#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 导入相关库
import copy, sys, math
import rospy, roslib, numpy
import moveit_commander, tf, tf2_ros
from moveit_commander import RobotCommander
from moveit_commander import MoveGroupCommander
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Point
from move_base_msgs.msg import MoveBaseActionResult
from copy import deepcopy
from moveit_python import MoveGroupInterface
# from moveit_msgs.msg import Constraints, JointConstraint
import moveit_python
import moveit_msgs.msg
import moveit_commander
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from ar_track_alvar_msgs.msg import AlvarMarkers
from visualization_msgs.msg import MarkerArray

# 定义AR标签的编号常量
PICK, OBJECT, PLACE = 0, 1, 2

# 主类定义
class MoveItPlanningDemo:
    def __init__(self):
        # 初始化moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node("cobot_visual_grab")

        # 存储检测到的AR标签的位置数据（用于取平均）
        self.temp = {PICK: Pose(), OBJECT: Pose(), PLACE: Pose()}
        self.seq_count = {PICK: 0, OBJECT: 0, PLACE: 0}
        self.real_position = {PICK: Pose(), OBJECT: Pose(), PLACE: Pose()}

        # 读取参数，用于区分抓取或放置阶段
        self.grab_param = rospy.get_param('/visual_grab/grab_param', default='grab_pose')
        self.yolo_grab = rospy.get_param("/visual_grab/yolo_grab",default="keyboard")
        # 初始化tf监听器
        self.tf_listener = tf.TransformListener()

        # 显示路径的Publisher
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # 订阅AR识别的marker消息
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_callback)

        rospy.Subscriber("/object_centroids", MarkerArray, self.yolo_callback)

        # 初始化机械臂控制组和夹爪控制组
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")

        # 初始化场景对象，用于添加/移除障碍物
        self.scene = moveit_commander.PlanningSceneInterface()

        # 获取末端执行器的link名称
        self.end_effector_link = self.arm.get_end_effector_link()

        # 设置误差容忍值
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)
        self.arm.set_max_velocity_scaling_factor(0.3)
        self.arm.set_max_acceleration_scaling_factor(0.3)

        # 创建MoveGroupInterface（备用）
        move_group = MoveGroupInterface("arm", "arm_base")

        # 当前识别到的marker ID
        self.id_num = int()

        # tf2静态坐标广播器（备用）
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        # 定义夹爪的偏移值，用于姿态调整
        self.gripper_z = 0.09
        self.gripper_x = 0.1
        self.gripper_y = 0.0

    # 打开夹爪
    def gripper_open(self):
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)
        rospy.sleep(1)
        self.gripper.stop()
        self.gripper.clear_pose_targets()

    # 关闭夹爪
    def gripper_close(self):
        self.gripper.set_named_target("close")
        self.gripper.go(wait=True)
        rospy.sleep(1)
        self.gripper.stop()
        self.gripper.clear_pose_targets()

    # 机械臂移动到home位
    def move2home(self):
        self.arm.set_named_target("home")
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

    # 机械臂移动到initial位
    def move2initial(self):
        self.arm.set_named_target("initial")
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        rospy.sleep(1)

    # AR标签回调函数
    def marker_callback(self, msg):
        if msg.markers:
            self.get_pickup_point(msg)

    # 获取AR标签的平均位置（连续识别3次取平均）
    def get_pickup_point(self, msg):
        for marker in msg.markers:
            self.id_num = marker.id
            if self.id_num in (PICK, OBJECT, PLACE):
                self.temp[self.id_num].position.x += marker.pose.pose.position.x
                self.temp[self.id_num].position.y += marker.pose.pose.position.y
                self.temp[self.id_num].position.z += marker.pose.pose.position.z

                count = self.seq_count[self.id_num]
                self.seq_count[self.id_num] += 1

                if count == 2:  # 第三次接收到数据，计算平均
                    self.real_position[self.id_num].position.x = self.temp[self.id_num].position.x / 3.0
                    self.real_position[self.id_num].position.y = self.temp[self.id_num].position.y / 3.0
                    self.real_position[self.id_num].position.z = self.temp[self.id_num].position.z / 3.0
                    self.temp[self.id_num] = Pose()
                    self.seq_count[self.id_num] = 0
       
    def yolo_callback(self, msg):
        for i, marker in enumerate(msg.markers):
            self.id_num = 0
            text = marker.text.strip()
            tokens = text.split()

            if len(tokens) < 2:
                continue  # 无效数据，跳过

            label = tokens[0]  # "keyboard"
            confidence = float(tokens[1])

            if self.id_num in (PICK, OBJECT, PLACE) and label == "mouse":
                self.temp[self.id_num].position.x += marker.pose.position.x
                self.temp[self.id_num].position.y += marker.pose.position.y
                self.temp[self.id_num].position.z += marker.pose.position.z

                count = self.seq_count[self.id_num]
                self.seq_count[self.id_num] += 1

                if count == 2:  # 第三次接收到数据，计算平均
                    self.real_position[self.id_num].position.x = self.temp[self.id_num].position.x / 3.0
                    self.real_position[self.id_num].position.y = self.temp[self.id_num].position.y / 3.0
                    self.real_position[self.id_num].position.z = self.temp[self.id_num].position.z / 3.0
                    self.real_position[self.id_num].position.z = self.real_position[self.id_num].position.z / 2

                    self.temp[self.id_num] = Pose()
                    self.seq_count[self.id_num] = 0

        # 使用笛卡尔路径方式规划移动路径
    def move_waypoints(self, target_pose):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        try:
            transform = tf_buffer.lookup_transform("arm_base", self.end_effector_link, rospy.Time(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to lookup transform from arm_base to end_effector_link")

        # 获取当前位姿
        start_current_pose = self.arm.get_current_pose().pose
        start_pose = start_current_pose

        # 构建路径点序列
        waypoints = []
        waypoints.append(start_pose)

        point2 = Pose()
        point2.position.x = start_pose.position.x
        point2.position.y = target_pose.position.y
        point2.position.z = start_pose.position.z
        point2.orientation = start_pose.orientation
        waypoints.append(copy.deepcopy(point2))

        point3 = Pose()
        point3.position.x = start_pose.position.x
        point3.position.y = target_pose.position.y
        point3.position.z = target_pose.position.z
        point3.orientation = start_pose.orientation
        waypoints.append(copy.deepcopy(point3))

        point1 = Pose()
        point1.position.x = target_pose.position.x
        point1.position.y = target_pose.position.y
        point1.position.z = target_pose.position.z
        point1.orientation = start_pose.orientation
        waypoints.append(copy.deepcopy(point1))

        # 多次尝试规划，直到成功
        fraction = 0.0
        maxtries = 100
        attempts = 0

        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path(
                waypoints, 0.01, 0.0, avoid_collisions=True
            )
            attempts += 1
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            rospy.sleep(3)
            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

        rospy.sleep(1)

        self.target_pose = Pose()

    # 控制第5个关节旋转一定角度
    def move_joint(self, angle_deg):
        joint_values = self.arm.get_current_joint_values()
        idx = 5  # 第五关节索引
        angle_rad = math.radians(angle_deg)
        joint_values[idx] -= angle_rad
        self.arm.set_joint_value_target(joint_values)
        success, plan, planning_time, error_code = self.arm.plan()
        if not success:
            rospy.logerr("规划失败，无法执行运动")
        else:
            exec_success = self.arm.execute(plan, wait=True)
            if exec_success:
                rospy.loginfo("第5关节已旋转 %.2f°", angle_deg)
            else:
                rospy.logerr("执行失败")

    # 沿直线向前移动一定距离
    def move_straight_pose(self, scale=1):
        rospy.sleep(2)
        rospy.loginfo("move_straight_pose")
        waypoints = []
        current_pose = self.arm.get_current_pose().pose
        wpose = copy.deepcopy(current_pose)
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z += scale * 0.0
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0, avoid_collisions=True)
        if fraction < 0.9:
            rospy.logwarn("compute_cartesian_path failed! fraction = %f", fraction)
        else:
            self.arm.execute(plan, wait=True)
            rospy.sleep(1)
        return fraction

    # 根据TF和AR位置计算目标位姿（用于抓取或放置）
    def get_tf(self, parent_frame, child_frame):
        try:
            self.target_pose = Pose()
            current_pose = self.arm.get_current_pose().pose
            if child_frame == "grab_pose" and self.id_num == 0:
                self.target_pose.position.x = current_pose.position.x + self.real_position[0].position.z - self.gripper_z
                self.target_pose.position.y = current_pose.position.y + (-self.real_position[0].position.y) + self.gripper_y
                self.target_pose.position.z = current_pose.position.z + (-self.real_position[0].position.x) + self.gripper_x
            elif child_frame == "place_pose" and self.id_num == 2:
                self.target_pose.position.x = current_pose.position.x + self.real_position[2].position.z - self.gripper_z
                self.target_pose.position.y = current_pose.position.y + (-self.real_position[2].position.y) + self.gripper_y
                self.target_pose.position.z = current_pose.position.z + (-self.real_position[2].position.x) + self.gripper_x
            else:
                return False
            rospy.loginfo(f"Current pose: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
            rospy.loginfo(f"real_position: x={self.real_position[self.id_num].position.x:.3f}, y={self.real_position[self.id_num].position.y:.3f}, z={self.real_position[self.id_num].position.z:.3f}")
            rospy.loginfo(self.target_pose)
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
            rospy.logwarn(e)
            return False

    # 主循环运行
    def run(self):
        while not rospy.is_shutdown():
            print("11111")
            self.move2initial()
            self.gripper_open()
            if self.grab_param == "grab_pose":
                if self.get_tf("arm_base", "grab_pose"):
                    print("grab_pose")
                    self.move_waypoints(self.target_pose)
                    self.gripper_close()
                    # self.move_joint5(20)
                    self.move_straight_pose()
                    self.move2home()
            print("2222")
            if self.grab_param == "place_pose":
                print("3333")
                self.move2initial()
                if self.get_tf("arm_base", "place_pose"):
                    print("place_pose")
                    self.move_waypoints(self.target_pose)
                    self.gripper_open()
                    rospy.sleep(1)
                    self.move_straight_pose()
                    self.move2home()
            rospy.sleep(1)

        # 程序结束时清理MoveIt资源
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


# 主程序入口
if __name__ == "__main__":
    node = MoveItPlanningDemo()
    node.run()
