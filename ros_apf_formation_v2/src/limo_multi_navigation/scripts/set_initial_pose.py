#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

def set_initial_pose():
    rospy.init_node('set_initial_pose', anonymous=True)
    pub = rospy.Publisher('/limo2/initialpose', PoseWithCovarianceStamped, queue_size=10)

    rospy.loginfo("请输入机器人的初始位置和朝向：")
    while not rospy.is_shutdown():
        try:
            # 获取用户输入
            x = -1.0
            y = 1.0
            yaw = 0.0

            # 将角度转换为弧度
            yaw_rad = yaw * (3.141592653589793 / 180.0)

            # 创建初始位置消息
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.frame_id = "map"
            initial_pose.header.stamp = rospy.Time.now()
            initial_pose.pose.pose.position.x = x
            initial_pose.pose.pose.position.y = y

            # 设置朝向（四元数表示）
            quat = quaternion_from_euler(0, 0, yaw_rad)
            initial_pose.pose.pose.orientation.x = quat[0]
            initial_pose.pose.pose.orientation.y = quat[1]
            initial_pose.pose.pose.orientation.z = quat[2]
            initial_pose.pose.pose.orientation.w = quat[3]
            rospy.sleep(3)
            # 发布初始位置
            pub.publish(initial_pose)
            rospy.loginfo(f"LIMO2已设置初始位置：x={x}, y={y}, yaw={yaw}°")

            # 退出程序
            break
        except ValueError:
            rospy.logwarn("输入格式错误，请输入有效的数字。")

if __name__ == '__main__':
    try:
        set_initial_pose()
    except rospy.ROSInterruptException:
        rospy.loginfo("设置初始位置程序被中断。")