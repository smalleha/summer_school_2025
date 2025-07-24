#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg
import sys
import copy
import time

def main():
    # 初始化 moveit_commander 和 ROS 节点
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_cartesian_path', anonymous=True)

    # 创建 MoveGroupCommander 接口，控制 "arm" 规划组
    move_group = moveit_commander.MoveGroupCommander("arm")

    # 获取终端 Link 名称
    end_effector_link = move_group.get_end_effector_link()

    # 设置参考坐标系
    reference_frame = move_group.get_planning_frame()
    move_group.set_pose_reference_frame(reference_frame)

    # 允许 replanning
    move_group.allow_replanning(True)

    # 设置位置和姿态的允许误差
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.01)

    # 设置最大速度和加速度缩放因子
    move_group.set_max_acceleration_scaling_factor(0.2)
    move_group.set_max_velocity_scaling_factor(0.2)

    # 控制机械臂先回到初始化位置（需要已配置好 "home" 位姿）
    move_group.set_named_target("home")
    move_group.go(wait=True)
    rospy.sleep(1)

    # 获取当前位姿
    start_pose = move_group.get_current_pose(end_effector_link).pose

    # 设置路点列表
    waypoints = []

    # 添加初始点
    waypoints.append(copy.deepcopy(start_pose))

    # 向上移动 z
    start_pose.position.z += 0.2
    waypoints.append(copy.deepcopy(start_pose))

    # 向 x 正方向移动
    start_pose.position.x += 0.1
    waypoints.append(copy.deepcopy(start_pose))

    # 向 y 正方向移动
    start_pose.position.y += 0.1
    waypoints.append(copy.deepcopy(start_pose))

    # 计算笛卡尔路径
    eef_step = 0.01
    jump_threshold = 0.0
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, eef_step, jump_threshold)

    # 执行轨迹
    move_group.execute(plan, wait=True)

    # 关闭
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
