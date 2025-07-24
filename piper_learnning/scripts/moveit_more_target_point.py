#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import sys
import copy

def main():
    # 初始化 MoveIt Commander 和 ROS 节点
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_more_point", anonymous=True)

    # 创建 MoveGroup 接口对象
    move_group = moveit_commander.MoveGroupCommander("arm")

    # 获取末端执行器链接名称
    end_effector_link = move_group.get_end_effector_link()

    # 设置容差、重规划、速度、加速度参数
    move_group.allow_replanning(True)
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.01)
    move_group.set_max_velocity_scaling_factor(0.2)
    move_group.set_max_acceleration_scaling_factor(0.2)

    # 回到初始化位置 "home"
    move_group.set_named_target("home")
    move_group.go(wait=True)
    rospy.sleep(1)

    # 设置起始点、过渡点、目标点
    start_pose = geometry_msgs.msg.Pose()
    start_pose.orientation.x = 1.0
    start_pose.orientation.y = -5.69991e-05
    start_pose.orientation.z = 4.14520e-05
    start_pose.orientation.w = 6.84446e-07
    start_pose.position.x = 0.183
    start_pose.position.y = -7.051e-6
    start_pose.position.z = 0.1542

    mid_pose = geometry_msgs.msg.Pose()
    mid_pose.orientation.x = 1.0
    mid_pose.orientation.y = -3.88606e-06
    mid_pose.orientation.z = -7.30472e-05
    mid_pose.orientation.w = 5.33069e-07
    mid_pose.position.x = 0.245193
    mid_pose.position.y = 1.09666e-6
    mid_pose.position.z = 0.212405

    goal_pose = geometry_msgs.msg.Pose()
    goal_pose.orientation.x = 1.0
    goal_pose.orientation.y = -2.70514e-05
    goal_pose.orientation.z = 3.82349e-05
    goal_pose.orientation.w = -1.8513e-05
    goal_pose.position.x = 0.245196
    goal_pose.position.y = -4.95272e-6
    goal_pose.position.z = 0.323646

    # 构造路点列表
    waypoints = [copy.deepcopy(start_pose), copy.deepcopy(mid_pose), copy.deepcopy(goal_pose)]

    # 降低速度以保证精度
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    # 计算笛卡尔轨迹
    eef_step = 0.01
    jump_threshold = 0.0
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, eef_step, jump_threshold)

    # 执行规划好的路径
    move_group.execute(plan, wait=True)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
