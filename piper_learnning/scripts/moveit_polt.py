#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import copy
import sys

def main():
    # 初始化 moveit_commander 和 ROS 节点
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_plot', anonymous=True)

    # 初始化 MoveGroup 和 PlanningScene
    move_group = moveit_commander.MoveGroupCommander("lite6_arm")
    planning_scene_interface = moveit_commander.PlanningSceneInterface()

    # 获取末端 link
    end_effector_link = move_group.get_end_effector_link()

    # 设置参考系和参数
    move_group.allow_replanning(True)
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.01)
    move_group.set_max_velocity_scaling_factor(0.2)
    move_group.set_max_acceleration_scaling_factor(0.2)

    # 回到初始位置 home
    move_group.set_named_target("home")
    move_group.go(wait=True)
    rospy.sleep(1)

    # 设置起点和终点姿态（两者方向一致）
    start_pose = geometry_msgs.msg.Pose()
    start_pose.orientation.x = 1.0
    start_pose.orientation.y = 1.81909e-05
    start_pose.orientation.z = -4.69167e-05
    start_pose.orientation.w = -8.68015e-05
    start_pose.position.x = 0.26628
    start_pose.position.y = 0.056854
    start_pose.position.z = 0.154221

    target_pose1 = copy.deepcopy(start_pose)
    target_pose1.position.y = 0.156854
    target_pose1.position.z = 0.154193  # 稍微调整 Z

    # 移动到起始点
    move_group.set_start_state_to_current_state()
    move_group.set_pose_target(start_pose)
    move_group.go(wait=True)

    # 构造轨迹路径（waypoints）
    waypoints = []

    waypoints.append(copy.deepcopy(start_pose))

    waypoint_pose = copy.deepcopy(start_pose)
    waypoint_pose.position.x = 0.36628
    waypoint_pose.position.y = 0.156854
    waypoint_pose.position.z = 0.154221
    waypoints.append(copy.deepcopy(waypoint_pose))

    waypoint_pose.position.y = -0.15
    waypoints.append(copy.deepcopy(waypoint_pose))

    waypoint_pose.position.x = 0.26628
    waypoint_pose.position.y = -0.15
    waypoints.append(copy.deepcopy(waypoint_pose))

    # 最后设定终点为目标位置
    waypoints.append(copy.deepcopy(target_pose1))

    # 笛卡尔路径规划
    eef_step = 0.01
    jump_threshold = 0.0
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, eef_step, jump_threshold)

    # 执行轨迹
    if fraction > 0.95:
        rospy.loginfo("Executing Cartesian path with fraction: %.2f", fraction)
        move_group.execute(plan, wait=True)
    else:
        rospy.logwarn("Path planning incomplete: only %.2f of path achieved", fraction)

    # 回到 home 位姿
    move_group.set_named_target("home")
    move_group.go(wait=True)
    rospy.sleep(1)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
