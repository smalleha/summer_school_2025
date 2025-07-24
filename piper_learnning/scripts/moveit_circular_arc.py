#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import copy
import math
import sys
import time

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_circular_arc', anonymous=True)

    # 初始化 MoveGroup 接口
    move_group = moveit_commander.MoveGroupCommander("arm")

    # 获取终端 Link 名
    end_effector_link = move_group.get_end_effector_link()

    # 设置参考坐标系
    reference_frame = "arm_base"
    move_group.set_pose_reference_frame(reference_frame)

    # 允许重新规划
    move_group.allow_replanning(True)

    # 设置容差
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.01)

    # 设置速度和加速度比例
    move_group.set_max_velocity_scaling_factor(0.8)
    move_group.set_max_acceleration_scaling_factor(0.8)

    # 回到初始化位置
    move_group.set_named_target("home")
    move_group.go(wait=True)
    rospy.sleep(1)

    # 设置目标点
    target_pose = geometry_msgs.msg.Pose()
    target_pose.orientation.x = 0.701835
    target_pose.orientation.y = -0.00258698
    target_pose.orientation.z = 0.71233
    target_pose.orientation.w = 0.00269154
    target_pose.position.x = 0.30384
    target_pose.position.y = 0.0
    target_pose.position.z = 0.235383

    move_group.set_pose_target(target_pose)
    move_group.go(wait=True)

    # 构建圆弧轨迹路点
    waypoints = []
    waypoints.append(copy.deepcopy(target_pose))

    centerA = target_pose.position.y
    centerB = target_pose.position.z
    radius = 0.1

    for th in [i * 0.01 for i in range(0, int(2 * math.pi / 0.01))]:
        new_pose = copy.deepcopy(target_pose)
        new_pose.position.y = centerA + radius * math.cos(th)
        new_pose.position.z = centerB + radius * math.sin(th)
        waypoints.append(new_pose)

    # 规划笛卡尔轨迹
    eef_step = 0.01
    jump_threshold = 0.0
    maxtries = 100
    attempts = 0
    fraction = 0.0
    plan = None

    while fraction < 1.0 and attempts < maxtries:
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, eef_step, jump_threshold)
        attempts += 1
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after %d attempts...", attempts)

    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the arm.")
        move_group.execute(plan, wait=True)
        rospy.sleep(1)
    else:
        rospy.logwarn("Path planning failed with only %0.6f success after %d attempts.", fraction, attempts)

    # 回到初始化位置
    move_group.set_named_target("home")
    move_group.go(wait=True)
    rospy.sleep(1)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
