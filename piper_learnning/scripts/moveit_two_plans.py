#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander

def main():
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_revise_trajectory", anonymous=True)

    move_group = moveit_commander.MoveGroupCommander("arm")
    robot = moveit_commander.RobotCommander()

    # 设置参数
    move_group.set_goal_joint_tolerance(0.001)
    acc_scale = 0.8
    vel_scale = 0.8
    move_group.set_max_acceleration_scaling_factor(acc_scale)
    move_group.set_max_velocity_scaling_factor(vel_scale)

    # 回 home 位姿
    move_group.set_named_target("home")
    move_group.go(wait=True)
    rospy.sleep(1)

    # 第一段轨迹：关节1 = -0.6
    joint_goal1 = move_group.get_current_joint_values()
    joint_goal1[0] = -0.6
    move_group.set_joint_value_target(joint_goal1)
    success1 = move_group.go(wait=True)
    rospy.sleep(1)

    if not success1:
        rospy.logerr("Failed to move to joint_goal1")
        return

    # 第二段轨迹：关节1 = 1.2, 关节2 = -0.5
    joint_goal2 = move_group.get_current_joint_values()
    joint_goal2[0] = 0.6
    joint_goal2[1] = 0.3
    move_group.set_joint_value_target(joint_goal2)
    success2 = move_group.go(wait=True)
    rospy.sleep(1)

    if not success2:
        rospy.logerr("Failed to move to joint_goal2")
        return

    rospy.loginfo("Trajectory executed successfully")

    # 回 home
    move_group.set_named_target("home")
    move_group.go(wait=True)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
