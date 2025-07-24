#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
import time

def main():
    # 初始化 moveit_commander 和 ROS 节点
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_random", anonymous=True)

    # 创建 MoveGroup 接口（对应规划组名）
    move_group = moveit_commander.MoveGroupCommander("arm")

    # 获取末端执行器 link 名
    end_effector_link = move_group.get_end_effector_link()

    # 设置参考参数
    move_group.allow_replanning(True)
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.01)
    move_group.set_max_velocity_scaling_factor(0.2)
    move_group.set_max_acceleration_scaling_factor(0.2)

    # 回到初始位置 "home"
    move_group.set_named_target("home")
    move_group.go(wait=True)
    rospy.sleep(1)

    # 设置随机目标
    move_group.set_random_target()

    rospy.loginfo("Moving to random target...")

    # 执行运动
    move_group.go(wait=True)

    # 清理和关闭
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
