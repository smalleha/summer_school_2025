#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
import time

def main():
    # 初始化 moveit_commander 和 ROS 节点
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_joint", anonymous=True)

    # 启动异步线程
    spinner = rospy.Rate(10)

    # 创建 MoveGroup 接口对象
    move_group = moveit_commander.MoveGroupCommander("arm")

    # 获取末端执行器的 link 名
    end_effector_link = move_group.get_end_effector_link()

    # 设置容差和规划参数
    move_group.allow_replanning(True)
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.01)
    move_group.set_max_acceleration_scaling_factor(0.2)
    move_group.set_max_velocity_scaling_factor(0.2)

    # 控制机械臂回到初始化位置 "home"
    move_group.set_named_target("home")
    move_group.go(wait=True)
    rospy.sleep(1)

    # 获取当前的关节角度（用于补全未设置的关节）
    joint_values = move_group.get_current_joint_values()

    # 设置第 6 个关节的角度
    joint_values[5] = 0.5897185686390605

    # 设置目标关节角
    move_group.set_joint_value_target(joint_values)

    rospy.loginfo("Moving to joint target...")
    move_group.go(wait=True)

    # 关闭 MoveIt
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
