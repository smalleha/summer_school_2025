#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import sys

def main():
    # 初始化 moveit_commander 和 ROS 节点
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_target_point', anonymous=True)

    # 创建 MoveGroup 接口对象，指定规划组名称为 "arm"
    move_group = moveit_commander.MoveGroupCommander("arm")

    # 获取末端执行器的 link 名称
    end_effector_link = move_group.get_end_effector_link()

    # 设置规划器参数
    move_group.allow_replanning(True)
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.01)
    move_group.set_max_velocity_scaling_factor(0.2)
    move_group.set_max_acceleration_scaling_factor(0.2)

    # 控制机械臂回到初始化位置 "home"
    move_group.set_named_target("home")
    move_group.go(wait=True)
    rospy.sleep(1)

    # 创建目标点
    target_point = geometry_msgs.msg.Pose()
    target_point.orientation.x = 1.0
    target_point.orientation.y = -1.98016e-05
    target_point.orientation.z = 6.61578e-05
    target_point.orientation.w = -8.60138e-07
    target_point.position.x = 0.2
    target_point.position.y = 0.001
    target_point.position.z = 0.25

    # 设置目标位姿
    move_group.set_pose_target(target_point)

    # 执行路径规划
    plan = move_group.plan()

    # 检查规划是否成功
    if plan and plan[0]:  # plan是一个tuple (success_flag, plan)
        rospy.loginfo("Planning succeeded, executing trajectory...")
        move_group.execute(plan[1], wait=True)
    else:
        rospy.logwarn("Failed to plan and execute!")

    # 清理资源
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
