#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi

def main():
    # 初始化 MoveIt API
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cartesian_path_demo', anonymous=True)

    # 初始化机械臂规划组（替换为你的 group name）
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"  # 修改为你的 MoveIt group 名称
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # 获取当前位姿作为起点
    start_pose = move_group.get_current_pose().pose

    # 设定一系列的目标点（相对于当前 pose 移动）
    waypoints = []

    # 初始位姿
    waypoints.append(start_pose)

    # 向 X 正方向移动 10cm
    wpose = geometry_msgs.msg.Pose()
    wpose.position.x = start_pose.position.x 
    wpose.position.y = start_pose.position.y 
    wpose.position.z = start_pose.position.z + 0.2
    wpose.orientation = start_pose.orientation  # 保持姿态不变
    waypoints.append(wpose)

    # 向 Z 方向上升 10cm
    # wpose2 = geometry_msgs.msg.Pose()
    # wpose2.position.x = wpose.position.x
    # wpose2.position.y = wpose.position.y
    # wpose2.position.z = wpose.position.z + 0.1
    # wpose2.orientation = start_pose.orientation
    # waypoints.append(wpose2)

    # 规划笛卡尔路径
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,   # 路径点列表
        0.01,        # 步长（m），越小越精细
        0.0          # 跳跃阈值（设为 0 关闭跳跃检测）
    )

    rospy.loginfo("Cartesian path planning complete, fraction: %.2f" % fraction)

    # 执行轨迹
    if fraction > 0.9:
        move_group.execute(plan, wait=True)
        rospy.loginfo("Path executed successfully.")
    else:
        rospy.logwarn("Only %.2f of the path was planned. Aborting." % fraction)

    # 清理并退出
    move_group.stop()
    move_group.clear_pose_targets()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
