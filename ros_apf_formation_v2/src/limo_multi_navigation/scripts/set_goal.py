#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

def cancel_current_goal(cancel_publisher):
    """取消当前任务"""
    cancel_msg = GoalID()
    cancel_msg.id = ""
    cancel_msg.stamp = rospy.Time.now()
    cancel_publisher.publish(cancel_msg)
    rospy.loginfo("Current goal canceled.")

def send_goal(goal_publisher, x, y):
    """发送目标点"""
    goal = MoveBaseActionGoal()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    goal.goal.target_pose.header.frame_id = "map"
    goal.goal.target_pose.header.stamp = rospy.Time.now()
    goal.goal.target_pose.pose.position.x = x
    goal.goal.target_pose.pose.position.y = y
    goal.goal.target_pose.pose.orientation.w = 1.0  # 默认朝向

    goal_publisher.publish(goal)
    rospy.loginfo(f"Goal sent to ({x}, {y})")

def main():
    rospy.init_node('send_goal_node')

    # 创建发布器
    goal_publisher = rospy.Publisher('/limo1/move_base/goal', MoveBaseActionGoal, queue_size=1)
    cancel_publisher = rospy.Publisher('/limo1/move_base/cancel', GoalID, queue_size=1)

    while not rospy.is_shutdown():
        try:
            # 目标点坐标
            x = float(input("输入x: "))  # 目标点的 x 坐标
            y = float(input("输入y: "))  # 目标点的 y 坐标

            # 取消当前任务
            cancel_current_goal(cancel_publisher)

            # 发送新目标点
            send_goal(goal_publisher, x, y)

            # 等待 10 秒
            rospy.sleep(1.0)

        except ValueError:
            rospy.loginfo("输入无效，请输入数字。")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass