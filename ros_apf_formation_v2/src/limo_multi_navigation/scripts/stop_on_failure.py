#!/usr/bin/env python
import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist

class StopOnFailure:
    def __init__(self):
        rospy.init_node('stop_on_failure', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/limo2/cmd_vel', Twist, queue_size=10)
        self.move_base_status_sub = rospy.Subscriber('/limo2/move_base/status', GoalStatusArray, self.status_callback)
        self.zero_vel_msg = Twist()  # 创建一个速度为0的消息
        self.count_time = 0  # 初始化计数器
        self.rate = rospy.Rate(20)  # 设置循环频率为20Hz

    def status_callback(self, data):
        # 检查move_base的状态
        if data.status_list:
            status = data.status_list[-1].status  # 获取最新的状态
            if status != 1:  # 状态1表示ACTIVE
                self.count_time += 1
                rospy.logwarn("count:%d!!!!!!!!!!!!!!!!!!!!!!!!!!!!", self.count_time)
                if self.count_time >= 4:  # 0.5秒内累计10次（20Hz下，0.5秒约10次）
                    rospy.logwarn("LIMO2 STOP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    self.cmd_vel_pub.publish(self.zero_vel_msg)  # 发布速度为0的消息
                    self.count_time = 0  # 重置计数器
            else:
                self.count_time = 0  # 重置计数器

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = StopOnFailure()
        node.run()
    except rospy.ROSInterruptException:
        pass