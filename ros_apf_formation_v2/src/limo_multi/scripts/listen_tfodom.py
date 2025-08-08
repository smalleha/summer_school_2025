#!/usr/bin/env python
# coding=utf-8
# 从车接收主车坐标以及主车速度，并通过话题multfodom发布出来 - 适配仿真环境，使用ROS话题代替UDP通信

import math
import rospy
from std_msgs.msg import Float32MultiArray

def multfodom_callback(msg):
    # 直接转发消息到本地命名空间
    local_pub.publish(msg)
    rospy.logdebug("Received and forwarded leader car data")

def FrameListener():
    global local_pub
    
    # 初始化节点，使用匿名参数以避免节点名冲突
    rospy.init_node('listen_tfodom', anonymous=True)
    
    # 创建本地发布者，在从车命名空间内发布主车信息
    local_pub = rospy.Publisher('multfodom', Float32MultiArray, queue_size=10)
    
    # 订阅主车发布的位置和速度信息
    rospy.Subscriber('/multfodom', Float32MultiArray, multfodom_callback)
    
    rospy.loginfo("Follower car listening for leader car data")
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    try:
        FrameListener()
    except rospy.ROSInterruptException:
        pass
