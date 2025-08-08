#!/usr/bin/env python
# coding=utf-8
# 主车发送主车坐标以及主车速度给从车 - 适配仿真环境，使用ROS话题代替UDP通信

import math
import rospy
import tf
from numpy import array
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry  

odom_vx = 0
odom_vy = 0
odom_az = 0

def odom_callback(msg):
    global odom_vx, odom_vy, odom_az
    odom_vx = msg.twist.twist.linear.x
    odom_vy = msg.twist.twist.linear.y
    odom_az = msg.twist.twist.angular.z
    rospy.logdebug("Odom velocity: vx=%f, vy=%f, az=%f", odom_vx, odom_vy, odom_az)

def publishOdom():
    global odom_vx, odom_vy, odom_az
    rospy.init_node('send_tfodom', anonymous=True)
    
    # 使用正确的命名空间前缀订阅odom话题
    rospy.Subscriber('odom', Odometry, odom_callback)
    
    # 创建发布者，发布主车位置和速度信息
    pub = rospy.Publisher('/multfodom', Float32MultiArray, queue_size=10)
    
    listener = tf.TransformListener()
    rate = rospy.Rate(15.0)
    
    # 等待tf树建立
    rospy.sleep(5.0)
    rospy.loginfo("开始发布主车位置和速度信息")
    
    while not rospy.is_shutdown():
        try:
            # 修改为使用limo1/odom到limo1/base_link的转换，这在仿真环境中是可用的
            (trans, rot) = listener.lookupTransform("limo1/odom", "limo1/base_link", rospy.Time(0))
            
            # 由于我们使用的是相对于odom的位置，这里直接使用trans和rot
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
            
            # 创建消息并发布
            msg = Float32MultiArray()
            msg.data = [trans[0], trans[1], yaw, odom_vx, odom_vy, odom_az]
            pub.publish(msg)
            
            rospy.loginfo("Published: x=%f, y=%f, yaw=%f, vx=%f, vy=%f, az=%f", 
                         trans[0], trans[1], yaw, odom_vx, odom_vy, odom_az)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF错误: %s", e)
            rospy.sleep(1.0)
            continue
            
        rate.sleep()

if __name__ == '__main__':
    try:
        publishOdom()
    except rospy.ROSInterruptException:
        pass
