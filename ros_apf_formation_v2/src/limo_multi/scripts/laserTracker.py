#!/usr/bin/env python
# coding=utf-8
# 激光雷达避障功能，提取最近的障碍物距离信息

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from limo_multi.msg import avoid

class LaserTracker:
    def __init__(self):
        self.winSize = rospy.get_param('~winSize', 2)
        self.deltaDist = rospy.get_param('~deltaDist', 0.2)
        self.min_distance = float('inf')
        self.min_angle = 0
        
        # 发布障碍物信息
        self.obstacle_pub = rospy.Publisher('object_tracker/current_position', avoid, queue_size=10)
        
        # 订阅激光雷达数据
        self.laser_sub = rospy.Subscriber('scan', LaserScan, self.laser_callback)
        
        rospy.loginfo("LaserTracker initialized with winSize: %d, deltaDist: %.2f", self.winSize, self.deltaDist)
        
    def laser_callback(self, data):
        # 获取激光雷达数据
        ranges = np.array(data.ranges)
        angles = np.arange(data.angle_min, data.angle_max + data.angle_increment, data.angle_increment)
        
        # 过滤无效数据
        valid_indices = np.isfinite(ranges)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        if len(valid_ranges) == 0:
            return
        
        # 找到最近的障碍物
        min_idx = np.argmin(valid_ranges)
        self.min_distance = valid_ranges[min_idx]
        self.min_angle = valid_angles[min_idx]
        
        # 发布障碍物信息
        obstacle_msg = avoid()
        obstacle_msg.distance = self.min_distance
        obstacle_msg.angleX = self.min_angle
        obstacle_msg.angleY = 0.0  # 不使用Y角度
        
        self.obstacle_pub.publish(obstacle_msg)
        
if __name__ == '__main__':
    rospy.init_node('laser_tracker')
    tracker = LaserTracker()
    rospy.spin()
