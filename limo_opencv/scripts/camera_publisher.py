#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def camera_publisher():
    # 初始化 ROS 节点
    rospy.init_node('camera_publisher', anonymous=True)
    
    # 创建图像发布者
    image_pub = rospy.Publisher("camera_image", Image, queue_size=10)
    
    # 创建 OpenCV 桥接对象
    bridge = CvBridge()
    
    # 打开摄像头
    cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
    if not cap.isOpened():
        rospy.logerr("Cannot open camera")
        return
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        
        if not ret:
            rospy.logerr("Failed to capture frame")
            break
        
        try:
            # 将 OpenCV 图像转换为 ROS 图像消息
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            continue
        
        # 发布图像消息
        image_pub.publish(ros_image)
        rate.sleep()
    
    # 释放摄像头
    cap.release()
