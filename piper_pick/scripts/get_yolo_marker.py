#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Point

PICK, OBJECT, PLACE = 0, 1, 2

temp = {PICK: Pose(), OBJECT: Pose(), PLACE: Pose()}
seq_count = {PICK: 0, OBJECT: 0, PLACE: 0}
real_position = {PICK: Pose(), OBJECT: Pose(), PLACE: Pose()}

def marker_callback(msg):
    for i, marker in enumerate(msg.markers):
        id_num = 0
        text = marker.text.strip()
        tokens = text.split()

        if len(tokens) < 2:
            continue  # 无效数据，跳过

        label = tokens[0]  # "keyboard"
        try:
            confidence = float(tokens[1])
        except ValueError:
            continue 
        if id_num in (PICK, OBJECT, PLACE) and label == "keyboard":
            temp[id_num].position.x += marker.pose.position.x
            temp[id_num].position.y += marker.pose.position.y
            temp[id_num].position.z += marker.pose.position.z
            
            count = seq_count[id_num]
            seq_count[id_num] += 1
            if count == 2:  # 第三次接收到数据，计算平均
                real_position[id_num].position.x = temp[id_num].position.x / 3.0
                real_position[id_num].position.y = temp[id_num].position.y / 3.0
                real_position[id_num].position.z = temp[id_num].position.z / 3.0
                temp[id_num] = Pose()
                seq_count[id_num] = 0
    rospy.loginfo(f"[{label}] 平均位置：x = {real_position[id_num].position.x:.3f}, "
                  f"y = {real_position[id_num].position.y:.3f}, "
                  f"z = {real_position[id_num].position.z:.3f}")


def listener():
    rospy.init_node('marker_array_listener', anonymous=True)
    rospy.Subscriber("/object_centroids", MarkerArray, marker_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
