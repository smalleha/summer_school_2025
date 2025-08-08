#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include <cmath>

#define FOLLOW_DISTANCE 0.5  // 目标跟随距离（米）
#define MAX_SPEED 0.5        // 最大线速度
#define KP_LINEAR 1.0        // 线速度比例增益
#define KP_ANGULAR 2.0       // 角速度比例增益

ros::Publisher velocity_publisher;

void followLeader()
{
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    tf::StampedTransform transform2;

    try
    {
        // 获取limo1相对于map的变换
        listener.waitForTransform("/map", "/limo1/base_link", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/map", "/limo1/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    // 获取limo1的偏航角（yaw）
    tf::Quaternion q;
    transform.getBasis().getRotation(q);  // 提取旋转部分
    double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));  // 从四元数中提取偏航角

    // 计算目标点位置（在limo1车头前进方向的右侧0.5米）
    double target_x = transform.getOrigin().x() + (0.5) * sin(yaw);  // 右侧方向
    double target_y = transform.getOrigin().y() + (0.5) * cos(yaw);  // 右侧方向

    try
    {
        // 获取limo2相对于map的变换
        listener.waitForTransform("/map", "/limo2", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/map", "/limo2", ros::Time(0), transform2);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    // 计算limo2到目标点的距离和角度
    double dx = target_x - transform2.getOrigin().x();
    double dy = target_y - transform2.getOrigin().y();
    double distance = sqrt(dx * dx + dy * dy);
    double angle_to_target = atan2(dy, dx);

    // 获取limo2的偏航角（yaw）
    tf::Quaternion q2;
    transform2.getBasis().getRotation(q2);  // 提取旋转部分
    double yaw2 = atan2(2.0 * (q2.w() * q2.z() + q2.x() * q2.y()), 1.0 - 2.0 * (q2.y() * q2.y() + q2.z() * q2.z()));  // 从四元数中提取偏航角

    // 计算需要调整的角度
    double angle_diff = angle_to_target - yaw2;
    if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    geometry_msgs::Twist vel_msg;

    if (distance > FOLLOW_DISTANCE)
    {
        vel_msg.linear.x = std::min(KP_LINEAR * (distance - FOLLOW_DISTANCE), MAX_SPEED);
        vel_msg.angular.z = KP_ANGULAR * angle_diff;
    }
    else
    {
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
    }

    velocity_publisher.publish(vel_msg);

    ROS_INFO("Target Pos: (%.2f, %.2f) | Distance: %.2f | Linear Vel: %.2f | Angular Vel: %.2f", 
             target_x, target_y, distance, vel_msg.linear.x, vel_msg.angular.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follower_robot");
    ros::NodeHandle nh;

    velocity_publisher = nh.advertise<geometry_msgs::Twist>("/limo2/cmd_vel", 10);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        followLeader();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}