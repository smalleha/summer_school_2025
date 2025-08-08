#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include <cmath>

#define FOLLOW_DISTANCE 1  // 目标跟随距离（米）
#define MAX_SPEED 0.5        // 最大线速度
#define KP_LINEAR 1.0        // 线速度比例增益
#define KP_ANGULAR 2.0       // 角速度比例增益

ros::Publisher velocity_publisher;

void followLeader()
{
    static tf::TransformListener listener;
    tf::StampedTransform transform;

    try
    {
        listener.waitForTransform("/limo3", "/limo1", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/limo3", "/limo1", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    double dx = transform.getOrigin().x();  // 主机在从机坐标系的 x 方向距离
    double dy = transform.getOrigin().y();  // 主机在从机坐标系的 y 方向距离
    double distance = sqrt(dx * dx + dy * dy);
    double angle_to_leader = atan2(dy, dx); // 计算主机相对从机的角度

    geometry_msgs::Twist vel_msg;

    if (distance > FOLLOW_DISTANCE)
    {
        vel_msg.linear.x = std::min(KP_LINEAR * (distance - FOLLOW_DISTANCE), MAX_SPEED);
        vel_msg.angular.z = KP_ANGULAR * angle_to_leader;
    }
    else
    {
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
    }

    velocity_publisher.publish(vel_msg);

    ROS_INFO("Leader Pos: (%.2f, %.2f) | Distance: %.2f | Linear Vel: %.2f | Angular Vel: %.2f", 
             dx, dy, distance, vel_msg.linear.x, vel_msg.angular.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follower2_robot");
    ros::NodeHandle nh;

    velocity_publisher = nh.advertise<geometry_msgs::Twist>("/limo3/cmd_vel", 10);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        followLeader();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
