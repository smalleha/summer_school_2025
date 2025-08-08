#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib>  // 用于随机数生成
#include <ctime>

#define SAFE_DISTANCE 0.8   // 停障的最小安全距离
#define AVOID_DISTANCE 0.8  // 避障的安全距离
#define FRONT_ANGLE_RANGE 90  // 前方 ±30° 角度范围
#define TURN_SPEED 1.5
#define TURN_DURATION 1.5

ros::Publisher velocity_publisher;
double min_distance = 100.0;  // 最小障碍物距离
double left_distance = 100.0; // 左侧障碍物距离
double right_distance = 100.0; // 右侧障碍物距离

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr &laser_msg)
{
    min_distance = 100.0;
    left_distance = 100.0;
    right_distance = 100.0;

    int total_lasers = laser_msg->ranges.size();
    int center_index = total_lasers / 2;
    int start_index = center_index - FRONT_ANGLE_RANGE / 2;
    int end_index = center_index + FRONT_ANGLE_RANGE / 2;
    int left_index = center_index - FRONT_ANGLE_RANGE / 4;
    int right_index = center_index + FRONT_ANGLE_RANGE / 4;

    for (int i = start_index; i <= end_index; i++)
    {
        if (laser_msg->ranges[i] < min_distance)
        {
            min_distance = laser_msg->ranges[i];
        }
    }

    for (int i = start_index; i <= left_index; i++)
    {
        if (laser_msg->ranges[i] < left_distance)
        {
            left_distance = laser_msg->ranges[i];
        }
    }

    for (int i = right_index; i <= end_index; i++)
    {
        if (laser_msg->ranges[i] < right_distance)
        {
            right_distance = laser_msg->ranges[i];
        }
    }

    ROS_INFO("Front min: %.3f m | Left min: %.3f m | Right min: %.3f m", min_distance, left_distance, right_distance);
}

void moveRobot(double linear_speed, double angular_speed)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = linear_speed;
    vel_msg.angular.z = angular_speed;
    velocity_publisher.publish(vel_msg);
}

void wander()
{
    if (min_distance < SAFE_DISTANCE)
    {
        ROS_WARN("Obstacle detected! Stopping...");
        moveRobot(0.0, 0.0);  // 停止
        ros::Duration(1.0).sleep();
        double distance = right_distance - left_distance;
        // 根据左右避障情况决定转向方向
        if (distance > 0)
        {
            ROS_WARN("Obstacle on RIGHT! Turning LEFT .");
            moveRobot(0.0, TURN_SPEED);  // 向左转
            ros::Duration(TURN_DURATION).sleep();        
        }
        else
        {
            ROS_WARN("Obstacle on LEFT! Turning RIGHT.");
            moveRobot(0.0, -TURN_SPEED);  // 向右转
            ros::Duration(TURN_DURATION).sleep();       
        }
    }
    else
    {
        ROS_INFO("Moving forward...");
        moveRobot(0.3, 0.0);
    }

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "master_robot");
    ros::NodeHandle nh;

    velocity_publisher = nh.advertise<geometry_msgs::Twist>("/limo1/cmd_vel", 10);
    ros::Subscriber laser_sub = nh.subscribe("/limo1/scan", 10, laserCallBack);

    srand(time(0));  // 随机数种子
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        wander();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
