#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <cmath>

// 取消当前导航任务
void cancelCurrentGoal(ros::Publisher& cancel_publisher)
{
    actionlib_msgs::GoalID cancel_msg;
    cancel_msg.id = "";
    cancel_msg.stamp = ros::Time::now();
    cancel_publisher.publish(cancel_msg);
    ROS_INFO("Current goal canceled.");
}

// 发送目标点到 move_base
void sendGoalToMoveBase(const tf::StampedTransform& transform, ros::Publisher& goal_publisher)
{
    move_base_msgs::MoveBaseActionGoal goal;

    goal.header.frame_id = "map"; 
    goal.header.stamp = ros::Time::now();

    goal.goal.target_pose.header.frame_id = "map";
    goal.goal.target_pose.header.stamp = ros::Time::now();

    // 获取limo1的偏航角（yaw）
    tf::Quaternion quat;
    transform.getBasis().getRotation(quat);
    double yaw = tf::getYaw(quat);

    // 计算目标点位置（在limo1车头前进方向的右侧0.5米）
    double x = transform.getOrigin().x() + 0.5 * sin(yaw);  // 右侧方向
    double y = transform.getOrigin().y() - 0.5 * cos(yaw);  // 右侧方向

    goal.goal.target_pose.pose.position.x = x;
    goal.goal.target_pose.pose.position.y = y;
    goal.goal.target_pose.pose.position.z = 0.0;

    // 保持limo2的朝向与limo1一致
    goal.goal.target_pose.pose.orientation.x = quat.x();
    goal.goal.target_pose.pose.orientation.y = quat.y();
    goal.goal.target_pose.pose.orientation.z = quat.z();
    goal.goal.target_pose.pose.orientation.w = quat.w();

    goal_publisher.publish(goal);
    ROS_INFO("LIMO2 Goal sent to (%.2f, %.2f)", x, y);
}

void followLeader(ros::Publisher& goal_publisher, ros::Publisher& cancel_publisher, const std::string& leader_frame, const std::string& map_frame)
{
    static tf::TransformListener listener;

    try
    {
        tf::StampedTransform transform;
        listener.waitForTransform(map_frame, leader_frame, ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform(map_frame, leader_frame, ros::Time(0), transform);

        if (transform.frame_id_ == "")
        {
            ROS_ERROR("Transform frame_id is empty. Check TF tree.");
            return;
        }

        cancelCurrentGoal(cancel_publisher);  // 取消当前目标
        sendGoalToMoveBase(transform, goal_publisher);  // 发送新目标
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "limo_follow1_triangle");
    ros::NodeHandle nh;

    std::string leader_frame, map_frame;
    nh.param<std::string>("leader_frame", leader_frame, "limo1/base_link");
    nh.param<std::string>("map_frame", map_frame, "map");

    ROS_INFO("Leader frame: %s", leader_frame.c_str());
    ROS_INFO("Map frame: %s", map_frame.c_str());

    ros::Publisher goal_publisher = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/limo2/move_base/goal", 1);
    ros::Publisher cancel_publisher = nh.advertise<actionlib_msgs::GoalID>("/limo2/move_base/cancel", 1);

    // 初始化后延迟15秒，确保TF树稳定
    ROS_INFO("Initialization complete. Waiting for 15 seconds...");
    ros::Duration(15.0).sleep();

    ros::Rate loop_rate(2); 

    while (ros::ok())
    {
        followLeader(goal_publisher, cancel_publisher, leader_frame, map_frame);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}