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

    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    goal.goal.target_pose.pose.position.x = x;
    goal.goal.target_pose.pose.position.y = y + 0.8;
    goal.goal.target_pose.pose.position.z = 0.0;

    tf::Quaternion quat;
    transform.getBasis().getRotation(quat); 
    quat.normalize(); 

    goal.goal.target_pose.pose.orientation.x = quat.x();
    goal.goal.target_pose.pose.orientation.y = quat.y();
    goal.goal.target_pose.pose.orientation.z = quat.z();
    goal.goal.target_pose.pose.orientation.w = quat.w();

    if (x != 0 && y != 0)
    {
        goal_publisher.publish(goal);
        ROS_INFO("LIMO3_Goal sent to (%.2f, %.2f)", x, y);
    }
    else
    {
        ROS_INFO("NOT START");
    }
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

        cancelCurrentGoal(cancel_publisher);
        sendGoalToMoveBase(transform, goal_publisher);
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "limo_follow2_triangle");
    ros::NodeHandle nh;

    std::string leader_frame, map_frame;
    nh.param<std::string>("leader_frame", leader_frame, "limo1/base_link");
    nh.param<std::string>("map_frame", map_frame, "map");

    ROS_INFO("Leader frame: %s", leader_frame.c_str());
    ROS_INFO("Map frame: %s", map_frame.c_str());

    ros::Publisher goal_publisher = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/limo3/move_base/goal", 1);
    ros::Publisher cancel_publisher = nh.advertise<actionlib_msgs::GoalID>("/limo3/move_base/cancel", 1);

    // 初始化后延迟10秒
    ROS_INFO("Initialization complete. Waiting for 10 seconds...");
    ros::Duration(15.0).sleep();  // 延迟10秒

    ros::Rate loop_rate(2); 

    while (ros::ok())
    {
        followLeader(goal_publisher, cancel_publisher, leader_frame, map_frame);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}