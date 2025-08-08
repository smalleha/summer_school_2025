#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <algorithm> // for std::min

// 回调函数，接收同步后的轮式里程计和激光里程计的里程数据
void syncCallback(const nav_msgs::Odometry::ConstPtr &limo1_odom_msg,
                  const nav_msgs::Odometry::ConstPtr &limo2_odom_msg)
{
    ros::Time sync_time = std::max(limo1_odom_msg->header.stamp, limo2_odom_msg->header.stamp);
    // ROS_INFO("Synchronized Limo1 Odometry timestamp: %f", limo1_odom_msg->header.stamp.toSec());
    // ROS_INFO("Synchronized Limo2 Odometry timestamp: %f", limo2_odom_msg->header.stamp.toSec());
    // ROS_INFO("Selected smallest timestamp: %f", sync_time.toSec());

    static tf::TransformBroadcaster limo1_br, limo2_br;
    tf::Transform limo1_transform, limo2_transform;
    tf::Quaternion limo1_q, limo2_q;

    // 发布 limo1 的 TF
    limo1_transform.setOrigin(tf::Vector3(limo1_odom_msg->pose.pose.position.x, 
                                          limo1_odom_msg->pose.pose.position.y, 
                                          limo1_odom_msg->pose.pose.position.z));
    limo1_q.setX(limo1_odom_msg->pose.pose.orientation.x);
    limo1_q.setY(limo1_odom_msg->pose.pose.orientation.y);
    limo1_q.setZ(limo1_odom_msg->pose.pose.orientation.z);
    limo1_q.setW(limo1_odom_msg->pose.pose.orientation.w);
    limo1_transform.setRotation(limo1_q);
    limo1_br.sendTransform(tf::StampedTransform(limo1_transform, sync_time, "limo1/odom", "limo1/base_footprint"));

    // 发布 limo2 的 TF
    limo2_transform.setOrigin(tf::Vector3(limo2_odom_msg->pose.pose.position.x, 
                                          limo2_odom_msg->pose.pose.position.y, 
                                          limo2_odom_msg->pose.pose.position.z));
    limo2_q.setX(limo2_odom_msg->pose.pose.orientation.x);
    limo2_q.setY(limo2_odom_msg->pose.pose.orientation.y);
    limo2_q.setZ(limo2_odom_msg->pose.pose.orientation.z);
    limo2_q.setW(limo2_odom_msg->pose.pose.orientation.w);
    limo2_transform.setRotation(limo2_q);
    limo2_br.sendTransform(tf::StampedTransform(limo2_transform, sync_time, "limo2/odom", "limo2/base_footprint"));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_sync_node");
    ros::NodeHandle nh;

    // 创建消息过滤器订阅器
    message_filters::Subscriber<nav_msgs::Odometry> limo1_odom_sub(nh, "/limo1/odom", 1);
    message_filters::Subscriber<nav_msgs::Odometry> limo2_odom_sub(nh, "/limo2/odom", 1);

    // 定义时间同步策略（使用约束时间同步）
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), limo1_odom_sub, limo2_odom_sub);

    // 注册回调函数
    sync.registerCallback(boost::bind(&syncCallback, _1, _2));

    ros::spin();
    return 0;
}
