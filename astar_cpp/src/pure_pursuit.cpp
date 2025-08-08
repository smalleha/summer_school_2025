#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

class PurePursuitController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Publisher cmd_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<geometry_msgs::PoseStamped> waypoints_;
    double lookahead_distance_ = 0.5;

public:
    PurePursuitController() : tf_listener_(tf_buffer_) {
        path_sub_ = nh_.subscribe("/astar_path", 10, &PurePursuitController::pathCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        waypoints_ = msg->poses;
    }

    void update() {
        if (waypoints_.empty()) return;

        geometry_msgs::TransformStamped tf_stamped;
        // try {
        //     tf_stamped = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
        // } catch (tf2::TransformException &ex) {
        //     ROS_WARN("%s", ex.what());
        //     return;
        // }

        // 找到 lookahead 点
        geometry_msgs::Point target;
        for (const auto& wp : waypoints_) {
            double dx = wp.pose.position.x - tf_stamped.transform.translation.x;
            double dy = wp.pose.position.y - tf_stamped.transform.translation.y;
            double dist = sqrt(dx * dx + dy * dy);
            if (dist >= lookahead_distance_) {
                target = wp.pose.position;
                break;
            }
        }

        // 计算转向角
        double angle = atan2(target.y - tf_stamped.transform.translation.y,
                             target.x - tf_stamped.transform.translation.x);

        // 发布控制指令
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.5;  // 线速度
        cmd.angular.z = 2.0 * (angle - tf_stamped.transform.rotation.z); // 角速度
        cmd_pub_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuitController controller;
    ros::Rate rate(10);
    while (ros::ok()) {
        controller.update();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
