#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class MPCController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber initialpose_sub_;  // 订阅 /initialpose
    ros::Publisher cmd_pub_;
    ros::Publisher marker_pub_;        // 发布 Marker
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    vector<Vector2d> waypoints_;
    OsqpEigen::Solver solver_;

    visualization_msgs::Marker marker_; // 机器人轨迹

    int N = 10;  // 预测步长
    double dt = 0.1; // 时间步长
    double robot_x = 0.0, robot_y = 0.0, robot_theta = 0.0;  // 机器人位姿

public:
    MPCController() : tf_listener_(tf_buffer_) {
        path_sub_ = nh_.subscribe("/astar_path", 10, &MPCController::pathCallback, this);
        initialpose_sub_ = nh_.subscribe("/initialpose", 10, &MPCController::initialPoseCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/robot_trajectory", 10);

        setupMarker();
        setupMPC();
    }

    // 初始化 marker
    void setupMarker() {
        marker_.header.frame_id = "map";
        marker_.ns = "robot_trajectory";
        marker_.type = visualization_msgs::Marker::LINE_STRIP;
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.scale.x = 0.05;  // 线宽
        marker_.color.r = 1.0;
        marker_.color.a = 1.0;
    }

    // 订阅路径
    void pathCallback(const nav_msgs::Path::ConstPtr &msg) {
        waypoints_.clear();
        for (const auto &pose : msg->poses) {
            waypoints_.emplace_back(Vector2d(pose.pose.position.x, pose.pose.position.y));
        }
    }

    // 订阅 /initialpose，获取机器人初始位姿
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
        robot_x = msg->pose.pose.position.x;
        robot_y = msg->pose.pose.position.y;
        robot_theta = tf::getYaw(msg->pose.pose.orientation);  // 角度转换
        ROS_INFO("Set initial pose: x=%.2f, y=%.2f, theta=%.2f", robot_x, robot_y, robot_theta);
    }

    // 设置 MPC
    void setupMPC() {
        int state_size = 4;  // 状态变量 [x, y, theta, v]
        int control_size = 2; // 控制变量 [加速度 a, 方向角 delta]

        int num_vars = N * (state_size + control_size);
        int num_constraints = N * state_size;

        solver_.data()->setNumberOfVariables(num_vars);
        solver_.data()->setNumberOfConstraints(num_constraints);

        // 目标函数权重
        MatrixXd Q = MatrixXd::Identity(state_size, state_size);
        Q(0, 0) = 10.0;  // x 误差权重
        Q(1, 1) = 10.0;  // y 误差权重
        Q(2, 2) = 1.0;   // 角度误差权重
        Q(3, 3) = 1.0;   // 速度误差权重

        MatrixXd R = MatrixXd::Identity(control_size, control_size);
        R(0, 0) = 0.1;  // 加速度权重
        R(1, 1) = 0.1;  // 方向角变化权重

        solver_.data()->setHessianMatrix(Q);
        solver_.data()->setGradient(R);
        solver_.initSolver();
    }

    void updateControl() {
        if (waypoints_.empty()) return;

        // 计算机器人当前位置
        geometry_msgs::TransformStamped tf_stamped;
        try {
            tf_stamped = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
            robot_x = tf_stamped.transform.translation.x;
            robot_y = tf_stamped.transform.translation.y;
            robot_theta = tf_stamped.transform.rotation.z;
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        // 机器人当前状态
        Vector4d x0;
        x0 << robot_x, robot_y, robot_theta, 0.5;

        // 求解优化问题，获取控制输入
        VectorXd u = solver_.getSolution();
        double a = u(0);
        double delta = u(1);

        // 发布控制指令
        geometry_msgs::Twist cmd;
        cmd.linear.x = x0(3) + a * dt;
        cmd.angular.z = delta;
        cmd_pub_.publish(cmd);

        // 更新轨迹
        updateMarker();
    }

    // 更新轨迹
    void updateMarker() {
        geometry_msgs::Point p;
        p.x = robot_x;
        p.y = robot_y;
        p.z = 0;
        marker_.points.push_back(p);
        marker_.header.stamp = ros::Time::now();
        marker_pub_.publish(marker_);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "mpc_controller");
    MPCController controller;
    ros::Rate rate(10);
    while (ros::ok()) {
        controller.updateControl();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
