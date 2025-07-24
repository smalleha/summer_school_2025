#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/RobotTrajectory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_relative_move_xyz");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化 MoveGroup 接口
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    std::string end_effector_link = move_group.getEndEffectorLink();

    // 配置参数
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.01);
    move_group.setMaxAccelerationScalingFactor(0.2);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.allowReplanning(true);
    // 控制机械臂先回到初始化位置
    move_group.setNamedTarget("home");
    move_group.move();
    sleep(1);
    // 获取当前末端执行器的位姿
    geometry_msgs::Pose current_pose = move_group.getCurrentPose(end_effector_link).pose;
    ROS_INFO_STREAM("Current pose: \n" << current_pose);

    // 构造路径点列表
    std::vector<geometry_msgs::Pose> waypoints;

    // Step 1: Z 轴上移 10cm
    geometry_msgs::Pose pose_1 = current_pose;
    pose_1.position.z += 0.10;
    waypoints.push_back(pose_1);

    // Step 2: Y 轴右移 10cm（在 pose_1 基础上）
    geometry_msgs::Pose pose_2 = pose_1;
    pose_2.position.x += 0.1;
    waypoints.push_back(pose_2);

    // Step 3: X 轴前移 10cm（在 pose_2 基础上）
    geometry_msgs::Pose pose_3 = pose_2;
    pose_3.position.y += 0.05;
    waypoints.push_back(pose_3);

    // 规划笛卡尔路径
    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.005;      // 每步最大末端位移
    const double jump_threshold = 0.0;  // 关节跳跃阈值
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    ROS_INFO("Cartesian path planning completed %.2f%% of the path", fraction * 100.0);

    // 执行路径
    if (fraction > 0.9) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        if (move_group.execute(plan)) {
            ROS_INFO("XYZ stepwise Cartesian path executed successfully.");
        } else {
            ROS_WARN("Execution failed.");
        }
    } else {
        ROS_WARN("Cartesian path planning incomplete. Only %.2f%% achieved.", fraction * 100.0);
    }

    spinner.stop();
    ros::shutdown();
    return 0;
}
