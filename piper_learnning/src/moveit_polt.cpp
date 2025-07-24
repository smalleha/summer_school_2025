#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "moveit_polt");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 创建 PlanningSceneInterface 对象，用于添加物体和障碍物
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	moveit::planning_interface::MoveGroupInterface move_group("arm");
    //获取终端link的名称
    std::string end_effector_link = move_group.getEndEffectorLink();
   //设置目标位置所使用的参考坐标系
    geometry_msgs::Pose eff_pose = move_group.getCurrentPose(end_effector_link).pose;
    //当运动规划失败后，允许重新规划
    move_group.allowReplanning(true);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.01);

    //设置允许的最大速度和加速度
    move_group.setMaxAccelerationScalingFactor(0.2);
    move_group.setMaxVelocityScalingFactor(0.2);

    // 控制机械臂先回到初始化位置
    move_group.setNamedTarget("home");
    move_group.move();
    sleep(1);

    // 设置机械臂的目标位置和姿态
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = 1.0;
    target_pose1.orientation.y = 1.81909e-05;
    target_pose1.orientation.z = -4.69167e-05;
    target_pose1.orientation.w = -8.68015e-05;
    target_pose1.position.x = 0.26628;
    target_pose1.position.y = 0.156854;
    target_pose1.position.z = 0.154193;

    // 设置机械臂的起始位置和姿态
    geometry_msgs::Pose start_pose;
    start_pose.orientation.x = 1.0;
    start_pose.orientation.y = 1.81909e-05;
    start_pose.orientation.z = -4.69167e-05;
    start_pose.orientation.w = -8.68015e-05;
    start_pose.position.x = 0.26628;
    start_pose.position.y = 0.056854;
    start_pose.position.z = 0.154221;

    // 将机械臂的起始位置添加到规划场景中
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(start_pose);
    move_group.move();

    // 设置圆弧轨迹中间点的位置和姿态
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose);

    geometry_msgs::Pose waypoint_pose;

    waypoint_pose.orientation.x = 1.0;
    waypoint_pose.orientation.y = 1.81909e-05;
    waypoint_pose.orientation.z = -4.69167e-05;
    waypoint_pose.orientation.w = -8.68015e-05;    
    waypoint_pose.position.x = 0.36628;
    waypoint_pose.position.y = 0.156854;
    waypoint_pose.position.z = 0.154221;
    waypoints.push_back(waypoint_pose);

    waypoint_pose.position.y = -0.15;
    waypoints.push_back(waypoint_pose);

    waypoint_pose.position.x = 0.26628;
    waypoint_pose.position.y = -0.15;
    waypoints.push_back(waypoint_pose);

    // 将机械臂的终止位置添加到规划场景中
    move_group.setPoseTarget(target_pose1);

    // 计算机械臂的轨迹
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    // 控制机械臂沿着圆弧轨迹运动
    move_group.execute(trajectory);

    // 控制机械臂先回到初始化位置
    move_group.setNamedTarget("home");
    move_group.move();
    sleep(1);

    spinner.stop(); 

    ros::shutdown();
    return 0;
}
