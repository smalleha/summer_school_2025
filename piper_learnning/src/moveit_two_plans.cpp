#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/OrientationConstraint.h>

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "moveit_revise_trajectory");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 创建一个 MoveGroup 接口对象，对应的机械臂是 "lite6"
    moveit::planning_interface::MoveGroupInterface move_group("arm");

    // 设置关节容差值为 0.001
    move_group.setGoalJointTolerance(0.001);

    // 设置加速度和速度的缩放因子
    double accScale = 0.8;
    double velScale = 0.8;
    move_group.setMaxAccelerationScalingFactor(accScale);
    move_group.setMaxVelocityScalingFactor(velScale);

    // 控制机械臂先回到初始化位置
    move_group.setNamedTarget("home");
    move_group.move();
    sleep(1);

    // 获取机器人的起始位置
    moveit::core::RobotStatePtr start_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = start_state->getJointModelGroup(move_group.getName());

    std::vector<double> joint_group_positions;
    start_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    //设置第一个目标点
    joint_group_positions[0] = -0.6;  // 弧度值
    move_group.setJointValueTarget(joint_group_positions);

    // 计算第一条轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    moveit::planning_interface::MoveItErrorCode success = move_group.plan(plan1);

    // 获取机器人的起始位置，并更新机器人的起始状态
    joint_model_group = start_state->getJointModelGroup(move_group.getName());    
    start_state->setJointGroupPositions(joint_model_group, joint_group_positions);
    move_group.setStartState(*start_state);

    //设置第二个目标点
    joint_group_positions[0] = 1.2;  // 弧度值
    joint_group_positions[1] = -0.5;  // 弧度值
    move_group.setJointValueTarget(joint_group_positions);

    // 计算第二条轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    success = move_group.plan(plan2);

    //连接两条轨迹
    moveit_msgs::RobotTrajectory trajectory;

    trajectory.joint_trajectory.joint_names = plan1.trajectory_.joint_trajectory.joint_names;
    trajectory.joint_trajectory.points = plan1.trajectory_.joint_trajectory.points;
    for (size_t j = 1; j < plan2.trajectory_.joint_trajectory.points.size(); j++)
    {
        trajectory.joint_trajectory.points.push_back(plan2.trajectory_.joint_trajectory.points[j]);
    }

    // 对连接后的轨迹进行时间参数化
    moveit::planning_interface::MoveGroupInterface::Plan joinedPlan;
    robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "arm");
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, velScale, accScale);

    rt.getRobotTrajectoryMsg(trajectory);
    joinedPlan.trajectory_ = trajectory;
    //执行连接后的轨迹
    if (!move_group.execute(joinedPlan))
    {
        ROS_ERROR("Failed to execute plan");
        return false;
    }

    sleep(1);

    ROS_INFO("Finished");

    // 控制机械臂先回到初始化位置
    move_group.setNamedTarget("home");
    move_group.move();
    sleep(1);

    ros::shutdown(); 

    return 0;
}
