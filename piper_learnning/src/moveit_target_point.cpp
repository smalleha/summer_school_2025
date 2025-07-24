// ROS 的 C++ 标准库头文件
#include <ros/ros.h> 
// ROS 的几何姿态消息头文件
#include <geometry_msgs/Pose.h> 
// MoveIt 接口头文件
#include "moveit/move_group_interface/move_group_interface.h" 
// 主函数
int main(int argc, char * argv[]) 
{
    // ROS 节点初始化，命名为 move_target_point
    ros::init(argc, argv, "moveit_target_point"); 
    // 声明异步的 spinner，其中 1 表示只有一个线程
    ros::AsyncSpinner spinner(1); 
    // 启动 spinner，开始异步处理 ROS 节点的回调函数
    spinner.start(); 
    // 声明 MoveGroupInterface 对象，指定规划组名称为 "arm"
	moveit::planning_interface::MoveGroupInterface move_group("arm");
    //获取终端link的名称
    std::string end_effector_link = move_group.getEndEffectorLink();
    //设置目标位置所使用的参考坐标系
    geometry_msgs::Pose start_pose = move_group.getCurrentPose(end_effector_link).pose;
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

    // 声明姿态目标的消息
    geometry_msgs::Pose target_point; 
    // 设置姿态目标的四元数方向向量
    target_point.orientation.x = 1.0; 
    target_point.orientation.y = -1.98016e-05;
    target_point.orientation.z = 6.61578e-05;
    target_point.orientation.w = -8.60138e-07;
    // 设置姿态目标的位置坐标
    target_point.position.x = 0.2; 
    target_point.position.y = 0.001;
    target_point.position.z = 0.25;
    // 设置机械臂的姿态目标为 target_point
    move_group.setPoseTarget(target_point); 
    // 声明一个规划方案
    moveit::planning_interface::MoveGroupInterface::Plan my_plan; 
    // 通过 MoveGroupInterface 对象 plan() 函数进行规划，返回值 success 表示规划是否成功
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); 
    // 如果规划成功，则执行规划方案
    if (success) 
    {   
        // 通过 MoveGroupInterface 对象 execute() 函数执行规划方案
        move_group.execute(my_plan); 
    }
    else // 如果规划失败，则输出错误信息
    {
        ROS_INFO("Failed to plan and execute!");
    }
    // 停止 spinner
    spinner.stop(); 
    // 关闭 ROS 节点
    ros::shutdown(); 
    // 返回 0 ，程序结束
    return 0; 
}
