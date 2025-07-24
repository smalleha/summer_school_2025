#include <math.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "moveit_circular_arc");
	ros::AsyncSpinner spinner(1);
	spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("arm");

    //获取终端link的名称
    std::string end_effector_link = move_group.getEndEffectorLink();

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "arm_base";
    move_group.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    move_group.allowReplanning(true);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.01);

    //设置允许的最大速度和加速度
    move_group.setMaxAccelerationScalingFactor(0.8);
    move_group.setMaxVelocityScalingFactor(0.8);

    // 控制机械臂先回到初始化位置
    move_group.setNamedTarget("home");
    move_group.move();
    sleep(1);

    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = 0.701835;
    target_pose.orientation.y = -0.00258698;
    target_pose.orientation.z = 0.71233;
    target_pose.orientation.w = 0.00269154;
    target_pose.position.x = 0.30384;
    target_pose.position.y = -0.0;
    target_pose.position.z = 0.235383;

    move_group.setPoseTarget(target_pose);
    move_group.move();

	std::vector<geometry_msgs::Pose> waypoints;

    //将初始位姿加入路点列表
	waypoints.push_back(target_pose);

    double centerA = target_pose.position.y;
    double centerB = target_pose.position.z;
    double radius = 0.1;

    for(double th=0.0; th<6.28; th=th+0.01)
    {
        target_pose.position.y = centerA + radius * cos(th);
        target_pose.position.z = centerB + radius * sin(th);
        waypoints.push_back(target_pose);
    }

	// 笛卡尔空间下的路径规划
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");

	    // 生成机械臂的运动规划数据
	    moveit::planning_interface::MoveGroupInterface::Plan plan;
	    plan.trajectory_ = trajectory;

	    // 执行运动
	    move_group.execute(plan);
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    // 控制机械臂先回到初始化位置
    move_group.setNamedTarget("home");
    move_group.move();
    sleep(1);

	ros::shutdown(); 
	return 0;
}