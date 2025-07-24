#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "moveit_cartesian_path", ros::init_options::AnonymousName);

	// 创建一个异步的自旋线程（spinning thread）
	ros::AsyncSpinner spinner(1);
	spinner.start();

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

	std::vector<geometry_msgs::Pose> waypoints;

    //将初始位姿加入路点列表
	waypoints.push_back(start_pose);
	
    start_pose.position.z += 0.2;
	waypoints.push_back(start_pose);

    start_pose.position.x += 0.1;
	waypoints.push_back(start_pose);

    start_pose.position.y += 0.1;
	waypoints.push_back(start_pose);

	
	// 笛卡尔空间下的路径规划
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

	// 生成机械臂的运动规划数据
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	plan.trajectory_ = trajectory;

	// 执行运动
	move_group.execute(plan);

	ros::shutdown(); 
	return 0;
}