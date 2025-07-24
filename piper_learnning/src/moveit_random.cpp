// ROS的头文件
#include <ros/ros.h>
// ROS中的MoveIt库的头文件
#include <moveit/move_group_interface/move_group_interface.h>

// 主函数
int main(int argc, char * argv[])
{
    // ROS节点初始化，节点名为"moveit_random"
    ros::init(argc, argv, "moveit_random");

    // 异步消息处理器，设置线程数为1
    ros::AsyncSpinner spinner(1);
    
    // 启动异步消息处理器
    spinner.start();

    // move_group接口，选择机械臂为"arm"
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

    // 将机械臂设置为随机目标点
    move_group.setRandomTarget();

    // 输出日志信息
    ROS_INFO("move");

    // 控制机械臂运动到随机目标点
    move_group.move();
    
    // 停止异步消息处理器
    spinner.stop();
    
    // 关闭ROS节点
    ros::shutdown();
    
    // 返回0，程序结束
    return 0;
}
