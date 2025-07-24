// 导入ROS头文件
#include <ros/ros.h>
// 导入move_group_interface头文件
#include <moveit/move_group_interface/move_group_interface.h>

// 主函数
int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "moveit_joint");

    // 异步消息处理器，设置线程数为1
    ros::AsyncSpinner spinner(1);
    
    // 启动异步消息处理器
    spinner.start();

    // move_group接口，选择机械臂为"lite6"
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

    // 声明一个double 型元素的vector容器 
    std::vector<double> joint_values(6);

    // 把关节运动位置填入vector容器 
    // joint_values[0]=-0.39441719709048184;
    // joint_values[1]= 0.4228564717254736;
    // joint_values[2]= 0.5897185686390605;
    // joint_values[3]= 0.5897185686390605;
    // joint_values[4]= 0.5897185686390605;
    joint_values[5]= 0.5897185686390605;

    // 设置运动目标
    move_group.setJointValueTarget(joint_values); 

    ROS_INFO("move");
    // 执行运动
    move_group.move(); 

    // 停止异步消息处理器
    spinner.stop();

    // 关闭ROS节点
    ros::shutdown();
    
    // 返回0，程序结束
    return 0;
}
