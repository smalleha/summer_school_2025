#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_collision_object");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 创建一个 PlanningSceneInterface 对象
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // 创建一个 CollisionObject 对象，指定其 ID 为“box”，并设置其位置、大小、形状等属性
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "box";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1; // 长
    primitive.dimensions[1] = 1; // 宽
    primitive.dimensions[2] = 1; // 高

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = -0.3;
    box_pose.position.z = 0.5;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // 将 CollisionObject 对象添加到 PlanningScene 中
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);

    spinner.stop();
    ros::shutdown();
    return 0;
}
