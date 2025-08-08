// 修改后的astar.cpp
#include "astar.h"
#include <visualization_msgs/MarkerArray.h>

void AStarPlanner::worldToMap(double wx, double wy, int& mx, int& my) {
    mx = static_cast<int>((wx - origin_.position.x) / resolution_);
    my = static_cast<int>((wy - origin_.position.y) / resolution_);
}

void AStarPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_ = msg;
    map_width_ = msg->info.width;
    map_height_ = msg->info.height;
    resolution_ = msg->info.resolution;
    origin_ = msg->info.origin;
    map_initialized_ = true;
    ROS_INFO("Map initialized");
}

void AStarPlanner::startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!map_initialized_) {
        ROS_WARN("Map not initialized yet!");
        return;
    }

    start_pose_ = msg->pose.pose;
    int start_x, start_y;
    worldToMap(msg->pose.pose.position.x, msg->pose.pose.position.y, start_x, start_y);
    
    if (!isValid(start_x, start_y)) {
        ROS_WARN("Start point out of map bounds!");
        return;
    }
    
    if (isObstacle(start_x, start_y)) {
        ROS_WARN("Start point is in obstacle!");
        return;
    }
    
    if (start_node_) delete start_node_;
    start_node_ = new Node(start_x, start_y);
    ROS_INFO("New start set at grid (%d, %d)", start_x, start_y);
    publishMarker(msg->pose.pose, true);

    tryPlanning();
}

void AStarPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!map_initialized_) {
        ROS_WARN("Map not initialized yet!");
        return;
    }
    goal_pose_ = msg->pose;
    int goal_x, goal_y;
    worldToMap(msg->pose.position.x, msg->pose.position.y, goal_x, goal_y);
    
    if (!isValid(goal_x, goal_y)) {
        ROS_WARN("Goal point out of map bounds!");
        return;
    }
    
    if (isObstacle(goal_x, goal_y)) {
        ROS_WARN("Goal point is in obstacle!");
        return;
    }
    
    if (goal_node_) delete goal_node_;
    goal_node_ = new Node(goal_x, goal_y);
    ROS_INFO("New goal set at grid (%d, %d)", goal_x, goal_y);
    publishMarker(msg->pose, false);

    AStarPlanner::tryPlanning();
}

void AStarPlanner::tryPlanning() {
    if (start_node_ && goal_node_) {
        if (aStarSearch()) {
            ROS_INFO("Path found!");
        } else {
            ROS_WARN("No path found!");
        }
    } else {
        if (!start_node_) ROS_WARN("Start point not set!");
        if (!goal_node_) ROS_WARN("Goal point not set!");
    }
}

// 保持其他函数实现不变，只需确保使用start_node_和goal_node_

bool AStarPlanner::aStarSearch() {
    if (!map_initialized_ || !start_node_ || !goal_node_) return false;

    auto cmp = [](Node* a, Node* b) { return a->f > b->f; };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open_list(cmp);
    std::unordered_map<Node, Node*> closed_list;
    std::vector<Node*> path;

    start_node_->h = calculateH(start_node_);
    start_node_->f = start_node_->g + start_node_->h;
    open_list.push(start_node_);

    while (!open_list.empty()) {
        Node* current = open_list.top();
        open_list.pop();

        if (current->x == goal_node_->x && current->y == goal_node_->y) {
            while (current != nullptr) {
                path.push_back(current);
                current = current->parent;
            }
            publishPath(path);
            followPath(path);

            return true;
        }

        for (Node* neighbor : getNeighbors(current)) {
            if (!isValid(neighbor->x, neighbor->y) || isObstacle(neighbor->x, neighbor->y)) {
                delete neighbor;
                continue;
            }

            double tentative_g = current->g + 1;
            if (tentative_g < neighbor->g || !closed_list.count(*neighbor)) {
                neighbor->parent = current;
                neighbor->g = tentative_g;
                neighbor->h = calculateH(neighbor);
                neighbor->f = neighbor->g + neighbor->h;
                
                open_list.push(neighbor);
                closed_list[*neighbor] = neighbor;
            }
        }
    }
    return false;
}

bool AStarPlanner::isValid(int x, int y) {
    return x >= 0 && x < map_width_ && y >= 0 && y < map_height_;
}

bool AStarPlanner::isObstacle(int x, int y) {
    return map_->data[y * map_width_ + x] > 50;  // 阈值设为50
}

double AStarPlanner::calculateH(Node* node) {
    // 使用曼哈顿距离
    return std::abs(node->x - goal_node_->x) + std::abs(node->y - goal_node_->y);
}

std::vector<Node*> AStarPlanner::getNeighbors(Node* node) {
    std::vector<Node*> neighbors;
    // 4邻域
    const int dx[] = {-1, 1, 0, 0};
    const int dy[] = {0, 0, -1, 1};
    
    for (int i = 0; i < 4; ++i) {
        neighbors.push_back(new Node(node->x + dx[i], node->y + dy[i]));
    }
    return neighbors;
}
void AStarPlanner::publishPath(const std::vector<Node*>& path) {
    nav_msgs::Path ros_path;
    ros_path.header.stamp = ros::Time::now();
    ros_path.header.frame_id = "map";

    for (int i = path.size() - 1; i >= 0; --i) {
        geometry_msgs::PoseStamped pose;
        pose.header = ros_path.header;
        pose.pose.position.x = origin_.position.x + (path[i]->x + 0.5) * resolution_;
        pose.pose.position.y = origin_.position.y + (path[i]->y + 0.5) * resolution_;
        pose.pose.orientation.w = 1.0;
        ros_path.poses.push_back(pose);
    }
    

    path_pub_.publish(ros_path);
}

void AStarPlanner::followPath(const std::vector<Node*>& path) {
    if (path.empty()) {
        ROS_WARN("Path is empty! Cannot follow.");
        return;
    }

    ros::Rate rate(10);  // 10 Hz 控制频率
    geometry_msgs::Twist cmd_vel;
    double last_yaw = tf2::getYaw(start_pose_.orientation);

    for (size_t i = path.size() - 1; i > 0; --i) {
        Node* target = path[i];

        double target_x = origin_.position.x + (target->x + 0.5) * resolution_;
        double target_y = origin_.position.y + (target->y + 0.5) * resolution_;

        double dx = target_x - start_pose_.position.x;
        double dy = target_y - start_pose_.position.y;
        double distance = std::hypot(dx, dy);
        double angle = std::atan2(dy, dx);

        double angle_diff = angle - last_yaw;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        if (std::abs(angle_diff) > 0.1) {
            cmd_vel.angular.z = 0.5 * angle_diff;
            cmd_vel.linear.x = 0.0;
        } else {
            cmd_vel.angular.z = 0.0;
            cmd_vel.linear.x = std::min(0.5, distance);
        }

        last_yaw = angle;
        geometry_msgs::Pose car_pose;
        car_pose.position.x = target_x;
        car_pose.position.y = target_y;
        tf2::Quaternion q;
        q.setRPY(0, 0, last_yaw);
        car_pose.orientation.x = q.x();
        car_pose.orientation.y = q.y();
        car_pose.orientation.z = q.z();
        car_pose.orientation.w = q.w();

        publishCarMarker(car_pose);
        cmd_vel_pub_.publish(cmd_vel);
        rate.sleep();
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel);
    ROS_INFO("Reached the goal!");
}

void AStarPlanner::publishMarker(const geometry_msgs::Pose& pose, bool is_start) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = is_start ? "start_marker" : "goal_marker";
    marker.id = is_start ? 0 : 1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = is_start ? 0.0 : 1.0;
    marker.color.g = is_start ? 1.0 : 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    marker_pub_.publish(marker);
}

void AStarPlanner::publishCarMarker(const geometry_msgs::Pose& pose) {
    visualization_msgs::Marker car_marker;
    car_marker.header.frame_id = "map";
    car_marker.header.stamp = ros::Time::now();
    car_marker.ns = "car_marker";
    car_marker.id = 2;
    car_marker.type = visualization_msgs::Marker::CUBE;  // 用立方体模拟小车
    car_marker.action = visualization_msgs::Marker::ADD;

    car_marker.pose = pose;
    car_marker.scale.x = 0.5;  // 车身大小
    car_marker.scale.y = 0.3;
    car_marker.scale.z = 0.2;
    car_marker.color.r = 0.0;
    car_marker.color.g = 0.0;
    car_marker.color.b = 1.0;  // 蓝色小车
    car_marker.color.a = 1.0;

    marker_pub_.publish(car_marker);
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner");
    AStarPlanner planner;
    ros::spin();
    return 0;
}
