// 修改后的astar.h
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <queue>
#include <unordered_map>
#include <cmath>

struct Node {
    int x, y;
    double g, h, f;
    Node* parent;
    
    Node(int x_, int y_) : x(x_), y(y_), g(0), h(0), f(0), parent(nullptr) {}
    
    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

namespace std {
    template<> struct hash<Node> {
        size_t operator()(const Node& node) const {
            return hash<int>()(node.x) ^ hash<int>()(node.y);
        }
    };
}

class AStarPlanner {
private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber start_sub_;
    
    ros::Publisher path_pub_;
    ros::Publisher start_marker_pub_;
    ros::Publisher goal_marker_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher cmd_vel_pub_;

    nav_msgs::OccupancyGrid::ConstPtr map_;
    bool map_initialized_;
    
    Node* start_node_;
    Node* goal_node_;
    
    // 地图参数
    int map_width_;
    int map_height_;
    float resolution_;
    geometry_msgs::Pose origin_;
    geometry_msgs::Pose start_pose_;
    geometry_msgs::Pose goal_pose_;


public:
    AStarPlanner() : 
        map_initialized_(false),
        start_node_(nullptr),
        goal_node_(nullptr) 
    {
        map_sub_ = nh_.subscribe("/map", 1, &AStarPlanner::mapCallback, this);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &AStarPlanner::goalCallback, this);
        start_sub_ = nh_.subscribe("/initialpose", 1, &AStarPlanner::startCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/astar_path", 1);
        // start_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/start_maker_points",1);
        // goal_marker_pub_= nh_.advertise<visualization_msgs::Marker>("/goal_maker_points",1);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    }

    ~AStarPlanner() {
        if (start_node_) delete start_node_;
        if (goal_node_) delete goal_node_;
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    bool aStarSearch();
    bool dijkstraSearch();
    void tryPlanning();

    void publishPath(const std::vector<Node*>& path);
    bool isValid(int x, int y);
    bool isObstacle(int x, int y);
    std::vector<Node*> getNeighbors(Node* node);
    double calculateH(Node* node);
    void worldToMap(double wx, double wy, int& mx, int& my);
    void publishMarker(const geometry_msgs::Pose& pose, bool is_start);
    void followPath(const std::vector<Node*>& path);
    void publishCarMarker(const geometry_msgs::Pose& pose);

};