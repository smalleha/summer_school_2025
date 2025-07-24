#ifndef MULTI_NAVI_GOAL_PANEL_H
#define MULTI_NAVI_GOAL_PANEL_H

#define COLUMN_COUNT 3

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <rviz/panel.h>

#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>
#include <QLabel>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>
#include <fstream>
#include <iostream>
#include <thread>

using namespace std;

namespace navi_multi_goals_pub_rviz_plugin {



    class MultiNaviGoalsPanel : public rviz::Panel {
    Q_OBJECT
    public:
        explicit MultiNaviGoalsPanel(QWidget *parent = 0);
        ~MultiNaviGoalsPanel();

    public Q_SLOTS:

        void setMaxNumGoal(const QString &maxNumGoal);

        void writeTable(geometry_msgs::Pose pose, string task);
        void markPose(geometry_msgs::PoseStamped &pose,int id);
        void deleteMark();
        void DeleteCubeMark(int id);
        void cancelSelectPoseTable();
    protected Q_SLOTS:

        void updateMaxNumGoal();             // update max number of goal
        void updateMaxNumGoal(int num);
        void initPoseTable();               // initialize the pose table
        void updatePoseTableSelectRow();
        void setPoseTableSelectRow(QTableWidgetItem *item);

        void updatePoseTable();             // update the pose table
        void startNavi();                   // start navigate for the first pose
        void cancelNavi();

        void goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose);  //goal count sub callback function

        void statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses); //status sub callback function

        void checkCycle();

        void completeNavi();               //after the first pose, continue to navigate the rest of poses
        void cycleNavi();

        bool checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list);  // check whether arrived the goal

        static void startSpin(); // spin for sub
        void InitTableItem(int num);
        void UpdateLine(QTableWidgetItem *item);

        /*****
        * add new function
        *******/
        void LoadFile(std::string filename);
        void SaveFile(std::string filename);
        void ShowSaveFileDialog();
        void ShowLoadFileDialog();
        void RecordWayPoint(geometry_msgs::PoseStamped &pose);

    protected:
        QLineEdit *output_maxNumGoal_editor_;
        QPushButton *output_maxNumGoal_button_, *output_reset_button_, *output_startNavi_button_, *output_cancel_button_,
        *save_config_button_,*load_config_button_;
        QTableWidget *poseArray_table_;
        QCheckBox *cycle_checkbox_;

        QString output_maxNumGoal_;

        // The ROS node handle.
        ros::NodeHandle nh_;
        ros::Publisher goal_pub_, cancel_pub_, marker_pub_;
        ros::Subscriber goal_sub_, status_sub_, scout_status_sub_;

        int maxNumGoal_ = 1;
        int poseTableSelectRow = 0;

        int curGoalIdx_ = 0, cycleCnt_ = 0;
        bool permit_ = false, cycle_ = false, arrived_ = false;
        geometry_msgs::PoseArray pose_array_;

        std::string waypoint_path, waypoint_directory;
        std::string map_frame = "map";

        FILE *fp;
        char buffer[1024]={0};

        actionlib_msgs::GoalID pre_goalid_, cur_goalid_;
        //QTableWidgetItem *table_item;
        int count = 0;
        bool write_flag = false;
        std::vector<QTableWidgetItem*> vector_table_items;

        std::vector<std::string> task_array;
    };

} // end namespace navi-multi-goals-pub-rviz-plugin

#endif // MULTI_NAVI_GOAL_PANEL_H
