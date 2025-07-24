#include <cstdio>

#include <ros/console.h>

#include <fstream>
#include <sstream>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/qheaderview.h>
#include <QFileDialog>

#include "multi_navi_goal_panel.h"
#include <yaml-cpp/yaml.h>

namespace navi_multi_goals_pub_rviz_plugin {

    MultiNaviGoalsPanel::~MultiNaviGoalsPanel(){}

    MultiNaviGoalsPanel::MultiNaviGoalsPanel(QWidget *parent)
            : rviz::Panel(parent), nh_(), maxNumGoal_(1)
    {
        goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal_temp", 1,
                                                              boost::bind(&MultiNaviGoalsPanel::goalCntCB, this, _1));

        status_sub_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("move_base/status", 1,
                                                                     boost::bind(&MultiNaviGoalsPanel::statusCB, this, _1));

        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

        cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);

        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        // 获取默认路径点的文件目录
        string command = "rospack find navi_multi_goals_pub_rviz_plugin";
        fp = popen(command.data(),"r");
        fread(buffer,1,sizeof(buffer),fp);
        pclose(fp);

        waypoint_directory = buffer;
        // 消除回车符
        waypoint_directory.pop_back();
        waypoint_directory += "/configs";
        waypoint_path = waypoint_directory+"/waypoint.txt";

        // ui
        QVBoxLayout *root_layout = new QVBoxLayout;

        // create a panel about "maxNumGoal"
        QHBoxLayout *maxNumGoal_layout = new QHBoxLayout;
        maxNumGoal_layout->addWidget(new QLabel("导航点最大数量"));
        output_maxNumGoal_editor_ = new QLineEdit;
        output_maxNumGoal_editor_->setText(QString::number(maxNumGoal_));
        maxNumGoal_layout->addWidget(output_maxNumGoal_editor_);
        output_maxNumGoal_button_ = new QPushButton("确定");
        maxNumGoal_layout->addWidget(output_maxNumGoal_button_);
        root_layout->addLayout(maxNumGoal_layout);

        cycle_checkbox_ = new QCheckBox("循环");
        if(cycle_)  cycle_checkbox_->setCheckState(Qt::Checked);

        // save,load file
        load_config_button_ = new QPushButton("加载路径点");
        save_config_button_ = new QPushButton("保存路径点");
        QHBoxLayout *file_config_button_layout = new QHBoxLayout;
        file_config_button_layout->addWidget(cycle_checkbox_);
        file_config_button_layout->addWidget(load_config_button_);
        file_config_button_layout->addWidget(save_config_button_);
        root_layout->addLayout(file_config_button_layout);

        // creat a QTable to contain the poseArray
        poseArray_table_ = new QTableWidget;
        initPoseTable();
        root_layout->addWidget(poseArray_table_);

        //creat a manipulate layout
        QHBoxLayout *manipulate_layout = new QHBoxLayout;
        output_reset_button_ = new QPushButton("清空");
        manipulate_layout->addWidget(output_reset_button_);
        output_cancel_button_ = new QPushButton("取消");
        manipulate_layout->addWidget(output_cancel_button_);
        output_startNavi_button_ = new QPushButton("开始导航!");
        manipulate_layout->addWidget(output_startNavi_button_);
        root_layout->addLayout(manipulate_layout);

        setLayout(root_layout);
        // set a Qtimer to start a spin for subscriptions
        QTimer *output_timer = new QTimer(this);
        output_timer->start(200);

        // 设置信号与槽的连接
        connect(output_maxNumGoal_button_, SIGNAL(clicked()), this,
                SLOT(updateMaxNumGoal()));
        connect(output_maxNumGoal_button_, SIGNAL(clicked()), this,
                SLOT(updatePoseTable()));
        connect(output_reset_button_, SIGNAL(clicked()), this, SLOT(initPoseTable()));
        connect(output_cancel_button_, SIGNAL(clicked()), this, SLOT(cancelNavi()));
        connect(output_startNavi_button_, SIGNAL(clicked()), this, SLOT(startNavi()));
        connect(cycle_checkbox_, SIGNAL(clicked(bool)), this, SLOT(checkCycle()));
        connect(output_timer, SIGNAL(timeout()), this, SLOT(startSpin()));
        connect(poseArray_table_, SIGNAL(itemChanged(QTableWidgetItem*)), this, SLOT(UpdateLine(QTableWidgetItem*)));
        connect(poseArray_table_, SIGNAL(itemClicked(QTableWidgetItem*)), this, SLOT(setPoseTableSelectRow(QTableWidgetItem*)));
        connect(poseArray_table_, SIGNAL(itemSelectionChanged()), this, SLOT(updatePoseTableSelectRow()));
        connect(load_config_button_,SIGNAL(clicked()),this,SLOT(ShowLoadFileDialog()));
        connect(save_config_button_,SIGNAL(clicked()),this,SLOT(ShowSaveFileDialog()));
    }

    void MultiNaviGoalsPanel::ShowLoadFileDialog()
    {
        qDebug()<<"open file...";
          //定义文件对话框类
          QFileDialog *fileDialog = new QFileDialog(this);
          //定义文件对话框标题
          fileDialog->setWindowTitle(tr("打开文件"));
          //设置默认文件路径
          fileDialog->setDirectory(QString::fromStdString(waypoint_directory));
          //设置文件过滤器
          //fileDialog->setNameFilter(tr("Images(*.png *.jpg *.jpeg *.bmp)"));
          fileDialog->setNameFilter(tr("text(*.txt)"));
          //设置可以选择多个文件,默认为只能选择一个文件QFileDialog::ExistingFiles
          fileDialog->setFileMode(QFileDialog::ExistingFiles);
          //fileDialog->setFileMode(QFileDialog::Directory);
          //设置视图模式
          fileDialog->setViewMode(QFileDialog::Detail);
          //打印所有选择的文件的路径
          QStringList fileNames;
          if (fileDialog->exec())
          {
            fileNames = fileDialog->selectedFiles();
            waypoint_path = fileNames[0].toStdString();
            waypoint_directory = waypoint_path.substr(0, waypoint_path.rfind("/"));
            nh_.setParam("/rviz_waypoint_path", waypoint_path);
            LoadFile(waypoint_path);
          }
          for (auto tmp : fileNames)
          {
            qDebug() << tmp << endl;
          }
    }

    void MultiNaviGoalsPanel::LoadFile(std::string filename)
    {
        fstream infile;
        infile.open(filename,ios::in);
        int pose_num = 0;
        string str;
        geometry_msgs::PoseArray pose_array_from_file;
        std::vector<std::string> task_array_form_file;
        while (!infile.eof())
        {
            getline(infile, str,'\n');
            if (pose_num == 0)
            {
                pose_num++;
                continue;
            }
            std::stringstream ss(str);
            std::string data[COLUMN_COUNT];

            for(int i = 0; i < COLUMN_COUNT; i++) getline(ss, data[i], ','); //ss>>data[i];

            geometry_msgs::Pose pose;
            pose.position.x = std::atof(data[0].c_str());
            pose.position.y = std::atof(data[1].c_str());
            pose.orientation = tf::createQuaternionMsgFromYaw(std::atof(data[2].c_str())*M_PI/180.0);
            pose_array_from_file.poses.push_back(pose);
            if (COLUMN_COUNT == 4)
            {
                if (data[3] != "")
                    task_array_form_file.push_back(data[3]);
                else
                    task_array_form_file.push_back("--");
            }
            pose_num++;
            //cout<<"x:"<<data[0]<<" y:"<<data[1]<<" yaw:"<< data[2]<<endl;
        }
        if (!pose_array_from_file.poses.empty())
            pose_array_from_file.poses.pop_back();
        if (!task_array_form_file.empty())
            task_array_form_file.pop_back();
        initPoseTable();
        //ROS_INFO("pose_num:%d",pose_num);
        if (pose_num-2 > 0)
        {
            updateMaxNumGoal(pose_num-2);
            updatePoseTable();
            for (size_t i = 0; i < task_array_form_file.size(); i++)
            {
                task_array.push_back(task_array_form_file[i]);
            }
            for (size_t i = 0; i < pose_array_from_file.poses.size(); i++)
            {
                geometry_msgs::PoseStamped waypoint;
                waypoint.pose = pose_array_from_file.poses[i];
                waypoint.header.frame_id = map_frame;
                RecordWayPoint(waypoint);
            }
        }

    }

    void MultiNaviGoalsPanel::ShowSaveFileDialog()
    {
        QFileDialog *fileDialog = new QFileDialog(this);
        fileDialog->setDirectory(QString::fromStdString(waypoint_directory));
        //fileDialog->setDefaultSuffix(".txt");
        QString filename = fileDialog->getSaveFileName(this,tr("Save file"),"",tr("*.txt"));
//            QString filename = QFileDialog::getSaveFileName(this,tr("Save file"),"",tr("*.txt"));
        if (filename.isEmpty())
        {
            return;
        }
        else
        {
            // set save_path as waypoint_path
            waypoint_path = filename.toStdString();
            if(waypoint_path.substr(waypoint_path.length()-4,waypoint_path.length()) != ".txt")
                waypoint_path += ".txt";
            nh_.setParam("/rviz_waypoint_path", waypoint_path);
            waypoint_directory = waypoint_path.substr(0, waypoint_path.rfind("/"));

            SaveFile(waypoint_path);
        }
    }

    void MultiNaviGoalsPanel::SaveFile(std::string filename)
    {
        ofstream outfile;
        outfile.open(filename,ios::trunc);
        if (COLUMN_COUNT == 4)
            outfile << "x" << ",y" << ",yaw" << ",task" << endl;
        else
            outfile << "x" << ",y" << ",yaw" << endl;
        for (int i = 0; i < pose_array_.poses.size(); i++)
        {
            if (COLUMN_COUNT == 4)
                outfile << pose_array_.poses[i].position.x << ","
                        << pose_array_.poses[i].position.y << ","
                        << tf::getYaw(pose_array_.poses[i].orientation) * 180.0 / M_PI << ","
                        << (task_array.size() > i && task_array[i] != "" ? task_array[i] : "--")
                        << endl;
            else
                outfile << pose_array_.poses[i].position.x << ","
                        << pose_array_.poses[i].position.y << ","
                        << tf::getYaw(pose_array_.poses[i].orientation) * 180.0 / M_PI
                        << endl;
        }
        outfile.close();
    }

// 更新maxNumGoal命名
    void MultiNaviGoalsPanel::updateMaxNumGoal()
    {
        setMaxNumGoal(output_maxNumGoal_editor_->text());
        ROS_INFO("maxgoal:%d",maxNumGoal_);
        InitTableItem(maxNumGoal_*COLUMN_COUNT);
    }

    void MultiNaviGoalsPanel::updateMaxNumGoal(int num)
    {
        setMaxNumGoal( QString::fromStdString(to_string(num)));
        ROS_INFO("maxgoal:%d",maxNumGoal_);
        InitTableItem(maxNumGoal_*COLUMN_COUNT);
    }

// set up the maximum number of goals
    void MultiNaviGoalsPanel::setMaxNumGoal(const QString &new_maxNumGoal)
    {
        // 检查maxNumGoal是否发生改变.
        if (new_maxNumGoal != output_maxNumGoal_) {
            output_maxNumGoal_ = new_maxNumGoal;
            output_maxNumGoal_editor_->setText(output_maxNumGoal_);

            // 如果命名为空，不发布任何信息
            if (output_maxNumGoal_ == "") {
                nh_.setParam("maxNumGoal_", 1);
                maxNumGoal_ = 1;
            } else {
//                velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>(output_maxNumGoal_.toStdString(), 1);
                nh_.setParam("maxNumGoal_", output_maxNumGoal_.toInt());
                maxNumGoal_ = output_maxNumGoal_.toInt();
            }
            Q_EMIT configChanged();
        }
    }

    // initialize the table of pose
    void MultiNaviGoalsPanel::initPoseTable()
    {
        ROS_INFO("Initialize");
        curGoalIdx_ = 0, cycleCnt_ = 0;
        permit_ = false;
        count = 0;
        poseTableSelectRow = 0;
        if (poseArray_table_->rowCount() > 0){
            vector_table_items.clear();
        }
        poseArray_table_->clear();
        InitTableItem(maxNumGoal_*COLUMN_COUNT);
        pose_array_.poses.clear();
        task_array.clear();
        deleteMark();
        poseArray_table_->setRowCount(maxNumGoal_);
        poseArray_table_->setColumnCount(COLUMN_COUNT);
        poseArray_table_->setEditTriggers(QAbstractItemView::DoubleClicked);
        poseArray_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        poseArray_table_->setSelectionBehavior(QTableWidget::SelectRows);//一次选中一行
//        poseArray_table_->setSelectionMode(QAbstractItemView::SingleSelection);
        QStringList pose_header;
        pose_header << "x" << "y" << "yaw" << "task";
        poseArray_table_->setHorizontalHeaderLabels(pose_header);
        //cycle_checkbox_->setCheckState(Qt::Unchecked);
    }

    // delete marks in the map
    void MultiNaviGoalsPanel::deleteMark()
    {
        visualization_msgs::Marker marker_delete;
        marker_delete.action = visualization_msgs::Marker::DELETEALL;
        marker_pub_.publish(marker_delete);
    }

    //update the table of pose
    void MultiNaviGoalsPanel::updatePoseTable() {
        poseArray_table_->setRowCount(maxNumGoal_);
//        pose_array_.poses.resize(maxNumGoal_);
        QStringList pose_header;
        pose_header << "x" << "y" << "yaw" << "task";
        poseArray_table_->setHorizontalHeaderLabels(pose_header);

        while (pose_array_.poses.size() > maxNumGoal_)
        {
            pose_array_.poses.pop_back();
            task_array.pop_back();
            DeleteCubeMark(pose_array_.poses.size());
        }

        poseTableSelectRow = pose_array_.poses.size();
        poseArray_table_->show();
    }

    void MultiNaviGoalsPanel::updatePoseTableSelectRow(){
      QList<QTableWidgetItem*>itemList = poseArray_table_->selectedItems();
      int select_row_size = poseArray_table_->selectedItems().size();
        if (select_row_size == 0)
        {
            poseTableSelectRow = pose_array_.poses.size();
            cancelSelectPoseTable();
//            ROS_INFO("cancel selset, set pose size as select row : %d ",poseTableSelectRow);
        }
    }

    void MultiNaviGoalsPanel::setPoseTableSelectRow(QTableWidgetItem *item){
        poseTableSelectRow = item->row();
//        ROS_INFO("select row : %d",poseTableSelectRow);

    }

    // call back function for counting goals
    void MultiNaviGoalsPanel::goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose) {
            geometry_msgs::PoseStamped waypoint;
            waypoint = *pose;
            RecordWayPoint(waypoint);
        }

    void MultiNaviGoalsPanel::RecordWayPoint(geometry_msgs::PoseStamped &pose)
    {
        int id = poseTableSelectRow;
        if (pose_array_.poses.size() < maxNumGoal_ || poseTableSelectRow < pose_array_.poses.size())
        {
            write_flag = true;
            if (poseTableSelectRow < pose_array_.poses.size())
            {
                pose_array_.poses[poseTableSelectRow] = pose.pose;
            }
            else
            {
                pose_array_.poses.push_back(pose.pose);
                poseTableSelectRow = pose_array_.poses.size();
                if (task_array.size() < pose_array_.poses.size())
                {
                    std::stringstream ss;
//                    ss << "task" << id+1;
                    ss << "--";
                    task_array.push_back(ss.str());
                }
                pose_array_.header.frame_id = pose.header.frame_id;
            }
            writeTable(pose.pose, task_array.at(id));
            markPose(pose,id);
            write_flag = false;
        } else {
            //ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
        }
    }

    // write the poses into the table
    void MultiNaviGoalsPanel::writeTable(geometry_msgs::Pose pose, string task)
    {
        disconnect(poseArray_table_, SIGNAL(itemChanged(QTableWidgetItem*)), this, SLOT(UpdateLine(QTableWidgetItem*)));

        int row = poseTableSelectRow == pose_array_.poses.size() ?
              poseTableSelectRow-1 : poseTableSelectRow;
        for (int i = COLUMN_COUNT * row; i < COLUMN_COUNT * row + COLUMN_COUNT; i++)
        {
            switch(i % COLUMN_COUNT)
            {
            case 0:
              vector_table_items[i]->setText(QString::number(pose.position.x, 'f', 2));
              break;
            case 1:
              vector_table_items[i]->setText(QString::number(pose.position.y, 'f', 2));
              break;
            case 2:
              vector_table_items[i]->setText(QString::number(tf::getYaw(pose.orientation) * 180.0 / M_PI, 'f', 2));
              break;
            case 3:
              vector_table_items[i]->setText(QString::fromStdString(task));
              break;
            }
            if (poseTableSelectRow == pose_array_.poses.size())
                poseArray_table_->setItem(row, i % COLUMN_COUNT, vector_table_items[i]);
        };
        poseTableSelectRow = pose_array_.poses.size();

        connect(poseArray_table_, SIGNAL(itemChanged(QTableWidgetItem*)), this, SLOT(UpdateLine(QTableWidgetItem*)));
    }

    // when setting a Navi Goal, it will set a mark on the map
    void MultiNaviGoalsPanel::markPose(geometry_msgs::PoseStamped &pose,int id) {
      //ROS_INFO("markpose");
        if (ros::ok()) {
            visualization_msgs::Marker arrow;
            visualization_msgs::Marker number;
            visualization_msgs::Marker circular;
            circular.header.frame_id = arrow.header.frame_id = number.header.frame_id = pose.header.frame_id;
            arrow.ns = "navi_point_arrow";
            number.ns = "navi_point_number";
            circular.ns = "navi_point_circular";
            circular.action = arrow.action = number.action = visualization_msgs::Marker::ADD;
            arrow.type = visualization_msgs::Marker::ARROW;
            number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            circular.type = visualization_msgs::Marker::SPHERE;
            circular.pose = arrow.pose = number.pose = pose.pose;
            number.pose.position.x += 0.3;
            number.pose.position.y -= 0.3;
            arrow.scale.x = 0.4;
            arrow.scale.y = 0.15;
            circular.scale.x = 0.25;
            circular.scale.y = 0.25;
            circular.scale.z = 0;
            number.scale.z = 0.5;

            number.color.r = arrow.color.r = circular.color.r = 0.1f;
            number.color.g = arrow.color.g = circular.color.g = 0.1f;
            number.color.b = arrow.color.b = circular.color.b = 1.0f;
            number.color.a = arrow.color.a = circular.color.a = 1;

            circular.id = arrow.id = number.id = id;
            number.text = std::to_string(id+1);
            marker_pub_.publish(arrow);
            marker_pub_.publish(number);
            marker_pub_.publish(circular);

            cancelSelectPoseTable();
        }
    }

    void MultiNaviGoalsPanel::DeleteCubeMark(int id)
    {
        visualization_msgs::Marker arrow;
        visualization_msgs::Marker number;
        visualization_msgs::Marker circular;
        //arrow.header.frame_id = number.header.frame_id = pose->header.frame_id;
        arrow.ns = "navi_point_arrow";
        number.ns = "navi_point_number";
        circular.ns = "navi_point_circular";
        arrow.action = number.action = circular.action = visualization_msgs::Marker::DELETE;
        arrow.type = visualization_msgs::Marker::ARROW;
        number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        circular.type = visualization_msgs::Marker::SPHERE;
        arrow.id = id;
        number.id = id;
        circular.id = id;
        marker_pub_.publish(arrow);
        marker_pub_.publish(number);
        marker_pub_.publish(circular);
    }

    void MultiNaviGoalsPanel::cancelSelectPoseTable()
    {
        disconnect(poseArray_table_, SIGNAL(itemSelectionChanged()), this, SLOT(updatePoseTableSelectRow()));
        poseArray_table_->setCurrentItem(NULL);
        connect(poseArray_table_, SIGNAL(itemSelectionChanged()), this, SLOT(updatePoseTableSelectRow()));
    }


    // check whether it is in the cycling situation
    void MultiNaviGoalsPanel::checkCycle() {
        cycle_ = cycle_checkbox_->isChecked();
        nh_.setParam("/rviz_navi_cycle", cycle_);
    }

    // start to navigate, and only command the first goal
    void MultiNaviGoalsPanel::startNavi() {
        curGoalIdx_ = curGoalIdx_ % pose_array_.poses.size();
        if (!pose_array_.poses.empty() && curGoalIdx_ < maxNumGoal_) {
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_.header;
            goal.pose = pose_array_.poses.at(curGoalIdx_);
            goal_pub_.publish(goal);
            ROS_INFO("Navi to the Goal%d", curGoalIdx_ + 1);
            poseArray_table_->item(curGoalIdx_, 0)->setBackgroundColor(QColor(255, 69, 0));
            poseArray_table_->item(curGoalIdx_, 1)->setBackgroundColor(QColor(255, 69, 0));
            poseArray_table_->item(curGoalIdx_, 2)->setBackgroundColor(QColor(255, 69, 0));
            curGoalIdx_ += 1;
            permit_ = true;
        } else {
            ROS_ERROR("Something Wrong");
        }
    }

    // complete the remaining goals
    void MultiNaviGoalsPanel::completeNavi() {
        if (curGoalIdx_ < pose_array_.poses.size()) {
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_.header;
            goal.pose = pose_array_.poses.at(curGoalIdx_);
            goal_pub_.publish(goal);
            ROS_INFO("Navi to the Goal%d", curGoalIdx_ + 1);
            poseArray_table_->item(curGoalIdx_, 0)->setBackgroundColor(QColor(255, 69, 0));
            poseArray_table_->item(curGoalIdx_, 1)->setBackgroundColor(QColor(255, 69, 0));
            poseArray_table_->item(curGoalIdx_, 2)->setBackgroundColor(QColor(255, 69, 0));
            curGoalIdx_ += 1;
            permit_ = true;
        } else {
            ROS_INFO("All goals are completed");
            permit_ = false;
        }
    }

    // command the goals cyclically
    void MultiNaviGoalsPanel::cycleNavi() {
        if (permit_) {
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_.header;
            goal.pose = pose_array_.poses.at(curGoalIdx_ % pose_array_.poses.size());
            goal_pub_.publish(goal);
            ROS_INFO("Navi to the Goal%lu, in the %dth cycle", curGoalIdx_ % pose_array_.poses.size() + 1,
                     cycleCnt_ + 1);
            bool even = ((cycleCnt_ + 1) % 2 != 0);
            QColor color_table;
            if (even) color_table = QColor(255, 69, 0); else color_table = QColor(100, 149, 237);
            poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 0)->setBackgroundColor(color_table);
            poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 1)->setBackgroundColor(color_table);
            poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 2)->setBackgroundColor(color_table);
            curGoalIdx_ += 1;
            cycleCnt_ = curGoalIdx_ / pose_array_.poses.size();
        }
    }

    // cancel the current command
    void MultiNaviGoalsPanel::cancelNavi() {
        if (!cur_goalid_.id.empty()) {
            cancel_pub_.publish(cur_goalid_);
            ROS_INFO("Navigation have been canceled");
        }
    }

    // call back for listening current state
    void MultiNaviGoalsPanel::statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses) {
        bool arrived_pre = arrived_;
        arrived_ = checkGoal(statuses->status_list);
        if (arrived_ && arrived_ != arrived_pre && ros::ok() && permit_) {
//            ros::Rate r(1);
//            r.sleep();
            if (cycle_) cycleNavi();
            else completeNavi();
        }
    }

    //check the current state of goal
    bool MultiNaviGoalsPanel::checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list) {
        bool done = false;
        if (!status_list.empty()) {
            for (auto &i : status_list) {
                if (i.status == 3
                    && i.goal_id.id == cur_goalid_.id
                    && i.goal_id.id != pre_goalid_.id)
                {
                    pre_goalid_ = i.goal_id;
                    done = true;
//                    ROS_INFO("completed Goal%d", curGoalIdx_);
                } else if (i.status == 4) {
                    ROS_WARN("Goal%d is Invalid, Navi to Next Goal%d", curGoalIdx_, curGoalIdx_ + 1);
                    return true;
                } else if (i.status == 0) {
                    done = true;
                } else if (i.status == 1) {
                    cur_goalid_ = i.goal_id;
                    done = false;
                } else if (i.status == 2) {
                    pre_goalid_ = i.goal_id;
                } else done = false;
            }
        } else {
//            ROS_INFO("Please input the Navi Goal");
            done = false;
        }
        return done;
    }

// spin for subscribing
    void MultiNaviGoalsPanel::startSpin() {
        if (ros::ok()) {
            ros::spinOnce();
        }
    }

    void MultiNaviGoalsPanel::InitTableItem(int num)
    {
        // 当表格行数减少，减少item
        while (vector_table_items.size() > num)
        {
            vector_table_items.pop_back();
        }

        // 当表格行数增加，创建item
        while (num > vector_table_items.size())
        {
            QTableWidgetItem *item = new QTableWidgetItem();
            item->setTextAlignment(Qt::AlignCenter);
            vector_table_items.push_back(item);
        }

    }

    void MultiNaviGoalsPanel::UpdateLine(QTableWidgetItem *item)
    {
        cancelSelectPoseTable();
        if(write_flag == false)
        {
            //ROS_INFO("**%d**",item->row());
            QTableWidgetItem *text;
            geometry_msgs::PoseArray modify_item;
            text = poseArray_table_->item(item->row(),item->column());
            int index = item->column();
            int row = item->row();
            if(text->text() != "")
            {
                if(row+1 <= pose_array_.poses.size())
                {
                    switch (index) {
                    case 0:{
                        pose_array_.poses.at(row).position.x = text->text().toDouble();
                    }break;
                    case 1:{
                        pose_array_.poses.at(row).position.y = text->text().toDouble();
                    }break;
                    case 2:{
                        pose_array_.poses.at(row).orientation =
                                tf::createQuaternionMsgFromYaw(text->text().toDouble() * M_PI/180);
                    }break;
                    case 3:{
                        task_array.at(row) = text->text().toStdString();
                    }break;
                    default:break;
                    }
                    //ROS_INFO("ROW:%d",row+1);
                    geometry_msgs::PoseStamped pose_temp;
                    pose_temp.header.frame_id = pose_array_.header.frame_id;
                    pose_temp.pose = pose_array_.poses.at(row);
                    markPose(pose_temp,row);
                    //ROS_WARN("id:%d",row+1);
                }
                else
                {
                    item->setText("");
                }
            }
            else
            {
                //poseArray_table_->setItem(item->row(),item->column(),item);
                ROS_WARN("cannot input null!!!");
                write_flag = true;
                if(!pose_array_.poses.empty() && row+1<=pose_array_.poses.size())
                {
                    switch (index)
                    {
                    case 0:{
                        item->setText(QString::number(pose_array_.poses.at(row).position.x, 'f', 2));
                    }break;
                    case 1:{
                        item->setText(QString::number(pose_array_.poses.at(row).position.y, 'f', 2));
                    }break;
                    case 2:{
                        item->setText(QString::number(tf::getYaw(pose_array_.poses.at(row).orientation) * 180.0 / 3.14, 'f', 2));
                    }break;
                    default:break;
                    }
                }
                write_flag = false;
            }
        }
    }
} // end namespace navi-multi-goals-pub-rviz-plugin

// 声明此类是一个rviz的插件

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(navi_multi_goals_pub_rviz_plugin::MultiNaviGoalsPanel, rviz::Panel)

