# LIMO多机器人编队功能包使用说明

## 1. 功能包概述

LIMO多机器人编队功能包(limo_multi)是一个用于实现多机器人协同编队和导航的ROS功能包。该功能包支持一台领导者机器人和多台跟随者机器人的编队控制，具有以下主要功能：

- 多机器人编队控制
- 基于TF的位置跟踪
- 障碍物避障
- 键盘遥控控制
- 多机器人协同导航

本功能包适用于LIMO系列机器人，支持实际硬件和仿真环境。

## 2. 安装步骤

### 2.1 前提条件

- Ubuntu 20.04
- ROS Noetic
- LIMO机器人底盘驱动已安装

### 2.2 安装过程

1. 将功能包复制到ROS工作空间的src目录下：

```bash
cp -r limo_multi ~/catkin_ws/src/
```

2. 编译功能包：

```bash
cd ~/catkin_ws
catkin_make
```

3. 设置环境变量：

```bash
source ~/catkin_ws/devel/setup.bash
```

## 3. 使用方法

### 3.1 编队控制

#### 3.1.1 实际硬件环境

在实际硬件环境中，需要至少两台LIMO机器人（一台领导者，一台或多台跟随者）。每台机器人需要安装ROS和本功能包。

1. 在领导者机器人上运行：

```bash
roslaunch limo_multi navigation.launch
```

2. 在跟随者机器人上运行：

```bash
roslaunch limo_multi limo_follower.launch
```

3. 使用键盘控制领导者机器人：

```bash
roslaunch limo_multi keyboard_teleop.launch
```

#### 3.1.2 仿真环境

在仿真环境中，可以在单台计算机上模拟多机器人编队：

```bash
roslaunch limo_multi limo_formation.launch
```

### 3.2 自动控制多台机器人

功能包提供了一个Python脚本用于自动控制多台机器人：

```bash
python3 ~/catkin_ws/src/limo_multi/cmd.py
```

按照脚本提示输入命令，可以控制所有机器人的启动和停止。

### 3.3 编队模式

功能包支持两种编队模式：

1. 模式1：领导者自转，跟随者围绕领导者运动
2. 模式2：领导者自转，跟随者原地自转（跟随者模仿领导者运动）

可以在launch文件中通过`formation_mode`参数设置编队模式：

```xml
<arg name="formation_mode" value="2" />
```

对于阿克曼转向的LIMO机器人，推荐使用模式2。

## 4. 参数配置

### 4.1 跟随者位置配置

可以在launch文件中配置跟随者相对于领导者的期望位置：

```xml
<arg name="follower_x" value="-0.8"/>  <!-- 领导者前方为正方向 -->
<arg name="follower_y" value="0.8"/>   <!-- 领导者左方为正方向 -->
```

### 4.2 避障参数配置

可以在launch文件中配置避障相关参数：

```xml
<param name="avoidance_kv" type="double" value="0.4" />      <!-- 线速度避障系数 -->
<param name="avoidance_kw" type="double" value="0.4" />      <!-- 角速度避障系数 -->
<param name="safe_distance" type="double" value="0.4" />     <!-- 安全距离界限(米) -->
<param name="danger_distance" type="double" value="0.2" />   <!-- 危险距离界限(米) -->
```

### 4.3 速度限制配置

可以在launch文件中配置速度限制：

```xml
<arg name="max_vel_x" value="0.8" />       <!-- 最大线速度限制(m/s) -->  
<arg name="min_vel_x" value="0.05" />      <!-- 最小线速度限制(m/s) -->
<arg name="max_vel_theta" value="0.8" />   <!-- 最大角速度限制(rad/s) -->  
<arg name="min_vel_theta" value="0.05" />  <!-- 最小角速度限制(rad/s) --> 
```

## 5. 文件结构说明

- `launch/`: 包含所有启动文件
  - `limo_follower.launch`: 跟随者启动文件
  - `limo_follower_sim.launch`: 仿真环境下的跟随者启动文件
  - `limo_formation.launch`: 多机器人编队启动文件
  - `keyboard_teleop.launch`: 键盘控制启动文件
  - `navigation.launch`: 导航启动文件
  - `navigation_sim.launch`: 仿真环境下的导航启动文件
  - `tf_setup.launch`: TF设置启动文件
  - `formation_rviz.launch`: RViz可视化启动文件

- `src/`: 包含C++源代码
  - `follower_tf_listener.cpp`: 跟随者TF监听节点
  - `collision_avoidance.cpp`: 碰撞避障节点

- `scripts/`: 包含Python脚本和Shell脚本
  - `receive_leader_info.py`: 接收领导者信息的节点
  - `send_tfodom.py`: 发送领导者信息的节点
  - `set_pose.py`: 设置初始位姿的节点
  - `robot_teleop_key.py`: 键盘遥控节点
  - `laserTracker.py`: 激光雷达障碍物跟踪节点
  - `leader_car.sh`: 启动领导者车辆的脚本
  - `follower_car.sh`: 启动跟随者车辆的脚本
  - `kill_all.sh`: 终止所有相关进程的脚本

- `msg/`: 包含自定义消息定义
  - `obstacle.msg`: 障碍物信息消息定义

- `param/`: 包含参数配置文件
  - `mux.yaml`: 多路复用器配置文件

- `rviz/`: 包含RViz配置文件
  - `multi_robot_formation.rviz`: 多机器人编队可视化配置

- `cmd.py`: 多机器人控制脚本

## 6. 常见问题解答

### 6.1 机器人不能正确跟随

**问题**: 跟随者机器人不能正确跟随领导者机器人。

**解决方案**:
- 检查网络连接，确保所有机器人在同一网络中
- 检查ROS主节点设置，确保所有机器人使用相同的ROS_MASTER_URI
- 检查TF变换是否正确发布和接收
- 调整跟随参数，如k_v、k_l和k_a

### 6.2 避障功能不工作

**问题**: 机器人不能正确避开障碍物。

**解决方案**:
- 检查激光雷达是否正常工作
- 检查laserTracker.py节点是否正常运行
- 调整避障参数，如avoidance_kv、avoidance_kw、safe_distance和danger_distance

### 6.3 编队模式选择

**问题**: 不确定应该使用哪种编队模式。

**解决方案**:
- 对于阿克曼转向的LIMO机器人，推荐使用模式2
- 对于差速驱动的机器人，两种模式都可以使用
- 在模式1下，领导者角速度不宜过大

## 7. 联系方式

如有任何问题或建议，请联系：limo_developer@example.com
