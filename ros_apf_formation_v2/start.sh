#!/bin/bash
# 启动脚本：按顺序启动limo仿真和编队控制
# 使用方法：将此脚本放在ros_apf_formation目录下，执行 ./start_formation.sh

# 设置颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 确保脚本在ros_apf_formation目录下运行
if [ ! -d "./src" ] || [ ! -d "./devel" ]; then
    echo -e "${YELLOW}错误：请在ros_apf_formation工作空间根目录下运行此脚本${NC}"
    exit 1
fi

# 设置ROS环境
echo -e "${GREEN}[1/4] 设置ROS环境wowowowowo...${NC}"
source ./devel/setup.bash

# 启动Gazebo仿真环境
echo -e "${GREEN}[2/4] 启动Gazebo仿真环境了喔喔喔...${NC}"
echo -e "${YELLOW}注意喔：Gazebo启动ing，请耐心等待喔${NC}"
roslaunch limo_gazebo_sim limo_four_diff.launch &

# 等待40秒，并显示计时
echo -e "${GREEN}[3/4] 等待45秒后开始运行编队喔...${NC}"
for ((i=1; i<=45; i++)); do
    echo -ne "${YELLOW}Gazebo启动中喔，完成后自动运行编队喔: $i/45秒\r${NC}"
    sleep 1
done
echo -e "\n${GREEN}等待完成，准备启动编队了喔${NC}"

# 在另一个终端中进入/ros_apf_formation目录并运行sd和roslaunch
gnome-terminal -- bash -c "
cd /ros_apf_formation;
echo -e '${GREEN}[4/4] 启动编队控制了喔...${NC}';
source devel/setup.bash;
sd;
roslaunch limo_multi limo_formation.launch;
"

echo -e "${GREEN}所有系统已启动完成!${NC}"
echo -e "${YELLOW}提示：${NC}"
echo -e "  - 使用 ${GREEN}rosrun rqt_robot_steering rqt_robot_steering${NC} 控制主车(limo1)"
echo -e "  - 设置Topic为 ${GREEN}/limo1/cmd_vel${NC}"
echo -e "  - 按Ctrl+C终止所有进程"

# 等待用户按Ctrl+C终止
echo -e "${YELLOW}按Ctrl+C终止所有进程${NC}"
wait
