#!/bin/bash
# 测试脚本：用于测试tf修复效果
# 使用方法：将此脚本放在ros_apf_formation目录下，执行 ./test_tf_fixes.sh

# 设置颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 确保脚本在ros_apf_formation目录下运行
if [ ! -d "./src" ] || [ ! -d "./devel" ]; then
    echo -e "${RED}错误：请在ros_apf_formation工作空间根目录下运行此脚本${NC}"
    exit 1
fi

# 编译工作空间
echo -e "${GREEN}[1/4] 编译工作空间...${NC}"
catkin_make || { echo -e "${RED}编译失败！${NC}"; exit 1; }

# 设置ROS环境
echo -e "${GREEN}[2/4] 设置ROS环境...${NC}"
source ./devel/setup.bash

# 测试tf_setup.launch
echo -e "${GREEN}[3/4] 测试tf_setup.launch...${NC}"
echo -e "${YELLOW}启动tf_setup.launch，将在5秒后检查tf树...${NC}"
roslaunch wheeltec_multi tf_setup.launch &
TF_SETUP_PID=$!
sleep 5

# 检查tf树
echo -e "${GREEN}[4/4] 检查tf树...${NC}"
rosrun tf view_frames
echo -e "${YELLOW}tf树已保存到当前目录下的frames.pdf文件${NC}"
echo -e "${YELLOW}请检查frames.pdf文件，确认map到各个机器人odom的转换关系是否正确建立${NC}"

# 终止tf_setup.launch
kill $TF_SETUP_PID

echo -e "${GREEN}测试完成！${NC}"
echo -e "${YELLOW}如果frames.pdf中显示map到各个机器人odom的转换关系已正确建立，则tf修复成功${NC}"
echo -e "${YELLOW}接下来可以使用start_formation.sh脚本启动完整系统进行测试${NC}"
