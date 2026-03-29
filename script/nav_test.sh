#!/bin/bash

# ============================================================
# 机器人系统分步启动脚本
# 功能：按顺序独立启动 Gazebo, Display, Filter, Hector, Nav
# ============================================================

# 定义颜色输出，方便查看状态
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 进入工作空间并
cd ~/robotcup2026

echo -e "${YELLOW}[1/5] 启动 gazebo.launch...${NC}"
# 启动 gazebo.launch (假设在 car 包中)
# 使用 & 放入后台，sleep 等待其初始化
roslaunch car gazebo.launch &
GAZEBO_PID=$!
sleep 8  # Gazebo 加载模型需要较长时间，建议等待 8-10 秒
echo -e "${GREEN}-> gazebo.launch (PID: $GAZEBO_PID)${NC}"

echo -e "${YELLOW}[2/5] 启动 display.launch (RViz/TF)...${NC}"
# 启动 display.launch (假设在 car 包中)
roslaunch car display.launch &
DISPLAY_PID=$!
sleep 2
echo -e "${GREEN}-> display.launch 已启动 (PID: $DISPLAY_PID)${NC}"

echo -e "${YELLOW}[3/5] 启动 box_filter.launch...${NC}"
# 启动 box_filter.launch (假设在 sim_hector 包中)
roslaunch sim_hector box_filter.launch &
FILTER_PID=$!
sleep 1
echo -e "${GREEN}-> box_filter.launch 已启动 (PID: $FILTER_PID)${NC}"

echo -e "${YELLOW}[4/5] 启动 hector.launch...${NC}"
# 启动 hector.launch (假设在 sim_hector 包中)
# 确保 use_sim_time 为 true
roslaunch sim_hector hector.launch &
HECTOR_PID=$!
sleep 3  # 等待 Hector 初始化坐标系
echo -e "${GREEN}-> hector.launch 已启动 (PID: $HECTOR_PID)${NC}"

echo -e "${YELLOW}[5/5] 启动 nav.launch (Move_base + TEB)...${NC}"
# 启动 nav.launch (假设在 sim_nav 包中)
roslaunch sim_nav nav.launch &
NAV_PID=$!
sleep 2
echo -e "${GREEN}-> nav.launch 已启动 (PID: $NAV_PID)${NC}"

echo -e "${GREEN}==========================================${NC}"
echo -e "${GREEN}   所有节点已启动！正在打开调试工具...   ${NC}"
echo -e "${GREEN}==========================================${NC}"

# 可选：自动打开 rqt_tf_tree 和 rqt_reconfigure (对应你截图中的需求)
# 如果不需要，可以注释掉下面两行
sleep 2
rosrun rqt_tf_tree rqt_tf_tree &
rosrun rqt_reconfigure rqt_reconfigure &

echo -e "${GREEN}系统启动完成！${NC}"
echo -e "提示：如需停止所有节点，请运行：${RED}killall -9 roslaunch${NC}"

# 保持脚本运行，以便捕获 Ctrl+C 停止所有后台进程
wait