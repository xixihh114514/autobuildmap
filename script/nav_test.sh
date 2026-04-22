#!/bin/bash

# ============================================================
# RoboCup 一键启动脚本
# 功能：按指定顺序启动雷达、相机、底盘显示、建图、导航、探索和目标决策
# ============================================================

set -u

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

WORKSPACE="/home/rera/robocup2026"
PIDS=()

cleanup() {
  echo -e "\n${YELLOW}正在停止本脚本启动的 roslaunch 进程...${NC}"
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null
    fi
  done
  wait 2>/dev/null
  echo -e "${GREEN}已停止。${NC}"
}

trap cleanup INT TERM EXIT

launch_step() {
  local index="$1"
  local total="$2"
  local wait_seconds="$3"
  shift 3

  echo -e "${YELLOW}[${index}/${total}] 启动：roslaunch $*${NC}"
  roslaunch "$@" &
  local pid=$!
  PIDS+=("$pid")
  sleep "$wait_seconds"
  echo -e "${GREEN}-> 已启动 PID: ${pid}${NC}"
}

# setup_gimbal_can() {
#   local index="$1"
#   local total="$2"

#   echo -e "${YELLOW}[${index}/${total}] 配置：sudo ip link set gimbalcan up type can bitrate 1000000${NC}"
#   if sudo ip link set gimbalcan up type can bitrate 1000000; then
#     echo -e "${GREEN}-> gimbalcan 已设置为 1000000 bitrate 并启动${NC}"
#   else
#     echo -e "${RED}-> gimbalcan 配置失败，停止后续启动。${NC}"
#     exit 1
#   fi
# }

cd "$WORKSPACE" || exit 1

if [ -f "$WORKSPACE/devel/setup.bash" ]; then
  source "$WORKSPACE/devel/setup.bash"
elif [ -f "$WORKSPACE/devel/setup.sh" ]; then
  source "$WORKSPACE/devel/setup.sh"
fi

TOTAL_STEPS=12

launch_step 1 "$TOTAL_STEPS" 3 rplidar_ros rplidar_a3.launch
setup_gimbal_can 2 "$TOTAL_STEPS"
launch_step 3 "$TOTAL_STEPS" 2 "$WORKSPACE/src/imu_run/launch/imu_position_speed.launch"
launch_step 4 "$TOTAL_STEPS" 5 orbbec_camera astra_adv.launch
launch_step 5 "$TOTAL_STEPS" 2 car display.launch
launch_step 6 "$TOTAL_STEPS" 2 sim_hector box_filter.launch
launch_step 7 "$TOTAL_STEPS" 5 sim_hector hector.launch
launch_step 8 "$TOTAL_STEPS" 4 sim_nav nav.launch
launch_step 9 "$TOTAL_STEPS" 3 rrt_exploration single.launch
launch_step 10 "$TOTAL_STEPS" 2 rrt_goal_decision rrt_goal_decision.launch
launch_step 11 "$TOTAL_STEPS" 2 visual_calibration visual_calibration.launch
launch_step 12 "$TOTAL_STEPS" 1 visual_grid_mapper visual_grid_mapper.launch

echo -e "${GREEN}==========================================${NC}"
echo -e "${GREEN}   所有 roslaunch 已按顺序启动完成。      ${NC}"
echo -e "${GREEN}==========================================${NC}"
echo -e "按 ${RED}Ctrl+C${NC} 可停止本脚本启动的所有进程。"

wait
