#!/bin/bash

# ============================================================
# 局部规划器切换脚本
# 用法：./switch_planner.sh [teb|dwa]
# ============================================================

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LAUNCH_FILE="$SCRIPT_DIR/sim_nav/launch/nav.launch"

# 检查参数
if [ $# -eq 0 ]; then
    echo "用法：$0 [teb|dwa]"
    echo ""
    echo "  teb  - 使用 TEB 局部规划器 (默认)"
    echo "  dwa  - 使用 DWA 局部规划器"
    echo ""
    
    # 显示当前配置
    current=$(grep 'local_planner.*default=' "$LAUNCH_FILE" | grep -oP 'default="\K[^"]+')
    echo "当前配置：$current"
    exit 0
fi

# 根据参数设置规划器
case "$1" in
    teb)
        sed -i 's|local_planner.*default="[^"]*"|local_planner default="teb_local_planner/TebLocalPlannerROS"|' "$LAUNCH_FILE"
        echo "✓ 已切换到 TEB 局部规划器"
        ;;
    dwa)
        sed -i 's|local_planner.*default="[^"]*"|local_planner default="dwa_local_planner/DWAPlannerROS"|' "$LAUNCH_FILE"
        echo "✓ 已切换到 DWA 局部规划器"
        ;;
    *)
        echo "错误：未知参数 '$1'"
        echo "请使用：$0 [teb|dwa]"
        exit 1
        ;;
esac

# 显示当前配置
current=$(grep 'local_planner.*default=' "$LAUNCH_FILE" | grep -oP 'default="\K[^"]+')
echo "当前配置：$current"
echo ""
echo "启动导航："
echo "  roslaunch sim_nav nav.launch"
