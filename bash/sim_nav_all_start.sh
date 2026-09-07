#!/bin/bash

# 工作空间根目录（脚本所在目录的上一级）
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SETUP_FILE="$WS_DIR/install/setup.bash"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WS_DIR/running_log/log}"

if [ ! -f "$SETUP_FILE" ]; then
    echo "[ERROR] 未找到工作空间 setup 文件: $SETUP_FILE"
    echo "请先执行: cd $WS_DIR && colcon build --symlink-install"
    exit 1
fi

mkdir -p "$ROS_LOG_DIR"

# WSL: Mesa 默认 llvmpipe，需强制 d3d12 才能用 RTX GPU 加速 GUI
WSL_GPU_ENV='export GALLIUM_DRIVER=d3d12; export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA;'

open_terminal() {
    local title="$1"
    local cmd="$2"
    local full_cmd="${WSL_GPU_ENV} source $SETUP_FILE && export ROS_LOG_DIR=\"$ROS_LOG_DIR\" && $cmd; exec bash"

    if command -v gnome-terminal &>/dev/null; then
        gnome-terminal --title="$title" -- bash -c "$full_cmd" &
    elif command -v xterm &>/dev/null; then
        xterm -T "$title" -e bash -c "$full_cmd" &
    elif command -v konsole &>/dev/null; then
        konsole --new-tab -p tabtitle="$title" -e bash -c "$full_cmd" &
    else
        echo "[ERROR] 未找到可用的终端模拟器 (gnome-terminal / xterm / konsole)"
        exit 1
    fi
}

echo "启动 SP Nav 教程仿真栈 ..."
echo "  上层决策树: click_nav.xml"
echo "  下层导航树: default_nav_with_fallback.xml"
echo "  规划器: A* | 控制器: PidController"
export QT_QPA_PLATFORM=xcb

open_terminal "tf2" "ros2 run tf2_ros static_transform_publisher 0 0.15 0 0 0 0 base_link livox_frame"
open_terminal "sp_nav" "ros2 launch sp_nav_bringup sp_nav.launch.py"
sleep 3
open_terminal "controller" "ros2 launch sp_controller_server controller.launch.py"
sleep 2
open_terminal "decision" "ros2 launch sp_decision decision.launch.py"
sleep 1
open_terminal "mgp_gui" "ros2 launch map_robot_gui map_drag_robot.launch.py"
echo "所有节点已在新终端中启动。"
