#!/bin/bash
set -euo pipefail

SESSION="indoor_slam"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-22}"
LIVOX_CONFIG_PATH="${LIVOX_CONFIG_PATH:-}"

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux is required: sudo apt install tmux" >&2
    exit 1
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n "SLAM主管"
tmux set-option -t "$SESSION" mouse on
tmux set-option -t "$SESSION" pane-border-status top

launch_command="export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && source install/setup.bash && ros2 launch bxi_slam_manager quiet_bringup.launch.py"
if [[ -n "$LIVOX_CONFIG_PATH" ]]; then
    launch_command+=" livox_config_path:=$LIVOX_CONFIG_PATH"
fi

tmux send-keys -t "$SESSION:0.0" "$launch_command" C-m
tmux select-pane -t "$SESSION:0.0" -T "雷达 + 静默模式主管"

echo "Indoor SLAM started in quiet idle mode (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)."
echo "Point-LIO, GICP, Nav2 and maps will start only after an App mode request."
tmux attach-session -t "$SESSION"
