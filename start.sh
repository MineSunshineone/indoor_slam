#!/bin/bash
SESSION="indoor_slam"

# ── ROS 域必须与 bxi_rc_ros2 网关一致 ──────────────────────────────
# App 的重定位/导航请求链路: App REST → bxi_rc_ros2 桥 (/opt/bxi/bxi_rc_ros2,
# env.conf 默认 ROS_DOMAIN_ID=22) → ROS 服务 /nav/init → 本栈。
# 此前这里硬编码 37, 与网关不同域 → DDS 互相发现不了, App 端重定位报
# "nav init service unavailable"、nav.pose 也收不到 (App 一直提示需要重定位)。
# 如机器人上 env.conf 改过域号, 启动前 export ROS_DOMAIN_ID=<同值> 覆盖。
# 注意: 网关默认 RMW 为 CycloneDDS (钉 lo 回环); 本栈若与网关跨 RMW 偶发
# 发现不了服务, 在下面追加 export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 对齐。
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-22}"

# 1. 检查并安装 tmux
if ! command -v tmux &> /dev/null; then
    sudo apt update && sudo apt install tmux -y
fi

# 2. 清理旧会话并创建新会话（这步绝对不能省！）
tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION

# 3. 基础配置
tmux set-option -g mouse on
tmux set-option -g pane-border-status top
tmux set-option -g pane-border-format " #[fg=black,bg=green] #T #[default] "

# ---------------------------------------------------------------
# 核心布局划分（标准 4 宫格田字格）
# ---------------------------------------------------------------
# 先左右对半切 (生成 0 和 1)
tmux split-window -h -p 50 -t $SESSION

# 把左半边上下对半切 (生成 2，位于左下)
tmux select-pane -t 0
tmux split-window -v -p 50 -t $SESSION

# 把右半边上下对半切 (生成 3，位于右下)
tmux select-pane -t 2
tmux split-window -v -p 50 -t $SESSION

# ---------------------------------------------------------------
# 命名窗格标题 (根据切分逻辑：0=左上, 2=左下, 1=右上, 3=右下)
# ---------------------------------------------------------------
tmux select-pane -t 0 -T "雷达驱动"
tmux select-pane -t 2 -T "里程计"
tmux select-pane -t 1 -T "重定位"
tmux select-pane -t 3 -T "导航"

# ---------------------------------------------------------------
# 发送 ROS2 指令
# ---------------------------------------------------------------
# 0号窗格 (左上): 雷达
tmux send-keys -t $SESSION:0.0 "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && source install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360s_launch.py" C-m
sleep 2

# 2号窗格 (左下): 里程计 (Point-LIO)
tmux send-keys -t $SESSION:0.2 "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && source install/setup.bash && ros2 launch point_lio point_lio_with_mapping_control.launch.py" C-m
sleep 2

# 1号窗格 (右上): 重定位 (small_gicp)
tmux send-keys -t $SESSION:0.1 "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && source install/setup.bash && ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py" C-m

# 3号窗格 (右下): 导航 (nav)
tmux send-keys -t $SESSION:0.3 "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && source install/setup.bash && ros2 launch nav indoor_navigation_launch.py" C-m

# App 接入网关：rosbridge websocket，App 通过 ws://机器人IP:9090 访问 ROS topic/service/action。
APP_WINDOW="App网关"
tmux new-window -d -t "$SESSION:" -n "$APP_WINDOW"
tmux send-keys -t "$SESSION:$APP_WINDOW" "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && source install/setup.bash && if ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then ros2 launch rosbridge_server rosbridge_websocket_launch.xml; else echo 'rosbridge_server 未安装，请先安装 ros-humble-rosbridge-server'; bash; fi" C-m

# 打开会话并聚焦在左上角
tmux select-window -t $SESSION:0
tmux select-pane -t 0
tmux attach-session -t $SESSION
