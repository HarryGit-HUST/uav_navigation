#!/bin/zsh

# 【环境修复】Zsh 默认环境变量设置
export LANG=zh_CN.UTF-8
export LC_ALL=zh_CN.UTF-8
export ROS_LOG_DIR=$HOME/.ros/log

# 配置 tmux 使用 zsh 并在启动时加载配置
tmux set-option -gq default-shell /bin/zsh 2>/dev/null

# Session 名称
SESSION="ego_planner_session"

# ================= 配置路径 =================
MAIN_WS=~/first_task_ws
PX4_PATH=/home/jetson/Libraries/PX4-Autopilot

# 清理旧环境
tmux kill-session -t $SESSION 2>/dev/null
sleep 1

echo "=============================================="
echo "   🚀 UAV Navigation ZSH 启动脚本"
echo "=============================================="
echo "工作空间：$MAIN_WS"
echo "PX4 路径  ：$PX4_PATH"
echo "=============================================="

# ====================================================
# 窗口 0: 基础仿真 (Sim + Core)
# ====================================================
tmux new-session -d -s $SESSION -n "sim_core"
# Pane 0.0: roscore (Zsh 环境)
tmux send-keys -t $SESSION:0.0 'roscore' C-m

# Pane 0.1: Gazebo 仿真加载
tmux split-window -h -t $SESSION:0
# 注意：这里 source 的是 setup.zsh
tmux send-keys -t $SESSION:0.1 "sleep 3; \
export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
source ${MAIN_WS}/devel/setup.zsh; \
source ${PX4_PATH}/Tools/setup_gazebo.zsh ${PX4_PATH} ${PX4_PATH}/build/px4_sitl_default; \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:${PX4_PATH}:${PX4_PATH}/Tools/sitl_gazebo; \
roslaunch tutorial_gazebo sim.launch" C-m
tmux select-layout -t $SESSION:0 tiled

# ====================================================
# 窗口 1: 感知与处理
# ====================================================
tmux new-window -t $SESSION:1 -n "perception"
# Pane 1.0: PCL 处理脚本 (如果内部是 bash 脚本，依然用 bash 调用)
tmux send-keys -t $SESSION:1.0 "sleep 8; source ${MAIN_WS}/devel/setup.zsh; cd ${MAIN_WS}/src/pcl_detection2/shell; /bin/zsh pcl_detection.sh" C-m

# Pane 1.1: 点云拉伸节点
tmux split-window -h -t $SESSION:1
tmux send-keys -t $SESSION:1.1 "sleep 17; source ${MAIN_WS}/devel/setup.zsh; rosrun uav_navigation cloud_extruder" C-m
tmux select-layout -t $SESSION:1 tiled

# ====================================================
# 窗口 2: 导航大脑与飞控执行 (主战场)
# ====================================================
tmux new-window -t $SESSION:2 -n "nav_and_control"

# Pane 2.0: 监控坐标
tmux send-keys -t $SESSION:2.0 "sleep 5; source ${MAIN_WS}/devel/setup.zsh; rostopic echo /mavros/local_position/pose" C-m

# Pane 2.1: Ego-Planner 导航
tmux split-window -h -t $SESSION:2
tmux send-keys -t $SESSION:2.1 "sleep 12; source ${MAIN_WS}/devel/setup.zsh; roslaunch uav_navigation ego_nav.launch" C-m
tmux select-layout -t $SESSION:2 tiled

# ====================================================
# 窗口 3: 状态监控面板
# ====================================================
tmux new-window -t $SESSION:3 -n "status_monitor"
tmux send-keys -t $SESSION:3.0 "sleep 15; source ${MAIN_WS}/devel/setup.zsh; rostopic echo /ego_controller/status" C-m

# 附加会话，并跳转到主控页面
tmux select-window -t $SESSION:2
tmux select-pane -t $SESSION:2.1
tmux attach-session -t $SESSION