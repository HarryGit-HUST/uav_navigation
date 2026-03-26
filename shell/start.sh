#!/bin/bash

# 【中文编码修复】强制设置 UTF-8 环境变量，并导出给 tmux 继承
export LANG=zh_CN.UTF-8
export LC_ALL=zh_CN.UTF-8
export ROS_LOG_DIR=$HOME/.ros/log

# 【关键修复】tmux 默认会清空环境变量，需要配置 tmux 保留 UTF-8 支持
# 在 tmux.conf 中设置或启动时传入
tmux set-option -gq default-command "/bin/bash" 2>/dev/null

# Session 名称
SESSION="ego_planner_session"

# ================= 配置路径 =================
MAIN_WS=~/first_task_ws
PX4_PATH=/home/jetson/Libraries/PX4-Autopilot

# 清理旧环境
tmux kill-session -t $SESSION 2>/dev/null
sleep 1

echo "=============================================="
echo "   UAV Navigation 启动脚本"
echo "=============================================="
echo "工作空间：$MAIN_WS"
echo "PX4 路径：$PX4_PATH"
echo "=============================================="
echo ""

# ====================================================
# 窗口 0: 基础仿真 (Sim + Core)
# ====================================================
tmux new-session -d -s $SESSION -n "sim_core"
tmux send-keys -t $SESSION:0.0 'export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; roscore' C-m
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "sleep 3; \
export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
source ${MAIN_WS}/devel/setup.bash; \
source ${PX4_PATH}/Tools/setup_gazebo.bash ${PX4_PATH} ${PX4_PATH}/build/px4_sitl_default; \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:${PX4_PATH}:${PX4_PATH}/Tools/sitl_gazebo; \
echo '============================================'; \
echo '  启动 Gazebo 仿真'; \
echo '  首次启动需要加载 LiDAR 模型 (约 2-3 分钟)'; \
echo '  请等待日志显示 [ros topic name:/livox/lidar]'; \
echo '============================================'; \
roslaunch tutorial_gazebo sim.launch" C-m
tmux select-layout -t $SESSION:0 tiled

# ====================================================
# 窗口 1: 感知与点云拉伸处理
# ====================================================
tmux new-window -t $SESSION:1 -n "perception"

# 1.0: PCL 检测脚本 - 等待 Gazebo 启动后再启动
tmux send-keys -t $SESSION:1.0 "export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
echo '等待 Gazebo 启动...'; sleep 15; \
source ${MAIN_WS}/devel/setup.bash; cd ${MAIN_WS}/src/pcl_detection2/shell; bash pcl_detection.sh" C-m

# 1.1: 点云拉伸节点
tmux split-window -h -t $SESSION:1
tmux send-keys -t $SESSION:1.1 "export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
echo '等待 Gazebo 启动...'; sleep 17; \
source ${MAIN_WS}/devel/setup.bash; rosrun uav_navigation cloud_extruder" C-m
tmux select-layout -t $SESSION:1 tiled

# ====================================================
# 窗口 2: 导航大脑与飞控执行
# ====================================================
tmux new-window -t $SESSION:2 -n "nav_and_control"

# Pane 2.0: 启动整合包 - 等待 Gazebo 完全启动
tmux send-keys -t $SESSION:2.0 "export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
echo '等待 Gazebo 启动...'; sleep 20; \
source ${MAIN_WS}/devel/setup.bash; roslaunch uav_navigation ego_nav.launch" C-m

# Pane 2.1: 监控 ego_controller 状态
tmux split-window -v -t $SESSION:2
tmux send-keys -t $SESSION:2.1 "export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
sleep 25; rostopic echo /ego_controller/status" C-m

# Pane 2.2: 监控 EGO 目标点
tmux split-window -h -t $SESSION:2
tmux send-keys -t $SESSION:2.2 "export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
sleep 25; rostopic echo /fsm/ego_goal" C-m
tmux select-layout -t $SESSION:2 tiled

# ====================================================
# 窗口 3: 综合监控窗口 (含 LiDAR 等待进度)
# ====================================================
tmux new-window -t $SESSION:3 -n "monitor"
tmux send-keys -t $SESSION:3.0 "export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
sleep 5; \
echo '=== 监控话题列表 ==='; \
rostopic list" C-m
tmux split-window -v -t $SESSION:3
tmux send-keys -t $SESSION:3.1 "export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
sleep 10; echo '=== 检查 /livox/lidar 话题 ==='; \
while ! rostopic info /livox/lidar &>/dev/null; do \
  echo \"[\$(date +%H:%M:%S)] 等待 /livox/lidar 话题发布...\"; \
  sleep 5; \
done; \
echo '=== /livox/lidar 话题已就绪 ==='; \
rostopic hz /livox/lidar" C-m
tmux select-layout -t $SESSION:3 tiled

# ====================================================
# 窗口 4: 日志调试窗口
# ====================================================
tmux new-window -t $SESSION:4 -n "debug_logs"
tmux send-keys -t $SESSION:4.0 "export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
sleep 25; \
echo '=== Ego Controller 日志 ==='; \
rosrun rqt_console rqt_console" C-m
tmux split-window -h -t $SESSION:4
tmux send-keys -t $SESSION:4.1 "export LANG=zh_CN.UTF-8; export LC_ALL=zh_CN.UTF-8; \
sleep 26; \
echo '=== FSM 状态机日志 ==='; \
rosrun rqt_console rqt_console" C-m
tmux select-layout -t $SESSION:4 tiled

# 附加到会话
tmux select-window -t $SESSION:0
echo ""
echo "=============================================="
echo "   启动完成！"
echo "=============================================="
echo ""
echo "窗口说明:"
echo "  [0] sim_core     - Gazebo 仿真 + roscore (主窗口)"
echo "  [1] perception   - PCL 点云处理"
echo "  [2] nav_control  - 导航与控制"
echo "  [3] monitor      - 话题监控 (含 LiDAR 等待进度)"
echo "  [4] debug_logs   - 调试日志"
echo ""
echo "操作提示:"
echo "  1. 在窗口 [0] 等待 Gazebo 加载 (约 2-3 分钟)"
echo "  2. 看到 [ros topic name:/livox/lidar] 表示启动成功"
echo "  3. 然后切换到窗口 [2] 按 [1] 启动任务"
echo "  4. 使用 Ctrl+B 然后按方向键切换窗口"
echo ""
echo "=============================================="
echo ""

tmux attach-session -t $SESSION
