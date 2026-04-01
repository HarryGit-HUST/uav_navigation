# uav_navigation


## 📦 1. 仓库配置与编译

本模块是一个纯净的 ROS Package，作为底层依赖存在。

1. **下载与编译**：
   将 `uav_navigation` 放入你的工作空间 `src` 目录下，直接编译即可：
   ```zsh
   cd ~/your_ws
   catkin_make
   source devel/setup.zsh
   ```
2. **比赛坐标配置 (YAML)**：
   导航模块涉及的地图边界尺寸存放在 `uav_navigation/launch/advanced_param.xml` 中（已锁死 Z 轴高度防撞天花板，勿动）。
   *注意：你们主控的比赛目标点（巡航点、投物点）请你们自己在主控包里维护 YAML，本导航包不干涉上层业务逻辑。*

---

## 🚀 2. 节点启动规范 (如何在你的 sh 脚本中调用我)

本包**不提供**总启动脚本，请在你们的**总控 `.sh` 脚本** 或 **主 `launch` 文件**中，拉起我的导航环境。

**启动方法：直接 include 我的总控 launch 文件**
在你的主控 launch 文件（或在 sh 脚本中开一个 tmux pane）加入以下代码，即可瞬间拉起“点云处理 + Ego-Planner + 飞控翻译官”一整套导航黑盒：
```xml
<!-- 启动底层避障与导航黑盒 -->
<include file="$(find uav_navigation)/launch/ego_nav.launch" />
```
*(注意：请确保此时 Gazebo 或真机雷达、MAVROS 节点已经提前启动，否则点云接收会报错。)*

---

## 📡 3. 核心 API 接口 (FSM 如何与导航黑盒交互)

我们之间严格通过 **ROS Topic** 进行高内聚低耦合的交互。主控节点需完成以下订阅与发布：

### 📤 动作 1：下发目标点 (FSM -> Navigation)
* **话题名称**: `/fsm/ego_goal`
* **消息类型**: `geometry_msgs/PoseStamped`
* **调用规范**: 
  当状态机进入 `NAV_xxx` 状态时，向该话题发布**一次**目标坐标。
  > ⚠️ *底层处理说明：导航执行器收到该指令后，会自动唤醒 Ego-Planner，自动发送解锁轨迹信号（`/finish_ego = false`），并接管飞控，你无需关心这些细节。*

### 📥 动作 2：监听导航状态 (Navigation -> FSM)
* **话题名称**: `/ego_controller/status`
* **消息类型**: `std_msgs/Int8`
* **状态码定义**:
  * `0 (IDLE)`: 待机中。
  * `1 (FLYING)`: 正在执行避障飞行。
  * `2 (ARRIVED)`: **已成功到达目标点附近（误差 < 0.2m）！** 
* **调用规范**:
  主控在 `NAV_xxx` 状态下死循环监听此话题。一旦收到 `2`，主控即可切换到下一个状态（如 `HOVER`，并开始执行视觉识别或舵机投物）。

---

## ⚠️ 4. 绝对红线（主控开发必读）

为了防止无人机在空中发生控制权冲突导致炸机，请严格遵守以下开发规范：

1. 🛑 **控制权互斥原则（最重要）**：
   * 当无人机处于 **导航状态 (飞行中)** 时，主控 FSM **必须闭嘴**！绝对不允许在此时向 `/mavros/setpoint_raw/local` 发布任何指令！此时底层完全由我的 `ego_controller_node` 接管。
   * 只有在 **起飞 (TAKEOFF)**、**降落 (LANDING)**、**任务悬停 (HOVER_xxx)** 状态时，主控 FSM 才可以夺回控制权，自行发送原位悬停指令。
2. 🛑 **高度安全规范**：
   * 下发给导航模块的目标点 `Z` 轴高度，必须是**正数**（例如 1.2 米）。
   * 目标点高度**必须**在 `0.0` 到 `1.5` 米之间（此为 Ego 设定的地图天花板）。超出此范围，底层会判定为“飞入外太空/地下”，直接拒绝规划并原地锁死。

---

## ⌨️ 5. 主控调用伪代码示例 (Copy-Paste)

主控逻辑可以参考以下模板编写：

```cpp
// 1. 初始化发布与订阅
ros::Publisher ego_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/fsm/ego_goal", 1);
// 监听状态，存入 nav_status 变量
ros::Subscriber nav_status_sub = nh.subscribe("/ego_controller/status", 10, statusCallback);

// 2. FSM 逻辑流转
switch (current_state) {
    case WAIT_FOR_MAP:
        // 等待点云雷达积累 2 秒钟后，下发目标点！
        if (time_out) {
            sendEgoGoal(target_x, target_y, target_z); 
            current_state = NAV_RECOG_AREA;
        }
        break;

    case NAV_RECOG_AREA:
        // 【挂机等待】主控闭嘴，等待导航模块汇报到达
        if (nav_status == 2) { 
            current_state = HOVER_RECOGNIZE;
        }
        break;

    case HOVER_RECOGNIZE:
        // 【夺回控制权】主控向 MAVROS 发布定点悬停指令
        publishHoverSetpoint(target_x, target_y, target_z);
        
        // 执行视觉识别... 识别成功后下发下一个目标点
        if (vision_success) {
            sendEgoGoal(next_x, next_y, next_z);
            current_state = NAV_AIRDROP_AREA;
        }
        break;
}
```
# 调参说明
在ego_nav.launch 中可以调整参数
map_size_z_
ground_height
这两个可调
这些也可以调

    <arg name="max_vel" value="1.0" />
    <arg name="max_acc" value="1.0" />
    <arg name="planning_horizon" value="5.0" />
    <arg name="use_distinctive_trajs" value="false" />

    