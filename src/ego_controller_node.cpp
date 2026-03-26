#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <algorithm>

ros::Publisher mavros_cmd_pub;
ros::Publisher ego_goal_pub;
ros::Publisher nav_status_pub;

nav_msgs::Odometry current_odom;
quadrotor_msgs::PositionCommand current_traj_cmd;
geometry_msgs::PoseStamped current_goal;

enum NavState
{
    IDLE = 0,
    FLYING = 1,
    ARRIVED = 2
};
NavState nav_state = IDLE;

// 控制与安全参数
double kp_pos = 0.8;        // 【修改】降低 P 参数，使其更柔和
double max_v = 1.0;         // 【新增】物理速度绝对限幅 (m/s)
double arrive_radius = 0.2; // 到达判定半径 (m)
bool has_odom = false;
bool has_traj = false;

// 【绝杀修复】保存固定悬停点，防止里程计正反馈导致向后直直乱飞！
double hover_x = 0.0;
double hover_y = 0.0;
double hover_z = 0.0;
double hover_yaw = 0.0;

// 1. 接收里程计
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_odom = *msg;
    has_odom = true;
}

// 2. 接收主控目标点
void fsmGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_goal = *msg;
    ROS_INFO("\n===========================================");
    ROS_INFO("[Ego_Controller] 收到长官命令！前往新目标: X=%.2f, Y=%.2f", msg->pose.position.x, msg->pose.position.y);
    ROS_INFO("===========================================\n");

    // 【绝杀修复】在收到命令的瞬间，锁死当前的真实物理坐标作为安全悬停点！
    hover_x = current_odom.pose.pose.position.x;
    hover_y = current_odom.pose.pose.position.y;
    hover_z = current_goal.pose.position.z; // 高度用目标的定高
    hover_yaw = current_odom.pose.pose.orientation.z;

    ego_goal_pub.publish(current_goal); // 转发给黑盒 Ego-Planner
    nav_state = FLYING;
    has_traj = false; // 重置轨迹接收标志
}

// 3. 接收 Ego 轨迹指令
void trajCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    current_traj_cmd = *msg;
    has_traj = true;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, ""); // 【修复】允许终端输出中文，防乱码！
    ros::init(argc, argv, "ego_controller_node");
    ros::NodeHandle nh("~");

    ros::Subscriber odom_sub = nh.subscribe("/mavros/local_position/odom", 10, odomCallback);
    ros::Subscriber fsm_goal_sub = nh.subscribe("/fsm/ego_goal", 1, fsmGoalCallback);
    ros::Subscriber traj_sub = nh.subscribe("/position_cmd", 10, trajCmdCallback);

    mavros_cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ego_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    nav_status_pub = nh.advertise<std_msgs::Int8>("/ego_controller/status", 10);

    ros::Rate rate(30); // 【重要】强制 30Hz 循环发布，绝不能断开 MAVROS

    while (ros::ok())
    {
        ros::spinOnce();
        if (!has_odom)
        {
            rate.sleep();
            continue;
        }

        mavros_msgs::PositionTarget setpoint;
        // 【尊重原著】使用你原来经过检验的 FRAME_LOCAL_NED = 1
        setpoint.coordinate_frame = 1; 

        if (nav_state == FLYING)
        {
            double dist = sqrt(pow(current_goal.pose.position.x - current_odom.pose.pose.position.x, 2) +
                               pow(current_goal.pose.position.y - current_odom.pose.pose.position.y, 2));

            if (dist < arrive_radius)
            {
                ROS_INFO("[Ego_Controller] 成功到达目标点附近！刹车！误差: %.2f 米", dist);
                nav_state = ARRIVED;
            }
            else if (has_traj)
            {
                // --- 丝滑且安全的轨迹追踪 ---
                // 【尊重原著】使用你原来的掩码 0b101111100011 控制 vx vy z yaw
                setpoint.type_mask = 0b101111100011;

                double err_x = current_traj_cmd.position.x - current_odom.pose.pose.position.x;
                double err_y = current_traj_cmd.position.y - current_odom.pose.pose.position.y;

                double cmd_vx = current_traj_cmd.velocity.x + kp_pos * err_x;
                double cmd_vy = current_traj_cmd.velocity.y + kp_pos * err_y;

                // 【绝杀修复】绝对速度限幅，防止前馈爆炸导致翻车！
                setpoint.velocity.x = std::max(-max_v, std::min(max_v, cmd_vx));
                setpoint.velocity.y = std::max(-max_v, std::min(max_v, cmd_vy));
                setpoint.position.z = current_traj_cmd.position.z;
                setpoint.yaw = current_traj_cmd.yaw;

                // 【新增监控】每秒打印一次底层控制数据，让你查错一目了然
                ROS_INFO_THROTTLE(1.0, "[Ego执行器] 距目标:%.2fm | 期望点(%.2f,%.2f) | 误差(%.2f,%.2f) | 发送速度(%.2f,%.2f)", 
                                  dist, current_traj_cmd.position.x, current_traj_cmd.position.y, err_x, err_y, setpoint.velocity.x, setpoint.velocity.y);

                mavros_cmd_pub.publish(setpoint);
            }
            else
            {
                // 【绝杀修复】还没收到轨迹时，使用锁死的固态悬停点！防止正反馈乱飞！
                ROS_WARN_THROTTLE(0.5, "[Ego_Controller] 等待 Ego-Planner 吐出轨迹，安全定点悬停中...");
                setpoint.type_mask = 0b101111111000; // 纯位置掩码
                setpoint.position.x = hover_x;
                setpoint.position.y = hover_y;
                setpoint.position.z = hover_z;
                setpoint.yaw = hover_yaw; 
                mavros_cmd_pub.publish(setpoint);
            }
        }
        else
        {
            // 如果处于 IDLE 或 ARRIVED 状态，本节点闭嘴，不发任何指令
            // 此时由主控 FSM 接管发悬停指令
        }

        std_msgs::Int8 status_msg;
        status_msg.data = nav_state;
        nav_status_pub.publish(status_msg);

        rate.sleep();
    }
    return 0;
}