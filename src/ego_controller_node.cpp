#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <algorithm>
#include <tf/transform_datatypes.h>

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
double kp_pos = 0.8;
double max_v = 1.0;
double arrive_radius = 0.2;
bool has_odom = false;
bool has_traj = false;
bool has_first_goal = false;

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
    has_first_goal = true;
    
    ROS_INFO(" ");
    ROS_INFO("===========================================");
    ROS_INFO("[Ego_Controller] 收到长官命令！前往新目标");
    ROS_INFO("    目标位置：X=%.2f, Y=%.2f, Z=%.2f", 
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("===========================================");
    ROS_INFO(" ");

    ego_goal_pub.publish(current_goal);
    nav_state = FLYING;
    has_traj = false;
    
    ROS_INFO("[Ego_Controller] 已转发目标给 EGO-Planner，等待轨迹...");
}

// 3. 接收 Ego 轨迹指令
void trajCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    current_traj_cmd = *msg;
    has_traj = true;
    
    ROS_DEBUG_STREAM_THROTTLE(2.0, 
        "[Ego_Controller] 收到轨迹点：" <<
        " pos=[" << msg->position.x << ", " << msg->position.y << ", " << msg->position.z << "] " <<
        " vel=[" << msg->velocity.x << ", " << msg->velocity.y << ", " << msg->velocity.z << "]");
}

// 从四元数提取 yaw 角（ENU 坐标系）
double getYawFromQuaternion(const geometry_msgs::Quaternion& q)
{
    tf::Quaternion tf_q;
    tf::quaternionMsgToTF(q, tf_q);
    tf::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

// 【核心修复】ENU 转 NED 坐标系转换
// EGO-Planner 输出 ENU 坐标，MAVROS 需要 NED 坐标
// ENU -> NED: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
// 速度同理
void enuToNed(double x_enu, double y_enu, double z_enu,
              double& x_ned, double& y_ned, double& z_ned)
{
    x_ned = y_enu;
    y_ned = x_enu;
    z_ned = -z_enu;
}

void enuVelToNed(double vx_enu, double vy_enu, double vz_enu,
                 double& vx_ned, double& vy_ned, double& vz_ned)
{
    vx_ned = vy_enu;
    vy_ned = vx_enu;
    vz_ned = -vz_enu;
}

// NED 坐标系下的 yaw 转换：yaw_ned = PI/2 - yaw_enu
double enuYawToNed(double yaw_enu)
{
    return M_PI_2 - yaw_enu;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ego_controller_node");
    ros::NodeHandle nh("~");

    ros::Subscriber odom_sub = nh.subscribe("/mavros/local_position/odom", 10, odomCallback);
    ros::Subscriber fsm_goal_sub = nh.subscribe("/fsm/ego_goal", 1, fsmGoalCallback);
    ros::Subscriber traj_sub = nh.subscribe("/position_cmd", 10, trajCmdCallback);

    mavros_cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ego_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 1);
    nav_status_pub = nh.advertise<std_msgs::Int8>("/ego_controller/status", 10);

    ros::Rate rate(30);

    ROS_INFO(" ");
    ROS_INFO("============================================");
    ROS_INFO("   [Ego Controller] 节点启动成功！         ");
    ROS_INFO("   监听话题：/fsm/ego_goal                 ");
    ROS_INFO("   输出话题：/mavros/setpoint_raw/local    ");
    ROS_INFO("============================================");
    ROS_INFO(" ");

    int loop_count = 0;
    
    while (ros::ok())
    {
        ros::spinOnce();
        if (!has_odom)
        {
            rate.sleep();
            continue;
        }

        loop_count++;
        mavros_msgs::PositionTarget setpoint;
        setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        double current_yaw_enu = getYawFromQuaternion(current_odom.pose.pose.orientation);
        
        if (loop_count % 30 == 0) {
            ROS_INFO_STREAM_THROTTLE(1.0,
                "[Ego_Controller] 状态：" << 
                " nav_state=" << nav_state <<
                " has_traj=" << has_traj <<
                " has_first_goal=" << has_first_goal <<
                " pos_enu=[" << current_odom.pose.pose.position.x << ", " 
                        << current_odom.pose.pose.position.y << ", " 
                        << current_odom.pose.pose.position.z << "]");
        }

        if (nav_state == FLYING)
        {
            double dist = sqrt(pow(current_goal.pose.position.x - current_odom.pose.pose.position.x, 2) +
                               pow(current_goal.pose.position.y - current_odom.pose.pose.position.y, 2));

            ROS_INFO_THROTTLE(1.0, "[Ego_Controller] 正在飞行... 距离目标点：%.2f 米", dist);

            if (dist < arrive_radius)
            {
                ROS_INFO("[Ego_Controller] 成功到达目标点附近！刹车！");
                nav_state = ARRIVED;
            }
            else if (has_traj)
            {
                // --- 核心修复：ENU 转 NED 坐标系 ---
                
                // 1. 计算 ENU 坐标系下的位置误差
                double err_x_enu = current_traj_cmd.position.x - current_odom.pose.pose.position.x;
                double err_y_enu = current_traj_cmd.position.y - current_odom.pose.pose.position.y;

                // 2. ENU 坐标系下的速度指令（前馈 + PD）
                double cmd_vx_enu = current_traj_cmd.velocity.x + kp_pos * err_x_enu;
                double cmd_vy_enu = current_traj_cmd.velocity.y + kp_pos * err_y_enu;

                // 3. 速度限幅（ENU）
                cmd_vx_enu = std::max(-max_v, std::min(max_v, cmd_vx_enu));
                cmd_vy_enu = std::max(-max_v, std::min(max_v, cmd_vy_enu));

                // 4. ENU 转 NED 坐标系转换
                double vx_ned, vy_ned, vz_ned;
                enuVelToNed(cmd_vx_enu, cmd_vy_enu, 0.0, vx_ned, vy_ned, vz_ned);
                
                double z_ned;
                enuToNed(0, 0, current_traj_cmd.position.z, setpoint.position.x, setpoint.position.y, z_ned);
                setpoint.position.z = z_ned;

                // 5. Yaw 转换（ENU -> NED）
                setpoint.yaw = enuYawToNed(current_traj_cmd.yaw);

                // 6. 设置掩码：使用速度控制 VX, VY，位置控制 Z，yaw 控制
                setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                     mavros_msgs::PositionTarget::IGNORE_PY |
                                     mavros_msgs::PositionTarget::IGNORE_VZ |
                                     mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

                setpoint.velocity.x = vx_ned;
                setpoint.velocity.y = vy_ned;

                ROS_DEBUG_STREAM_THROTTLE(1.0,
                    "[Ego_Controller] 发布控制量 (NED): " <<
                    " vx=" << setpoint.velocity.x << 
                    " vy=" << setpoint.velocity.y <<
                    " z=" << setpoint.position.z <<
                    " yaw=" << setpoint.yaw <<
                    " | ENU_vel=[" << cmd_vx_enu << ", " << cmd_vy_enu << "]");

                mavros_cmd_pub.publish(setpoint);
            }
            else
            {
                ROS_WARN_THROTTLE(0.5, "[Ego_Controller] 等待 Ego-Planner 吐出轨迹，安全悬停中...");
                
                // 悬停时也进行坐标转换
                double z_ned;
                enuToNed(current_odom.pose.pose.position.x, 
                         current_odom.pose.pose.position.y, 
                         current_goal.pose.position.z, 
                         setpoint.position.x, setpoint.position.y, z_ned);
                setpoint.position.z = z_ned;
                setpoint.yaw = enuYawToNed(current_yaw_enu);
                
                setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                     mavros_msgs::PositionTarget::IGNORE_VY |
                                     mavros_msgs::PositionTarget::IGNORE_VZ |
                                     mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

                mavros_cmd_pub.publish(setpoint);
            }
        }
        else
        {
            if (loop_count % 60 == 0) {
                ROS_INFO_THROTTLE(2.0, "[Ego_Controller] 等待目标点指令... (nav_state=%d)", nav_state);
            }
        }

        std_msgs::Int8 status_msg;
        status_msg.data = nav_state;
        nav_status_pub.publish(status_msg);

        rate.sleep();
    }
    return 0;
}
