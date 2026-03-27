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
ros::Publisher ego_goal_pub_fallback; // 备用话题
ros::Publisher trigger_pub;           // 【新增】唤醒器！
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

double arrive_radius = 0.2;
bool has_odom = false;
bool has_traj = false;

double hover_x = 0.0;
double hover_y = 0.0;
double hover_z = 0.0;
double hover_yaw = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_odom = *msg;
    has_odom = true;
}

void fsmGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_goal = *msg;
    ROS_INFO("\n===========================================");
    ROS_INFO("[Ego执行器] 收到长官命令！前往新目标: X=%.2f, Y=%.2f, Z=%.2f",
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("===========================================\n");

    hover_x = current_odom.pose.pose.position.x;
    hover_y = current_odom.pose.pose.position.y;
    hover_z = current_goal.pose.position.z;
    hover_yaw = current_odom.pose.pose.orientation.z;

    // 1. 发送目标点 (双管齐下，防止 Ego 没听到)
    ego_goal_pub.publish(current_goal);
    ego_goal_pub_fallback.publish(current_goal);

    // 2. 【绝杀修复】发送 Trigger 唤醒 Ego-Planner！！
    geometry_msgs::PoseStamped trigger_msg;
    trigger_msg.header.stamp = ros::Time::now();
    trigger_msg.header.frame_id = "world";
    trigger_pub.publish(trigger_msg);
    ROS_INFO("[Ego执行器] 已发送 /traj_start_trigger 强行唤醒小脑！");

    nav_state = FLYING;
    has_traj = false;
}

void trajCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    current_traj_cmd = *msg;
    has_traj = true;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "ego_controller_node");
    ros::NodeHandle nh("~");

    ros::Subscriber odom_sub = nh.subscribe("/mavros/local_position/odom", 10, odomCallback);
    ros::Subscriber fsm_goal_sub = nh.subscribe("/fsm/ego_goal", 1, fsmGoalCallback);
    // 监听小脑的轨迹
    ros::Subscriber traj_sub = nh.subscribe("/position_cmd", 10, trajCmdCallback);

    mavros_cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    nav_status_pub = nh.advertise<std_msgs::Int8>("/ego_controller/status", 10);

    // 发给 Ego 的话题
    ego_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ego_goal_pub_fallback = nh.advertise<geometry_msgs::PoseStamped>("/goal", 1);
    trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 1);

    ros::Rate rate(30);

    while (ros::ok())
    {
        ros::spinOnce();
        if (!has_odom)
        {
            rate.sleep();
            continue;
        }

        mavros_msgs::PositionTarget setpoint;
        setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 1
        setpoint.type_mask = 3040;

        if (nav_state == FLYING)
        {
            double dist = sqrt(pow(current_goal.pose.position.x - current_odom.pose.pose.position.x, 2) +
                               pow(current_goal.pose.position.y - current_odom.pose.pose.position.y, 2));

            if (dist < arrive_radius)
            {
                ROS_INFO("[Ego执行器] 成功到达目标点附近！交还控制权！");
                nav_state = ARRIVED;
            }
            else if (has_traj)
            {
                setpoint.position.x = current_traj_cmd.position.x;
                setpoint.position.y = current_traj_cmd.position.y;
                setpoint.position.z = current_traj_cmd.position.z;

                setpoint.velocity.x = current_traj_cmd.velocity.x;
                setpoint.velocity.y = current_traj_cmd.velocity.y;
                setpoint.velocity.z = current_traj_cmd.velocity.z;

                setpoint.yaw = current_traj_cmd.yaw;

                ROS_INFO_THROTTLE(1.0, "[Ego执行器] 距目标:%.2fm | 发送点(%.2f,%.2f) | 引导速度(%.2f,%.2f)",
                                  dist, setpoint.position.x, setpoint.position.y, setpoint.velocity.x, setpoint.velocity.y);

                mavros_cmd_pub.publish(setpoint);
            }
            else
            {
                ROS_WARN_THROTTLE(0.5, "[Ego执行器] 等待小脑轨迹，安全定点悬停中...");
                setpoint.position.x = hover_x;
                setpoint.position.y = hover_y;
                setpoint.position.z = hover_z;
                setpoint.velocity.x = 0;
                setpoint.velocity.y = 0;
                setpoint.velocity.z = 0;
                setpoint.yaw = hover_yaw;
                mavros_cmd_pub.publish(setpoint);
            }
        }
        std_msgs::Int8 status_msg;
        status_msg.data = nav_state;
        nav_status_pub.publish(status_msg);

        rate.sleep();
    }
    return 0;
}