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
ros::Publisher trigger_pub; // 唤醒器
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

// 固定悬停点，防止未收到轨迹时漂移
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
    ROS_INFO("[Ego执行器] 收到长官命令！前往新目标: X=%.2f, Y=%.2f", msg->pose.position.x, msg->pose.position.y);
    ROS_INFO("===========================================\n");

    hover_x = current_odom.pose.pose.position.x;
    hover_y = current_odom.pose.pose.position.y;
    hover_z = current_goal.pose.position.z;
    hover_yaw = current_odom.pose.pose.orientation.z;

    ego_goal_pub.publish(current_goal);

    geometry_msgs::PoseStamped trigger_msg;
    trigger_msg.header.stamp = ros::Time::now();
    trigger_msg.header.frame_id = "world";
    trigger_pub.publish(trigger_msg);

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
    ros::Subscriber traj_sub = nh.subscribe("/position_cmd", 10, trajCmdCallback);

    mavros_cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ego_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 1);
    nav_status_pub = nh.advertise<std_msgs::Int8>("/ego_controller/status", 10);

    ros::Rate rate(30);
    ros::Time last_retry_time = ros::Time::now();

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
        setpoint.type_mask = 3040;                                                // 掩码3040：同时监听 position 和 velocity！

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
                // 【拨乱反正】不再手动翻转！直接把 Ego 的输出塞给飞控，MAVROS 会自动处理！
                setpoint.position.x = current_traj_cmd.position.x;
                setpoint.position.y = current_traj_cmd.position.y;
                setpoint.position.z = current_traj_cmd.position.z;

                setpoint.velocity.x = current_traj_cmd.velocity.x;
                setpoint.velocity.y = current_traj_cmd.velocity.y;
                setpoint.velocity.z = 0;

                setpoint.yaw = current_traj_cmd.yaw;

                ROS_INFO_THROTTLE(1.0, "[Ego执行器] 距目标:%.2fm | 原生系发送速度(%.2f,%.2f)", dist, setpoint.velocity.x, setpoint.velocity.y);

                mavros_cmd_pub.publish(setpoint);
            }
            else
            {
                ROS_WARN_THROTTLE(1.0, "[Ego执行器] 等待小脑轨迹，安全定点悬停中...");

                if ((ros::Time::now() - last_retry_time).toSec() > 1.0)
                {
                    ego_goal_pub.publish(current_goal);
                    geometry_msgs::PoseStamped trigger_msg;
                    trigger_msg.header.stamp = ros::Time::now();
                    trigger_msg.header.frame_id = "world";
                    trigger_pub.publish(trigger_msg);
                    ROS_WARN("[Ego执行器] 小脑未响应！已重新发送目标与唤醒信号...");
                    last_retry_time = ros::Time::now();
                }

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