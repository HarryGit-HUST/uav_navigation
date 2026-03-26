#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher cloud_pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI> input_cloud;
    pcl::fromROSMsg(*msg, input_cloud); // 接收你以前代码处理好的 2D 纯净点云

    pcl::PointCloud<pcl::PointXYZI> output_cloud;

    // 核心：Z 轴拉伸法术！假设定高 0.6m，我们生成 0.2 到 1.0 米的障碍物柱子
    for (const auto &pt : input_cloud.points)
    {
        for (float z = 0.2; z <= 1.0; z += 0.1)
        {
            pcl::PointXYZI new_pt;
            new_pt.x = pt.x;
            new_pt.y = pt.y;
            new_pt.z = z; // 赋予新的高度
            new_pt.intensity = pt.intensity;
            output_cloud.points.push_back(new_pt);
        }
    }

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);
    output_msg.header = msg->header; // 保持坐标系不变
    // 发送给 Ego-Planner 默认接收的话题
    cloud_pub.publish(output_msg);
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, ""); // 防乱码
    ros::init(argc, argv, "cloud_extruder");
    ros::NodeHandle nh;
    // 订阅以前的 2D 话题
    ros::Subscriber sub = nh.subscribe("/projected_accumulated_cloud", 1, cloudCallback);
    // 发布给 Ego-Planner 的话题
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 1);
    ros::spin();
    return 0;
}