#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher cloud_pub;

// 【新增】定义一个全局的强度过滤阈值
double intensity_threshold = 10.0; 

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // 【核心修改 1】将 PointXYZ 改为 PointXYZI，提取并保留强度(Intensity)信息
    pcl::PointCloud<pcl::PointXYZI> input_cloud;
    pcl::fromROSMsg(*msg, input_cloud);

    // 【修复闪烁】防空包过滤！如果 PCL 这帧没处理出来障碍物，宁愿不发，也不能发空包清空 Ego 的视野
    if (input_cloud.empty())
        return;

    pcl::PointCloud<pcl::PointXYZI> output_cloud;

    // 假设定高 0.6m，拉伸范围 0.0 到 2.0 米
    for (const auto &pt : input_cloud.points)
    {
        // 过滤无穷远无效点
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y))
            continue;

        // ==============================================================
        // 【新增】强度过滤！如果该点的反射强度低于阈值，视为噪点，直接抛弃！
        // ==============================================================
        if (pt.intensity < intensity_threshold)
            continue;

        // 【修复闪烁】间距从 0.1 放大到 0.2，极大减轻点云运算负担，防止延迟卡顿
        for (float z = 0.0; z <= 2.0; z += 0.2)
        {
            pcl::PointXYZI new_pt; // 【核心修改 2】新建的点也必须是 PointXYZI
            new_pt.x = pt.x;
            new_pt.y = pt.y;
            new_pt.z = z;
            new_pt.intensity = pt.intensity; // 【核心修改 3】继承原生的高强度值
            
            output_cloud.points.push_back(new_pt);
        }
    }

    // 如果过滤完噪点后点云空了，就不发布，维持 Ego 脑子里的旧地图
    if (output_cloud.empty())
        return;

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);

    // 【修复闪烁】强行绑定 world 坐标系，绕过 TF 变换带来的跳变延迟
    output_msg.header.frame_id = "world";
    output_msg.header.stamp = ros::Time::now();

    cloud_pub.publish(output_msg);
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "cloud_extruder");
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~"); // 用于读取私有参数

    // 【新增】从 launch 文件中读取 intensity_threshold 参数，如果没有设置，默认值为 10.0
    nh_private.param<double>("intensity_threshold", intensity_threshold, 10.0);
    ROS_INFO("[点云拉伸与过滤节点] 启动成功！当前过滤低强度噪点的阈值为: %.2f", intensity_threshold);

    ros::Subscriber sub = nh.subscribe("/projected_accumulated_cloud", 1, cloudCallback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_pcl", 1);

    ros::spin();
    return 0;
}