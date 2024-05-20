// ################################################
// #####        main function for ep_qrcode
// #####        yuanxun@ep-ep.com
// #####        updateTime:  2024.05.01
// ################################################
#include "hik_utility.hpp"

std::ofstream log_file;

void doMsg(const nav_msgs::Odometry::ConstPtr& msg)
{
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfListener(buffer);
    geometry_msgs::TransformStamped trans_base2map;
    try
    {
        trans_base2map = buffer.lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch (tf::TransformException &exception)
    {
        ROS_WARN("%s; retrying...", exception.what());
        return;
    }

    double xerror = msg->pose.pose.position.x - trans_base2map.transform.translation.x;
    double yerror = msg->pose.pose.position.y - trans_base2map.transform.translation.y;
    double zerror = msg->pose.pose.position.z - trans_base2map.transform.translation.z;

    log_file<< format_time(msg->header.stamp) 
            << " " << sqrt(pow(xerror, 2) + pow(yerror, 2) + pow(zerror, 2))
            << " | " << xerror
            << " " << yerror
            << " " << zerror
            << " " << getYaw(msg->pose.pose) - getYaw(trans_base2map.transform.rotation)
            << " | " << msg->pose.pose.position.x
            << " " << msg->pose.pose.position.y
            << " " << msg->pose.pose.position.z
            << " " << getYaw(msg->pose.pose)
            << " | " << trans_base2map.transform.translation.x
            << " " << trans_base2map.transform.translation.y
            << " " << trans_base2map.transform.translation.z
            << " " << getYaw(trans_base2map.transform.rotation)
            << std::endl;
}

// 主函数
int main(int argc, char *argv[])
{
    // 初始化
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "loc_error");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("hik2005/odom_map",100,doMsg);

    // 日志文件初始化
    std::string log_name = "/var/xmover/log/hik2005/LocError" + format_time(ros::Time::now()) + ".txt";
    log_file.open(log_name);
    if (!log_file.is_open())
    {
        std::cout << "can't open: " << log_name << std::endl;
    }
    else
    {
        std::cout << "open: " << log_name << std::endl;
    }
    ros::spin();
}