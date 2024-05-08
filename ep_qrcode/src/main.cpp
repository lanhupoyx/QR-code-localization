// ################################################
// #####        main function for ep_qrcode
// #####        yuanxun@ep-ep.com
// #####        updateTime:  2024.05.01
// ################################################
#include "ep_qrcode_utility.hpp"
#include "MV_SC2005AM.hpp"

// 主函数
int main(int argc, char *argv[])
{
    // 初始化
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "ep_qrcode");
    ros::NodeHandle nh;

    // 读取参数
    std::string odomTopic;
    std::string msgTopic;

    bool show_msg;
    bool collect_QRcode;
    std::string port;
    std::string log_dir;
    std::string cfg_dir;
    nh.param<std::string>("ep_qrcode/odomTopic", odomTopic, "qrcode/odom");
    nh.param<std::string>("ep_qrcode/msgTopic", msgTopic, "qrcode/msg");
    nh.param<bool>("ep_qrcode/show_msg", show_msg, false);
    nh.param<bool>("ep_qrcode/collect_QRcode", collect_QRcode, false);
    nh.param<std::string>("ep_qrcode/port", port, "1024");
    nh.param<std::string>("ep_qrcode/log_dir", log_dir, "/var/xmover/log/qrcode");
    nh.param<std::string>("ep_qrcode/cfg_dir", cfg_dir, "/var/xmover/params");

    // 发布器
    ros::Publisher pub_qrCodeMsg = nh.advertise<std_msgs::String>(msgTopic, 1000);
    ros::Publisher pub_qrcode_odom = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);

    // 日志文件初始化
    std::string log_name = log_dir + "/" + format_time(ros::Time::now()) + ".txt";
    std::ofstream log_file(log_name);
    if (!log_file.is_open())
    {
        std::cout << "can't open: " << log_name << std::endl;
    }
    else
    {
        std::cout << "open: " << log_name << std::endl;
    }

    // baselink---->camera的变换关系
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfListener(buffer);
    geometry_msgs::TransformStamped transform;
    bool tferr = true;
    while (tferr)
    {
        tferr = false;
        try
        {
            transform = buffer.lookupTransform("sc2005am_link", "base_link", ros::Time(0));
        }
        catch (tf::TransformException &exception)
        {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep();
            continue;
        }
    }

    coordinate_table table(cfg_dir + "/qrcode_coordinate_table.txt", &log_file, transform);
    MV_SC2005AM camera(port, &log_file);

    ros::Subscriber sub_pos = nh.subscribe<nav_msgs::Odometry>("/ep_localization/odometry/lidar", 1000, &coordinate_table::tfCallback);

    // 主循环 100Hz
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        // 查询相机数据
        frame pic;
        static nav_msgs::Odometry odom;
        if (camera.getframe(&pic))
        {
            // 发布 /qrCodeMsg
            if (show_msg)
            {
                std::stringstream pub_ss;
                pub_ss << format_time(pic.stamp) << " [" << pic.sender << "] " << pic.index << " " << pic.duration << "s " // ip
                       << " " << pic.error_x << "mm " << pic.error_y << "mm " << pic.error_yaw << std::endl;
                std_msgs::String msg;
                msg.data = pub_ss.str();
                pub_qrCodeMsg.publish(msg);
            }

            // 查询二维码坐标
            qrcode_info code_info;
            if (table.find(pic, &code_info))
            {
                // 计算相机坐标
                double camera_x = code_info.x + pic.error_x;
                double camera_y = code_info.y + pic.error_y;
                double camera_yaw = code_info.yaw + pic.error_yaw;
                tf::Quaternion q;
                q.setRPY(0, 0, camera_yaw);

                // 发布相机坐标
                odom.header.stamp = pic.stamp;
                odom.header.frame_id = "map";
                odom.header.seq = pic.index;
                odom.pose.pose.position.x = camera_x;
                odom.pose.pose.position.y = camera_y;
                odom.pose.pose.position.z = 0.1;
                odom.pose.pose.orientation.x = q.getX();
                odom.pose.pose.orientation.y = q.getY();
                odom.pose.pose.orientation.z = q.getZ();
                odom.pose.pose.orientation.w = q.getW();
                odom.pose.covariance[0] = 1;            // 此帧是否可用，1：可用，0：不可用
                odom.pose.covariance[1] = pic.duration; // 相机处理图像用时(s)
                pub_qrcode_odom.publish(odom);
            }
            else
            {
                odom.pose.covariance[0] = 0; // 此帧是否可用，1：可用，0：不可用
                pub_qrcode_odom.publish(odom);
            }
        }
        else
        {
            odom.pose.covariance[0] = 0; // 此帧是否可用，1：可用，0：不可用
            pub_qrcode_odom.publish(odom);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
