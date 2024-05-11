// ################################################
// #####        main function for ep_qrcode
// #####        yuanxun@ep-ep.com
// #####        updateTime:  2024.05.01
// ################################################
#include "hik_utility.hpp"
#include "MV_SC2005AM.hpp"

// 主函数
int main(int argc, char *argv[])
{
    // 初始化
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "hik2005");
    ros::NodeHandle nh;

    // 读取参数
    std::string odomTopic;
    std::string msgTopic;

    bool show_msg;
    bool collect_QRcode;
    std::string port;
    std::string log_dir;
    std::string cfg_dir;
    nh.param<std::string>("hik2005/odomTopic", odomTopic, "hik2005/odom");
    nh.param<std::string>("hik2005/msgTopic", msgTopic, "hik2005/msg");
    nh.param<bool>("hik2005/show_msg", show_msg, false);
    nh.param<bool>("hik2005/collect_QRcode", collect_QRcode, false);
    nh.param<std::string>("hik2005/port", port, "1024");
    nh.param<std::string>("hik2005/log_dir", log_dir, "/var/xmover/log/hik2005");
    nh.param<std::string>("hik2005/cfg_dir", cfg_dir, "/var/xmover/params");

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
    geometry_msgs::TransformStamped trans_base2camera;
    geometry_msgs::TransformStamped trans_camera2base;
    bool tferr = true;
    while (tferr)
    {
        tferr = false;
        try
        {
            trans_base2camera = buffer.lookupTransform("sc2005am_link", "base_link", ros::Time(0));
            trans_camera2base = buffer.lookupTransform("base_link", "sc2005am_link", ros::Time(0));
        }
        catch (tf::TransformException &exception)
        {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep();
            continue;
        }
    }

    QRcodeTable table(cfg_dir + "/qrcode_table.txt", &log_file, trans_camera2base);
    MV_SC2005AM camera(port, &log_file);

    ros::Subscriber sub_pos = nh.subscribe<nav_msgs::Odometry>("/ep_localization/odometry/lidar", 1000, &QRcodeTable::tfCallback);

    // 主循环 100Hz
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        frame pic; // 相机数据
        static nav_msgs::Odometry odom;
        if (camera.getframe(&pic))
        {
            // 查询二维码坐标
            qrcode_info code_info;
            if (table.find(pic, &code_info))
            {
                // 二维码到map变换关系
                geometry_msgs::Pose pose_qrcode2map;
                pose_qrcode2map.position.x = code_info.x;
                pose_qrcode2map.position.y = code_info.y;
                pose_qrcode2map.position.z = 0.0;
                tf::Quaternion q;
                q.setRPY(0.0, 0.0, code_info.yaw*M_PI/180);
                tf::quaternionTFToMsg(q, pose_qrcode2map.orientation);
                double yaw_qrcode2map = getYaw(pose_qrcode2map);
                // 相机到二维码变换关系
                geometry_msgs::Pose pose_camera2qrcode;
                q.setRPY(0.0, 0.0, pic.error_yaw*M_PI/180);
                tf::quaternionTFToMsg(q, pose_camera2qrcode.orientation);
                double yaw_camera2qrcode = getYaw(pose_camera2qrcode);
                tf::Matrix3x3 m(q);
                tf::Matrix3x3 inv_m(m);
                tf::Vector3 v(pic.error_x/1000.0, pic.error_y/-1000.0, 0.0);
                inv_m.inverse();  // 求逆
                pose_camera2qrcode.position.x = -inv_m.getRow(0).dot(v);//(row0(0)*v.getX() + inv_m(0,1)*v.getX() +inv_m(0,2)*v.getZ());
                pose_camera2qrcode.position.y = -inv_m.getRow(1).dot(v);//-(inv_m(1,0)*v.getX() + inv_m(1,1)*v.getX() +inv_m(1,2)*v.getZ());
                pose_camera2qrcode.position.z = 0.0;
                // 相机到map位姿
                geometry_msgs::Pose pose_camera2map;
                tf2::doTransform(pose_camera2qrcode, pose_camera2map, p2t(pose_qrcode2map));
                double yaw_camera2map = getYaw(pose_camera2map);
                // base到map位姿
                geometry_msgs::Pose pose_base2map;
                tf2::doTransform(t2p(trans_base2camera), pose_base2map, p2t(pose_camera2map));
                double yaw_base2map = getYaw(pose_base2map);

                //发布base_link坐标
                odom.header.stamp = pic.stamp;
                odom.header.frame_id = "map";
                odom.child_frame_id = "base_link";
                odom.header.seq = pic.index;
                odom.pose.pose = pose_base2map;
                odom.pose.covariance[0] = 1;            // 此帧是否可用，1：可用，0：不可用
                odom.pose.covariance[1] = pic.duration; // 相机处理图像用时(s)
                pub_qrcode_odom.publish(odom);

                // 发布 /qrCodeMsg
                if (show_msg)
                {
                    std::stringstream pub_ss;
                    pub_ss << format_time(pic.stamp) << " [" << pic.sender << "] " << pic.index << " " << pic.duration << "s " // ip
                        << " " << pic.error_x << "mm " << pic.error_y << "mm " << pic.error_yaw << " yaw_base2map:" << yaw_base2map;
                    std_msgs::String msg;
                    msg.data = pub_ss.str();
                    pub_qrCodeMsg.publish(msg);
                }
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
