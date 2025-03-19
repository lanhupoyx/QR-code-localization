#include "Mode.hpp"

// 构造函数
Mode_CollectQRCodePose::Mode_CollectQRCodePose(ParamServer &param, MV_SC2005AM *camera) : QRcodeLoc(param, camera)
{
    // 实例化功能对象
    qrcode_table = new QRcodeTableV2(param.cfg_dir, trans_camera2base, param);
    wheel_odom = new WheelSpeedOdometer(trans_camera2base, param);
}

Mode_CollectQRCodePose::~Mode_CollectQRCodePose() {}

// 采集二维码位姿模式
void Mode_CollectQRCodePose::loop()
{
    logger->info("Mode_CollectQRCodePose::loop()");
    QRcodeTable Lidarmap_tab(param.cfg_dir + "CaptureTable.txt", param);
    Lidarmap_tab.subPose();
    pub_qrCodeMsg = param.nh.advertise<std_msgs::String>(param.msgTopic, 1);

    ros::Rate loop_rate(100); // 主循环 100Hz
    while (ros::ok())
    {
        if (camera->getframe(&pic))
        {
            // 查询二维码坐标
            QRcodeInfo code_info;
            if (Lidarmap_tab.find_add(pic, &code_info))
            {
                // 获取base_link位姿
                geometry_msgs::Pose pose_base2map = get_pose_lidarmap(pic, code_info);

                // 发布 /qrCodeMsg
                std::stringstream pub_ss;
                pub_ss << format_time(pic.stamp) << " [" << pic.sender << "] " << pic.index << " " << pic.duration << "s " // ip
                       << " " << pic.error_x << "mm " << pic.error_y << "mm " << pic.error_yaw << " yaw_base2map:" << getYaw(pose_base2map);
                std_msgs::String msg;
                msg.data = pub_ss.str();
                pub_qrCodeMsg.publish(msg);
            }
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
}
