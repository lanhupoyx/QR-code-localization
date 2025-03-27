#include "Mode.hpp"

// 构造函数
Mode_Calibration::Mode_Calibration(ParamServer &param, MV_SC2005AM *camera) : QRcodeLoc(param, camera)
{
    logger->info(std::string(__FUNCTION__) + "() start");

    qrcode_table = new QRcodeTableV2(param.cfg_dir, trans_camera2base, param);

    logger->info(std::string(__FUNCTION__) + "() return");
}

Mode_Calibration::~Mode_Calibration() {}

void Mode_Calibration::loop()
{
    logger->info(std::string(__FUNCTION__) + "() start");

    ros::Rate loop_rate(200); // 主循环 200Hz
    while (ros::ok())
    {
        if (camera->getframe(&pic))
        {
            nav_msgs::Odometry odom = qrcode_table->getCurLidarPose();

            odom.pose.covariance[7] = pic.error_x;   // 地码与相机横向偏差
            odom.pose.covariance[8] = pic.error_y;   // 地码与相机纵向偏差
            odom.pose.covariance[9] = pic.error_yaw; // 地码与相机角度偏差

            std::vector<nav_msgs::Odometry> v_odom;
            v_odom.push_back(odom);
            v_odom.push_back(odom);

            // 发布消息
            pubOdom(v_odom);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    logger->info(std::string(__FUNCTION__) + "() return");
}