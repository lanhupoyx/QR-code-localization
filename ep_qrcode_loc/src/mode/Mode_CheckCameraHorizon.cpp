#include "Mode.hpp"

// 构造函数
Mode_CheckCameraHorizon::Mode_CheckCameraHorizon(ParamServer &param, MV_SC2005AM *camera) : QRcodeLoc(param, camera)
{
    logger->info("Mode_CheckCameraHorizon() start");
    // 实例化功能对象
    qrcode_table = new QRcodeTableV2(param.cfg_dir, trans_camera2base, param);
    wheel_odom = new WheelSpeedOdometer(trans_camera2base, param);

    logger->info("Mode_CheckCameraHorizon() return");
}

Mode_CheckCameraHorizon::~Mode_CheckCameraHorizon() {}

// 检查相机水平模式
void Mode_CheckCameraHorizon::loop()
{
    logger->info("Mode_CheckCameraHorizon::loop()");

    QRcodeInfo code_info;     // 查询二维码坐标
    ros::Rate loop_rate(200); // 主循环 200Hz
    while (ros::ok())
    {
        if (camera->getframe(&pic))
        {
            nav_msgs::Odometry odom;
            std::vector<nav_msgs::Odometry> v_odom;

            odom.pose.covariance[7] = pic.error_x;   // 地码与相机横向偏差
            odom.pose.covariance[8] = pic.error_y;   // 地码与相机纵向偏差
            odom.pose.covariance[9] = pic.error_yaw; // 地码与相机角度偏差

            // map ---> base_link
            odom.header.frame_id = "map";
            odom.child_frame_id = "base_link";
            v_odom.push_back(odom);
            v_odom.push_back(odom);

            // 发布消息
            pubOdom(v_odom);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
}