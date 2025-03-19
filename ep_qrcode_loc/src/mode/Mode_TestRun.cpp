#include "Mode.hpp"

// 构造函数
Mode_TestRun::Mode_TestRun(ParamServer &param, MV_SC2005AM *camera) : QRcodeLoc(param, camera)
{
    // 实例化功能对象
    qrcode_table = new QRcodeTableV2(param.cfg_dir, trans_camera2base, param);
    wheel_odom = new WheelSpeedOdometer(trans_camera2base, param);
}

Mode_TestRun::~Mode_TestRun() {}

// 采集二维码编号模式
void Mode_TestRun::loop()
{
    logger->info("Mode_TestRun::loop()");

    QRcodeInfo code_info;     // 查询二维码坐标
    ros::Rate loop_rate(200); // 主循环 200Hz
    while (ros::ok())
    {
        if (camera->getframe(&pic))
        {
            code_info.frame = pic;
            std::vector<geometry_msgs::Pose> pose;

            // 打包生成消息
            std::vector<nav_msgs::Odometry> v_odom = packageMsg(pose, code_info);

            // 发布消息
            pubOdom(v_odom);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
}
