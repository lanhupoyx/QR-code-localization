#include "qrcodeTable.hpp"

QRcodeTable::QRcodeTable(ParamServer &param) : param(param)
{
    // 记录器
    logger = &epLogger::getInstance();
    logger->info(std::string(__FUNCTION__) + "() start");

    // sub_pos = param.nh.subscribe<nav_msgs::Odometry>("/ep_localization/odometry/lidar", 1,
    //                                                  &QRcodeTable::lidarPoseCallback, this,
    //                                                  ros::TransportHints().tcpNoDelay());
    // logger->info("sub: /ep_localization/odometry/lidar");

    logger->info(std::string(__FUNCTION__) + "() return");
}

QRcodeTable::~QRcodeTable() {}

// bool QRcodeTable::find_add(CameraFrame frame, QRcodeInfo *info){}
// bool QRcodeTable::add(const CameraFrame frame){}
// void QRcodeTable::tfCallback(const nav_msgs::Odometry::ConstPtr &msg){}
// void QRcodeTable::subPose(){}
// QRcodeInfo calPose(){}
// bool QRcodeTable::correct_yaw(uint32_t code, double yaw_err){}
// void QRcodeTable::readYawErr(std::string cfg_path){}
// nav_msgs::Odometry QRcodeTable::getCurLidarPose(){}
// std::vector<uint32_t> QRcodeTable::get_all_head(){}
// bool QRcodeTable::is_in_queue(nav_msgs::Odometry base2map, double head_offset){}
// bool QRcodeTable::onlyfind(CameraFrame frame, QRcodeInfo *info)
// {
//     logger->debug("QRcodeTable::onlyfind() start end");
// }
// bool QRcodeTable::onlyfind(CameraFrame frame)
// {
//     logger->debug("QRcodeTable::onlyfind() start end");
// }
// bool QRcodeTable::isAGVInQueue(double head_offset){}
