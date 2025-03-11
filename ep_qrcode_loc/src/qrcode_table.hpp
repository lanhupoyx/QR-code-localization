#pragma once
#include "utility_qloc.hpp"

// 二维码坐标对照表
class QRcodeTable
{
public:
    QRcodeTable(std::string dir, ParamServer& param);

    ~QRcodeTable();

    // 根据二维码编号查表，得到位姿信息
    bool find_add(CameraFrame frame, QRcodeInfo *info);

    // 根据二维码编号查表，得到位姿信息
    bool onlyfind(CameraFrame frame, QRcodeInfo *info);

    // 添加新的二维码
    bool add(const CameraFrame frame);

    // callback获取baselink位姿
    void tfCallback(const nav_msgs::Odometry::ConstPtr &msg);

private:
    // 计算二维码位姿
    QRcodeInfo calPose();

private:
    std::ifstream ifs;
    std::map<uint32_t, QRcodeInfo> map;
    std::map<uint32_t, QRcodeInfo> new_map;
    std::ofstream log_file;
    std::list<CameraFrame> frame_buffer;

    geometry_msgs::TransformStamped trans_camera2base;
    std::string path;

    ros::Subscriber sub_pos;

    std::mutex mtx;
    std::list<nav_msgs::Odometry> tf_buffer;

    Logger *logger;
    std::stringstream stream;
    ParamServer& param;
};
