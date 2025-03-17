#pragma once
#include "utility_qloc.hpp"
#include "LocCamera.h"

// 相机标定
class Calibration
{
public:
Calibration(ParamServer& param);

    ~Calibration();

    // 用于主循环获取相机消息
    bool getframe(CameraFrame *frame);

    // 相机循环
    void cameraLoop();

private:
    // callback获取baselink位姿
    void tfCallback(const nav_msgs::Odometry::ConstPtr &msg)

private:
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket *socket;
    boost::asio::ip::udp::endpoint sender_endpoint;
    std::array<char, 1024> recv_buf;
    boost::system::error_code error;
    std::ofstream log_file;
    Logger *logger;
    std::mutex mtx; // 互斥锁
    std::stringstream stream;
    bool state;
    ParamServer &param;
    ros::Publisher pub_frame;
    ros::Subscriber sub_frame;
    bool is_subNewFrame;
    CameraFrame subFrameData;
};
