#pragma once
#include "utility_qloc.hpp"
#include "ParamServer.hpp"
#include "logger.hpp"
#include "LocCamera.h"

// 相机数据预处理
class MV_SC2005AM
{
public:
    MV_SC2005AM(ParamServer& param);

    ~MV_SC2005AM();

    // 用于主循环获取相机消息
    bool getframe(CameraFrame *frame);

    // 相机循环
    void cameraLoop();

private:
    // 海康定位相机，固件版本：2.5.0
    bool getframe_v1(CameraFrame *frame);

    // 海康定位相机，固件版本：2.7.0
    bool getframe_v2(CameraFrame *frame);

    // 发布数据
    void publishFrame(CameraFrame frame);

    // 获取/ep_qrcode_loc/cemera/frame的回调函数
    void LocCameraCallback(const ep_qrcode_loc::LocCamera::ConstPtr &p_frame_msg);

private:
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket *socket;
    boost::asio::ip::udp::endpoint sender_endpoint;
    std::array<char, 1024> recv_buf;
    boost::system::error_code error;
    std::ofstream log_file;
    epLogger *logger;
    std::mutex mtx; // 互斥锁
    std::stringstream stream;
    bool state;
    ParamServer &param;
    ros::Publisher pub_frame;
    ros::Subscriber sub_frame;
    bool is_subNewFrame;
    CameraFrame subFrameData;
};
