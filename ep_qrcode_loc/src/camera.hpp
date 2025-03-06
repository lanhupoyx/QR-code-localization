#pragma once

#include "utility_qloc.hpp"
#include "logger.hpp"

// 相机数据预处理
class MV_SC2005AM
{
public:
    MV_SC2005AM(ParamServer& param);

    ~MV_SC2005AM();

    bool getframe(CameraFrame *frame);

private:
    // 海康定位相机，固件版本：2.5.0
    bool getframe_v1(CameraFrame *frame);

    // 海康定位相机，固件版本：2.7.0
    bool getframe_v2(CameraFrame *frame);

private:
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket *socket;
    boost::asio::ip::udp::endpoint sender_endpoint;
    std::array<char, 1024> recv_buf;
    boost::system::error_code error;
    std::ofstream log_file;
    Logger *logger;
    std::stringstream stream;
    bool state;
    ParamServer& param;
};
