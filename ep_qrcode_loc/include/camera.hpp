#pragma once

#include "utility_qloc.hpp"

// 相机数据预处理
class MV_SC2005AM : public ParamServer
{
public:
    MV_SC2005AM()
    {
        logger = &Logger::getInstance();
        stream.str("");
        stream << "MV_SC2005AM Start"
               << std::endl;
        logger->log(stream.str());

        // UDP端口监测初始化
        socket = new boost::asio::ip::udp::socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), std::atoi(port.c_str())));
    }
    ~MV_SC2005AM() {}

    bool getframe(CameraFrame *frame)
    {
        if (socket->available())
        {
            socket->receive_from(boost::asio::buffer(recv_buf), sender_endpoint, 0, error);
            if (!error)
            {
                ros::Time now = ros::Time::now();
                std::string sender = sender_endpoint.address().to_string();
                frame->hex = charArrayToHex(recv_buf, 15);
                // std::cout << "hex is: " << frame->hex << std::endl;
                frame->sender = sender;

                // todo:判断帧头和字节校验和
                frame->head = frame->hex.substr(2, 2) + frame->hex.substr(0, 2);
                frame->sum = frame->hex.substr(28, 2); // 转换为整数

                // 数据提取与转换
                frame->index = convert_16_to_10(frame->hex.substr(4, 2));

                u_int32_t d = convert_16_to_10(frame->hex.substr(6, 2));
                frame->duration = d / 1000.0;
                frame->stamp = now; // - ros::Duration(frame->duration);

                frame->code = convert_16_to_10(frame->hex.substr(14, 2) + frame->hex.substr(12, 2) + frame->hex.substr(10, 2) + frame->hex.substr(8, 2));

                int16_t pixel_x = std::stoi(frame->hex.substr(18, 2) + frame->hex.substr(16, 2), 0, 16);
                int16_t pixel_y = std::stoi(frame->hex.substr(22, 2) + frame->hex.substr(20, 2), 0, 16);
                u_int32_t pixel_yaw = convert_16_to_10(frame->hex.substr(26, 2) + frame->hex.substr(24, 2));
                frame->error_x = pixel_x * 0.2125;
                frame->error_y = pixel_y * 0.2166667;
                frame->error_yaw = pixel_yaw / 100.0;

                if (show_msg)
                {
                    stream.str("");
                    stream << format_time(frame->stamp) << " [" << sender.c_str() << "] " << frame->code << " " << frame->index << " " << frame->duration << "s " // ip
                           << " " << frame->error_x << "mm " << frame->error_y << "mm " << frame->error_yaw;
                    logger->log(stream.str());
                }

                // std::cout << format_time(frame->stamp) << " [" << sender.c_str() << "] " << frame->code << " " << frame->index << " " << frame->duration << "s " // ip
                //           << " " << frame->error_x << "mm " << frame->error_y << "mm " << frame->error_yaw << std::endl;

                return true;
            }
            // log_file << "error receiving UDP data: " << error.message().c_str() << std::endl;
            stream.str("");
            stream << "error receiving UDP data: " << error.message().c_str();
            logger->log(stream.str());
        }
        return false;
    }

private:
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket *socket;
    boost::asio::ip::udp::endpoint sender_endpoint;
    std::array<char, 1024> recv_buf;
    boost::system::error_code error;
    std::ofstream log_file;
    Logger *logger;
    std::stringstream stream;
};
