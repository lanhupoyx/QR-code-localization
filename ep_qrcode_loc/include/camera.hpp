#pragma once

#include "utility_qloc.hpp"

// 相机数据预处理
class MV_SC2005AM : public ParamServer
{
public:
    MV_SC2005AM()
    {
        logger = &Logger::getInstance();
        logger->info("MV_SC2005AM");

        // UDP端口监测初始化
        socket = new boost::asio::ip::udp::socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), std::atoi(port.c_str())));
    }
    ~MV_SC2005AM() {}

    bool getframe(CameraFrame *frame)
    {
        return getframe_v2(frame);
    }

private:
    // 海康定位相机，固件版本：2.5.0
    bool getframe_v1(CameraFrame *frame)
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

                float rate = 0.9;
                if (75 * rate < abs(frame->error_x))
                {
                    std::cout << frame->error_x << std::endl;
                    return false;
                }

                if (show_original_msg)
                {
                    stream.str("");
                    stream << format_time(frame->stamp) << " [" << sender.c_str() << "] " << frame->code << " " << frame->index << " " << frame->duration << "s " // ip
                           << " " << frame->error_x << "mm " << frame->error_y << "mm " << frame->error_yaw;
                    logger->info(stream.str());
                }

                // std::cout << format_time(frame->stamp) << " [" << sender.c_str() << "] " << frame->code << " " << frame->index << " " << frame->duration << "s " // ip
                //           << " " << frame->error_x << "mm " << frame->error_y << "mm " << frame->error_yaw << std::endl;

                return true;
            }
            // log_file << "error receiving UDP data: " << error.message().c_str() << std::endl;
            stream.str("");
            stream << "error receiving UDP data: " << error.message().c_str();
            logger->info(stream.str());
        }
        return false;
    }

    // 海康定位相机，固件版本：2.7.0
    bool getframe_v2(CameraFrame *frame)
    {
        if (socket->available())
        {
            socket->receive_from(boost::asio::buffer(recv_buf), sender_endpoint, 0, error);
            if (!error)
            {
                // 时间戳
                frame->stamp = ros::Time::now();

                // 相机IP
                std::string sender = sender_endpoint.address().to_string();
                frame->sender = sender;

                // 读取数据帧内容（16进制）
                frame->hex = charArrayToHex(recv_buf, 89);
                // std::cout << "hex is: " << frame->hex << std::endl;

                // 帧头
                frame->head = frame->hex.substr(0, 8);

                // 序号
                frame->index = convert_16_to_10(frame->hex.substr(8, 8));

                // 算法耗时
                u_int32_t d = convert_16_to_10(frame->hex.substr(48, 8));
                frame->duration = d / 1000.0;

                // 是否译码成功
                if ("01" == frame->hex.substr(56, 2))
                {
                    frame->is_decode = true;
                }
                else
                {
                    frame->is_decode = false;
                }

                // 是否定位成功
                if ("01" == frame->hex.substr(58, 2))
                {
                    frame->is_accurate_loc = true;
                }
                else
                {
                    frame->is_accurate_loc = false;

                    // 未精确定位成功，返回
                    return false;
                }

                // 是否粗定位成功
                if ("01" == frame->hex.substr(60, 2))
                {
                    frame->is_broad_loc = true;
                }
                else
                {
                    frame->is_broad_loc = false;
                }

                // 关键信息转换成ASCII码
                std::string keyinfo = hexToAscii(frame->hex.substr(72, 106));
                std::string keyinfo_backup = keyinfo;
                uint8_t band_num = 0;

                // 二维码内容
                if ('<' != keyinfo[0]) // 起始字符
                {
                    std::cout << "data error" << std::endl;
                    return false;
                }
                keyinfo.erase(0, 1); // <
                std::string code_ascii;
                band_num = 0;
                while ('@' != keyinfo[0]) // 结束字符
                {
                    code_ascii += keyinfo[0];
                    keyinfo.erase(0, 1);

                    band_num++;
                    if (band_num > 50)
                    {
                        std::cout << "data decode error -> @" << std::endl;
                        return false;
                    }
                }
                frame->code = std::stoi(code_ascii);

                // 中心坐标x
                band_num = 0;
                while ('(' != keyinfo[0]) // 起始字符
                {
                    keyinfo.erase(0, 1);

                    band_num++;
                    if (band_num > 50)
                    {
                        std::cout << "data decode error -> (" << std::endl;
                        return false;
                    }
                }
                keyinfo.erase(0, 1);
                std::string pixel_x_ascii;
                band_num = 0;
                while ('.' != keyinfo[0]) // 结束字符
                {
                    pixel_x_ascii += keyinfo[0];
                    keyinfo.erase(0, 1);

                    band_num++;
                    if (band_num > 50)
                    {
                        std::cout << "data decode error -> ." << std::endl;
                        return false;
                    }
                }
                //std::cout << pixel_x_ascii << std::endl;
                frame->error_x = (std::stoi(pixel_x_ascii) - 400) * 0.2125;

                // 中心视野x+-7cm范围，防止角度跳变
                double disdis_x = frame->error_x * frame->error_x;
                if(disdis_x > 4900)
                {
                    return false;
                }

                // 中心坐标y
                band_num = 0;
                while ('x' != keyinfo[0]) // 起始字符
                {
                    keyinfo.erase(0, 1);

                    band_num++;
                    if (band_num > 50)
                    {
                        std::cout << "data decode error -> x" << std::endl;
                        return false;
                    }
                }
                keyinfo.erase(0, 1);
                std::string pixel_y_ascii;
                band_num = 0;
                while ('.' != keyinfo[0]) // 结束字符
                {
                    pixel_y_ascii += keyinfo[0];
                    keyinfo.erase(0, 1);

                    band_num++;
                    if (band_num > 50)
                    {
                        std::cout << "data decode error -> .(2)" << std::endl;
                        return false;
                    }
                }
                //std::cout << pixel_y_ascii << std::endl;
                frame->error_y = (std::stoi(pixel_y_ascii) - 300) * 0.2166667;

                // 中心视野y+-5cm范围，防止角度跳变
                double disdis_y = frame->error_y * frame->error_y;
                if(disdis_y > 2500)
                {
                    return false;
                }

                // 中心yaw
                band_num = 0;
                while (')' != keyinfo[0]) // 起始字符
                {
                    keyinfo.erase(0, 1);

                    band_num++;
                    if (band_num > 50)
                    {
                        std::cout << "data decode error -> )" << std::endl;
                        return false;
                    }
                }
                keyinfo.erase(0, 1);
                std::string yaw_ascii;
                band_num = 0;
                while ('@' != keyinfo[0]) // 结束字符
                {
                    yaw_ascii += keyinfo[0];
                    keyinfo.erase(0, 1);

                    band_num++;
                    if (band_num > 50)
                    {
                        std::cout << "data decode error -> @(2)" << std::endl;
                        return false;
                    }
                }
                frame->error_yaw = std::stod(yaw_ascii);

                // 记录
                if (show_original_msg)
                {
                    stream.str("");
                    stream << " [" << sender.c_str() << "] " << frame->code << " " << frame->index << " " << frame->duration << "s " // ip
                            << " " << frame->error_x << "mm " << frame->error_y << "mm " << frame->error_yaw;
                    logger->info(stream.str());
                }

                return true;
            }
            // 接收到的数据异常
            stream.str("");
            stream << "error receiving UDP data: " << error.message().c_str();
            logger->info(stream.str());
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
    bool state;
};
