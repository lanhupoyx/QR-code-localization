// ################################################
// #####        main function for ep_qrcode
// #####        yuanxun@ep-ep.com
// #####        updateTime:  2024.05.01
// ################################################
#include "ep_qrcode_utility.hpp"

class MV_SC2005AM
{
public:
    MV_SC2005AM(std::string port, std::ofstream *log_os_)
    {
        log_os = log_os_;
        // UDP端口监测初始化
        socket = new boost::asio::ip::udp::socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), std::atoi(port.c_str())));
    }
    ~MV_SC2005AM() {}

    bool getframe(frame *pic)
    {
        if (socket->available())
        {
            socket->receive_from(boost::asio::buffer(recv_buf), sender_endpoint, 0, error);
            if (!error)
            {
                ros::Time now = ros::Time::now();
                std::string sender = sender_endpoint.address().to_string();
                pic->hex = charArrayToHex(recv_buf, 15);
                // std::cout << "hex is: " << pic->hex << std::endl;
                pic->sender = sender;

                // todo:判断帧头和字节校验和
                pic->head = pic->hex.substr(2, 2) + pic->hex.substr(0, 2);
                pic->sum = pic->hex.substr(28, 2); // 转换为整数

                // 数据提取与转换
                pic->index = convert_16_to_10(pic->hex.substr(4, 2));

                u_int32_t d = convert_16_to_10(pic->hex.substr(6, 2));
                pic->duration = d / 1000.0;
                pic->stamp = now - ros::Duration(pic->duration);

                pic->code = convert_16_to_10(pic->hex.substr(14, 2) + pic->hex.substr(12, 2) + pic->hex.substr(10, 2) + pic->hex.substr(8, 2));

                int16_t pixel_x = std::stoi(pic->hex.substr(18, 2) + pic->hex.substr(16, 2), 0, 16);
                int16_t pixel_y = std::stoi(pic->hex.substr(22, 2) + pic->hex.substr(20, 2), 0, 16);
                int16_t pixel_yaw = std::stoi(pic->hex.substr(26, 2) + pic->hex.substr(24, 2), 0, 16);
                pic->error_x = pixel_x * 0.2125;
                pic->error_y = pixel_y * 0.2166667;
                pic->error_yaw = pixel_yaw / 100.0;

                // 保存帧log
                *log_os << format_time(pic->stamp) << " [" << sender.c_str() << "] " << pic->code << pic->index << " " << pic->duration << "s " // ip
                        << " " << pic->error_x << "mm " << pic->error_y << "mm " << pic->error_yaw << std::endl;
                std::cout << format_time(pic->stamp) << " [" << sender.c_str() << "] " << pic->code << pic->index << " " << pic->duration << "s " // ip
                          << " " << pic->error_x << "mm " << pic->error_y << "mm " << pic->error_yaw << std::endl;

                return true;
            }
            *log_os << "error receiving UDP data: " << error.message().c_str() << std::endl;
        }
        return false;
    }

private:
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket *socket;
    boost::asio::ip::udp::endpoint sender_endpoint;
    std::array<char, 1024> recv_buf;
    boost::system::error_code error;
    std::ofstream *log_os;
};