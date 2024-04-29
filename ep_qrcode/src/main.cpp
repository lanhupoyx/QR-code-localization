#include "ros/ros.h"
#include <ros/console.h>
#include <boost/asio.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <array>
#include "ep_qrcode/qrcodedebug.h"
#include "std_msgs/String.h"

//数据帧转换到16进制
std::string charArrayToHex(std::array<char, 1024> array, size_t size) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < size; ++i) {
        ss << std::setw(2) << static_cast<int>(static_cast<unsigned char>(array[i]));
    }
    return ss.str();
}

//十六进制转换为十进制
uint32_t convert_16_to_10(std::string str2)
{

    double sum = 0, times;
    double m;
    std::string::size_type sz = str2.size();
    for (std::string::size_type index = 0; index != sz; ++index)
    {
        //变为小写，这个思路很好
        str2[index] = tolower(str2[index]);
        if (str2[index] >= 'a' && str2[index] <= 'f')
        {
            //这里让a~f进行转换为数字字符，很奇妙
            m = str2[index] - 'a' + 10;
            //求幂次方
            times = pow(16, (sz - 1 - index));
            sum += m * times;
 
        }else if (isdigit(str2[index]))
        {
            //需要将字符类型转换为数字类型
            //因为0的ASCII码是48，所以转换为相应的数字，减去48即可
            m= str2[index] - 48;
            times = pow(16, (sz - 1 - index));
            sum += m * times;
            
        }else
        {
            std::cout << "无法识别的十六进制!";
            break;
        }
    }
    return uint32_t(sum);
}

//时间格式化输出
std::string format_time(ros::Time t){
    std::stringstream ss;
    ros::WallTime wall_time = ros::WallTime(t.toSec());
    ss << wall_time.toBoost().date().year() << '-';
    ss << wall_time.toBoost().date().month() << '-';
    ss << wall_time.toBoost().date().day() << ' ';
    ss << wall_time.toBoost().time_of_day().hours() + 8 << ':';
    ss << wall_time.toBoost().time_of_day().minutes() << ':';
    int second = wall_time.toBoost().time_of_day().seconds();
    ss << (double(second)*1e3 + double(t.nsec)/1000000.0)/1000.0; // 毫秒
    return ss.str();
}

int main(int argc, char *argv[])
{
    //初始化
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"ep_qrcode");
    ros::NodeHandle nh;

    //发布器
    ros::Publisher pub = nh.advertise<std_msgs::String>("qrCodeMsg",1000);

    //UDP端口监测初始化
    std::string port = "1024";
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), std::atoi(port.c_str())));
    boost::asio::ip::udp::endpoint sender_endpoint;
    std::array<char, 1024> recv_buf;
    boost::system::error_code error;

    //日志文件初始化
    std::string log_name = format_time(ros::Time::now()) +".txt";
    std::ofstream file("/var/xmover/log/qrcode/"+ log_name);
    std::cout << format_time(ros::Time::now()) << std::endl;
    if (!file.is_open()) {
        std::cout << "can't open: " << log_name << std::endl;
    }
    std::ostream& output = file; // 将文件流转换为输出流对象


    while (ros::ok())
    {
        socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint, 0, error);
        if (!error)
        {
            std::string sender = sender_endpoint.address().to_string();
            std::string data = std::string(recv_buf.data());
            //std::cout << recv_buf.data() << std::endl;
            std::string hexString = charArrayToHex(recv_buf, 1024);
            //std::cout << "Hex representation: " << hexString << std::endl;
            std::string subStr = hexString.substr(0, 30);
            //std::cout << "Hex representation: " << subStr << std::endl;

            std::stringstream ss;

            // 获取当前时间戳
            ros::Time current_time = ros::Time::now();
            // 将时间戳转换为秒和纳秒
            int32_t seconds = current_time.sec;
            int32_t nanoseconds = current_time.nsec;
            // 将秒转换为日期和时间
            ros::WallTime wall_time = ros::WallTime(current_time.toSec());
            int year = wall_time.toBoost().date().year();
            int month = wall_time.toBoost().date().month();
            int day = wall_time.toBoost().date().day();
            int hour = wall_time.toBoost().time_of_day().hours() + 8;
            int minute = wall_time.toBoost().time_of_day().minutes();
            int second = wall_time.toBoost().time_of_day().seconds();
            double millisecond = (double(second)*1e3 + double(nanoseconds)/1000000.0)/1000.0; // 毫秒


            //pub_data.stamp = ros::Time::now();
            std::string head = hexString.substr(2, 2) + hexString.substr(0, 2);
            //std::cout << "head is: " << head << std::endl;
            output << year << "-" << month << "-" << day << " " << hour << ":" << minute << ":" << std::fixed << std::setprecision(6) << millisecond ; // 写入内容
            std::cout.unsetf(std::ios::fixed);
            //std::string sindex = hexString.substr(4, 2);
            // std::cout << "shead is: " << sindex << std::endl;
            //int16_t index_temp = std::stoi(hexString.substr(4, 2), 0, 16); // 转换为整数
            uint32_t index = convert_16_to_10(hexString.substr(4, 2));
            ss << " "  << index ; // 写入内容
            output << " " << index; // 写入内容
            //pub_data.index = uint8_t(index_temp);
            //std::cout << "index is: " << index << std::endl;
            //output << "  index: " << pub_data.index ; // 写入内容

            //std::string sduration = hexString.substr(6, 2);
            //std::cout << "shead is: " << sduration << std::endl;
            //pub_data.duration = std::stoi(hexString.substr(6, 2), 0, 16); // 转换为整数
            //std::cout << "duration is: " << duration << std::endl;

            //std::string scode = hexString.substr(14, 2) + hexString.substr(12, 2) + hexString.substr(10, 2) + hexString.substr(8, 2);
            //std::cout << "shead is: " << scode << std::endl;
            uint32_t code = convert_16_to_10(hexString.substr(14, 2) + hexString.substr(12, 2) + hexString.substr(10, 2) + hexString.substr(8, 2)); // 转换为整数
            ss << " "  << code ; // 写入内容
            output << " " << code ; // 写入内容

            int16_t error_x = std::stoi(hexString.substr(18, 2) + hexString.substr(16, 2), 0, 16); // 转换为整数
            ss << " dx:" << error_x*0.2125 ; // 写入内容
            output << " dx:" << error_x*0.2125 ; // 写入内容

            int16_t error_y = std::stoi(hexString.substr(22, 2) + hexString.substr(20, 2), 0, 16); // 转换为整数
            ss << " dy:" << error_y*0.2166667 ; // 写入内容
            output << " dy:" << error_y*0.2166667 ; // 写入内容

            uint32_t error_yaw = convert_16_to_10(hexString.substr(26, 2) + hexString.substr(24, 2)); // 转换为整数
            ss << " dyaw:" << error_yaw/100.0 ; // 写入内容
            output << " dyaw:" << error_yaw/100.0 ; // 写入内容

            ss << " dyaw_hex: 0x" << hexString.substr(26, 2) << hexString.substr(24, 2) ; // 写入内容

            u_int8_t sum = std::stoi(hexString.substr(28, 2), 0, 16); // 转换为整数
            //std::cout << "sum is: " << sum << std::endl;
            
            std_msgs::String msg;
            msg.data = ss.str();
            pub.publish(msg);

            output << std::endl; // 写入内容
            
            //ROS_INFO("Received %s from %s", data.c_str(), sender.c_str());
        }
        else
        {
            ROS_WARN("Error receiving UDP data: %s", error.message().c_str());
        }
 
        ros::spinOnce();
    }

    return 0;
}


