#include "ros/ros.h"
#include <tf/tf.h>
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
#include <nav_msgs/Odometry.h>

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

//帧格式
struct frame{
    std::string hex;
    std::string head;
    uint32_t index;
    uint32_t duration;
    uint32_t code;
    double error_x;
    double error_y;
    double error_yaw;
    std::string sum;
    ros::Time stamp;
};

//二维码信息
struct qrcode_info{
    uint32_t code;
    double x;
    double y;
    double yaw;
};

//主函数
int main(int argc, char *argv[])
{
    //初始化
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"ep_qrcode");
    ros::NodeHandle nh;

    //发布器
    ros::Publisher pub_qrCodeMsg = nh.advertise<std_msgs::String>("qrcode_msg",1000);
    ros::Publisher pub_qrcode_odom = nh.advertise<nav_msgs::Odometry> ("qrcode_odom", 2000);

    //UDP端口监测初始化
    std::string port = "1024";
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), std::atoi(port.c_str())));
    boost::asio::ip::udp::endpoint sender_endpoint;
    std::array<char, 1024> recv_buf;
    boost::system::error_code error;

    //日志文件初始化
    std::string log_name = "/var/xmover/log/qrcode/"+ format_time(ros::Time::now()) +".txt";
    std::ofstream log_file(log_name);
    if (!log_file.is_open()) {
        std::cout << "can't open: " << log_name << std::endl;
    }else{
        std::cout << "open: " << log_name << std::endl;
    }
    std::ostream& log_os = log_file; // 将文件流转换为输出流对象

    //读取二维码-坐标对照表
    std::string table_name = "/home/xun/work/ep-qrcode/src/ep_qrcode/config/qrcode_coordinate_table.txt";
    std::ifstream ifs;
    ifs.open(table_name, std::ios::in);
    if (!ifs.is_open()){
        std::cout << table_name + "打开失败!" << std::endl;
    }
    std::map<uint32_t, qrcode_info> qrcode_table;
    std::string buf;//将数据存放到c++ 中的字符串中
    while (std::getline(ifs,buf)) //使用全局的getline()函数，其里面第一个参数代表输入流对象，第一个参数代表准备好的字符串，每次读取一行内容到buf
    {
        std::stringstream line_ss;
        line_ss << buf;
        qrcode_info info;
        line_ss >> info.code >> info.x >> info.y >> info.yaw;
        qrcode_table.insert(std::pair<uint32_t, qrcode_info>(info.code, info));
    }
    log_os << "qrcode_table的大小为: " << qrcode_table.size() << std::endl;

    //主循环
    ros::Rate loop_rate(100);
    while (ros::ok())
    {   
        frame f; 
        bool isNewFrame = false;
        if(socket.available()){
            socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint, 0, error);
            if (!error)
            {
                f.stamp = ros::Time::now();
                std::string sender = sender_endpoint.address().to_string();
                f.hex = charArrayToHex(recv_buf, 15);
                std::cout << "hex is: " << f.hex << std::endl;
                
                //todo:判断帧头和字节校验和
                f.head = f.hex.substr(2, 2) + f.hex.substr(0, 2);
                f.sum = f.hex.substr(28, 2); // 转换为整数

                //数据提取与转换
                f.index = convert_16_to_10(f.hex.substr(4, 2));
                f.duration = convert_16_to_10(f.hex.substr(6, 2));
                f.code = convert_16_to_10(f.hex.substr(14, 2) + f.hex.substr(12, 2) + f.hex.substr(10, 2) + f.hex.substr(8, 2));
                f.error_x = double(std::stoi(f.hex.substr(18, 2) + f.hex.substr(16, 2), 0, 16))*0.2125; 
                f.error_y = double(std::stoi(f.hex.substr(22, 2) + f.hex.substr(20, 2), 0, 16))*0.2166667;
                f.error_yaw = double(convert_16_to_10(f.hex.substr(26, 2) + f.hex.substr(24, 2)))/100.0;

                //发布 /qrCodeMsg
                std::stringstream pub_ss;
                pub_ss  << format_time(f.stamp) << " [" << sender.c_str() << "] " << f.index << " " << f.duration << "ms " // ip
                        << " " << f.error_x << "mm "<< f.error_y << "mm "<< f.error_yaw << std::endl;
                std_msgs::String msg;
                msg.data = pub_ss.str();
                pub_qrCodeMsg.publish(msg);
                
                //保存帧log
                log_os  << format_time(f.stamp) << " [" << sender.c_str() << "] " << f.index << " " << f.duration << "ms " // ip
                        << " " << f.error_x << "mm "<< f.error_y << "mm "<< f.error_yaw << std::endl;
                
                isNewFrame = true;
            }
            else
            {
                ROS_WARN("Error receiving UDP data: %s", error.message().c_str());
            }
        }
        if(isNewFrame){
            isNewFrame = false;
            
            //获取二维码坐标
            static qrcode_info info;
            static qrcode_info info_last;
            if(f.code != info_last.code){
                std::map<uint32_t, qrcode_info>::iterator it = qrcode_table.find(f.code);
                if (it != qrcode_table.end()) {
                    info_last = info;
                    info = (*it).second;
                } else {
                    log_os << "can not identify code:" << f.code << std::endl;
                }
            }

            //计算相机坐标
            double camera_x = info.x + f.error_x;
            double camera_y = info.y + f.error_y;
            double camera_yaw = info.yaw + f.error_yaw;

            tf::Quaternion q;
            q.setRPY(0, 0, camera_yaw);




            //发布相机坐标

            nav_msgs::Odometry odom;

            odom.header.stamp = f.stamp;
            odom.header.frame_id = "map";

            odom.pose.pose.position.x = camera_x;
            odom.pose.pose.position.y = camera_y;
            odom.pose.pose.position.z = 0;
            odom.pose.pose.orientation.x = q.getX();
            odom.pose.pose.orientation.y = q.getY();
            odom.pose.pose.orientation.z = q.getZ();
            odom.pose.pose.orientation.w = q.getW();

            //odom.pose.covariance.
            
            pub_qrcode_odom.publish(odom);

        }
 
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}


