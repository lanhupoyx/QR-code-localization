#pragma once
#ifndef _SC2005AM_QRCODE_H_
#define _SC2005AM_QRCODE_H_

#include "ros/ros.h"
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <ros/console.h>
#include <boost/asio.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <mutex>
#include <vector>
#include <list>
#include <string>
#include <array>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/PoseStamped.h>



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

//扫码相机数据帧格式
struct frame{
    std::string hex;
    std::string head;
    uint32_t index;
    double duration;
    uint32_t code;
    double error_x;
    double error_y;
    double error_yaw;
    std::string sum;
    ros::Time stamp;
    std::string sender;
};

//二维码信息
struct qrcode_info{
    uint32_t code;
    double x;
    double y;
    double yaw;
};

//二维码坐标对照表
class coordinate_table{
public:
    coordinate_table(std::string path, std::ostream& log_os_){
        log_os = &log_os_;
        ifs.open(path, std::ios::in);
        if (!ifs.is_open()){
            *log_os << path + "打开失败!" << std::endl;
        }
        std::string buf;//将数据存放到c++ 中的字符串中
        while (std::getline(ifs,buf)) //使用全局的getline()函数，其里面第一个参数代表输入流对象，第一个参数代表准备好的字符串，每次读取一行内容到buf
        {
            std::stringstream line_ss;
            line_ss << buf;
            qrcode_info info;
            line_ss >> info.code >> info.x >> info.y >> info.yaw;
            map.insert(std::pair<uint32_t, qrcode_info>(info.code, info));
        }
        ifs.close();
        *log_os << "qrcode_table的大小为: " << map.size() << std::endl;
        for(std::map<uint32_t, qrcode_info>::iterator it = map.begin(); it != map.end(); it++) {
            *log_os << (*it).second.code << " " <<  (*it).second.x << " " <<  (*it).second.y << " " <<  (*it).second.yaw << std::endl;
        }
    }

    ~coordinate_table(){}

    bool find(frame pic, qrcode_info* info){
        std::map<uint32_t, qrcode_info>::iterator it = map.find(code);
        if (it != map.end()) {
            *info = (*it).second;
            return true;
        } else {
            *log_os << "can not identify code:" << pic.code << std::endl;
            addframe(pic);
        }
        return false;
    }

    bool addframe(frame pic){
        std::lock_guard<std::mutex> locker(mtx);
        //
        return false;
    }

    /* 
    //采集二维码信息
    std::ofstream table_ofs;
    if(collect_QRcode){
        std::string log_name = log_dir + "/"+ format_time(ros::Time::now()) +".txt";
        std::ofstream log_file(log_name);
        if (!log_file.is_open()) {
            std::cout << "can't open: " << log_name << std::endl;
        }else{
            std::cout << "open: " << log_name << std::endl;
        }
        
    }
    std::ostream& log_os = log_file; // 将文件流转换为输出流对象 */

    static void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
        std::lock_guard<std::mutex> locker(mtx);
        for (int i = 0; i < msg->transforms.size(); ++i) {
            // 处理每个变换, 例如打印出来
            ROS_INFO("Received transform: %s", msg->transforms[i].child_frame_id.c_str());
            tf_buffer.push_back(msg->transforms[i]);
        }
        while(20 < tf_buffer.size()){
            tf_buffer.pop_front();
        }
    }

private:
    std::ifstream ifs;
    std::map<uint32_t, qrcode_info> map;
    std::map<uint32_t, qrcode_info> new_map;
    std::ostream* log_os;
    static std::list<tf2_msgs::TFMessage::transforms> tf_buffer;
    std::mutex mtx;

};


#endif
