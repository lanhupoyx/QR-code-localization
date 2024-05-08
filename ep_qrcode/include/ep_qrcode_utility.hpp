#pragma once
#ifndef _SC2005AM_QRCODE_H_
#define _SC2005AM_QRCODE_H_

#include "ros/ros.h"
#include <tf/tf.h>
#include <ros/console.h>
#include <boost/asio.hpp>
#include <iostream>
#include <cmath>
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
#include <tf2_msgs/TFMessage.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/PoseStamped.h>
#include "message_filters/subscriber.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"

// 数据帧转换到16进制
std::string charArrayToHex(std::array<char, 1024> array, size_t size)
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < size; ++i)
    {
        ss << std::setw(2) << static_cast<int>(static_cast<unsigned char>(array[i]));
    }
    return ss.str();
}

// 十六进制转换为十进制
uint32_t convert_16_to_10(std::string str2)
{

    double sum = 0, times;
    double m;
    std::string::size_type sz = str2.size();
    for (std::string::size_type index = 0; index != sz; ++index)
    {
        // 变为小写，这个思路很好
        str2[index] = tolower(str2[index]);
        if (str2[index] >= 'a' && str2[index] <= 'f')
        {
            // 这里让a~f进行转换为数字字符，很奇妙
            m = str2[index] - 'a' + 10;
            // 求幂次方
            times = pow(16, (sz - 1 - index));
            sum += m * times;
        }
        else if (isdigit(str2[index]))
        {
            // 需要将字符类型转换为数字类型
            // 因为0的ASCII码是48，所以转换为相应的数字，减去48即可
            m = str2[index] - 48;
            times = pow(16, (sz - 1 - index));
            sum += m * times;
        }
        else
        {
            std::cout << "无法识别的十六进制!";
            break;
        }
    }
    return uint32_t(sum);
}

// 时间格式化输出
std::string format_time(ros::Time t)
{
    std::stringstream ss;
    ros::WallTime wall_time = ros::WallTime(t.toSec());
    ss << wall_time.toBoost().date().year() << '-';
    ss << wall_time.toBoost().date().month() << '-';
    ss << wall_time.toBoost().date().day() << ' ';
    ss << wall_time.toBoost().time_of_day().hours() + 8 << ':';
    ss << wall_time.toBoost().time_of_day().minutes() << ':';
    int second = wall_time.toBoost().time_of_day().seconds();
    ss << (double(second) * 1e3 + double(t.nsec) / 1000000.0) / 1000.0; // 毫秒
    return ss.str();
}

// 扫码相机数据帧格式
struct frame
{
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

// 二维码信息
struct qrcode_info
{
    uint32_t code;
    double x;
    double y;
    double yaw;
};

// 二维码坐标对照表
class coordinate_table
{
public:
    coordinate_table(std::string path_, std::ofstream *log_os_, geometry_msgs::TransformStamped transform_)
    {
        log_os = log_os_;
        tf_b2c = transform_;
        path = path_;
        ifs.open(path, std::ios::in);
        if (!ifs.is_open())
        {
            *log_os << path + "打开失败!" << std::endl;
        }
        std::string buf;               // 将数据存放到c++ 中的字符串中
        while (std::getline(ifs, buf)) // 使用全局的getline()函数，其里面第一个参数代表输入流对象，第一个参数代表准备好的字符串，每次读取一行内容到buf
        {
            std::stringstream line_ss;
            line_ss << buf;
            qrcode_info info;
            line_ss >> info.code >> info.x >> info.y >> info.yaw;
            map.insert(std::pair<uint32_t, qrcode_info>(info.code, info));
        }
        ifs.close();
        *log_os << "qrcode_table的大小为: " << map.size() << std::endl;
        for (std::map<uint32_t, qrcode_info>::iterator it = map.begin(); it != map.end(); it++)
        {
            *log_os << (*it).second.code << " " << (*it).second.x << " " << (*it).second.y << " " << (*it).second.yaw << std::endl;
        }
    }

    ~coordinate_table() {}

    // 根据二维码编号查表，得到位姿信息
    bool find(frame pic, qrcode_info *info)
    {
        std::map<uint32_t, qrcode_info>::iterator it = map.find(pic.code);
        if (it != map.end())
        {
            *info = (*it).second;
            return true;
        }
        else
        {
            *log_os << "can not identify code:" << pic.code << std::endl;
            add(pic);
        }
        return false;
    }

    // 添加新的二维码
    bool add(const frame pic)
    {
        // std::lock_guard<std::mutex> locker(coordinate_table::mtx);
        if (frame_buffer.size() > 120)
        {
            frame_buffer.pop_front();
        }
        static uint32_t lastcode = 0;
        if (pic.code != lastcode)
        {
            lastcode = pic.code;
            frame_buffer.clear();
        }
        frame_buffer.push_back(pic);

        // 计算并保存新二维码信息
        if ((tf_buffer.size() > 9) || (frame_buffer.size() > 110))
        {
            qrcode_info new_qrcode = calPose();
            new_qrcode.code = pic.code;
            map.insert(std::pair<uint32_t, qrcode_info>(new_qrcode.code, new_qrcode));
            std::ofstream save(path, std::ios::app);
            std::stringstream line_ss;
            std::ostream &save_os = save; // 将文件流转换为输出流对象
            save_os << new_qrcode.code << " " << new_qrcode.x << " " << new_qrcode.y << " " << new_qrcode.yaw << std::endl;
            save.close();
        }

        return true;
    }

    // 计算二维码位姿
    qrcode_info calPose()
    {
        std::lock_guard<std::mutex> locker(coordinate_table::mtx);
        double x_sum = 0, y_sum = 0, yaw_sum = 0;
        int i = 0;
        std::list<qrcode_info> poselog;
        for (std::list<nav_msgs::Odometry>::iterator t_it = tf_buffer.begin(); t_it != tf_buffer.end(); t_it++)
        {
            double min_time_err = 1.0;
            std::list<frame>::iterator d_it = frame_buffer.begin();
            for (std::list<frame>::iterator f_it = frame_buffer.begin(); f_it != frame_buffer.end(); f_it++)
            {
                ros::Duration time_err = f_it->stamp - t_it->header.stamp;
                double d_time_err = std::fabs(time_err.toSec());
                if (d_time_err > min_time_err)
                {
                    continue;
                }
                else
                {
                    min_time_err = d_time_err;
                    d_it = f_it;
                }
            }
            // baselink---->camera
            geometry_msgs::Pose pose_out;
            tf2::doTransform(t_it->pose.pose, pose_out, tf_b2c);
            double roll = 0.0, pitch = 0.0, yaw = 0.0;                                                                        // 初始化欧拉角
            tf::Quaternion q(pose_out.orientation.x, pose_out.orientation.y, pose_out.orientation.z, pose_out.orientation.w); // 初始化四元数
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);                                                                        // 四元数转欧拉角

            // camera---->qrcode
            qrcode_info frame;
            frame.x = pose_out.position.x - d_it->error_x;
            frame.y = pose_out.position.y - d_it->error_y;
            frame.yaw = yaw - d_it->error_yaw;
            poselog.push_back(frame);

            x_sum += frame.x;
            y_sum += frame.y;
            yaw_sum += frame.yaw;
            i++;
        }

        // //tode:检查10帧计算结果之间的偏差
        // for(std::list<qrcode_info> it = poselog.begin() + 1; it != poselog.end(); it++){
        // }

        // 输出平均后的结果
        qrcode_info new_frame;
        new_frame.x = x_sum / i;
        new_frame.y = y_sum / i;
        new_frame.yaw = yaw_sum / i;

        return (new_frame);
    }

    // callback获取baselink位姿
    static void tfCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> locker(coordinate_table::mtx);
        const nav_msgs::Odometry transform = *msg; //->transforms[i];
        // *log_os << "base_link at:" << transform.pose.pose.position.x << " "
        //         << transform.pose.pose.position.y << " "
        //         << transform.pose.pose.position.z << " "
        //         << std::endl;
        if (tf_buffer.size() != 0)
        {
            double x_error = std::fabs(transform.pose.pose.position.x - tf_buffer.back().pose.pose.position.x);
            double y_error = std::fabs(transform.pose.pose.position.y - tf_buffer.back().pose.pose.position.y);
            if ((x_error > 0.05) || (y_error > 0.05))
            {
                tf_buffer.clear();
            }
        }
        tf_buffer.push_back(transform);
        while (10 < tf_buffer.size())
        {
            tf_buffer.pop_front();
        }
    }

private:
    std::ifstream ifs;
    std::map<uint32_t, qrcode_info> map;
    std::map<uint32_t, qrcode_info> new_map;
    std::ofstream *log_os;
    std::list<frame> frame_buffer;

    geometry_msgs::TransformStamped tf_b2c;
    std::string path;

    static std::mutex mtx;
    static std::list<nav_msgs::Odometry> tf_buffer;
};

std::mutex coordinate_table::mtx;
std::list<nav_msgs::Odometry> coordinate_table::tf_buffer;

#endif
