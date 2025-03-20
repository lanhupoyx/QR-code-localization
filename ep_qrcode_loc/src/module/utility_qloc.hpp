#pragma once

// 标准库
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
#include <thread>
#include <algorithm>
#include <chrono>
#include <map>
#include <functional>
#include <cstddef>
#include <math.h>

// ros基础
#include "ros/ros.h"
#include "ros/console.h"

// ros/tf消息
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"

// ros消息
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "message_filters/subscriber.h"

// 第三方
#include "Eigen/Dense"
#include "boost/asio.hpp"
#include "boost/filesystem.hpp"

// VCS
#include "file/file.h"
#include "file/path.h"
#include "process/signal_manager.h"
#include "vcs_manager.h"
#include "yaml-cpp/yaml.h"
#include "datetime/datetime.h"

// 数据帧转换到16进制
std::string charArrayToHex(std::array<char, 1024> array, std::size_t size);

// 十六进制转换为十进制
uint32_t convert_16_to_10(std::string str2);

// 十六进制转换为ASCII
std::string hexToAscii(const std::string &hex);

// 时间格式化输出
std::string format_time(ros::Time t);

// 小时数输出
std::string getHour(ros::Time t);

// 日期格式化输出
std::string format_date(ros::Time t);

// 日期数输出
std::string getDay(ros::Time t);

// string 替换字符
std::string replaceChar(std::string str, char toReplace, char replacement);

// 删除末尾若干个字符
std::string del_n_end(std::string str, uint16_t n);

// pose to transform
geometry_msgs::TransformStamped p2t(geometry_msgs::Pose pose);

// transform to pose
geometry_msgs::Pose t2p(geometry_msgs::TransformStamped trans);

// get yaw frome pose
double getYaw(geometry_msgs::Pose pose);

// get yaw frome pose
double getYaw(geometry_msgs::Quaternion q);

// get yaw frome TransformStamped
double getYaw(geometry_msgs::TransformStamped trans);

// get yaw(rad) frome pose
double getYawRad(geometry_msgs::Pose pose);

// get yaw(rad) frome pose
double getYawRad(geometry_msgs::Quaternion q);

/// @brief 得到pose，yaw单位deg
/// @param x 
/// @param y 
/// @param yaw 
/// @return pose
geometry_msgs::Pose toPoseDeg(double x, double y, double yawDeg);

/// @brief 得到pose，yaw单位rad
/// @param x 
/// @param y 
/// @param yaw 
/// @return pose
geometry_msgs::Pose toPoseRad(double x, double y, double yawRad);

// get yaw frome pose
geometry_msgs::Pose poseInverse(geometry_msgs::Pose source);

// 扫码相机数据帧格式
struct CameraFrame
{
    ros::Time stamp;    // 时间辍
    std::string sender; // 相机IP
    std::string hex;    // 数据帧内容（16进制）
    std::string head;   // 帧头
    std::string sum;    // 字节校验和
    uint32_t index;     // 序号
    double duration;    // 算法耗时

    uint32_t code;    // 二维码编号
    double error_x;   // 中心坐标x
    double error_y;   // 中心坐标y
    double error_yaw; // 中心yaw

    bool is_decode;       // 是否译码成功
    bool is_accurate_loc; // 是否精准定位成功
    bool is_broad_loc;    // 是否粗定位成功
};

// 二维码信息帧格式，查询到的地码信息
struct QRcodeInfo
{
    uint32_t code;
    double x;
    double y;
    double yaw;
    double yaw_init;
    double yaw_err_average;
    uint32_t yaw_err_num;

    CameraFrame frame;
    bool is_head;

    uint8_t type; // 列内 0， 列首 1

    QRcodeInfo() {};

    QRcodeInfo(uint32_t code_, double x_, double y_, double yaw_, bool is_head_ = false);

    ~QRcodeInfo() {}
};

// 贴在地上的二维码信息
struct QRcodeGround
{
public:
    uint32_t index_;           // 序号
    geometry_msgs::Pose pose_; // 位姿

    uint8_t type; // 列内 0， 列首 1
    bool is_head;

    double x_err_;   // x方向补偿值，单位m
    double y_err_;   // y方向补偿值，单位m
    double yaw_err_; // yaw方向补偿值,单位角度

    void turn(double d_yaw);
};
