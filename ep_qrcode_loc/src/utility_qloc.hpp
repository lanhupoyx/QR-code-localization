#pragma once

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

#include "ros/ros.h"
#include "ros/console.h"

#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"

#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "message_filters/subscriber.h"

#include "Eigen/Dense"

#include "boost/asio.hpp"
#include "boost/filesystem.hpp"

#include "file/file.h"
#include "file/path.h"
#include "process/signal_manager.h"
#include "vcs_manager.h"
#include "yaml-cpp/yaml.h"
#include "datetime/datetime.h"

#include "logger.hpp"

using namespace vcs;

namespace fs = boost::filesystem;

// 数据帧转换到16进制
std::string charArrayToHex(std::array<char, 1024> array, size_t size);

// 十六进制转换为十进制
uint32_t convert_16_to_10(std::string str2);

// 十六进制转换为ASCII
std::string hexToAscii(const std::string &hex);

// 时间格式化输出
std::string format_time(ros::Time t);

// 日期格式化输出
std::string format_date(ros::Time t);

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

// get yaw frome pose
geometry_msgs::Pose poseInverse(geometry_msgs::Pose source);

// 扫码相机数据帧格式
struct CameraFrame
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

    bool is_decode;
    bool is_accurate_loc;
    bool is_broad_loc;
};

// 二维码信息帧格式
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

class Logger;

// 参数服务器

class ParamServer
{
public:
    ros::NodeHandle nh;
    std::string yamlData;
    std::string logData;

    std::string logLevel;

    std::string odomMapBase;
    std::string odomMapCamera;
    std::string pathMapBase;
    std::string pathMapCamera;
    std::string msgTopic;

    int operating_mode;
    bool show_original_msg;
    bool is_pub_tf;
    double low_speed_UL;
    std::string port;
    std::string log_dir;
    std::string cfg_dir;
    float maxEstimationDis;

    bool read_yaw_err;

    double yaw_jump_UL;
    double x_jump_UL;
    double y_jump_UL;

    double rec_p1;
    double wheel_diameter;
    double wheel_reduction_ratio;
    double wheel_base_dis;
    double wheel_angular_offset;
    double wheel_angular_forward;
    double wheel_angular_backward;

    double err_ratio_offline;

    double detect_site_dis;
    double aux_site_dis;
    double forkaction_site_dis;
    double site_site_dis;

    double avliable_yaw;

    bool is_debug;
    bool check_sequence;
    bool cal_yaw;
    double ground_code_yaw_offset;

    std::string mainParamPath;
    std::string siteTablePath;

    size_t logKeepDays;

    ParamServer(ros::NodeHandle &nh);

    std::string loadMainParamPath();

    std::string loadSitetableParamPath();

    // 加载单个参数条目
    template <typename T>
    void importItem(YAML::Node &config, std::string FirstName, std::string LastName, T &TargetParam, T DefaultVal);

    // 保存log
    void saveLog(Logger *logger);
};
