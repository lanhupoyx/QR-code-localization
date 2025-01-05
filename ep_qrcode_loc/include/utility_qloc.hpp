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
#include <boost/filesystem.hpp>
#include <algorithm>
#include <chrono>

#include "ros/ros.h"
#include <tf/tf.h>
#include <ros/console.h>
#include <boost/asio.hpp>

#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_msgs/TFMessage.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/PoseStamped.h>
#include "message_filters/subscriber.h"
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include "geometry_msgs/PointStamped.h"

namespace fs = boost::filesystem;

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

// 十六进制转换为ASCII
std::string hexToAscii(const std::string &hex) {
    std::string ascii;
    std::stringstream ss;
 
    // 将16进制字符串转换为整数
    for (size_t i = 0; i < hex.length(); i += 2) {
        std::string byte = hex.substr(i, 2);
        unsigned int value = 0;
        ss << std::hex << byte;
        ss >> value;
        ss.clear();
 
        // 将整数转换为对应的ASCII字符
        ascii += static_cast<char>(value);
    }
 
    return ascii;
}

// 时间格式化输出
std::string format_time(ros::Time t)
{
    std::stringstream ss;
    ros::WallTime wall_time = ros::WallTime(t.toSec());
    ss << wall_time.toBoost().date().year() << '-';
    ss << wall_time.toBoost().date().month() << '-';
    ss << wall_time.toBoost().date().day() << '_';
    ss << wall_time.toBoost().time_of_day().hours() + 8 << ':';
    ss << wall_time.toBoost().time_of_day().minutes() << ':';
    int second = wall_time.toBoost().time_of_day().seconds();
    ss << std::fixed << std::setprecision(6) << (double(second) * 1e3 + double(t.nsec) / 1000000.0) / 1000.0; // 毫秒
    return ss.str();
}

// 日期格式化输出
std::string format_date(ros::Time t)
{
    std::stringstream ss;
    ros::WallTime wall_time = ros::WallTime(t.toSec());
    ss << wall_time.toBoost().date().year() << '-';
    ss << wall_time.toBoost().date().month() << '-';
    ss << wall_time.toBoost().date().day();
    return ss.str();
}

// string 替换字符
std::string replaceChar(std::string str, char toReplace, char replacement) 
{
    size_t start_pos = 0;
    while ((start_pos = str.find(toReplace, start_pos)) != std::string::npos) {
        str.replace(start_pos, 1, 1, replacement);
        ++start_pos;
    }
    return str;
}

// 删除末尾若干个字符
std::string del_n_end(std::string str, uint16_t n)
{
    if (str.size() <= n)
    {
        return str;
    }
    else
    {
        return str.substr(0, str.size()-n);
    }
}

// pose to transform
geometry_msgs::TransformStamped p2t(geometry_msgs::Pose pose){
    geometry_msgs::TransformStamped trans;
    trans.transform.translation.x = pose.position.x;
    trans.transform.translation.y = pose.position.y;
    trans.transform.translation.z = pose.position.z;
    trans.transform.rotation = pose.orientation;
    return trans;
}

// transform to pose
geometry_msgs::Pose t2p(geometry_msgs::TransformStamped trans){
    geometry_msgs::Pose pose;
    pose.position.x = trans.transform.translation.x;
    pose.position.y = trans.transform.translation.y;
    pose.position.z = trans.transform.translation.z;
    pose.orientation = trans.transform.rotation;
    return pose;
}

// get yaw frome pose
double getYaw(geometry_msgs::Pose pose){
    tf::Quaternion q(   pose.orientation.x, 
                        pose.orientation.y, 
                        pose.orientation.z, 
                        pose.orientation.w); // 初始化四元数
    double roll = 0.0, pitch = 0.0, yaw = 0.0;          // 初始化欧拉角
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);         // 四元数转欧拉角
    return yaw*180/M_PI;
}

// get yaw frome pose
double getYaw(geometry_msgs::Quaternion q){
    tf::Quaternion quaternion(q.x, q.y, q.z, q.w); // 初始化四元数
    double roll = 0.0, pitch = 0.0, yaw = 0.0;          // 初始化欧拉角
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);         // 四元数转欧拉角
    return yaw*180/M_PI;
}

// get yaw frome TransformStamped
double getYaw(geometry_msgs::TransformStamped trans){
    tf::Quaternion q(   trans.transform.rotation.x, 
                        trans.transform.rotation.y, 
                        trans.transform.rotation.z, 
                        trans.transform.rotation.w); // 初始化四元数
    double roll = 0.0, pitch = 0.0, yaw = 0.0;          // 初始化欧拉角
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);         // 四元数转欧拉角
    return yaw*180/M_PI;
}

// get yaw(rad) frome pose
double getYawRad(geometry_msgs::Pose pose){
    tf::Quaternion quaternion(  pose.orientation.x, 
                                pose.orientation.y, 
                                pose.orientation.z, 
                                pose.orientation.w); // 初始化四元数
    double roll = 0.0, pitch = 0.0, yaw = 0.0;          // 初始化欧拉角
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);         // 四元数转欧拉角
    return yaw;
}

// get yaw(rad) frome pose
double getYawRad(geometry_msgs::Quaternion q){
    tf::Quaternion quaternion(q.x, q.y, q.z, q.w); // 初始化四元数
    double roll = 0.0, pitch = 0.0, yaw = 0.0;          // 初始化欧拉角
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);         // 四元数转欧拉角
    return yaw;
}

// get yaw frome pose
geometry_msgs::Pose poseInverse(geometry_msgs::Pose source)
{
    geometry_msgs::Pose result;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, -getYawRad(source.orientation));
    tf::quaternionTFToMsg(q, result.orientation);
    tf::Matrix3x3 m(q);
    tf::Matrix3x3 inv_m(m);
    tf::Vector3 v(source.position.x, source.position.y, 0.0);
    inv_m.inverse();                                         // 求逆
    result.position.x = -inv_m.getRow(0).dot(v); //(row0(0)*v.getX() + inv_m(0,1)*v.getX() +inv_m(0,2)*v.getZ());
    result.position.y = -inv_m.getRow(1).dot(v); //-(inv_m(1,0)*v.getX() + inv_m(1,1)*v.getX() +inv_m(1,2)*v.getZ());
    result.position.z = 0.0;
    return result;
}

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

    QRcodeInfo(){};

    QRcodeInfo(uint32_t code_, double x_, double y_, double yaw_, bool is_head_ = false)
    {
        code = code_;
        x = x_;
        y = y_;
        yaw = yaw_;
        is_head = is_head_;
        type = 0;
    }

    ~QRcodeInfo(){}
};

// 参数服务器
class ParamServer
{
public:
    ros::NodeHandle nh;

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

    ParamServer()
    {
        // log级别
        nh.param<std::string>("ep_qrcode_loc/logLevel", logLevel, "INFO");
        nh.param<int>("ep_qrcode_loc/operating_mode", operating_mode, 3);
        // topic名称
        nh.param<std::string>("ep_qrcode_loc/odomMapBase", odomMapBase, "ep_qrcode_loc/odometry/base");
        nh.param<std::string>("ep_qrcode_loc/odomMapCamera", odomMapCamera, "ep_qrcode_loc/odometry/locCamera");
        nh.param<std::string>("ep_qrcode_loc/pathMapBase", pathMapBase, "ep_qrcode_loc/path/base");
        nh.param<std::string>("ep_qrcode_loc/pathMapCamera", pathMapCamera, "ep_qrcode_loc/path/locCamera");
        nh.param<std::string>("ep_qrcode_loc/msgTopic", msgTopic, "ep_qrcode_loc/msg");
        // 运行模式
        nh.param<bool>("ep_qrcode_loc/show_original_msg", show_original_msg, false);
        nh.param<bool>("ep_qrcode_loc/is_pub_tf", is_pub_tf, false);
        nh.param<double>("ep_qrcode_loc/yaw_jump_UL", yaw_jump_UL, 2.0);
        nh.param<double>("ep_qrcode_loc/x_jump_UL", x_jump_UL, 0.05);
        nh.param<double>("ep_qrcode_loc/y_jump_UL", y_jump_UL, 0.05);
        nh.param<bool>("ep_qrcode_loc/read_yaw_err", read_yaw_err, false);
        if (5 == operating_mode) read_yaw_err = false;
        nh.param<double>("ep_qrcode_loc/err_ratio_offline", err_ratio_offline, 1.0);
        nh.param<double>("ep_qrcode_loc/rec_p1", rec_p1, 0.0);
        nh.param<double>("ep_qrcode_loc/avliable_yaw", avliable_yaw, 0.0);
        nh.param<double>("ep_qrcode_loc/wheel_diameter", wheel_diameter, 0.0);
        nh.param<double>("ep_qrcode_loc/wheel_reduction_ratio", wheel_reduction_ratio, 1.0);
        nh.param<double>("ep_qrcode_loc/wheel_base_dis", wheel_base_dis, 0.0);
        nh.param<double>("ep_qrcode_loc/wheel_angular_offset", wheel_angular_offset, 0.0);
        nh.param<double>("ep_qrcode_loc/wheel_angular_forward", wheel_angular_forward, 0.0);
        nh.param<double>("ep_qrcode_loc/wheel_angular_backward", wheel_angular_backward, 0.0);
        nh.param<double>("ep_qrcode_loc/low_speed_UL", low_speed_UL, 0.2);
        nh.param<std::string>("ep_qrcode_loc/port", port, "1024");
        nh.param<std::string>("ep_qrcode_loc/log_dir", log_dir, "/var/xmover/log/QR_code_loc/");
        nh.param<std::string>("ep_qrcode_loc/cfg_dir", cfg_dir, "/var/xmover/params/ep-qrcode-loc/");
        nh.param<float>("ep_qrcode_loc/maxEstimationDis", maxEstimationDis, 1.0);
        nh.param<double>("ep_qrcode_loc/detect_site_dis", detect_site_dis, 1.8);
        nh.param<double>("ep_qrcode_loc/aux_site_dis", aux_site_dis, 1.365);
        nh.param<double>("ep_qrcode_loc/forkaction_site_dis", forkaction_site_dis, 0.93);
        nh.param<double>("ep_qrcode_loc/site_site_dis", site_site_dis, 1.36);
        nh.param<bool>("ep_qrcode_loc/is_debug", is_debug, false);
        nh.param<bool>("ep_qrcode_loc/check_sequence", check_sequence, true);
        nh.param<bool>("ep_qrcode_loc/cal_yaw", cal_yaw, true);
        nh.param<double>("ep_qrcode_loc/ground_code_yaw_offset", ground_code_yaw_offset, 0.0);
    }
};


// 记录服务器
class Logger : public ParamServer
{
private:
    static std::ofstream logFile_;
    static std::mutex mutex;

    static std::ofstream poseFile_;
    static std::mutex mutex_pose;

    static std::ofstream otherFile_;
    static std::mutex mutex_other;

    static std::ofstream yawerrFile_;
    static std::mutex mutex_yawerr;

    static std::ofstream jumperrFile_;
    static std::mutex mutex_jumperr;
    
    std::string loglevel_;
 
    // 私有构造函数确保不能直接创建Logger实例
    Logger() 
    {
        init(log_dir);
        loglevel_ = logLevel;
    }
 
    // 防止拷贝构造和赋值
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
 
    // 静态成员函数，用于初始化静态变量
    static void init(std::string log_dir_) 
    {
        // 创建文件夹
        std::string log_dir_today = log_dir_ + format_date(ros::Time::now()) + "/";
        fs::path dir(log_dir_today); 
        if (fs::create_directory(dir)) {
            std::cout << "Folder created successfully." << std::endl;
        } else {
            std::cout << "Folder already exists or cannot be created." << std::endl;
        }

        // log文件
        std::string log_path = log_dir_today + format_time(ros::Time::now()) + "_qrcode_log.txt";
        logFile_.open(log_path, std::ios::app);
        if (!logFile_.is_open()) 
        {
            std::cout << "can't open: " << log_path << std::endl;
        }
        else
        {
            std::cout << "open: " << log_path << std::endl;
        }

        // pose文件
        std::string pose_path = log_dir_today + "pose.txt";
        poseFile_.open(pose_path, std::ios::app);
        if (!poseFile_.is_open()) 
        {
            std::cout << "can't open: " << pose_path << std::endl;
        }
        else
        {
            std::cout << "open: " << pose_path << std::endl;
        }

        // other文件
        std::string other_path = log_dir_today + "other.csv";
        otherFile_.open(other_path, std::ios::app);
        if (!otherFile_.is_open()) 
        {
            std::cout << "can't open: " << other_path << std::endl;
        }
        else
        {
            std::cout << "open: " << other_path << std::endl;
        }

        // yawerr文件
        std::string yawerr_path = log_dir_ + "yawerr.txt";
        yawerrFile_.open(yawerr_path, std::ios::app);
        if (!yawerrFile_.is_open()) 
        {
            std::cout << "can't open: " << yawerr_path << std::endl;
        }
        else
        {
            std::cout << "open: " << yawerr_path << std::endl;
        }

        // jumperr文件
        std::string jumperr_path = log_dir_today + "jumperr.txt";
        jumperrFile_.open(jumperr_path, std::ios::app);
        if (!jumperrFile_.is_open()) 
        {
            std::cout << "can't open: " << jumperr_path << std::endl;
        }
        else
        {
            std::cout << "open: " << jumperr_path << std::endl;
        }
    }
 
public:
    // 获取单例对象
    static Logger& getInstance() {
        static Logger instance; // 懒汉式，在第一次调用时实例化
        return instance;
    }
 
    // 记录日志的方法
    void log(const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex); // 线程安全
        logFile_ << message << std::endl;
    }

    void fatal(const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex); // 线程安全
        logFile_ << format_time(ros::Time::now()) << "[FATAL]" << message << std::endl;
    }

    void error(const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex); // 线程安全
        logFile_ << format_time(ros::Time::now()) << " [ERROR] " << message << std::endl;
    }

    void warn(const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex); // 线程安全
        logFile_ << format_time(ros::Time::now()) << " [WARN] " << message << std::endl;
    }

    void info(const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex); // 线程安全
        logFile_ << format_time(ros::Time::now()) << " [INFO] " << message << std::endl;
    }

    void debug_endl() {
        if("DEBUG" == loglevel_)
        {
            std::lock_guard<std::mutex> lock(mutex); // 线程安全
            logFile_ << std::endl;
        }
    }

    void debug(const std::string& message) {
        if("DEBUG" == loglevel_)
        {
            std::lock_guard<std::mutex> lock(mutex); // 线程安全
            logFile_ << format_time(ros::Time::now()) << " [DEBUG] " << message << std::endl;
        }
    }

    void pose(const std::string& message) 
    {
        std::lock_guard<std::mutex> lock(mutex_pose); // 线程安全
        poseFile_ << del_n_end(format_time(ros::Time::now()), 3) << "," << message << std::endl;
    }

    void other(const std::string& message) 
    {
        std::lock_guard<std::mutex> lock(mutex_other); // 线程安全
        otherFile_ << format_time(ros::Time::now()) << " " << message << std::endl;
    }

    void yawerr(const std::string& message) 
    {
        std::lock_guard<std::mutex> lock(mutex_yawerr); // 线程安全
        yawerrFile_ << format_time(ros::Time::now()) << " " << message << std::endl;
    }

    void close_yawerr()
    {
        yawerrFile_.close();
    }

    void jumperr(const std::string& message) 
    {
        std::lock_guard<std::mutex> lock(mutex_jumperr); // 线程安全
        jumperrFile_ << format_time(ros::Time::now()) << " " << message << std::endl;
    }

    // 滚动删除历史文件夹
    void roll_delete_old_folders(size_t keep_count)
    {
        fs::path dir_path(log_dir);

        std::vector<fs::directory_entry> directories;
        for (const auto &entry : fs::directory_iterator(dir_path))
        {
            if (fs::is_directory(entry))
            {
                directories.push_back(entry);
            }
        }

        if (directories.size() <= keep_count)
        {
            this->info("No old log folders to delete. Total folders: " + std::to_string(directories.size()));
            return;
        }

        std::sort(directories.begin(), directories.end(), [](const fs::directory_entry &a, const fs::directory_entry &b)
                  { return fs::last_write_time(a.path()) < fs::last_write_time(b.path()); });

        for (size_t i = 0; i < directories.size() - keep_count; ++i)
        {
            fs::remove_all(directories[i].path());
            this->info("Deleted: " + directories[i].path().string());
        }
    }
};

std::ofstream Logger::logFile_;
std::mutex Logger::mutex;
std::ofstream Logger::poseFile_;
std::mutex Logger::mutex_pose;
std::ofstream Logger::otherFile_;
std::mutex Logger::mutex_other;
std::ofstream Logger::yawerrFile_;
std::mutex Logger::mutex_yawerr;
std::ofstream Logger::jumperrFile_;
std::mutex Logger::mutex_jumperr;

