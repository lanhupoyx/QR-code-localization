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
    CameraFrame frame;
    bool is_head;

    QRcodeInfo(){};

    QRcodeInfo(uint32_t code_, double x_, double y_, double yaw_, bool is_head_ = false)
    {
        code = code_;
        x = x_;
        y = y_;
        yaw = yaw_;
        is_head = is_head_;
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

    std::string odomQrmapBase;
    std::string odomQrmapCamera;
    std::string msgTopic;

    int operating_mode;
    bool show_original_msg;
    bool is_pub_tf;
    bool ignore_area;
    double low_speed_UL;
    std::string port;
    std::string log_dir;
    std::string cfg_dir;
    std::vector<double> qrmap2mapTrans;
    std::vector<float> enableArea;
    float maxEstimationDis;
    geometry_msgs::Pose pose_qrmap2mapcopy;
    std::ofstream log_file;

    double realVelRatio_x;
    double realVelRatio_z;
    double realVelOffset_x;
    double realVelOffset_z;

    int yawAverageNum;

    double detect_site_dis;
    double aux_site_dis;
    double forkaction_site_dis;
    double site_site_dis;

    ParamServer(){
        
        nh.param<std::string>("ep_qrcode_loc/logLevel",   logLevel,   "INFO");

        nh.param<std::string>("ep_qrcode_loc/odomMapBase",   odomMapBase,   "ep_qrcode_loc/odometry/base");
        nh.param<std::string>("ep_qrcode_loc/odomMapCamera", odomMapCamera, "ep_qrcode_loc/odometry/locCamera");
        nh.param<std::string>("ep_qrcode_loc/pathMapBase",   pathMapBase,   "ep_qrcode_loc/path/base");
        nh.param<std::string>("ep_qrcode_loc/pathMapCamera", pathMapCamera, "ep_qrcode_loc/path/locCamera");
        nh.param<std::string>("ep_qrcode_loc/odomQrmapBase",   odomQrmapBase,   "ep_qrcode_loc/qrmap/odometry/base");
        nh.param<std::string>("ep_qrcode_loc/odomQrmapCamera", odomQrmapCamera, "ep_qrcode_loc/qrmap/odometry/locCamera");
        nh.param<std::string>("ep_qrcode_loc/msgTopic", msgTopic, "ep_qrcode_loc/msg");
  
        nh.param<int>("ep_qrcode_loc/operating_mode", operating_mode, 3);
        nh.param<bool>("ep_qrcode_loc/show_original_msg", show_original_msg, false);
        nh.param<bool>("ep_qrcode_loc/is_pub_tf", is_pub_tf, false);
        
        nh.param<bool>("ep_qrcode_loc/ignore_area", ignore_area, false);
        nh.param<double>("ep_qrcode_loc/low_speed_UL", low_speed_UL, 0.2);
        nh.param<std::string>("ep_qrcode_loc/port", port, "1024");
        nh.param<std::string>("ep_qrcode_loc/log_dir", log_dir, "/var/xmover/log/QR_code_loc/");
        nh.param<std::string>("ep_qrcode_loc/cfg_dir", cfg_dir, "/var/xmover/params/ep-qrcode-loc/");
        nh.param<std::vector<double>>("ep_qrcode_loc/qrmap2mapTrans", qrmap2mapTrans, std::vector<double>());
        pose_qrmap2mapcopy.position.x = qrmap2mapTrans[0];
        pose_qrmap2mapcopy.position.y = qrmap2mapTrans[1];
        pose_qrmap2mapcopy.position.z = qrmap2mapTrans[2];
        pose_qrmap2mapcopy.orientation.x = qrmap2mapTrans[3];
        pose_qrmap2mapcopy.orientation.y = qrmap2mapTrans[4];
        pose_qrmap2mapcopy.orientation.z = qrmap2mapTrans[5];
        pose_qrmap2mapcopy.orientation.w = qrmap2mapTrans[6];
        nh.param<double>("ep_qrcode_loc/realVelRatio_x", realVelRatio_x, 1.0);
        nh.param<double>("ep_qrcode_loc/realVelRatio_z", realVelRatio_z, 1.0);
        nh.param<double>("ep_qrcode_loc/realVelOffset_x", realVelOffset_x, 0.0);
        nh.param<double>("ep_qrcode_loc/realVelOffset_z", realVelOffset_z, 0.0);
        nh.param<int>("ep_qrcode_loc/yawAverageNum", yawAverageNum, 5);
        nh.param<std::vector<float>>("ep_qrcode_loc/enableArea", enableArea, std::vector<float>());
        nh.param<float>("ep_qrcode_loc/maxEstimationDis", maxEstimationDis, 2.0);

        nh.param<double>("ep_qrcode_loc/detect_site_dis", detect_site_dis, 1.8);
        nh.param<double>("ep_qrcode_loc/aux_site_dis", aux_site_dis, 1.365);
        nh.param<double>("ep_qrcode_loc/forkaction_site_dis", forkaction_site_dis, 0.93);
        nh.param<double>("ep_qrcode_loc/site_site_dis", site_site_dis, 1.36);

    }
};

// 记录服务器
class Logger : public ParamServer
{
private:
    static std::ofstream logFile_;
    static std::mutex mutex;
    std::string loglevel_;
 
    // 私有构造函数确保不能直接创建Logger实例
    Logger() 
    {
        init(log_dir + "/" + format_time(ros::Time::now()) + "qrcode_log.csv");
        loglevel_ = logLevel;
    }
 
    // 防止拷贝构造和赋值
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
 
    // 静态成员函数，用于初始化静态变量
    static void init(std::string log_name) 
    {
        logFile_.open(log_name, std::ios::app);
        if (!logFile_.is_open()) 
        {
            std::cout << "can't open: " << log_name << std::endl;
        }
        else
        {
            std::cout << "open: " << log_name << std::endl;
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
// "FATAL" "ERROR" "WARN" "INFO" "DEBUG"
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

    void debug(const std::string& message) {
        if("DEBUG" == loglevel_)
        {
            std::lock_guard<std::mutex> lock(mutex); // 线程安全
            logFile_ << format_time(ros::Time::now()) << " [DEBUG] " << message << std::endl;
        }
    }
};
std::ofstream Logger::logFile_;
std::mutex Logger::mutex;

// 向量
class vec  
{
public:
	double v[3];
	vec(){}
	vec(double x,double y ,double z)
	{
		 v[0] = x; v[1] = y; v[2] = z;
	}
	const double &operator [] (int i) const
	{
		return v[i];
	}
	double &operator [] (int i)
	{
		return v[i];
	}
};
