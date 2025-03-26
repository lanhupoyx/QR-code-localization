#pragma once
#include "utilityQloc.hpp"
#include "ParamServer.hpp"
#include "logger.hpp"

/// @brief 地码表类，用于处理地码表相关功能
class QRcodeTable
{
protected:
    std::mutex mtx;
    epLogger *logger;
    ParamServer &param;

    ros::Subscriber sub_pos;
    std::list<nav_msgs::Odometry> tf_buffer;

    std::map<uint32_t, QRcodeInfo> map; // 最终生成的二维码位姿表

    // 1、加载数据
    // 2、basePose

public:
    /// @brief QRcodeTableV3构造函数
    /// @param param 参数管理器对象
    QRcodeTable(ParamServer &param);

    /// @brief QRcodeTableV3析构函数
    ~QRcodeTable();

    // virtual bool is_head(uint32_t code_new) = 0;
    // virtual std::vector<uint32_t> get_neighbor(uint32_t base_code) = 0;
    // virtual bool check_is_code_in_order(uint32_t code_new, double vel_x, bool reset = false)=0;

    // bool find_add(CameraFrame frame, QRcodeInfo *info);
    // bool add(const CameraFrame frame);
    // void tfCallback(const nav_msgs::Odometry::ConstPtr &msg);
    // void subPose();
    // QRcodeInfo calPose();
    // bool correct_yaw(uint32_t code, double yaw_err);
    // void readYawErr(std::string cfg_path);
    // nav_msgs::Odometry getCurLidarPose();
    // std::vector<uint32_t> get_all_head();
    // bool is_in_queue(nav_msgs::Odometry base2map, double head_offset);
    // bool onlyfind(CameraFrame frame, QRcodeInfo *info);
    // bool onlyfind(CameraFrame frame);
    // bool isAGVInQueue(double head_offset);
    
};
