#pragma once
#include "utility_qloc.hpp"

class WheelSpeedOdometer
{
private:
    std::mutex mtx;                                     // 互斥锁
    std::mutex init_mtx;                                // 初值互斥锁
    Logger *logger;                                     // 计录器
    bool state_;                                        // 轮速递推器状态
    bool map_o_init_;                                   // 刚上电时，使用地图原点初始化递推器
    geometry_msgs::TransformStamped trans_camera2base_; // 相机到base的变换
    nav_msgs::Odometry odom_estimation_init_;           // 轮速递推器初值
    std::list<geometry_msgs::TwistStamped> speed_data;  // 轮速数据缓存队列
    geometry_msgs::TwistStamped speed_data_new_;        // 最新轮速数据
    std::mutex speed_data_new_mtx;                      // 最新轮速数据互斥锁
    double new_speed_x;                                 // 最新线速度-base
    geometry_msgs::Twist new_msg;                       // 最新/real_vel消息
    ros::Subscriber sub_realvel;                        // /real_vel消息订阅器
    double path_dis;                                    // 本段递推中轮子走过的路径长度
    bool path_dis_overflow;                             // 递推路径过长
    ParamServer &param;

public:
    WheelSpeedOdometer(geometry_msgs::TransformStamped trans_camera2base, ParamServer &param);

    ~WheelSpeedOdometer();

    // 获取/realvel的回调函数
    void realvelCallback(const geometry_msgs::Twist::ConstPtr &p_velmsg);

    // 运行轮速递推器
    bool run_odom(std::vector<nav_msgs::Odometry> &v_odom);

    // 获取车轮速度
    double get_vel_x();

    // 获取最新/real_vel消息
    geometry_msgs::Twist get_vel_msg();

    // 是否递推路径过长
    bool is_path_dis_overflow();

    // 递推距离归零
    void reset_path_dis();

    // 使用二维码定位结果设置递推初值
    void setEstimationInitialPose(nav_msgs::Odometry odom);

    // 获取递推初值
    nav_msgs::Odometry getCurOdom();

    // 启动轮速递推器
    void start();

    // 确认轮速递推器启动状态
    bool is_start();

    // 关闭轮速递推器
    void close();

private:
    // 根据初值、速度、时间进行位姿递推
    nav_msgs::Odometry poseEstimation(nav_msgs::Odometry odom_init, geometry_msgs::Twist vel, ros::Time time_now);

    // 解析轮速数据，前主动轮模型
    geometry_msgs::TwistStamped decode_msg(geometry_msgs::Twist vel_msg, ros::Time time);
};
