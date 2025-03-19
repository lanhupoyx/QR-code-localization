#pragma once
#include "utility_qloc.hpp"

class QRcodeColumn
{
public:
    uint32_t index;                    // 库位列编号
    geometry_msgs::Pose first_pose;    // 列首地码位姿
    double space;                      // 地码间距
    std::vector<QRcodeGround> qrcodes; // 对应二维码
    double yawOffset; //所有地码均增加的
    double pose_x,pose_y,pose_yaw; // 原始pose数据


public:
    QRcodeColumn(uint32_t index_, double x_, double y_, double yaw_, double space_, double yawOffset_); 

    ~QRcodeColumn();

    void addQRcode(uint32_t code_index, double x_offset, double y_offset, double yaw_offset);

    geometry_msgs::Pose poseMove(geometry_msgs::Pose pose, double dis_front, double dis_left);
};



// 二维码坐标对照表
class QRcodeTableV3
{
private:
    std::mutex mtx;
    epLogger *logger;
    ParamServer& param;

    std::vector<QRcodeColumn> columns; // 存放地码初始信息
    
    std::map<uint32_t, QRcodeInfo> map; // 最终生成的二维码位姿表

    ros::Subscriber sub_pos;
    std::list<nav_msgs::Odometry> tf_buffer;
    

public:
    QRcodeTableV3(ParamServer& param);

    ~QRcodeTableV3();

    bool loadCodeTable();

    // 根据二维码编号查表，得到位姿信息
    bool onlyfind(CameraFrame frame, QRcodeInfo *info);

    // 根据二维码编号查表，得到位姿信息
    bool onlyfind(CameraFrame frame);

    // callback获取baselink位姿
    void tfCallback(const nav_msgs::Odometry::ConstPtr &msg);

    // 最新基于lidar的baselink位姿
    nav_msgs::Odometry getCurLidarPose();

    // 获取所有列首二维码编号
    std::vector<uint32_t> get_all_head();

    // 检查是否为列首地码
    bool is_head(uint32_t code_new);

    // 获取前后二维码
    std::vector<uint32_t> get_neighbor(uint32_t base_code);

    // 是否在列内
    bool isAGVInQueue(nav_msgs::Odometry base2map, double head_offset);

    // 是否在矩形内
    bool isPointInColumn(const geometry_msgs::Pose point, const QRcodeColumn& column, double offset_x, double range_y);

    // 平面坐标系变换，将点从坐标系A转换到坐标系B
    geometry_msgs::Pose transformPoint(const geometry_msgs::Pose pointA, double T_x, double T_y, double theta);

    void test();
};
