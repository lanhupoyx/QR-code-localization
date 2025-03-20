#pragma once
#include "utility_qloc.hpp"
#include "ParamServer.hpp"
#include "logger.hpp"

// 库位
struct Site
{
    uint32_t list_index_;      // 所在列编号
    uint32_t index_;           // 库位编号
    geometry_msgs::Pose pose_; // 位姿

    QRcodeGround detect_point_qrcode_;
    QRcodeGround aux_point_qrcode_;
    QRcodeGround action_point_qrcode_;

    Site(uint32_t list_index,                               // 所在列编号
         uint32_t index,                                    // 库位编号
         geometry_msgs::Pose pose,                          // 库位位姿
         std::vector<QRcodeGround> qrcodes,                 // 对应二维码
         std::vector<double> dis_vec,                       // 三个距离参数
         geometry_msgs::TransformStamped trans_base_camera); // 相机到base的变换

    ~Site();

    geometry_msgs::Pose move(double dis_front, double dis_left);


};

// 库位列
struct SiteList
{
    uint32_t index_; // 库位列编号
    geometry_msgs::Pose first_pose_; // 列首库位位姿
    std::vector<double> dis_vec_;
    double site_dis_;
    geometry_msgs::TransformStamped trans_base_camera_;
    std::list<Site> sites_;                    // 列内的库位
    std::list<QRcodeGround> front_aux_points_; // 列首的辅助二维码
    geometry_msgs::Pose pose_cross_;           // 轴线与主干道垂直交点

    SiteList(uint32_t index, geometry_msgs::Pose first_pose, std::vector<double> dis_vec, geometry_msgs::TransformStamped trans_base_camera);

    ~SiteList();

    void set_first_site(geometry_msgs::Pose pose, std::vector<QRcodeGround> qrcodes);

    void add_site(std::vector<QRcodeGround> qrcodes);

    void add_aux_points(std::vector<QRcodeGround> qrcodes);

    geometry_msgs::Pose poseMove(geometry_msgs::Pose pose, double dis_front, double dis_left);
};

// 单列地码列表
struct ColumnCodeList
{
    uint32_t column_index_;     // 所在列编号
    std::list<uint32_t> codes_; // 地码编号列表
};

// 二维码坐标对照表
class QRcodeTableV2
{
private:
    std::vector<SiteList> siteList_lib; // 各个列
    std::map<uint32_t, QRcodeInfo> map; // 最终生成的二维码位姿表

    std::mutex mtx;

    epLogger *logger;
    std::stringstream stream;

    std::string cfg_path_;

    ros::Subscriber sub_pos;
    std::list<nav_msgs::Odometry> tf_buffer;
    std::list<ColumnCodeList> ground_codes;
    ParamServer& param;

public:
    QRcodeTableV2(std::string cfg_path, geometry_msgs::TransformStamped trans_base_camera, ParamServer& param);

    ~QRcodeTableV2();

    // 根据二维码编号查表，得到位姿信息
    bool onlyfind(CameraFrame frame, QRcodeInfo *info);

    // 根据二维码编号查表，得到位姿信息
    bool onlyfind(CameraFrame frame);

    // 校正单个地码方向角
    bool correct_yaw(uint32_t code, double yaw_err);

    // 读取地码方向角补偿数据
    void readYawErr(std::string cfg_path);

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
    bool is_in_queue(nav_msgs::Odometry base2map, double head_offset);
};
