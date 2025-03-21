#pragma once
#include "utilityQloc.hpp"
#include "ParamServer.hpp"
#include "logger.hpp"


/// @brief 地码表类，用于处理地码表相关功能
class QRcodeTable
{
public:
    std::mutex mtx;
    epLogger *logger;
    ParamServer &param;

    std::map<uint32_t, QRcodeInfo> map; // 最终生成的二维码位姿表

    ros::Subscriber sub_pos;
    std::list<nav_msgs::Odometry> tf_buffer;

    // 1、加载数据
    // 2、basePose


    /// @brief 加载地码信息文件
    /// @return 是否成功
    bool loadCodeTable();

    /// @brief 订阅lidar pose的回调函数
    /// @param msg nav_msgs::Odometry消息
    void lidarPoseCallback(const nav_msgs::Odometry::ConstPtr &msg);

    /// @brief 获取最新相机pose
    /// @return 相机pose
    geometry_msgs::Pose getCameraPose();

    /// @brief 获取所有列首地码编号
    /// @return 列首地码编号集合vector
    std::vector<uint32_t> get_all_head();

    /// @brief 找到本地码前后的邻居地码
    /// @param base_code 本地码
    /// @return 邻居地码vector
    std::vector<uint32_t> get_neighbor(uint32_t base_code);

    /// @brief 判断一个点是否在列范围内
    /// @param point 目标点
    /// @param column 目标列
    /// @param offset_x 列首偏移
    /// @param range_y 横向范围
    /// @return bool
    bool isPointInColumn(const geometry_msgs::Pose point, const QRcodeColumn &column, double offset_x, double range_y);

    /// @brief 平面坐标系变换，将点从坐标系A转换到坐标系B
    /// @param pointA 坐标系A中的点
    /// @param T_x 坐标系A在B中的位置x
    /// @param T_y 坐标系A在B中的位置x
    /// @param theta 坐标系A在B中的方向角
    /// @return point在坐标系B中的位姿
    geometry_msgs::Pose transformPoint(const geometry_msgs::Pose pointA, double T_x, double T_y, double theta);

    /// @brief 测试函数
    void test();

    /// @brief QRcodeTableV3构造函数
    /// @param param 参数管理器对象
    QRcodeTable(ParamServer &param);

    /// @brief QRcodeTableV3析构函数
    ~QRcodeTable();

    /// @brief 根据二维码编号查表，得到位姿信息
    /// @param frame 相机帧
    /// @param info 查询到的地码信息
    /// @return 是否成功
    bool onlyfind(CameraFrame frame, QRcodeInfo *info);

    /// @brief 根据二维码编号查表
    /// @param frame 相机帧
    /// @return 该地码是否在表格内
    bool onlyfind(CameraFrame frame);

    /// @brief 检查是否为列首地码
    /// @param code_new 地码编号
    /// @return 结果
    bool is_head(uint32_t code_new);

    /// @brief 判断AGV是否在列（可运行区域）内
    /// @param head_offset 列首偏移量，基于第一个地码的位置，正前负后
    /// @return bool
    bool isAGVInQueue(double head_offset);

    /// @brief 是否在按顺序扫码
    /// @param code_new 新识别到的地码
    /// @param vel_x 轮速x
    /// @param reset 是否重置为初始状态
    /// @return bool
    bool is_code_in_order(uint32_t code_new, double vel_x, bool reset = false);
};
