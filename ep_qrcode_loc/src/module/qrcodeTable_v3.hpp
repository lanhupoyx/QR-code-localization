#pragma once
#include "utilityQloc.hpp"
#include "ParamServer.hpp"
#include "logger.hpp"

/// @brief 地码列类，用于存储一列地码的信息
class QRcodeColumn
{
public:
    uint32_t index;                    // 库位列编号
    geometry_msgs::Pose first_pose;    // 列首地码位姿
    double space;                      // 地码间距
    std::vector<QRcodeGround> qrcodes; // 对应二维码
    double yawOffset;                  // 所有地码均增加的方向角补偿
    double pose_x, pose_y, pose_yaw;   // 原始pose数据

public:
    /// @brief 构造函数
    /// @param index_ 库位列编号
    /// @param x_ 列首地码pose-x
    /// @param y_ 列首地码pose-y
    /// @param yaw_ 列首地码pose-yaw
    /// @param space_ 地码间距
    /// @param yawOffset_ 所有地码均增加的方向角补偿
    QRcodeColumn(uint32_t index_, double x_, double y_, double yaw_, double space_, double yawOffset_);

    /// @brief 析构函数
    ~QRcodeColumn();

    /// @brief QRcodeColumn地码列类：依次添加新的地码，从列首到列尾
    /// @param code_index 地码编号
    /// @param row_index 地码行号
    /// @param x_offset 位姿补偿x
    /// @param y_offset 位姿补偿y
    /// @param yaw_offset 位姿补偿yaw
    void addQRcode(uint32_t code_index, uint32_t row_index, double x_offset, double y_offset, double yaw_offset);

    /// @brief QRcodeColumn地码列类：pose平移
    /// @param pose 目标pose
    /// @param dis_front 前后移动距离，正前负后
    /// @param dis_left 前后移动距离，正左负右
    /// @return 平移后的pose
    geometry_msgs::Pose poseMove(geometry_msgs::Pose pose, double dis_front, double dis_left);
};

/// @brief 地码表类，用于处理地码表相关功能
class QRcodeTableV3
{
private:
    std::mutex mtx;
    epLogger *logger;
    ParamServer &param;

    std::vector<QRcodeColumn> columns;  // 存放地码初始信息
    std::map<uint32_t, QRcodeInfo> map; // 最终生成的二维码位姿表

    ros::Subscriber sub_pos;
    std::list<nav_msgs::Odometry> tf_buffer;

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

public:
    /// @brief QRcodeTableV3构造函数
    /// @param param 参数管理器对象
    QRcodeTableV3(ParamServer &param);

    /// @brief QRcodeTableV3析构函数
    ~QRcodeTableV3();

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
