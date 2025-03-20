#pragma once

#include "utility_qloc.hpp"
#include "wheel_odom.hpp"
#include "camera.hpp"
#include "qrcode_table.hpp"
#include "qrcode_table_v2.hpp"
#include "qrcode_table_v3.hpp"
#include "BasicState.h"

// 偏差值信息
struct err_val
{
    double err_;
    uint32_t num_;

    err_val(double err, uint32_t num);
};

// 二维码定位
class QRcodeLoc
{
public:

    // 互斥锁
    std::mutex mtx;

    // 记录器
    epLogger *logger;

    // ros发布器、接收器
    ros::Publisher pub_odom_map_base;
    ros::Publisher pub_odom_map_camera;
    ros::Publisher pub_path_map_base;
    ros::Publisher pub_path_map_camera;
    ros::Publisher pub_qrCodeMsg;
    ros::Subscriber sub_BasicState;      // /xmover_basic_state消息订阅器

    // tf转换相关
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener *tfListener;
    geometry_msgs::TransformStamped trans_base2camera;
    geometry_msgs::TransformStamped trans_camera2base;
    geometry_msgs::TransformStamped trans_base2map;
    geometry_msgs::TransformStamped trans_camera2map;
    tf::TransformBroadcaster br;

    // 功能对象
    ParamServer& param;
    MV_SC2005AM *camera;
    QRcodeTableV2 *qrcode_table;
    WheelSpeedOdometer *wheel_odom;
    
    //相机当前frame数据
    CameraFrame pic; 
    
    // 是否手动状态
    bool is_handle;

public:
    // 构造函数
    QRcodeLoc(ParamServer &param, MV_SC2005AM *camera);
    ~QRcodeLoc();

    void getTrans_BaseToCamera();

    // 获取/xmover_basic_state的回调函数
    void BasicStateCallback(const xmover_msgs::BasicState::ConstPtr &p_base_state_msg);

    // 发布定位结果
    void pubOdom(std::vector<nav_msgs::Odometry> v_odom);

    // 发布TF
    void pubTf(const nav_msgs::Odometry odom);

    // 计算base_link在map坐标系下的坐标
    geometry_msgs::Pose get_pose_lidarmap(CameraFrame pic, QRcodeInfo code_info);

    // 计算base_link在qrmap和map坐标系下的坐标
    std::vector<geometry_msgs::Pose> get_pose(QRcodeInfo code_info);

    // pose1 预测值，pose2 观测值
    geometry_msgs::Pose kalman_f_my(geometry_msgs::Pose pose_recursion, double p1, 
                                    geometry_msgs::Pose pose_observe,   double p2, 
                                    double dis_U);

    // 车身方向角是否在允许识别二维码的范围内
    bool is_yaw_available(geometry_msgs::Quaternion q, double yaw_des, double range);

    // 打包需要输出的消息
    std::vector<nav_msgs::Odometry> packageMsg(std::vector<geometry_msgs::Pose> pose, QRcodeInfo code_info);

    // 扫到码时，判断是否要需要输出这一帧
    bool do_not_jump_this_frame(CameraFrame pic_new, bool reset = false);

    // 判断是否跳变过大
    bool is_pose_jump(geometry_msgs::Pose pose_last, geometry_msgs::Pose pose_now);

    // 是否按顺序扫码
    bool is_code_in_order(uint32_t code_new, double vel_x, bool reset = false);

    // 输出记录
    void output_log(CameraFrame pic_latest, 
                    QRcodeInfo code_info, 
                    geometry_msgs::Twist wheel_msg,
                    std::vector<nav_msgs::Odometry> publist_front);

    /// @brief 输出扫码跳变数据到指定文件
    /// @param output
    /// @param output_last
    void output_jump_err(QRcodeInfo code_info,
                         std::vector<nav_msgs::Odometry> output,
                         std::vector<nav_msgs::Odometry> output_last);

//-------------------------------------------各种运行模式----------------------------------------------//


    // // 模式:采集二维码位姿
    // void CollectQRCodePose_mode();

    // // 模式:采集二维码编号
    // void CollectQRCodeIndex_mode();

    // // 模式:正常运行
    // void NormalRun_mode();
        
    // // 模式:正常运行v4
    // void NormalRun_mode_v4();

    // // 模式:测试
    // void TestRun_mode();

    // // 模式:采集地码角度
    // void GetYaw_mode();

    // // 模式:补偿地码角度
    // void CalYawErr_mode();

    // // 模式:检查相机水平
    // void CheckCameraHorizon_mode();

    // // 模式:代码测试
    // void CodeDebug_mode();

    // // 模式:辅助驾驶
    // void AssistedDriving_mode();

    // // 主循环
    // void mainloop();
    
    // 循环
    virtual void loop()=0;
};

