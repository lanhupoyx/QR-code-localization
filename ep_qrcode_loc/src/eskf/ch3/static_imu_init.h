//
// Created by xiang on 2021/11/11.
//

#ifndef SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H
#define SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/odom.h"

#include <deque>

namespace sad {

/**
 * IMU水平静止状态下初始化器
 * 使用方法：调用AddIMU, AddOdom添加数据，使用InitSuccess获取初始化是否成功
 * 成功后，使用各Get函数获取内部参数
 *
 * 初始化器在每次调用AddIMU时尝试对系统进行初始化。在有odom的场合，初始化要求odom轮速读数接近零；没有时，假设车辆初期静止。
 * 初始化器收集一段时间内的IMU读数，按照书本3.5.4节估计初始零偏和噪声参数，提供给ESKF或者其他滤波器
 */
class StaticIMUInit {
   public:
    struct Options {
        Options() {}
        double init_time_seconds_ = 10.0;     // 静止时间
        int init_imu_queue_max_size_ = 2000;  // 初始化IMU队列最大长度
        int static_odom_pulse_ = 5;           // 静止时轮速计输出噪声
        double static_odom_vel_ = 0.0001;     // 静止时轮速计输出噪声
        double max_static_gyro_var = 0.5;     // 静态下陀螺测量方差
        double max_static_acce_var = 0.05;    // 静态下加计测量方差
        double gravity_norm_ = 9.81;          // 重力大小
        bool use_speed_for_static_checking_ = true;  // 是否使用odom来判断车辆静止（部分数据集没有odom选项）
    };

    /// 构造函数
    StaticIMUInit(Options options = Options()) : options_(options) {
        SetPara();
    }

    /// 添加IMU数据
    bool AddIMU(const IMU& imu);
    /// 添加轮速数据
    bool AddOdom(const Odom& odom);

    /// 判定初始化是否成功
    bool InitSuccess() const { return init_success_; }

    /// 获取各Cov, bias, gravity
    Vec3d GetCovGyro() const { return cov_gyro_; }
    Vec3d GetCovAcce() const { return cov_acce_; }
    Vec3d GetInitBg() const { return init_bg_; }
    Vec3d GetInitBa() const { return init_ba_; }
    Vec3d GetGravity() const { return gravity_; }

    bool SetPara(){

        //91号车
        cov_gyro_ << 7.96204e-07, 1.79635e-06, 4.65166e-07;  // 陀螺测量噪声协方差（初始化时评估）
        cov_acce_ << 6.55993e-05, 8.35848e-05, 7.43041e-05;  // 加计测量噪声协方差（初始化时评估）
        init_bg_  << -0.000630219, 0-0.00102344, 03.15111e-05;   // 陀螺初始零偏
        init_ba_  << 0-0.00110075, -0.000288036, 000-0.102252;   // 加计初始零偏
        gravity_  << 0-0.105599, -0.0276324, 00-9.80939;   // 重力
        init_success_ = true;
        return true;

        // //15号车滤波前
        // cov_gyro_ << 5.68144e-07, 1.44988e-06, 9.07988e-07;  // 陀螺测量噪声协方差（初始化时评估）
        // cov_acce_ << 7.01235e-05, 8.44337e-05, 7.93104e-05;  // 加计测量噪声协方差（初始化时评估）
        // init_bg_  << 00.000309102, -0.000234087, 02.00943e-06;   // 陀螺初始零偏
        // init_ba_  << 00.00448489, -0.00173519, 00-0.160523;   // 加计初始零偏
        // gravity_  << 00.273961, -0.105995, 00-9.8056;   // 重力
        // init_success_ = true;
        // return true;

        // //15号车滤波后
        // cov_gyro_ << 5.68144e-07, 1.44988e-06, 9.07988e-07;  // 陀螺测量噪声协方差（初始化时评估）
        // cov_acce_ << 7.01235e-05, 8.44337e-05, 7.93104e-05;  // 加计测量噪声协方差（初始化时评估）
        // init_bg_  << 2.950802505353335e-04, -2.313698065667380e-04, 1.724505710206993e-05;   // 陀螺初始零偏
        // init_ba_  << -0.269272875089220, 0.103490313918629, 9.644994354032772;   // 加计初始零偏
        // gravity_  << 00.273961, -0.105995, 00-9.8056;   // 重力
        // init_success_ = true;
        // return true;
    }

   private:
    /// 尝试对系统初始化
    bool TryInit();

    Options options_;                 // 选项信息
    bool init_success_ = false;       // 初始化是否成功
    Vec3d cov_gyro_ = Vec3d::Zero();  // 陀螺测量噪声协方差（初始化时评估）
    Vec3d cov_acce_ = Vec3d::Zero();  // 加计测量噪声协方差（初始化时评估）
    Vec3d init_bg_ = Vec3d::Zero();   // 陀螺初始零偏
    Vec3d init_ba_ = Vec3d::Zero();   // 加计初始零偏
    Vec3d gravity_ = Vec3d::Zero();   // 重力
    bool is_static_ = false;          // 标志车辆是否静止
    std::deque<IMU> init_imu_deque_;  // 初始化用的数据
    double current_time_ = 0.0;       // 当前时间
    double init_start_time_ = 0.0;    // 静止的初始时间
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H
