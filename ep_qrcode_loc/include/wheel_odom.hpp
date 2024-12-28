#pragma once

#include "utility_qloc.hpp"

// struct wheel_speed
// {
//     ros::Time timestamp_;
//     geometry_msgs::Twist vel_;

//     wheel_speed(ros::Time timestamp, geometry_msgs::Twist vel)
//     {
//         timestamp_ = timestamp;
//         vel_ = vel;
//     }
// };

class WheelSpeedOdometer : public ParamServer
{
private:
    std::mutex mtx;                                     // 互斥锁
    std::mutex init_mtx;                                     // 初值互斥锁
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

public:

    WheelSpeedOdometer(geometry_msgs::TransformStamped trans_camera2base)
    {
        logger = &Logger::getInstance();
        logger->info("WheelSpeedOdometer");
        trans_camera2base_ = trans_camera2base;
        sub_realvel = nh.subscribe<geometry_msgs::Twist>("/real_vel", 1, &WheelSpeedOdometer::realvelCallback,
                                                         this, ros::TransportHints().tcpNoDelay());
        logger->info("sub: /real_vel");
        new_speed_x=0;
        state_ = false;
        path_dis = 0;
        path_dis_overflow = false;

        // 设置地图原点为初值，开始递推
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = 0;
        odom.pose.pose.position.y = 0;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation.x = 0;
        odom.pose.pose.orientation.y = 0;
        odom.pose.pose.orientation.z = 0;
        odom.pose.pose.orientation.w = 1;
        setEstimationInitialPose(odom);
        map_o_init_ = true;
    }

    ~WheelSpeedOdometer() {}

    // 获取/realvel的回调函数
    void realvelCallback(const geometry_msgs::Twist::ConstPtr &p_velmsg)
    {
        logger->debug("realvelCallback");
        ros::Time time_now = ros::Time::now();
        std::lock_guard<std::mutex> locker(mtx);

        geometry_msgs::Twist vel_msg = *p_velmsg;
        geometry_msgs::TwistStamped vel_msg_stamped = decode_msg(vel_msg, time_now);
        static geometry_msgs::TwistStamped vel_msg_stamped_last = vel_msg_stamped;

        // 增加base运动轨迹长度
        double dt = vel_msg_stamped.header.stamp.toSec() - vel_msg_stamped_last.header.stamp.toSec();
        path_dis += dt * abs(vel_msg_stamped.twist.linear.x);
        if(path_dis > maxEstimationDis)
        {
            path_dis_overflow = true;
        }
        vel_msg_stamped_last = vel_msg_stamped;

        speed_data.push_back(vel_msg_stamped);
        new_speed_x = vel_msg_stamped.twist.linear.x;
        new_msg = vel_msg_stamped.twist;

        // 内存保护
        while (speed_data.size() > 5)
        {
            speed_data.pop_front();
            logger->debug("speed_data.pop_front();");
        }
    }

    // 运行轮速递推器
    bool run_odom(std::vector<nav_msgs::Odometry> &v_odom)
    {
        logger->debug("run_odom() : start");
        std::lock_guard<std::mutex> locker(mtx);
        if (0 == speed_data.size())
        {
            logger->debug("run_odom() : 0 == speed_data.size(), return false");
            return false;
        }

        // 获取最新轮速信息
        geometry_msgs::TwistStamped cur_speed = speed_data.front();
        speed_data.pop_front();

        // 估计base当前时刻位姿(map--->base_link)
        nav_msgs::Odometry odom_est = poseEstimation(odom_estimation_init_, cur_speed.twist, cur_speed.header.stamp);

        // 附加数据
        odom_est.header.stamp = cur_speed.header.stamp;// 时间戳
        odom_est.header.frame_id = "map";
        odom_est.child_frame_id = "base_link";

        // 设置标志
        if(path_dis_overflow)
        {
            odom_est.pose.covariance[0] = 0; // 不可用
            odom_est.pose.covariance[3] = 1; // 递推长度超过限制
        }
        else if(map_o_init_)
        {
            odom_est.pose.covariance[0] = 0; // 不可用
            odom_est.pose.covariance[3] = 2; // 使用地图原点初始化递推器
        }
        else
        {
            odom_est.pose.covariance[0] = 1; // 可用
            odom_est.pose.covariance[3] = 0; // 递推长度未超过限制
        }
        odom_est.pose.covariance[4] = path_dis; // 递推距离
        odom_est.pose.covariance[5] = 0; // 数据源：轮速计

        // 设置下一段的初值
        std::lock_guard<std::mutex> init_locker(init_mtx);
        odom_estimation_init_ = odom_est; 

        // 存入base姿态
        v_odom.clear();
        v_odom.push_back(odom_est);

        // 存入相机姿态
        geometry_msgs::Pose pose_camera2map;
        tf2::doTransform(t2p(trans_camera2base_), pose_camera2map, p2t(odom_est.pose.pose));
        odom_est.pose.pose = pose_camera2map;
        odom_est.child_frame_id = "locCamera_link";
        v_odom.push_back(odom_est);

        logger->debug("run_odom() : return true");
        return true;
    }

    // 获取车轮速度
    double get_vel_x()
    {
        std::lock_guard<std::mutex> locker(mtx);
        return new_speed_x;
    }

    // 获取最新/real_vel消息
    geometry_msgs::Twist get_vel_msg()
    {
        std::lock_guard<std::mutex> locker(mtx);
        return new_msg;
    }

    // 是否递推路径过长
    bool is_path_dis_overflow()
    {
        std::lock_guard<std::mutex> locker(mtx);
        return path_dis_overflow;
    }

    // 递推距离归零
    void reset_path_dis()
    {
        logger->debug("reset_path_dis");
        std::lock_guard<std::mutex> locker(mtx);
        path_dis = 0;
        path_dis_overflow = false;
        return;
    }
    
    // 使用二维码定位结果设置递推初值
    void setEstimationInitialPose(nav_msgs::Odometry odom)
    {
        logger->debug("setEstimationInitialPose");
        std::lock_guard<std::mutex> locker(mtx);
        odom_estimation_init_ = odom;
        path_dis = 0;
        state_ = true;
        map_o_init_ = false;
    }

    // 获取递推初值
    nav_msgs::Odometry getCurOdom()
    {
        std::lock_guard<std::mutex> locker(init_mtx);
        return odom_estimation_init_;
    }
    
    // 启动轮速递推器
    void start()
    {
        std::lock_guard<std::mutex> locker(mtx);
        state_ = true;
    }

    // 确认轮速递推器启动状态
    bool is_start()
    {
        std::lock_guard<std::mutex> locker(mtx);
        return state_;
    }

    // 关闭轮速递推器
    void close()
    {
        std::lock_guard<std::mutex> locker(mtx);
        state_ = false;
    }

private:

    // 根据初值、速度、时间进行位姿递推
    nav_msgs::Odometry poseEstimation(nav_msgs::Odometry odom_init, geometry_msgs::Twist vel, ros::Time time_now)
    {
        logger->debug("poseEstimation");
        nav_msgs::Odometry odom_final;

        // 计算时间间隔，更新时间戳
        double dt = (time_now - odom_init.header.stamp).toSec();

        // 方向递推
        double yaw = getYawRad(odom_init.pose.pose.orientation) + vel.angular.z * dt;
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        tf::quaternionTFToMsg(q, odom_final.pose.pose.orientation);

        // 位置递推
        odom_final.pose.pose.position.x = odom_init.pose.pose.position.x + vel.linear.x * dt * cos(yaw);
        odom_final.pose.pose.position.y = odom_init.pose.pose.position.y + vel.linear.x * dt * sin(yaw);

        // 返回值
        return odom_final;
    }

    // 解析轮速数据，前主动轮模型
    geometry_msgs::TwistStamped decode_msg(geometry_msgs::Twist vel_msg, ros::Time time)
    {   
        logger->debug("decode_msg");
        geometry_msgs::TwistStamped vel_new;

        // vel_msg.linear.x  :base_link实际线速度 (m/s)
        // vel_msg.linear.y  :舵轮目标线速度 (m/s)
        // vel_msg.linear.z  :舵轮电机的实际转速 (rpm)
        // vel_msg.angular.x :舵轮的目标角度 (度)
        // vel_msg.angular.y :舵轮实际角度 (度)
        // vel_msg.angular.z :base_link实际角速度 (rad/s)

        // 获取时间
        vel_new.header.stamp = time;
        // 保存原始数据
        vel_new.twist = vel_msg;
        // 轮子角速度：弧度
        double wheel_angular = (vel_msg.angular.y + wheel_angular_offset) * M_PI / 180;
        // 轮子进退电机转速 rpm
        double wheel_moter_speed = vel_msg.linear.z;
        // 轮子速度 m/s
        double wheel_vel = M_PI * wheel_diameter * wheel_moter_speed / (wheel_reduction_ratio * 60.0);
        // base线速度 m/s
        double base_vel_x = 0;
        // base角速度 red/s
        double base_vel_yaw = 0;

        // 根据轮子转角大小分情况解析
        if (abs(wheel_angular) < 0.001) //角度过小
        {
            base_vel_x = wheel_vel;
            base_vel_yaw = 0;
        }
        else if (abs(wheel_angular) < 1.570) // 一般角度
        {
            base_vel_yaw = wheel_vel * std::sin(wheel_angular) / wheel_base_dis;
            base_vel_x = base_vel_yaw * (wheel_base_dis / std::tan(wheel_angular));
        }
        else // 角度过大
        {
            if (wheel_angular > 0)
            {
                base_vel_yaw = wheel_vel / wheel_base_dis;
            }
            else
            {
                base_vel_yaw = -1 * wheel_vel / wheel_base_dis;
            }
            base_vel_x = 0;
        }

        // 结果存入结构体
        vel_new.twist.linear.x = base_vel_x;    // base线速递
        vel_new.twist.linear.y = wheel_vel;     // 轮子速度
        vel_new.twist.angular.z = base_vel_yaw; // base角速度

        // 返回值
        return vel_new;
    }

};
