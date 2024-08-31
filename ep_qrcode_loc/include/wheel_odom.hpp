#pragma once

#include "utility_qloc.hpp"

struct wheel_speed
{
    ros::Time timestamp_;
    geometry_msgs::Twist vel_;

    wheel_speed(ros::Time timestamp, geometry_msgs::Twist vel)
    {
        timestamp_ = timestamp;
        vel_ = vel;
    }
};

class WheelSpeedOdometer : public ParamServer
{
private:
    geometry_msgs::TransformStamped trans_camera2base_;
    nav_msgs::Odometry odom_estimation_init_;
    std::list<wheel_speed> speed_data;
    ros::Subscriber sub_realvel;
    std::mutex mtx;

private:
    // 获取递推初始值
    nav_msgs::Odometry getEstimationInitialPose()
    {
        std::lock_guard<std::mutex> locker(mtx);
        return odom_estimation_init_;
    }

public:
    WheelSpeedOdometer(geometry_msgs::TransformStamped trans_camera2base)
    {
        trans_camera2base_ = trans_camera2base;
        sub_realvel = nh.subscribe<geometry_msgs::Twist>("/real_vel", 1, &WheelSpeedOdometer::realvelCallback,
                                                         this, ros::TransportHints().tcpNoDelay());
    }

    ~WheelSpeedOdometer() {}

    // 设置递推初始值
    void setEstimationInitialPose(nav_msgs::Odometry odom)
    {
        std::lock_guard<std::mutex> locker(mtx);
        odom_estimation_init_ = odom;
    }

    // 使用轮速进行位姿递推
    nav_msgs::Odometry poseEstimation(nav_msgs::Odometry odom_init, geometry_msgs::Twist vel, ros::Time time_now)
    {
        nav_msgs::Odometry odom_final;

        // 计算时间间隔，更新时间戳
        double dt = (time_now - odom_init.header.stamp).toSec();

        // 方向递推
        double yaw = getYawRad(odom_init.pose.pose.orientation) + (vel.angular.z + realVelOffset_z) * realVelRatio_z * dt;
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        tf::quaternionTFToMsg(q, odom_final.pose.pose.orientation);

        // 位置递推
        odom_final.pose.pose.position.x = odom_init.pose.pose.position.x + (vel.linear.x + realVelOffset_x) * realVelRatio_x * dt * cos(yaw);
        odom_final.pose.pose.position.y = odom_init.pose.pose.position.y + (vel.linear.x + realVelOffset_x) * realVelRatio_x * dt * sin(yaw);

        // 数据来源
        odom_final.pose.covariance = odom_init.pose.covariance;
        odom_final.pose.covariance[2] = 1; // 此帧数据来源，0：二维码 1：轮速计递推

        // 时间戳
        odom_final.header.stamp = time_now;
        odom_final.header.frame_id = "map";
        odom_final.child_frame_id = "base_link";

        // 返回值
        return odom_final;
    }

    // 获取/realvel的回调函数
    void realvelCallback(const geometry_msgs::Twist::ConstPtr &velmsg)
    {
        std::lock_guard<std::mutex> locker(mtx);
        wheel_speed new_speed(ros::Time::now(), *velmsg);
        speed_data.push_back(new_speed);
    }

    bool run_odom(std::vector<nav_msgs::Odometry> &v_odom)
    {
        if (0 == speed_data.size())
        {
            return false;
        }

        // 获取速度信息
        wheel_speed cur_speed = speed_data.front();
        speed_data.pop_front();

        // 获取初始位姿
        nav_msgs::Odometry odom_init = getEstimationInitialPose();

        // 估计base当前时刻位姿(map--->base_link)
        nav_msgs::Odometry odom_est = poseEstimation(odom_init, cur_speed.vel_, cur_speed.timestamp_);
        setEstimationInitialPose(odom_est);
        v_odom.clear();
        v_odom.push_back(odom_est);

        // 计算定位相机当前时刻位姿(map--->locCamera_link)
        geometry_msgs::Pose pose_camera2map;
        tf2::doTransform(t2p(trans_camera2base_), pose_camera2map, p2t(odom_est.pose.pose));
        odom_est.pose.pose = pose_camera2map;
        odom_est.child_frame_id = "locCamera_link";
        v_odom.push_back(odom_est);

        return true;
    }

    double get_vel_x()
    {
        if(0 != speed_data.size())
        {
            return speed_data.back().vel_.linear.x;
        }
        else
        {
            return 0;
        }
    }
};
