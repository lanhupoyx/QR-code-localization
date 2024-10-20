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
    double new_speed_x;
    geometry_msgs::Twist new_msg;
    ros::Subscriber sub_realvel;
    std::mutex mtx;
    Logger *logger;
    bool state_;

public:
    WheelSpeedOdometer(geometry_msgs::TransformStamped trans_camera2base)
    {
        trans_camera2base_ = trans_camera2base;
        sub_realvel = nh.subscribe<geometry_msgs::Twist>("/real_vel", 1, &WheelSpeedOdometer::realvelCallback,
                                                         this, ros::TransportHints().tcpNoDelay());
        new_speed_x=0;
        state_ = false;
        logger = &Logger::getInstance();
    }

    ~WheelSpeedOdometer() {}

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

    // 获取递推状态值
    nav_msgs::Odometry getCurOdom()
    {
        std::lock_guard<std::mutex> locker(mtx);
        return odom_estimation_init_;
    }

    // 设置递推初始值
    void setEstimationInitialPose(nav_msgs::Odometry odom)
    {
        std::lock_guard<std::mutex> locker(mtx);
        odom_estimation_init_ = odom;
    }

    // 设置递推状态值
    void setCurOdom(nav_msgs::Odometry odom)
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
        double yaw = getYawRad(odom_init.pose.pose.orientation) + vel.angular.z * realVelRatio_z * dt + realVelAdd_z;
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        tf::quaternionTFToMsg(q, odom_final.pose.pose.orientation);

        // 位置递推
        odom_final.pose.pose.position.x = odom_init.pose.pose.position.x + (vel.linear.x + realVelOffset_x) * realVelRatio_x * dt * cos(yaw);
        odom_final.pose.pose.position.y = odom_init.pose.pose.position.y + (vel.linear.x + realVelOffset_x) * realVelRatio_x * dt * sin(yaw);

        // 数据来源
        odom_final.pose.covariance = odom_init.pose.covariance;

        // 时间戳
        odom_final.header.stamp = time_now;
        odom_final.header.frame_id = "map";
        odom_final.child_frame_id = "base_link";

        // 返回值
        return odom_final;
    }

    // 获取/realvel的回调函数
    void realvelCallback(const geometry_msgs::Twist::ConstPtr &p_velmsg)
    {
        logger->debug("realvelCallback() : start");
        std::lock_guard<std::mutex> locker(mtx);

        if(state_)
        {
            geometry_msgs::Twist vel_msg = *p_velmsg;

            logger->debug("realvelCallback(): vel_msg: " 
                        + ' ' + std::to_string(vel_msg.linear.x)
                        + ' ' + std::to_string(vel_msg.angular.z));

            // 自行车模型
            // geometry_msgs::Twist vel_new;
            // double wheel_angular = (vel_msg.angular.y + wheel_angular_offset) * M_PI / 180;
            // double wheel_rpm = vel_msg.angular.y;
            // double wheel_vel = M_PI * wheel_diameter * wheel_rpm / 60.0;
            // vel_new.angular.z = wheel_vel * std::atan(wheel_angular) / wheel_base_dis;
            // vel_new.linear.x = wheel_vel * std::cos(wheel_angular);

            geometry_msgs::Twist vel_new;

            double wheel_angular = (vel_msg.angular.y + wheel_angular_offset) * M_PI / 180;
            logger->debug("wheel_angular: " + std::to_string(wheel_angular));
            double wheel_rpm = vel_msg.linear.z;
            logger->debug("wheel_rpm: " + std::to_string(wheel_rpm));
            double wheel_vel = M_PI * wheel_diameter * wheel_rpm / 60.0;
            logger->debug("wheel_vel: " + std::to_string(wheel_vel));

            if (abs(wheel_angular) < 0.001)
            {
                logger->debug("abs(wheel_angular) < 0.001");
                vel_new.linear.x = wheel_vel;
                vel_new.angular.z = 0;
                logger->debug("vel_new.angular.z: " + std::to_string(vel_new.angular.z));
                logger->debug("vel_new.linear.x: " + std::to_string(vel_new.linear.x));
            }
            else if (abs(wheel_angular) < 1.570)
            {
                logger->debug("abs(wheel_angular) < 1.570");
                vel_new.angular.z = wheel_vel * std::sin(wheel_angular) / wheel_base_dis;
                vel_new.linear.x = vel_new.angular.z * (wheel_base_dis / std::tan(wheel_angular));
                logger->debug("vel_new.angular.z: " + std::to_string(vel_new.angular.z));
                logger->debug("vel_new.linear.x: " + std::to_string(vel_new.linear.x));
            }
            else
            {
                logger->debug("abs(wheel_angular) > 1.570");
                if (wheel_angular > 0)
                {
                    vel_new.angular.z = wheel_vel / wheel_base_dis;
                }
                else
                {
                    vel_new.angular.z = -1 * wheel_vel / wheel_base_dis;
                }
                vel_new.linear.x = 0;

                logger->debug("vel_new.angular.z: " + std::to_string(vel_new.angular.z));
                logger->debug("vel_new.linear.x: " + std::to_string(vel_new.linear.x));
            }

            // 加入队列
            wheel_speed new_speed(ros::Time::now(), vel_new);
            speed_data.push_back(new_speed);
            new_speed_x=new_speed.vel_.linear.x;

            logger->debug("realvelCallback(): new_speed: " 
                        + ' ' + std::to_string(new_speed.vel_.linear.x)
                        + ' ' + std::to_string(new_speed.vel_.angular.z));

            // 保护
            while (speed_data.size() > 5)
            {
                speed_data.pop_front();
            }
        }
        else
        {
            new_speed_x = p_velmsg->linear.x;
            new_msg = *p_velmsg;
        }
    }

    bool run_odom(std::vector<nav_msgs::Odometry> &v_odom)
    {
        //logger->debug("run_odom() : start");
        if (0 == speed_data.size())
        {
            //logger->debug("run_odom() : 0 == speed_data.size(), return false");
            return false;
        }

        // 获取速度信息
        wheel_speed cur_speed = speed_data.front();
        speed_data.pop_front();

        // 获取初始位姿
        nav_msgs::Odometry odom_init = getCurOdom();

        // 估计base当前时刻位姿(map--->base_link)
        nav_msgs::Odometry odom_est = poseEstimation(odom_init, cur_speed.vel_, cur_speed.timestamp_);
        setCurOdom(odom_est);
        v_odom.clear();
        v_odom.push_back(odom_est);

        // 计算定位相机当前时刻位姿(map--->locCamera_link)
        geometry_msgs::Pose pose_camera2map;
        tf2::doTransform(t2p(trans_camera2base_), pose_camera2map, p2t(odom_est.pose.pose));
        odom_est.pose.pose = pose_camera2map;
        odom_est.child_frame_id = "locCamera_link";
        v_odom.push_back(odom_est);

        //logger->debug("run_odom() : return true");
        return true;
    }

    double get_vel_x()
    {
        std::lock_guard<std::mutex> locker(mtx);
        return new_speed_x;
    }

    geometry_msgs::Twist get_vel_msg()
    {
        std::lock_guard<std::mutex> locker(mtx);
        return new_msg;
    }
};
