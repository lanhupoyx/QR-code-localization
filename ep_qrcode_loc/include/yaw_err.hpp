#pragma once

#include "utility_qloc.hpp"

 
// 定义一个二维点结构
struct Point {
    double x, y;
};

class CorrectionCase
{
public:
    uint32_t index_;           // 序号
    geometry_msgs::Pose pose_start_;
    geometry_msgs::Pose pose_end_;
    geometry_msgs::Pose pose_target_;

    double yaw_compensation_;

    CorrectionCase( uint32_t index,
                    geometry_msgs::Pose pose_start,
                    geometry_msgs::Pose pose_end,
                    geometry_msgs::Pose pose_target)
    {
        index_ = index;
        pose_start_ = pose_start;
        pose_end_ = pose_end;
        pose_target_ = pose_target;

        // 定义三个点A, B, C
        Point A = {pose_target_.position.x, pose_target_.position.y};
        Point B = {pose_start_.position.x, pose_start_.position.y};
        Point C = {pose_end_.position.x, pose_end_.position.y};
    
        // 计算向量BA和BC
        Point BA = vectorBetweenPoints(B, A);
        Point BC = vectorBetweenPoints(B, C);
    
        // 计算∠ABC的角度（弧度）
        double angleRadians = angleBetweenVectors(BA, BC);
    
        // 将角度转换为度
        double yaw_compensation_ = radiansToDegrees(angleRadians);
    
        // 输出结果
        //std::cout << "The angle ABC is: " << angleDegrees << " degrees" << std::endl;
    }

    ~CorrectionCase(){}

private:
    // 计算两点之间的向量
    Point vectorBetweenPoints(const Point& a, const Point& b) {
        return {b.x - a.x, b.y - a.y};
    }
    
    // 计算向量的点积
    double dotProduct(const Point& a, const Point& b) {
        return a.x * b.x + a.y * b.y;
    }
    
    // 计算向量的模长
    double magnitude(const Point& a) {
        return std::sqrt(a.x * a.x + a.y * a.y);
    }
    
    // 计算两个向量之间的角度（弧度）
    double angleBetweenVectors(const Point& a, const Point& b) {
        double dot = dotProduct(a, b);
        double magA = magnitude(a);
        double magB = magnitude(b);
        return std::acos(dot / (magA * magB));
    }
    
    // 将弧度转换为度
    double radiansToDegrees(double radians) {
        return radians * (180.0 / M_PI);
    }

};

// 
class YawErrorCorrecter : public ParamServer
{
private:
    std::list<geometry_msgs::Pose> pose_from_wheel_;
    std::list<geometry_msgs::Pose> pose_from_qrcode_;
    std::list<CorrectionCase> correction_cases_;

    std::mutex mtx;

public:
    YawErrorCorrecter()
    {
    }

    ~YawErrorCorrecter() {}

    void add_pose_from_wheel(geometry_msgs::Pose pose)
    {
        std::lock_guard<std::mutex> locker(mtx);
        pose_from_wheel_.push_back(pose);
        if(pose_from_wheel_.size() > 10)
        {
            pose_from_wheel_.pop_front();
        }
    }

    void add_qrcode(uint32_t index,
                    geometry_msgs::Pose new_qrcode_pose)
    {
        CorrectionCase cur_case(index, pose_from_qrcode_.back(), pose_from_wheel_.back(), new_qrcode_pose);
        correction_cases_.push_back(cur_case);
        pose_from_qrcode_.push_back(new_qrcode_pose);
        if(pose_from_qrcode_.size() > 10)
        {
            pose_from_qrcode_.pop_front();
        }

    }

};
