#pragma once

#include "utility_qloc.hpp"
#include "camera.hpp"

// 贴在地上的二维码
struct QRcodeGround
{
    uint32_t index;                                    // 序号
    geometry_msgs::TransformStamped trans_site_qrcode; // 与绑定站点之间的变换关系
    double x_err;                                      // x方向补偿值，单位m
    double y_err;                                      // y方向补偿值，单位m
    double yaw_err;                                    // yaw方向补偿值,单位角度
    nav_msgs::Odometry pose_map_qrcode;                // 位姿
};

// 库位
struct Site
{
    uint32_t list_index;     // 所在列编号
    uint32_t index;          // 库位编号
    nav_msgs::Odometry pose; // 位姿
    QRcodeGround action_point;
    QRcodeGround aux_point;
    QRcodeGround detect_point;
};

// 库位列
struct SiteList
{
    uint32_t index;                           // 库位列编号
    nav_msgs::Odometry pose_benchmark;        // 列基准点
    nav_msgs::Odometry pose_first_site;       // 列首库位位姿
    uint16_t site_num;                        // 列内库位数量
    std::list<QRcodeGround> front_aux_points; // 列首的辅助二维码
    std::list<Site> sites;                    // 列内的库位
};

// 二维码坐标对照表
class QRcodeTableV2 : public ParamServer
{
private:
    geometry_msgs::TransformStamped trans_site_action; // 库位到货叉动作点变换
    geometry_msgs::TransformStamped trans_site_aux;    // 库位到辅助点变换
    geometry_msgs::TransformStamped trans_site_detect; // 库位到检测点变换
    std::list<SiteList> sitelists;                     // 各个列
    std::map<uint32_t, QRcodeInfo> map;                // 最终生成的二维码位姿表

    std::mutex mtx;

    Logger *logger;
    std::stringstream stream;

    std::ifstream ifs;

public:
    QRcodeTableV2(std::string site_info_path, std::string aux_point_info_path)
    {
        // 读取文本文件中库位相关信息
        logger = &Logger::getInstance();
        logger->log("QRcodeTable Start");
        ifs.open(site_info_path, std::ios::in);
        if (!ifs.is_open())
        {
            logger->log(site_info_path + "打开失败!");
        }
        std::list<Site> site_lib;
        std::string buf;               // 将数据存放到c++ 中的字符串中
        while (std::getline(ifs, buf)) // 使用全局的getline()函数，其里面第一个参数代表输入流对象，第一个参数代表准备好的字符串，每次读取一行内容到buf
        {
            std::stringstream line_ss;
            line_ss << buf;
            Site cur_site;
            line_ss >> info.code >> info.x >> info.y >> info.yaw;
            map.insert(std::pair<uint32_t, QRcodeInfo>(info.code, info));
        }
        ifs.close();
        stream.str("");
        stream << "qrcode_table的大小为: " << map.size();
        logger->log(stream.str());
        for (std::map<uint32_t, QRcodeInfo>::iterator it = map.begin(); it != map.end(); it++)
        {
            stream.str("");
            stream << (*it).second.code << " " << (*it).second.x << " " << (*it).second.y << " " << (*it).second.yaw;
            logger->log(stream.str());
        }

        // 读取文本文件中额外二维码的信息


        // 读取参数中的变换关系


        // 计算二维码位姿，存在map中

    }

    ~QRcodeTableV2(){}

    // 根据二维码编号查表，得到位姿信息
    bool onlyfind(CameraFrame frame, QRcodeInfo *info)
    {
        std::lock_guard<std::mutex> locker(mtx);
        std::map<uint32_t, QRcodeInfo>::iterator it = map.find(frame.code);
        if (it != map.end())
        {
            *info = (*it).second;
            info->frame = frame;
            return true;
        }
        else
        {
            std::cout << "can not identify code:" << frame.code << std::endl;
        }
        return false;
    }
};
