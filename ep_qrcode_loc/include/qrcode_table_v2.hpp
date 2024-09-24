#pragma once

#include "utility_qloc.hpp"
#include "camera.hpp"

// 贴在地上的二维码
struct QRcodeGround
{
    uint32_t index_;           // 序号
    geometry_msgs::Pose pose_; // 位姿

    double x_err_;   // x方向补偿值，单位m
    double y_err_;   // y方向补偿值，单位m
    double yaw_err_; // yaw方向补偿值,单位角度

    void turn(double d_yaw)
    {
        double yaw = getYawRad(pose_.orientation);
        yaw += d_yaw * M_PI / 180.0;
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        tf::quaternionTFToMsg(q, pose_.orientation);
    }
};

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
         geometry_msgs::TransformStamped trans_base_camera) // 相机到base的变换
    {
        list_index_ = list_index;
        index_ = index;
        pose_ = pose;

        // 三个对应的二维码
        detect_point_qrcode_ = qrcodes[0];
        aux_point_qrcode_ = qrcodes[1];
        action_point_qrcode_ = qrcodes[2];

        // 计算二维码位姿
        detect_point_qrcode_.pose_ = this->move(dis_vec[0] + trans_base_camera.transform.translation.x + detect_point_qrcode_.x_err_, 
                                                             trans_base_camera.transform.translation.y + detect_point_qrcode_.y_err_);
        aux_point_qrcode_.pose_    = this->move(dis_vec[1] + trans_base_camera.transform.translation.x + aux_point_qrcode_.x_err_, 
                                                             trans_base_camera.transform.translation.y + aux_point_qrcode_.y_err_);
        action_point_qrcode_.pose_ = this->move(dis_vec[2] + trans_base_camera.transform.translation.x + action_point_qrcode_.x_err_, 
                                                             trans_base_camera.transform.translation.y + action_point_qrcode_.y_err_);
        // 增加二维码位姿补偿
        detect_point_qrcode_.turn(detect_point_qrcode_.yaw_err_);
        aux_point_qrcode_.turn(aux_point_qrcode_.yaw_err_);
        action_point_qrcode_.turn(action_point_qrcode_.yaw_err_);
    }

    ~Site(){}

    geometry_msgs::Pose move(double dis_front, double dis_left)
    {
        geometry_msgs::Pose pose_move;
        
        pose_move.position.x = pose_.position.x + dis_front * cos(getYawRad(pose_));
        pose_move.position.y = pose_.position.y + dis_front * sin(getYawRad(pose_));

        pose_move.position.x +=  dis_left * (-1)*sin(getYawRad(pose_)); //cos(x+90) = -sin(x)
        pose_move.position.y +=  dis_left * cos(getYawRad(pose_)); // sin(x+90) = cos(x)

        pose_move.orientation = pose_.orientation;
        return pose_move;
    }


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

    SiteList(uint32_t index, geometry_msgs::Pose first_pose, std::vector<double> dis_vec, geometry_msgs::TransformStamped trans_base_camera)
    {
        index_ = index;
        first_pose_ = first_pose;
        site_dis_ = dis_vec.back();
        dis_vec.pop_back();
        dis_vec_ = dis_vec;
        trans_base_camera_ = trans_base_camera;
    }

    ~SiteList() {}

    void set_first_site(geometry_msgs::Pose pose, std::vector<QRcodeGround> qrcodes)
    {
        Site site_new(index_, sites_.size(), pose, qrcodes, dis_vec_, trans_base_camera_);
        sites_.clear();
        sites_.push_back(site_new);
    }

    void add_site(std::vector<QRcodeGround> qrcodes)
    {
        geometry_msgs::Pose pose_site;

        if(0 == sites_.size())
        {
            pose_site = first_pose_;
        }
        else
        {
            double num = -1 * int(sites_.size()) * site_dis_;
            pose_site = sites_.front().move(num, 0.0); // 库位位姿
        }

        Site site_new(index_, sites_.size(), pose_site, qrcodes, dis_vec_, trans_base_camera_);
        sites_.push_back(site_new);
    }

    void add_aux_points(std::vector<QRcodeGround> qrcodes)
    {
        for (std::vector<QRcodeGround>::iterator it = qrcodes.begin(); it != qrcodes.end(); it++)
        {
            QRcodeGround newcode;
            newcode.index_ = it->index_;
            newcode.pose_ = poseMove(first_pose_, it->x_err_, it->y_err_);
            newcode.turn(it->yaw_err_);
            front_aux_points_.push_back(newcode);
        }
    }

    geometry_msgs::Pose poseMove(geometry_msgs::Pose pose, double dis_front, double dis_left)
    {
        geometry_msgs::Pose pose_move;
        
        pose_move.position.x = pose.position.x + dis_front * cos(getYawRad(pose));
        pose_move.position.y = pose.position.y + dis_front * sin(getYawRad(pose));

        pose_move.position.x +=  dis_left * (-1)*sin(getYawRad(pose)); //cos(x+90) = -sin(x)
        pose_move.position.y +=  dis_left * cos(getYawRad(pose)); // sin(x+90) = cos(x)

        pose_move.orientation = pose.orientation;
        return pose_move;
    }    
};

// 二维码坐标对照表
class QRcodeTableV2 : public ParamServer
{
private:
            
    std::vector<SiteList> siteList_lib; // 各个列
    std::map<uint32_t, QRcodeInfo> map;                // 最终生成的二维码位姿表

    std::mutex mtx;

    Logger *logger;
    std::stringstream stream;

    std::ifstream ifs;

public:
    QRcodeTableV2(std::string site_info_path, geometry_msgs::TransformStamped trans_base_camera)
    {
        // 读取文本文件中库位相关信息
        logger = &Logger::getInstance();
        logger->log("QRcodeTable Start");
        ifs.open(site_info_path, std::ios::in);
        if (!ifs.is_open())
        {
            logger->log(site_info_path + "打开失败!");
        }
        else
        {
            logger->log(site_info_path + "打开成功!");
        }
        std::string buf;               // 将数据存放到c++ 中的字符串中
        while (std::getline(ifs, buf)) // 使用全局的getline()函数，其里面第一个参数代表输入流对象，第一个参数代表准备好的字符串，每次读取一行内容到buf
        {
            std::stringstream line_ss;
            line_ss << buf;

            // 跳过空行
            if("" == buf)
            {
                continue;
            }

            // 定义变量
            uint32_t list_index, site_index, code_index;
            double site_x_first, site_y_first, site_yaw_first;
            QRcodeGround detect, aux, action;

            // 提取信息
            line_ss >> list_index >> site_index >> code_index;

            if(0 == site_index)
            {
                if(0 == code_index)
                {
                    line_ss >> site_x_first >> site_y_first >> site_yaw_first;

                    // 列首库位pose
                    geometry_msgs::Pose pose_first_site;
                    pose_first_site.position.x = site_x_first;
                    pose_first_site.position.y = site_y_first;
                    tf::Quaternion q;
                    q.setRPY(0.0, 0.0, site_yaw_first);
                    tf::quaternionTFToMsg(q, pose_first_site.orientation);

                    // 几个距离参数
                    std::vector<double> dis_vec;
                    dis_vec.push_back(detect_site_dis);
                    dis_vec.push_back(aux_site_dis);
                    dis_vec.push_back(forkaction_site_dis);
                    dis_vec.push_back(site_site_dis);

                    // 新建列
                    SiteList siteList_new(list_index, pose_first_site, dis_vec, trans_base_camera);
                    siteList_lib.push_back(siteList_new);

                }
                else
                {
                    line_ss >> aux.index_ >> aux.x_err_ >> aux.y_err_ >> aux.yaw_err_;

                    std::vector<QRcodeGround> qrcodes;
                    qrcodes.push_back(aux);

                    // 列首添加额外辅助点---------需添加计算这些点位姿的程序！！！！
                    siteList_lib[siteList_lib.size() - 1].add_aux_points(qrcodes);
                }
            }
            else if(0 < site_index)
            {
                static std::vector<QRcodeGround> qrcodes;

                if(1 == code_index)
                {
                    line_ss >> detect.index_ >> detect.x_err_ >> detect.y_err_ >> detect.yaw_err_ ;
                    qrcodes.push_back(detect);
                }
                else if(2 == code_index)
                {
                    line_ss >> aux.index_ >> aux.x_err_ >> aux.y_err_ >> aux.yaw_err_ ;
                    qrcodes.push_back(aux);
                }
                else if(3 == code_index)
                {
                    line_ss >> action.index_ >> action.x_err_ >> action.y_err_ >> action.yaw_err_;
                    qrcodes.push_back(action);

                    siteList_lib[siteList_lib.size() - 1].add_site(qrcodes);
                    qrcodes.clear();
                }
                else
                {
                    std::cout << "code_index 编号有误！" << std::endl;
                }            
            }
            else
            {
                std::cout << "列编号有误！" << std::endl;
            }
        }
        ifs.close();
        logger->log(site_info_path + "读取完毕!");

        // 遍历提取每个二维码的信息
        for (std::vector<SiteList>::iterator list_it = siteList_lib.begin(); list_it != siteList_lib.end(); list_it++)
        {
            // 每个库位对应的3个点
            for (std::list<Site>::iterator site_it = list_it->sites_.begin(); site_it != list_it->sites_.end(); site_it++)
            {
                stream.str("");
                stream << site_it->list_index_ << " " 
                << site_it->index_ << " " 
                << site_it->pose_.position.x << " " 
                << site_it->pose_.position.y << " " 
                << getYaw(site_it->pose_.orientation);
                logger->log(stream.str());

                // 识别点对应二维码
                QRcodeInfo info_detect(site_it->detect_point_qrcode_.index_,
                                       site_it->detect_point_qrcode_.pose_.position.x,
                                       site_it->detect_point_qrcode_.pose_.position.y,
                                       getYaw(site_it->detect_point_qrcode_.pose_));
                map.insert(std::pair<uint32_t, QRcodeInfo>(info_detect.code, info_detect));

                // 辅助点对应二维码
                QRcodeInfo info_aux(site_it->aux_point_qrcode_.index_,
                                    site_it->aux_point_qrcode_.pose_.position.x,
                                    site_it->aux_point_qrcode_.pose_.position.y,
                                    getYaw(site_it->aux_point_qrcode_.pose_));
                map.insert(std::pair<uint32_t, QRcodeInfo>(info_aux.code, info_aux));

                // 货叉动作点对应二维码
                QRcodeInfo info_action(site_it->action_point_qrcode_.index_,
                                       site_it->action_point_qrcode_.pose_.position.x,
                                       site_it->action_point_qrcode_.pose_.position.y,
                                       getYaw(site_it->action_point_qrcode_.pose_));
                map.insert(std::pair<uint32_t, QRcodeInfo>(info_action.code, info_action));
            }

            // 列首辅助二维码
            for (std::list<QRcodeGround>::iterator it = list_it->front_aux_points_.begin(); it != list_it->front_aux_points_.end(); it++)
            {
                QRcodeInfo info_head(it->index_, it->pose_.position.x, it->pose_.position.y, getYaw(it->pose_), true);
                map.insert(std::pair<uint32_t, QRcodeInfo>(info_head.code, info_head));
            }
        }

        //readYawErr();

        stream.str("");
        stream << "qrcode_table的大小为: " << map.size();
        logger->log(stream.str());
        for (std::map<uint32_t, QRcodeInfo>::iterator it = map.begin(); it != map.end(); it++)
        {
            stream.str("");
            stream << (*it).second.code << " " << (*it).second.x << " " << (*it).second.y << " " << (*it).second.yaw;
            logger->log(stream.str());
        }

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

    bool jiaozheng(uint32_t code, double y_err)
    {
        std::lock_guard<std::mutex> locker(mtx);
        std::map<uint32_t, QRcodeInfo>::iterator it = map.find(code);
        if (it != map.end())
        {
            it->second.yaw += y_err;
            return true;
        }
        else
        {
            std::cout << "can not identify code:" << code << std::endl;
        }
        return false;
    }

    // void readYawErr()
    // {
    //     std::string yaw_err_path = "/home/zl/work/ep-qrcode-loc/src/ep_qrcode_loc/config/yawerr.csv";
    //     logger->log("QRcodeTable Start");
    //     ifs.open(yaw_err_path, std::ios::in);
    //     if (!ifs.is_open())
    //     {
    //         logger->log(yaw_err_path + "打开失败!");
    //     }
    //     else
    //     {
    //         logger->log(yaw_err_path + "打开成功!");
    //     }
    //     std::string buf;               // 将数据存放到c++ 中的字符串中
    //     while (std::getline(ifs, buf)) // 使用全局的getline()函数，其里面第一个参数代表输入流对象，第一个参数代表准备好的字符串，每次读取一行内容到buf
    //     {
    //         std::stringstream line_ss;
    //         line_ss << buf;

    //         // 跳过空行
    //         if("" == buf)
    //         {
    //             continue;
    //         }

    //         // 定义变量
    //         uint32_t list_index, site_index;
    //         double site_x_first, site_y_first, site_yaw_first;
    //         QRcodeGround detect, aux, action;

    //         // 提取信息
    //         line_ss >> list_index >> site_index ;

    //     }
    //     ifs.close();
    //     logger->log(yaw_err_path + "读取完毕!");
    // }
};
