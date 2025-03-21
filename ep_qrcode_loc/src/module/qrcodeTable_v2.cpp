#include "qrcodeTable_v2.hpp"

Site::Site(uint32_t list_index,                               // 所在列编号
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
    aux_point_qrcode_.pose_ = this->move(dis_vec[1] + trans_base_camera.transform.translation.x + aux_point_qrcode_.x_err_,
                                         trans_base_camera.transform.translation.y + aux_point_qrcode_.y_err_);
    action_point_qrcode_.pose_ = this->move(dis_vec[2] + trans_base_camera.transform.translation.x + action_point_qrcode_.x_err_,
                                            trans_base_camera.transform.translation.y + action_point_qrcode_.y_err_);
    // 增加二维码位姿补偿
    detect_point_qrcode_.turn(detect_point_qrcode_.yaw_err_);
    aux_point_qrcode_.turn(aux_point_qrcode_.yaw_err_);
    action_point_qrcode_.turn(action_point_qrcode_.yaw_err_);
}

Site::~Site() {}

geometry_msgs::Pose Site::move(double dis_front, double dis_left)
{
    geometry_msgs::Pose pose_move;

    pose_move.position.x = pose_.position.x + dis_front * cos(getYawRad(pose_));
    pose_move.position.y = pose_.position.y + dis_front * sin(getYawRad(pose_));

    pose_move.position.x += dis_left * (-1) * sin(getYawRad(pose_)); // cos(x+90) = -sin(x)
    pose_move.position.y += dis_left * cos(getYawRad(pose_));        // sin(x+90) = cos(x)

    pose_move.orientation = pose_.orientation;
    return pose_move;
}

SiteList::SiteList(uint32_t index, geometry_msgs::Pose first_pose, std::vector<double> dis_vec, geometry_msgs::TransformStamped trans_base_camera)
{
    index_ = index;
    first_pose_ = first_pose;
    site_dis_ = dis_vec.back();
    dis_vec.pop_back();
    dis_vec_ = dis_vec;
    trans_base_camera_ = trans_base_camera;
}

SiteList::~SiteList() {}

void SiteList::set_first_site(geometry_msgs::Pose pose, std::vector<QRcodeGround> qrcodes)
{
    Site site_new(index_, sites_.size(), pose, qrcodes, dis_vec_, trans_base_camera_);
    sites_.clear();
    sites_.push_back(site_new);
}

void SiteList::add_site(std::vector<QRcodeGround> qrcodes)
{
    geometry_msgs::Pose pose_site;

    if (0 == sites_.size())
    {
        pose_site = first_pose_;
    }
    else
    {
        double num = -1 * int(sites_.size()) * site_dis_;
        pose_site = sites_.front().move(num, 0.0); // 库位位姿
    }

    Site site_new(index_, sites_.size() + 1, pose_site, qrcodes, dis_vec_, trans_base_camera_);
    sites_.push_back(site_new);
}

void SiteList::add_aux_points(std::vector<QRcodeGround> qrcodes)
{
    for (std::vector<QRcodeGround>::iterator it = qrcodes.begin(); it != qrcodes.end(); it++)
    {
        QRcodeGround newcode;
        newcode.type = 1;
        newcode.index_ = it->index_;
        newcode.pose_ = poseMove(first_pose_, it->x_err_, it->y_err_);
        newcode.turn(it->yaw_err_);
        front_aux_points_.push_back(newcode);
    }
}

geometry_msgs::Pose SiteList::poseMove(geometry_msgs::Pose pose, double dis_front, double dis_left)
{
    geometry_msgs::Pose pose_move;

    pose_move.position.x = pose.position.x + dis_front * cos(getYawRad(pose));
    pose_move.position.y = pose.position.y + dis_front * sin(getYawRad(pose));

    pose_move.position.x += dis_left * (-1) * sin(getYawRad(pose)); // cos(x+90) = -sin(x)
    pose_move.position.y += dis_left * cos(getYawRad(pose));        // sin(x+90) = cos(x)

    pose_move.orientation = pose.orientation;
    return pose_move;
}

QRcodeTableV2::QRcodeTableV2(std::string cfg_path, geometry_msgs::TransformStamped trans_base_camera, ParamServer &param) : param(param)
{
    // 配置文件路径
    cfg_path_ = cfg_path;

    // 记录器
    logger = &epLogger::getInstance();
    logger->info("QRcodeTableV2() start");

    // 打开库位信息文件
    //std::string site_info_path = cfg_path + "SiteTable.txt";
    std::string site_info_path = param.siteTablePath;
    std::ifstream ifs;
    ifs.open(site_info_path, std::ios::in);
    if (!ifs.is_open())
    {
        logger->info(site_info_path + "打开失败!");
    }
    else
    {
        logger->info(site_info_path + "打开成功!");
    }

    // 读取库位及其绑定的二维码信息
    std::string buf;
    while (std::getline(ifs, buf))
    {
        buf = replaceChar(buf, ',', ' '); // ','替换为' '
        std::stringstream line_ss;
        line_ss << buf;

        // 跳过空行
        if (("" == buf) || (' ' == buf[0]))
        {
            continue;
        }

        // 定义变量
        uint32_t list_index, site_index, code_index;
        double site_x_first, site_y_first, site_yaw_first;
        static double list_yaw_offset;
        QRcodeGround detect, aux, action, function_qrcode_ground;

        // 提取信息
        line_ss >> list_index >> site_index >> code_index;

        if (0 == site_index)
        {
            if (0 == code_index)
            {
                line_ss >> site_x_first >> site_y_first >> site_yaw_first >> list_yaw_offset;

                // 列首库位pose
                geometry_msgs::Pose pose_first_site;
                pose_first_site.position.x = site_x_first;
                pose_first_site.position.y = site_y_first;
                tf::Quaternion q;
                q.setRPY(0.0, 0.0, site_yaw_first);
                tf::quaternionTFToMsg(q, pose_first_site.orientation);

                // 几个距离参数
                std::vector<double> dis_vec;
                dis_vec.push_back(param.detect_site_dis);
                dis_vec.push_back(param.aux_site_dis);
                dis_vec.push_back(param.forkaction_site_dis);
                dis_vec.push_back(param.site_site_dis);

                // 新建列
                SiteList siteList_new(list_index, pose_first_site, dis_vec, trans_base_camera);
                siteList_lib.push_back(siteList_new);
            }
            else
            {
                line_ss >> aux.index_ >> aux.x_err_ >> aux.y_err_ >> aux.yaw_err_;
                aux.yaw_err_ *= param.err_ratio_offline;
                aux.yaw_err_ += list_yaw_offset;
                aux.yaw_err_ += param.ground_code_yaw_offset;

                std::vector<QRcodeGround> qrcodes;
                qrcodes.push_back(aux);

                // 列首添加额外辅助点---------需添加计算这些点位姿的程序！！！！
                siteList_lib[siteList_lib.size() - 1].add_aux_points(qrcodes);
            }
        }
        else if (0 < site_index)
        {
            static std::vector<QRcodeGround> qrcodes;

            if ((1 == code_index) || (2 == code_index) || (3 == code_index))
            {
                line_ss >> function_qrcode_ground.index_ >> function_qrcode_ground.x_err_ >> function_qrcode_ground.y_err_ >> function_qrcode_ground.yaw_err_;
                function_qrcode_ground.yaw_err_ *= param.err_ratio_offline;
                function_qrcode_ground.yaw_err_ += list_yaw_offset;
                function_qrcode_ground.yaw_err_ += param.ground_code_yaw_offset;
                qrcodes.push_back(function_qrcode_ground);
            }
            else
            {
                std::cout << "code_index 编号有误！" << std::endl;
            }

            if (3 == code_index)
            {
                siteList_lib[siteList_lib.size() - 1].add_site(qrcodes);
                qrcodes.clear();
            }
        }
        else
        {
            std::cout << "列编号有误！" << std::endl;
        }
    }
    ifs.close();
    logger->info(site_info_path + "读取完毕!");

    // 遍历提取每个二维码的信息
    for (std::vector<SiteList>::iterator list_it = siteList_lib.begin(); list_it != siteList_lib.end(); list_it++)
    {
        ColumnCodeList this_column;
        this_column.column_index_ = list_it->index_;

        // 每个库位对应的3个点
        for (std::list<Site>::iterator site_it = list_it->sites_.begin(); site_it != list_it->sites_.end(); site_it++)
        {
            stream.str("");
            stream << site_it->list_index_ << " "
                   << site_it->index_ << " "
                   << site_it->pose_.position.x << " "
                   << site_it->pose_.position.y << " "
                   << getYaw(site_it->pose_.orientation);
            logger->info(stream.str());

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
            info_head.type = 1;
            map.insert(std::pair<uint32_t, QRcodeInfo>(info_head.code, info_head));
        }
    }

    // if(param.read_yaw_err)
    // {
    //     // 读取矫正值
    //     readYawErr(cfg_path_);
    // }

    stream.str("");
    stream << "qrcode_table的大小为: " << map.size();
    logger->info(stream.str());
    for (std::map<uint32_t, QRcodeInfo>::iterator it = map.begin(); it != map.end(); it++)
    {
        stream.str("");
        stream << (*it).second.code << " " << (*it).second.x << " " << (*it).second.y << " " << (*it).second.yaw;
        logger->info(stream.str());
    }

    sub_pos = param.nh.subscribe<nav_msgs::Odometry>("/ep_localization/odometry/lidar", 1,
                                                     &QRcodeTableV2::tfCallback, this,
                                                     ros::TransportHints().tcpNoDelay());
    logger->debug("sub: /ep_localization/odometry/lidar");

    logger->info("QRcodeTableV2() return");
}

QRcodeTableV2::~QRcodeTableV2() {}

// 根据二维码编号查表，得到位姿信息
bool QRcodeTableV2::onlyfind(CameraFrame frame, QRcodeInfo *info)
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

// 根据二维码编号查表，得到位姿信息
bool QRcodeTableV2::onlyfind(CameraFrame frame)
{
    std::lock_guard<std::mutex> locker(mtx);
    std::map<uint32_t, QRcodeInfo>::iterator it = map.find(frame.code);
    if (it != map.end())
    {
        return true;
    }
    else
    {
        std::cout << "can not identify code:" << frame.code << std::endl;
        return false;
    }
}

// 校正单个地码方向角
bool QRcodeTableV2::correct_yaw(uint32_t code, double yaw_err)
{
    std::lock_guard<std::mutex> locker(mtx);
    std::map<uint32_t, QRcodeInfo>::iterator it = map.find(code);
    if ((it != map.end()) && (1 != it->second.type))
    {
        it->second.yaw += yaw_err; // 单位：角度
        logger->info("correct_yaw: " +
                     std::to_string(code) + " " +
                     std::to_string(yaw_err) + " " +
                     std::to_string(it->second.yaw));
        return true;
    }
    else
    {
        logger->info("correct_yaw() can not identify code: " + std::to_string(code));
    }
    return false;
}

// 读取地码方向角补偿数据
void QRcodeTableV2::readYawErr(std::string cfg_path)
{
    logger->debug("QRcodeTableV2::readYawErr() start");

    std::string yaw_err_path = cfg_path + "yaw_err.txt";
    std::ifstream ifs;
    ifs.open(yaw_err_path, std::ios::in);
    if (!ifs.is_open())
    {
        logger->info(yaw_err_path + "打开失败!");
    }
    else
    {
        logger->info(yaw_err_path + "打开成功!");
    }
    std::string buf;               // 将数据存放到c++ 中的字符串中
    while (std::getline(ifs, buf)) // 使用全局的getline()函数，其里面第一个参数代表输入流对象，第一个参数代表准备好的字符串，每次读取一行内容到buf
    {
        std::stringstream line_ss;
        line_ss << buf;

        // 跳过空行
        if (("" == buf) || (' ' == buf[0]))
        {
            continue;
        }

        // 定义变量
        std::string time;
        uint32_t index;
        double yaw_err;

        // 提取信息
        line_ss >> time >> index >> yaw_err;

        correct_yaw(index, yaw_err * param.err_ratio_offline);
    }
    ifs.close();
    logger->info(yaw_err_path + "读取完毕!");

    logger->debug("QRcodeTableV2::readYawErr() return");
}

// callback获取baselink位姿
void QRcodeTableV2::tfCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> locker(mtx);
    const nav_msgs::Odometry transform = *msg; //->transforms[i];

    tf_buffer.push_back(transform);
    while (20 < tf_buffer.size())
    {
        tf_buffer.pop_front();
    }
    logger->debug("QRcodeTableV2 tfCallback()");
}

// 最新基于lidar的baselink位姿
nav_msgs::Odometry QRcodeTableV2::getCurLidarPose()
{
    std::lock_guard<std::mutex> locker(mtx);
    if (tf_buffer.size() > 0)
    {
        logger->info("QRcodeTableV2::getCurLidarPose(): normal");
        return tf_buffer.back();
    }
    else
    {
        logger->info("QRcodeTableV2::getCurLidarPose(): zero");
        nav_msgs::Odometry zero;
        return zero;
    }
}

// 获取所有列首二维码编号
std::vector<uint32_t> QRcodeTableV2::get_all_head()
{
    static std::vector<uint32_t> v_out;

    if (v_out.size() == 0) // 只在第一次执行时计算一次
    {
        // 遍历每列
        for (std::vector<SiteList>::iterator list_it = siteList_lib.begin(); list_it != siteList_lib.end(); list_it++)
        {
            // 遍历每个库位
            for (std::list<Site>::iterator site_it = list_it->sites_.begin(); site_it != list_it->sites_.end(); site_it++)
            {
                // if(2 == site_it->index_) // 石花项目：第二排库位的辅助二维码为列首
                // {
                //     v_out.push_back(site_it->aux_point_qrcode_.index_);
                //     break;
                // }
                if (0 != site_it->detect_point_qrcode_.index_)
                {
                    v_out.push_back(site_it->detect_point_qrcode_.index_);
                    break;
                }
                if (0 != site_it->aux_point_qrcode_.index_)
                {
                    v_out.push_back(site_it->aux_point_qrcode_.index_);
                    break;
                }
                if (0 != site_it->action_point_qrcode_.index_)
                {
                    v_out.push_back(site_it->action_point_qrcode_.index_);
                    break;
                }
            }
        }
    }

    return v_out;
}

// 检查是否为列首地码
bool QRcodeTableV2::is_head(uint32_t code_new)
{
    std::vector<uint32_t> code_could_be = get_all_head();
    std::vector<uint32_t>::iterator code_it;
    for (code_it = code_could_be.begin(); code_it != code_could_be.end(); code_it++)
    {
        if (code_new == *code_it)
        {
            return true;
        }
    }

    return false;
}

// 获取前后二维码
std::vector<uint32_t> QRcodeTableV2::get_neighbor(uint32_t base_code)
{
    // 创建二维码矩阵
    static std::vector<std::vector<uint32_t>> code_matrix;
    if (code_matrix.size() == 0) // 只在第一次执行时计算一次
    {
        logger->info("get code_matrix:");
        // 遍历每列
        for (std::vector<SiteList>::iterator list_it = siteList_lib.begin(); list_it != siteList_lib.end(); list_it++)
        {
            logger->info("list " + std::to_string(list_it->index_) + ":");
            std::vector<uint32_t> code_list;
            // 遍历每个库位
            for (std::list<Site>::iterator site_it = list_it->sites_.begin(); site_it != list_it->sites_.end(); site_it++)
            {
                logger->info("site " + std::to_string(site_it->index_) + ":");
                if (0 != site_it->detect_point_qrcode_.index_)
                {
                    code_list.push_back(site_it->detect_point_qrcode_.index_);
                    logger->info(std::to_string(site_it->detect_point_qrcode_.index_));
                }
                if (0 != site_it->aux_point_qrcode_.index_)
                {
                    code_list.push_back(site_it->aux_point_qrcode_.index_);
                    logger->info(std::to_string(site_it->aux_point_qrcode_.index_));
                }
                if (0 != site_it->action_point_qrcode_.index_)
                {
                    code_list.push_back(site_it->action_point_qrcode_.index_);
                    logger->info(std::to_string(site_it->action_point_qrcode_.index_));
                }
            }
            code_matrix.push_back(code_list);
        }
    }

    // 二维码矩阵，查找前后近邻地码
    std::vector<uint32_t> v_out;
    for (uint32_t ln = 0; ln < code_matrix.size(); ln++)
    {
        for (uint32_t cn = 0; cn < code_matrix[ln].size(); cn++)
        {
            if (base_code == code_matrix[ln][cn])
            {
                if (0 == cn) // 列首
                {
                    v_out.push_back(0);
                    v_out.push_back(code_matrix[ln][cn + 1]);
                }
                else if (code_matrix[ln].size() - 1 == cn) // 列尾
                {
                    v_out.push_back(code_matrix[ln][cn - 1]);
                    v_out.push_back(0);
                }
                else // 列中
                {
                    v_out.push_back(code_matrix[ln][cn - 1]);
                    v_out.push_back(code_matrix[ln][cn + 1]);
                }
            }
        }
    }

    return v_out;
}

// 是否在列内
bool QRcodeTableV2::is_in_queue(nav_msgs::Odometry base2map, double head_offset)
{
    // 遍历每个库位
    // double l_x_err
    std::vector<SiteList>::iterator list_it;
    for (list_it = siteList_lib.begin(); list_it != siteList_lib.end(); list_it++)
    {
        // 石花定制：先看列首x值
        double l_x_err = base2map.pose.pose.position.x - list_it->sites_.begin()->pose_.position.x;

        if (l_x_err < head_offset)
        {
            continue;
        }
        // 石花定制：再看列首y值
        double l_y_err = base2map.pose.pose.position.y - list_it->sites_.begin()->pose_.position.y;

        if (abs(l_y_err) > 3.0)
        {
            continue;
        }

        // 每个库位
        for (std::list<Site>::iterator site_it = list_it->sites_.begin(); site_it != list_it->sites_.end(); site_it++)
        {
            double x_err = base2map.pose.pose.position.x - site_it->pose_.position.x;
            double y_err = base2map.pose.pose.position.y - site_it->pose_.position.y;

            if (abs(x_err) < 1.0 && abs(y_err) < 1.0) // 覆盖范围大于单个库位
            {
                logger->debug(
                    "is_in_queue: true  head_offset: " + std::to_string(head_offset) +
                    "  l_x_err: " + std::to_string(l_x_err) +
                    "  l_y_err: " + std::to_string(l_y_err) +
                    "  x_err: " + std::to_string(x_err) +
                    "  y_err: " + std::to_string(y_err));
                return true;
            }
        }
    }
    logger->debug("is_in_queue: false  head_offset: " + std::to_string(head_offset));
    return false;
}
