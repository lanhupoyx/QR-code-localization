#include "qrcodeTable_v1.hpp"

QRcodeTableV1::QRcodeTableV1(std::string dir, ParamServer &param) : param(param)
{
    path = dir; // 文件位置
    logger = &epLogger::getInstance();
    logger->info("QRcodeTableV1() Start");
    ifs.open(dir, std::ios::in);
    if (!ifs.is_open())
    {
        logger->info(dir + "打开失败!");
    }
    else
    {
        logger->info(dir + "打开成功!");
        std::string buf;               // 将数据存放到c++ 中的字符串中
        while (std::getline(ifs, buf)) // 使用全局的getline()函数，其里面第一个参数代表输入流对象，第一个参数代表准备好的字符串，每次读取一行内容到buf
        {
            std::stringstream line_ss;
            line_ss << buf;
            QRcodeInfo info;
            line_ss >> info.code >> info.x >> info.y >> info.yaw;
            map.insert(std::pair<uint32_t, QRcodeInfo>(info.code, info));
        }
        ifs.close();
        stream.str("");
        stream << "qrcode_table的大小为: " << map.size();
        logger->info(stream.str());
        for (std::map<uint32_t, QRcodeInfo>::iterator it = map.begin(); it != map.end(); it++)
        {
            stream.str("");
            stream << (*it).second.code << " " << (*it).second.x << " " << (*it).second.y << " " << (*it).second.yaw;
            logger->info(stream.str());
        }
    }

    // baselink---->camera的变换关系
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfListener(buffer);
    bool tferr = true;
    while (tferr)
    {
        tferr = false;
        try
        {
            trans_camera2base = buffer.lookupTransform("base_link", "locCamera_link", ros::Time(0));
        }
        catch (tf::TransformException &exception)
        {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep();
            continue;
        }
    }

    logger->info("QRcodeTableV1() End");
}

QRcodeTableV1::~QRcodeTableV1() {}

// 开始订阅lidar输出位姿
void QRcodeTableV1::subPose()
{
    sub_pos = param.nh.subscribe<nav_msgs::Odometry>("/ep_localization/odometry/lidar", 1,
                                                     &QRcodeTableV1::tfCallback, this,
                                                     ros::TransportHints().tcpNoDelay());
    logger->info("QRcodeTableV1::subPose() sub: /ep_localization/odometry/lidar");
}

// 根据二维码编号查表，得到位姿信息
bool QRcodeTableV1::find_add(CameraFrame frame, QRcodeInfo *info)
{
    std::map<uint32_t, QRcodeInfo>::iterator it = map.find(frame.code);
    if (it != map.end())
    {
        *info = (*it).second;
        info->frame = frame;
        return true;
    }
    else
    {
        // stream.str("");
        // stream  << "can not identify code:" << frame.code;
        // logger->info(stream.str());
        std::cout << "can not identify code:" << frame.code << std::endl;
        add(frame);
    }
    return false;
}

// 根据二维码编号查表，得到位姿信息
bool QRcodeTableV1::onlyfind(CameraFrame frame, QRcodeInfo *info)
{
    std::map<uint32_t, QRcodeInfo>::iterator it = map.find(frame.code);
    if (it != map.end())
    {
        *info = (*it).second;
        info->frame = frame;
        return true;
    }
    else
    {
        // stream.str("");
        // stream  << "can not identify code:" << frame.code;
        // logger->info(stream.str());
        std::cout << "can not identify code:" << frame.code << std::endl;
    }
    return false;
}

// 添加新的二维码
bool QRcodeTableV1::add(const CameraFrame frame)
{
    // std::lock_guard<std::mutex> locker(QRcodeTableV1::mtx);
    if (frame_buffer.size() > 240)
    {
        frame_buffer.pop_front();
    }
    static uint32_t lastcode = 0;
    if (frame.code != lastcode)
    {
        lastcode = frame.code;
        frame_buffer.clear();
    }
    frame_buffer.push_back(frame);

    // 计算并保存新二维码信息
    if ((tf_buffer.size() > 19) && (frame_buffer.size() > 230))
    {
        QRcodeInfo new_qrcode = calPose();
        new_qrcode.code = frame.code;
        map.insert(std::pair<uint32_t, QRcodeInfo>(new_qrcode.code, new_qrcode));
        std::ofstream table(path, std::ios::app);
        std::ostream &table_os = table; // 将文件流转换为输出流对象
        table_os << new_qrcode.code << " "
                 << new_qrcode.x << " "
                 << new_qrcode.y << " "
                 << new_qrcode.yaw << std::endl;
        table.close();
        // log_file    << "add to dable:"
        //             << new_qrcode.code << " "
        //             << new_qrcode.x << " "
        //             << new_qrcode.y << " "
        //             << new_qrcode.yaw << std::endl;
        stream.str("");
        stream << "add to dable:"
               << new_qrcode.code << " "
               << new_qrcode.x << " "
               << new_qrcode.y << " "
               << new_qrcode.yaw;
        logger->info(stream.str());
        std::cout << stream.str() << std::endl;
    }

    return true;
}

// callback获取baselink位姿
void QRcodeTableV1::tfCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    logger->debug("QRcodeTableV1::tfCallback()");
    std::lock_guard<std::mutex> locker(QRcodeTableV1::mtx);
    const nav_msgs::Odometry transform = *msg; //->transforms[i];
    if (tf_buffer.size() != 0)
    {
        double x_error = std::fabs(transform.pose.pose.position.x - tf_buffer.begin()->pose.pose.position.x);
        double y_error = std::fabs(transform.pose.pose.position.y - tf_buffer.begin()->pose.pose.position.y);
        if ((x_error > 0.01) || (y_error > 0.01))
        {
            tf_buffer.clear();
        }
    }
    tf_buffer.push_back(transform);
    while (21 < tf_buffer.size())
    {
        tf_buffer.pop_front();
    }
}

// 计算二维码位姿
QRcodeInfo QRcodeTableV1::calPose()
{
    std::lock_guard<std::mutex> locker(QRcodeTableV1::mtx);
    QRcodeInfo sum;
    sum.x = 0.0;
    sum.y = 0.0;
    sum.yaw = 0.0;
    sum.code = 0;
    std::list<QRcodeInfo> poselog;
    for (std::list<nav_msgs::Odometry>::iterator t_it = tf_buffer.begin(); t_it != tf_buffer.end(); t_it++)
    {
        double min_time_err = 1.0;
        std::list<CameraFrame>::iterator d_it = frame_buffer.begin();
        for (std::list<CameraFrame>::iterator f_it = frame_buffer.begin(); f_it != frame_buffer.end(); f_it++)
        {
            ros::Duration time_err = f_it->stamp - t_it->header.stamp;
            double d_time_err = std::fabs(time_err.toSec());
            if (d_time_err > min_time_err)
            {
                continue;
            }
            else
            {
                min_time_err = d_time_err;
                d_it = f_it;
            }
        }
        if (min_time_err > 0.01)
        {
            continue;
        }
        // 相机位姿(map)
        geometry_msgs::Pose pose_camera2map;
        tf2::doTransform(t2p(trans_camera2base), pose_camera2map, p2t(t_it->pose.pose));
        // double yaw_camera2base = getYaw(t_it->pose.pose);
        // double yaw_base2map = getYaw(t_it->pose.pose);
        // double yaw_camera2map = getYaw(pose_camera2map);
        //  二维码与相机变换关系
        geometry_msgs::Pose pose_qrcode2camera;
        pose_qrcode2camera.position.x = d_it->error_x / 1000.0;
        pose_qrcode2camera.position.y = d_it->error_y / -1000.0;
        pose_qrcode2camera.position.z = 0.0;
        tf::Quaternion q1;
        q1.setRPY(0.0, 0.0, d_it->error_yaw * M_PI / -180.0);
        tf::quaternionTFToMsg(q1, pose_qrcode2camera.orientation);
        // 二维码位姿(map)
        geometry_msgs::Pose pose_qrcode2map;
        tf2::doTransform(pose_qrcode2camera, pose_qrcode2map, p2t(pose_camera2map));
        // double yaw_qrcode2camera = getYaw(pose_qrcode2camera);
        double yaw_qrcode2map = getYaw(pose_qrcode2map);
        // 提取关键数据
        QRcodeInfo cur_qrcode;
        cur_qrcode.x = pose_qrcode2map.position.x;
        cur_qrcode.y = pose_qrcode2map.position.y;
        cur_qrcode.yaw = yaw_qrcode2map;
        poselog.push_back(cur_qrcode);
        // 累加
        sum.x = sum.x + cur_qrcode.x;
        sum.y = sum.y + cur_qrcode.y;
        sum.yaw = sum.yaw + cur_qrcode.yaw;
        sum.code = sum.code + 1; // 累加次数
    }

    // 输出平均后的结果
    QRcodeInfo average;
    average.x = sum.x / sum.code;
    average.y = sum.y / sum.code;
    average.yaw = sum.yaw / sum.code;
    stream.str("");
    stream << "sum.code: "
           << sum.code;
    logger->info(stream.str());
    return (average);
}
