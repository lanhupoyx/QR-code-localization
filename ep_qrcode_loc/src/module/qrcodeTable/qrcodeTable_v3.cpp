#include "qrcodeTable_v3.hpp"

QRcodeColumn::QRcodeColumn(uint32_t index_, double x_, double y_, double yaw_, double space_, double yawOffset_)
{
    index = index_;
    space = space_;
    yawOffset = yawOffset_;
    pose_x = x_;
    pose_y = y_;
    pose_yaw = yaw_;
    first_pose = toPoseRad(x_, y_, yaw_);
}

QRcodeColumn::~QRcodeColumn() {}

void QRcodeColumn::addQRcode(uint32_t code_index, uint32_t row_index, double x_offset, double y_offset, double yaw_offset)
{
    QRcodeGround newcode;
    if (qrcodes.size() == 0)
    {
        newcode.is_head = true;
    }
    else
    {
        newcode.is_head = false;
    }
    newcode.index_ = code_index;
    newcode.column_index_ = this->index;
    newcode.row_index_ = row_index;
    newcode.pose_ = poseMove(first_pose, -1 * space * (row_index - 1) + x_offset, y_offset);
    newcode.turn(yaw_offset);
    qrcodes.push_back(newcode);
}

geometry_msgs::Pose QRcodeColumn::poseMove(geometry_msgs::Pose pose, double dis_front, double dis_left)
{
    geometry_msgs::Pose pose_move;

    pose_move.position.x = pose.position.x + dis_front * cos(getYawRad(pose));
    pose_move.position.y = pose.position.y + dis_front * sin(getYawRad(pose));

    pose_move.position.x += dis_left * (-1) * sin(getYawRad(pose)); // cos(x+90) = -sin(x)
    pose_move.position.y += dis_left * cos(getYawRad(pose));        // sin(x+90) = cos(x)

    pose_move.orientation = pose.orientation;
    return pose_move;
}

QRcodeTableV3::QRcodeTableV3(ParamServer &param) : QRcodeTable(param)
{
    // 记录器
    logger->info(std::string(__FUNCTION__) + "() start");

    loadCodeTable();

    sub_pos = param.nh.subscribe<nav_msgs::Odometry>("/ep_localization/odometry/lidar", 1,
                                                     &QRcodeTableV3::lidarPoseCallback, this,
                                                     ros::TransportHints().tcpNoDelay());
    logger->info("sub: /ep_localization/odometry/lidar");

    logger->info(std::string(__FUNCTION__) + "() return");
}

QRcodeTableV3::~QRcodeTableV3() {}

bool QRcodeTableV3::loadCodeTable()
{
    logger->info("QRcodeTableV3::loadCodeTable() start");

    // 打开文件
    std::string GroundCodeTablePath = param.GroundCodeTablePath;
    std::ifstream ifs;
    ifs.open(GroundCodeTablePath, std::ios::in);
    if (!ifs.is_open())
    {
        logger->info(GroundCodeTablePath + "打开失败!");
        return false;
    }
    else
    {
        logger->info(GroundCodeTablePath + "打开成功!");
    }

    // 读取二维码信息
    std::string buf;
    while (std::getline(ifs, buf))
    {
        logger->info(buf);//记录原始文本

        buf = replaceChar(buf, ',', ' '); // ','替换为' '
        if (("" == buf) || (' ' == buf[0]))
            continue; // 跳过空行

        std::stringstream line_ss;
        line_ss << buf;

        // 行列信息
        uint32_t column_index, unused, row_index;
        line_ss >> column_index >> unused >> row_index;

        if (0 == row_index) // 0行存储本列公共信息
        {
            // 处理列信息
            double x_first, y_first, yaw_first, space, list_yaw_offset;
            line_ss >> x_first >> y_first >> yaw_first >> space >> list_yaw_offset;
            columns.push_back(QRcodeColumn(column_index, x_first, y_first, yaw_first, space, list_yaw_offset));
        }
        else // 非0行存储本行地码信息
        {
            // 处理地码信息
            uint32_t code_index;
            double x_offset, y_offset, yaw_offset;
            line_ss >> code_index >> x_offset >> y_offset >> yaw_offset;
            columns.back().addQRcode(code_index, row_index, x_offset, y_offset, yaw_offset + columns.back().yawOffset);
        }
    }
    ifs.close();
    logger->info(GroundCodeTablePath + "读取完毕!");

    // 遍历每列
    for (std::vector<QRcodeColumn>::iterator column_it = columns.begin(); column_it != columns.end(); column_it++)
    {
        logger->info(std::to_string(column_it->index) + ", " +
                     std::to_string(column_it->pose_x) + ", " +
                     std::to_string(column_it->pose_y) + ", " +
                     std::to_string(column_it->pose_yaw) + ", " +
                     std::to_string(column_it->space) + ", " +
                     std::to_string(column_it->yawOffset));

        // 遍历每排
        for (std::vector<QRcodeGround>::iterator item_it = column_it->qrcodes.begin(); item_it != column_it->qrcodes.end(); item_it++)
        {
            logger->info(std::to_string(column_it->index) + ", " +
                         std::to_string(item_it->row_index_) + ", " +
                         std::to_string(item_it->index_) + ", " +
                         std::to_string(item_it->pose_.position.x) + ", " +
                         std::to_string(item_it->pose_.position.y) + ", " +
                         std::to_string(getYaw(item_it->pose_.orientation)));

            QRcodeInfo info(item_it->index_,
                            item_it->pose_.position.x,
                            item_it->pose_.position.y,
                            getYaw(item_it->pose_),
                            item_it->is_head);
            map.insert(std::pair<uint32_t, QRcodeInfo>(info.code, info));
        }
    }

    logger->info("qrcode_table的大小为: " + std::to_string(map.size()));

    for (std::map<uint32_t, QRcodeInfo>::iterator it = map.begin(); it != map.end(); it++)
    {
        logger->debug(  std::to_string((*it).second.code) + ", " +
                        std::to_string((*it).second.x) + ", " +
                        std::to_string((*it).second.y) + ", " +
                        std::to_string((*it).second.yaw));
    }

    logger->info("QRcodeTableV3::loadCodeTable() return");
    return true;
}

bool QRcodeTableV3::onlyfind(CameraFrame frame, QRcodeInfo *info)
{
    logger->debug("QRcodeTableV3::onlyfind() start");
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
    logger->debug("QRcodeTableV3::onlyfind() end false");
    return false;
}

bool QRcodeTableV3::onlyfind(CameraFrame frame)
{
    logger->debug("QRcodeTableV3::onlyfind() start");
    std::lock_guard<std::mutex> locker(mtx);
    std::map<uint32_t, QRcodeInfo>::iterator it = map.find(frame.code);
    if (it != map.end())
    {
        return true;
    }
    else
    {
        std::cout << "can not identify code:" << frame.code << std::endl;
        logger->debug("QRcodeTableV3::onlyfind() end false");
        return false;
    }
}

void QRcodeTableV3::lidarPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> locker(mtx);
    const nav_msgs::Odometry transform = *msg; //->transforms[i];

    tf_buffer.push_back(transform);
    while (20 < tf_buffer.size())
    {
        tf_buffer.pop_front();
    }
    logger->debug("QRcodeTableV3 lidarPoseCallback()");
}

bool QRcodeTableV3::getCameraPose(geometry_msgs::Pose &pose_camera2map)
{
    logger->debug("QRcodeTableV3::getCameraPose(): start");
    std::lock_guard<std::mutex> locker(mtx);
    if (tf_buffer.size() > 0)
    {
        nav_msgs::Odometry base2map = tf_buffer.back();
        geometry_msgs::Pose pose_camera2base = t2p(param.trans_camera2base);
        pose_camera2map = transformPoint(   pose_camera2base,
                                            base2map.pose.pose.position.x,
                                            base2map.pose.pose.position.y,
                                            getYawRad(base2map.pose.pose.orientation));
        logger->debug("QRcodeTableV3::getCameraPose(): return true");
        return true;
    }
    else
    {
        logger->info("QRcodeTableV3::getCameraPose(): return false");
        return false;
    }
}

std::vector<uint32_t> QRcodeTableV3::get_all_head()
{
    static std::vector<uint32_t> v_out;

    if (v_out.size() == 0) // 只在第一次执行时计算一次
    {
        for (std::map<uint32_t, QRcodeInfo>::iterator it = map.begin(); it != map.end(); it++)
        {
            if ((*it).second.is_head)
            {
                v_out.push_back((*it).second.code);
            }
        }
    }

    return v_out;
}

bool QRcodeTableV3::is_head(uint32_t code_new)
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

std::vector<uint32_t> QRcodeTableV3::get_neighbor(uint32_t base_code)
{
    // 创建二维码矩阵
    static std::vector<std::vector<uint32_t>> code_matrix;
    if (code_matrix.size() == 0) // 只在第一次执行时计算一次
    {
        logger->info("get code_matrix:");
        // 遍历每列
        for (std::vector<QRcodeColumn>::iterator column_it = columns.begin(); column_it != columns.end(); column_it++)
        {
            logger->info("column " + std::to_string(column_it->index) + ":");
            std::vector<uint32_t> code_list;
            // 遍历每个库位
            for (std::vector<QRcodeGround>::iterator code_it = column_it->qrcodes.begin(); code_it != column_it->qrcodes.end(); code_it++)
            {
                code_list.push_back(code_it->index_);
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

bool QRcodeTableV3::isAGVInQueue(double head_offset)
{
    logger->debug("QRcodeTableV3::isAGVInQueue() start");
    // 获取相机位姿
    geometry_msgs::Pose cameraPose;
    bool res = getCameraPose(cameraPose);
    if(!res)
    {
        logger->debug("QRcodeTableV3::isAGVInQueue() return false");
        return false;
    }

    // 遍历每列
    for (std::vector<QRcodeColumn>::iterator column_it = columns.begin(); column_it != columns.end(); column_it++)
    {
        if (isPointInColumn(cameraPose, *column_it, head_offset, 0.6))
        {
            logger->debug("QRcodeTableV3::isAGVInQueue() return true");
            return true;
        }
    }
    logger->debug("QRcodeTableV3::isAGVInQueue() return false");
    return false;
}

bool QRcodeTableV3::isPointInColumn(const geometry_msgs::Pose point, const QRcodeColumn &column, double offset_x, double range_y)
{
    logger->debug("QRcodeTableV3::isPointInColumn() start");
    if(column.qrcodes.size() == 0)
    {
        logger->debug("QRcodeTableV3::isPointInColumn() : column.qrcodes.size()=" + std::to_string(column.qrcodes.size()));
        return false;
    }
    // 局部坐标系在map中的原点和方向，以列尾地码为原点，列方向为x轴方向
    double local_ox = column.qrcodes.back().pose_.position.x;
    double local_oy = column.qrcodes.back().pose_.position.y;
    double local_theta = getYawRad(column.first_pose);

    // 目标点变换到局部坐标系
    geometry_msgs::Pose point_local = transformPoint(point, local_ox, local_oy, local_theta);
    // 列首地码坐标变换到局部坐标系
    geometry_msgs::Pose head_local = transformPoint(column.qrcodes.front().pose_, local_ox, local_oy, local_theta);

    // 列首约束
    double cameraToFrontLine = point_local.position.x - (head_local.position.x + offset_x);
    if (cameraToFrontLine > 0)
    {
        logger->debug("QRcodeTableV3::isPointInColumn() return false");
        return false;
    }
        
    // 列尾约束
    double cameraToBackLine = point_local.position.x - (-0.7);
    if (cameraToBackLine < 0)
    {
        logger->debug("QRcodeTableV3::isPointInColumn() return false");
        return false;
    }
        
    // 横向约束
    if (abs(point_local.position.y) > range_y)
    {
        logger->debug("QRcodeTableV3::isPointInColumn() return false");
        return false;
    }
        
    logger->debug("QRcodeTableV3::isPointInColumn() return true");
    return true;
}

geometry_msgs::Pose QRcodeTableV3::transformPoint(const geometry_msgs::Pose pointA, double T_x, double T_y, double theta)
{
    // 平移
    double x_B_prime = pointA.position.x - T_x;
    double y_B_prime = pointA.position.y - T_y;

    // 旋转
    double cos_theta = cos(-1 * theta);
    double sin_theta = sin(-1 * theta);

    geometry_msgs::Pose pointB;
    pointB.position.x = x_B_prime * cos_theta - y_B_prime * sin_theta;
    pointB.position.y = x_B_prime * sin_theta + y_B_prime * cos_theta;

    return pointB;
}

void QRcodeTableV3::test()
{
    logger->info("QRcodeTableV3::test()");

    logger->info("QRcodeTableV3::test() end");
}

bool QRcodeTableV3::check_is_code_in_order(uint32_t code_new, double vel_x, bool reset)
{
    static uint32_t last_code = 0;

    if (!param.check_sequence)
    {
        return true;
    }

    if (reset)
    {
        last_code = 0;
        return true;
    }
    else if (last_code == code_new)
    {
        logger->debug("check_is_code_in_order: same code, return true");
        return true;
    }
    else if (0 == last_code) // 首次扫码
    {
        if (is_head(code_new))
        {
            last_code = code_new;
            logger->debug("check_is_code_in_order: is head, return true");
            return true;
        }
        else
        {
            logger->info("check_is_code_in_order: not head, return false  code_new=" + std::to_string(code_new));
            last_code = code_new;
            // return false;//必须扫到列首地码
            return true; // 不需要扫到列首地码
        }
    }
    else
    {
        std::vector<uint32_t> nbr = get_neighbor(last_code);
        if ((vel_x > 0) && (code_new == nbr[0]))
        {
            if (is_head(code_new))
            {
                last_code = 0;
                logger->debug("check_is_code_in_order: forward out, return true");
            }
            else
            {
                last_code = code_new;
                logger->debug("check_is_code_in_order: forward, return true");
            }
            return true;
        }
        else if ((vel_x < 0) && (code_new == nbr[1]))
        {
            last_code = code_new;
            logger->debug("check_is_code_in_order: retreat, return true");
            return true;
        }
        else
        {
            last_code = code_new;
            logger->info("check_is_code_in_order: other, return false  code_new=" + std::to_string(code_new) +
                         " vel_x=" + std::to_string(vel_x) +
                         " nbr[0]=" + std::to_string(nbr[0]) +
                         " nbr[1]=" + std::to_string(nbr[1]));
            return false;
        }
    }
}
