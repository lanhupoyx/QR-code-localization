
#include "utility_qloc.hpp"

// 二维码坐标对照表
class QRcodeTable : public ParamServer
{
public:
    QRcodeTable(std::string dir)
    {
        path = dir; // 文件位置
        logger = &Logger::getInstance();
        logger->log("QRcodeTable Start");
        ifs.open(dir, std::ios::in);
        if (!ifs.is_open())
        {
            logger->log(dir + "打开失败!");
        }
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
        logger->log(stream.str());
        for (std::map<uint32_t, QRcodeInfo>::iterator it = map.begin(); it != map.end(); it++)
        {
            stream.str("");
            stream << (*it).second.code << " " << (*it).second.x << " " << (*it).second.y << " " << (*it).second.yaw;
            logger->log(stream.str());
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
        if (1 == mode)
        {
            sub_pos = nh.subscribe<nav_msgs::Odometry>("/ep_localization/odometry/lidar", 1,
                                                       &QRcodeTable::tfCallback, this,
                                                       ros::TransportHints().tcpNoDelay());
        }
    }

    ~QRcodeTable() {}

    // 根据二维码编号查表，得到位姿信息
    bool find_add(CameraFrame frame, QRcodeInfo *info)
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
            // logger->log(stream.str());
            std::cout << "can not identify code:" << frame.code << std::endl;
            add(frame);
        }
        return false;
    }

    // 根据二维码编号查表，得到位姿信息
    bool onlyfind(CameraFrame frame, QRcodeInfo *info)
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
            // logger->log(stream.str());
            std::cout << "can not identify code:" << frame.code << std::endl;
        }
        return false;
    }

    // 添加新的二维码
    bool add(const CameraFrame frame)
    {
        // std::lock_guard<std::mutex> locker(QRcodeTable::mtx);
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
            logger->log(stream.str());
            std::cout << stream.str() << std::endl;
        }

        return true;
    }

    // callback获取baselink位姿
    void tfCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> locker(QRcodeTable::mtx);
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

private:
    // 计算二维码位姿
    QRcodeInfo calPose()
    {
        std::lock_guard<std::mutex> locker(QRcodeTable::mtx);
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
        logger->log(stream.str());
        return (average);
    }

private:
    std::ifstream ifs;
    std::map<uint32_t, QRcodeInfo> map;
    std::map<uint32_t, QRcodeInfo> new_map;
    std::ofstream log_file;
    std::list<CameraFrame> frame_buffer;

    geometry_msgs::TransformStamped trans_camera2base;
    std::string path;

    ros::Subscriber sub_pos;

    std::mutex mtx;
    std::list<nav_msgs::Odometry> tf_buffer;

    Logger *logger;
    std::stringstream stream;
};

// 相机数据预处理
class MV_SC2005AM : public ParamServer
{
public:
    MV_SC2005AM()
    {
        logger = &Logger::getInstance();
        stream.str("");
        stream << "MV_SC2005AM Start"
               << std::endl;
        logger->log(stream.str());

        // UDP端口监测初始化
        socket = new boost::asio::ip::udp::socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), std::atoi(port.c_str())));
    }
    ~MV_SC2005AM() {}

    bool getframe(CameraFrame *frame)
    {
        if (socket->available())
        {
            socket->receive_from(boost::asio::buffer(recv_buf), sender_endpoint, 0, error);
            if (!error)
            {
                ros::Time now = ros::Time::now();
                std::string sender = sender_endpoint.address().to_string();
                frame->hex = charArrayToHex(recv_buf, 15);
                // std::cout << "hex is: " << frame->hex << std::endl;
                frame->sender = sender;

                // todo:判断帧头和字节校验和
                frame->head = frame->hex.substr(2, 2) + frame->hex.substr(0, 2);
                frame->sum = frame->hex.substr(28, 2); // 转换为整数

                // 数据提取与转换
                frame->index = convert_16_to_10(frame->hex.substr(4, 2));

                u_int32_t d = convert_16_to_10(frame->hex.substr(6, 2));
                frame->duration = d / 1000.0;
                frame->stamp = now; // - ros::Duration(frame->duration);

                frame->code = convert_16_to_10(frame->hex.substr(14, 2) + frame->hex.substr(12, 2) + frame->hex.substr(10, 2) + frame->hex.substr(8, 2));

                int16_t pixel_x = std::stoi(frame->hex.substr(18, 2) + frame->hex.substr(16, 2), 0, 16);
                int16_t pixel_y = std::stoi(frame->hex.substr(22, 2) + frame->hex.substr(20, 2), 0, 16);
                u_int32_t pixel_yaw = convert_16_to_10(frame->hex.substr(26, 2) + frame->hex.substr(24, 2));
                frame->error_x = pixel_x * 0.2125;
                frame->error_y = pixel_y * 0.2166667;
                frame->error_yaw = pixel_yaw / 100.0;

                if (show_msg)
                {
                    stream.str("");
                    stream << format_time(frame->stamp) << " [" << sender.c_str() << "] " << frame->code << " " << frame->index << " " << frame->duration << "s " // ip
                           << " " << frame->error_x << "mm " << frame->error_y << "mm " << frame->error_yaw;
                    logger->log(stream.str());
                }

                // std::cout << format_time(frame->stamp) << " [" << sender.c_str() << "] " << frame->code << " " << frame->index << " " << frame->duration << "s " // ip
                //           << " " << frame->error_x << "mm " << frame->error_y << "mm " << frame->error_yaw << std::endl;

                return true;
            }
            // log_file << "error receiving UDP data: " << error.message().c_str() << std::endl;
            stream.str("");
            stream << "error receiving UDP data: " << error.message().c_str();
            logger->log(stream.str());
        }
        return false;
    }

private:
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket *socket;
    boost::asio::ip::udp::endpoint sender_endpoint;
    std::array<char, 1024> recv_buf;
    boost::system::error_code error;
    std::ofstream log_file;
    Logger *logger;
    std::stringstream stream;
};

// 线段对象，用于划定运行区域
class LineSegment
{
private:
    float x1_, y1_, x2_, y2_;
    float a_, b_;

    Logger *logger;

public:
    LineSegment(float x1, float y1, float x2, float y2) : x1_(x1), y1_(y1), x2_(x2), y2_(y2)
    {
        logger = &Logger::getInstance();
        if (x1 == x2)
        {
            logger->log("error: x1 == x2");
        }
        a_ = (y1_ - y2_) / (x1_ - x2_);
        b_ = y1_ - a_ * x1_;
    }

    ~LineSegment() {}

    // 判定从该点出发的射线是否与本线段相交
    bool isCross(float x, float y)
    {
        float y_cross = a_ * x + b_;
        if (y_cross < y)
        {
            return false;
        }
        if (((y1_ < y_cross) && (y_cross < y2_)) || ((y2_ < y_cross) && (y_cross < y1_)))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};

// 区域对象，用于表示运行区域
class Polygon : public ParamServer
{
private:
    std::list<LineSegment> linelist;

public:
    Polygon()
    {
        int pointNum = enableArea.size() / 2;

        for (int i = 0; i < pointNum - 1; i++)
        {
            LineSegment line(enableArea[i * 2], enableArea[i * 2 + 1], enableArea[i * 2 + 2], enableArea[i * 2 + 3]);
            linelist.push_back(line);
        }
        LineSegment line2(enableArea[pointNum * 2], enableArea[pointNum * 2 + 1], enableArea[0], enableArea[1]);
        linelist.push_back(line2);
    }

    ~Polygon() {}

    // 判定该点是否为本区域内的点，区域内部点出发的射线与区域边相交的次数为奇数
    bool isInArea(float x, float y)
    {
        int corss_num = 0;
        for (std::list<LineSegment>::iterator it = linelist.begin(); it != linelist.end(); it++)
        {
            if (it->isCross(x, y))
            {
                corss_num++;
            }
        }
        if ((corss_num % 2) == 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};

// 二维码定位
class QRcodeLoc : public ParamServer
{
public:
    // 发布器
    ros::Publisher pub_odom_map_base;
    ros::Publisher pub_odom_map_camera;
    ros::Publisher pub_path_map_base;
    ros::Publisher pub_path_map_camera;

    ros::Publisher pub_odom_qrmap_base;
    ros::Publisher pub_odom_qrmap_camera;
    ros::Publisher pub_qrCodeMsg;

    // 接收器
    ros::Subscriber sub_realvel;
    ros::Subscriber sub_pos;

    QRcodeTable *QRmap_tab;
    QRcodeTable *Lidarmap_tab;
    MV_SC2005AM *camera;
    Polygon *operating_area;

    std::ofstream log_err;
    Logger *logger;
    std::stringstream stream;

    geometry_msgs::TransformStamped trans_base2camera;
    geometry_msgs::TransformStamped trans_camera2base;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener *tfListener;
    geometry_msgs::TransformStamped trans_base2map;

    std::mutex mtx;
    std::mutex odom_init_mtx;
    nav_msgs::Odometry odom_estimation_init;
    
    nav_msgs::Odometry odom_map_base;
    nav_msgs::Odometry odom_map_camera;
    nav_msgs::Path     path_map_base;
    nav_msgs::Path     path_map_camera;

    nav_msgs::Odometry odom_qrmap_base;
    nav_msgs::Odometry odom_qrmap_camera;

    nav_msgs::Odometry odom_map;
    nav_msgs::Odometry odom_map_last;
    nav_msgs::Odometry odom_map_last_last;
    nav_msgs::Odometry odom_qrmap;
    std::list<nav_msgs::Odometry> odom_his;

    CameraFrame pic; // 相机数据

    bool start;

    tf::TransformBroadcaster br;

    QRcodeLoc()
    {
        // 发布器
        logger = &Logger::getInstance();
        stream.str("");
        stream << "QRcodeLoc Start";
        logger->log(stream.str());

        // 初始化发布器
        pub_odom_map_base = nh.advertise<nav_msgs::Odometry>(odomMapBase, 2000);
        pub_odom_map_camera = nh.advertise<nav_msgs::Odometry>(odomMapCamera, 2000);
        pub_path_map_base = nh.advertise<nav_msgs::Path>(pathMapBase, 2000);
        pub_path_map_camera = nh.advertise<nav_msgs::Path>(pathMapCamera, 2000);

        pub_odom_qrmap_base = nh.advertise<nav_msgs::Odometry>(odomQrmapBase, 2000);
        pub_odom_qrmap_camera = nh.advertise<nav_msgs::Odometry>(odomQrmapCamera, 2000);
        pub_qrCodeMsg = nh.advertise<std_msgs::String>(msgTopic, 1000);

        // 订阅轮速计消息
        if (3 == mode) // 使用论速计递推
        {
            sub_realvel = nh.subscribe<geometry_msgs::Twist>("/real_vel", 1, &QRcodeLoc::realvelCallback,
                                                             this, ros::TransportHints().tcpNoDelay());
        }

        // 实例化两个表格、定位相机、运行区域
        QRmap_tab = new QRcodeTable(cfg_dir + "ep-qrcode-QRCodeMapTable.txt");
        Lidarmap_tab = new QRcodeTable(cfg_dir + "ep-qrcode-LidarMapTable.txt");
        camera = new MV_SC2005AM();
        operating_area = new Polygon();

        // 从TF获取baselink---->camera的变换关系
        tfListener = new tf2_ros::TransformListener(buffer);
        bool tferr = true;
        while (tferr)
        {
            tferr = false;
            try
            {
                trans_base2camera = buffer.lookupTransform("locCamera_link", "base_link", ros::Time(0));
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

        stream.str("");
        stream << " init "
               << std::endl;
        logger->log(stream.str());
    }

    ~QRcodeLoc()
    {
        delete QRmap_tab;
        delete Lidarmap_tab;
        delete camera;
    }

    // 计算lidar坐标系与二维码坐标系之间的转换关系，奇异值分解法
    std::vector<double> calTrans_SVD()
    {
        // 读取Lidarmap_table.txt
        std::ifstream ifs;
        std::map<uint32_t, QRcodeInfo> map;
        std::string path = cfg_dir + "/Lidarmap_table.txt";
        ifs.open(path, std::ios::in);
        if (!ifs.is_open())
        {
            std::cout << path + "打开失败!" << std::endl;
        }
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
        std::cout << "Lidarmap_table的大小为: " << map.size() << std::endl;
        for (std::map<uint32_t, QRcodeInfo>::iterator it = map.begin(); it != map.end(); it++)
        {
            std::cout << (*it).second.code << " " << (*it).second.x << " " << (*it).second.y << " " << (*it).second.yaw << std::endl;
        }

        // 读取QRmap_table.txt
        std::map<uint32_t, QRcodeInfo> qrmap;
        path = cfg_dir + "/QRmap_table.txt";
        ifs.open(path, std::ios::in);
        if (!ifs.is_open())
        {
            std::cout << path + "打开失败!" << std::endl;
        }
        while (std::getline(ifs, buf)) // 使用全局的getline()函数，其里面第一个参数代表输入流对象，第一个参数代表准备好的字符串，每次读取一行内容到buf
        {
            std::stringstream line_ss;
            line_ss << buf;
            QRcodeInfo info;
            line_ss >> info.code >> info.x >> info.y >> info.yaw;
            qrmap.insert(std::pair<uint32_t, QRcodeInfo>(info.code, info));
        }
        ifs.close();
        std::cout << "QRmap_table的大小为: " << qrmap.size() << std::endl;
        for (std::map<uint32_t, QRcodeInfo>::iterator it = qrmap.begin(); it != qrmap.end(); it++)
        {
            std::cout << (*it).second.code << " " << (*it).second.x << " " << (*it).second.y << " " << (*it).second.yaw << std::endl;
        }

        // 提取点坐标
        int n = map.size();
        if (n < 3)
        {
            std::cout << "less points" << std::endl;
            std::vector<double> result;
            return result;
        }
        std::vector<vec> qrmap_points;
        std::vector<vec> lidarmap_points;
        for (std::map<uint32_t, QRcodeInfo>::iterator lidar_it = map.begin(); lidar_it != map.end(); lidar_it++)
        {
            std::map<uint32_t, QRcodeInfo>::iterator qr_it = qrmap.find(lidar_it->first);
            if (qr_it == qrmap.end())
            {
                std::cout << "SVD can not identify code:" << lidar_it->first << std::endl;
                continue;
            }
            vec qrpoint(qr_it->second.x, qr_it->second.y, 0.0);
            vec lidarpoint(lidar_it->second.x, lidar_it->second.y, 0.0);
            qrmap_points.push_back(qrpoint);
            lidarmap_points.push_back(lidarpoint);
        }

        // 求质心
        n = qrmap_points.size();
        if (n < 3)
        {
            std::cout << "less points" << std::endl;
            std::vector<double> result;
            return result;
        }
        vec al(0, 0, 0); // lidar map平均坐标
        vec ar(0, 0, 0); // qrcode map平均坐标
        for (int i = 0; i < n; ++i)
        {
            al[0] += lidarmap_points[i][0] / n;
            al[1] += lidarmap_points[i][1] / n;
            al[2] = 0.0;

            ar[0] += qrmap_points[i][0] / n;
            ar[1] += qrmap_points[i][1] / n;
            ar[2] = 0.0;
        }

        // 去质心坐标装填矩阵
        Eigen::Matrix3d mleft = Eigen::Matrix3d::Zero();
        for (int i = 0; i < n; ++i)
        {
            mleft += Eigen::Vector3d(lidarmap_points[i][0] - al[0], lidarmap_points[i][1] - al[1], 0.0) * Eigen::Vector3d(qrmap_points[i][0] - ar[0], qrmap_points[i][1] - ar[1], 0.0).transpose();
        }

        // 奇异值分解求(R,t)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(mleft, Eigen::ComputeFullU | Eigen::ComputeFullV); // 对矩阵分解
        Eigen::Matrix3d U_ = svd.matrixU();
        Eigen::Matrix3d V_ = svd.matrixV();
        Eigen::Matrix3d R_ = U_ * (V_.transpose());
        // if(R_.determinant() < 0)
        // {
        //     R_ = -R_;
        // }
        // Eigen::Matrix3d mR_ = -R_;
        Eigen::Vector3d t_ = Eigen::Vector3d(al[0], al[1], 0.0) - R_ * Eigen::Vector3d(ar[0], ar[1], 0.0);
        // Eigen::Vector3d t_2 = Eigen::Vector3d(al[0], al[1], 0.0) - mR_ * Eigen::Vector3d(ar[0], ar[1], 0.0);

        std::vector<double> output;
        output.push_back(t_.x());
        output.push_back(t_.y());
        output.push_back(t_.z());
        Eigen::Quaterniond quaternion(R_);
        output.push_back(quaternion.x());
        output.push_back(quaternion.y());
        output.push_back(quaternion.z());
        output.push_back(quaternion.w());
        return output;
    }

    // 设置递推初始值
    void setEstimationInitialPose(nav_msgs::Odometry odom)
    {
        std::lock_guard<std::mutex> locker(QRcodeLoc::odom_init_mtx);
        odom_estimation_init = odom;
    }
    
    // 获取递推初始值
    nav_msgs::Odometry getEstimationInitialPose()
    {
        std::lock_guard<std::mutex> locker(QRcodeLoc::odom_init_mtx);
        return odom_estimation_init;
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
        odom_final.pose.covariance[2] = 1; // 此帧数据来源，0：二维码 1：轮速计递推

        // 时间戳
        odom_final.header.stamp = time_now;

        // 返回值
        return odom_final;
    }

    // 获取/realvel的回调函数
    void realvelCallback(const geometry_msgs::Twist::ConstPtr &velmsg)
    {
        if (3 == mode) // 使用轮速计递推
        {
            // 获取轮速(base_link)
            geometry_msgs::Twist vel; // 当前论速记数据
            vel = *velmsg;

            // 获取初始位姿
            nav_msgs::Odometry odom_init = getEstimationInitialPose();

            // 估计base当前时刻位姿(map--->base_link)
            nav_msgs::Odometry odom_est = poseEstimation(odom_init, vel, ros::Time::now());
            std::vector<nav_msgs::Odometry> v_odom;
            v_odom.push_back(odom_est);

            // 计算定位相机当前时刻位姿(map--->locCamera_link)
            geometry_msgs::Pose pose_camera2map;
            std::lock_guard<std::mutex> locker(QRcodeLoc::mtx);
            tf2::doTransform(t2p(trans_camera2base), pose_camera2map, p2t(odom_est.pose.pose));
            odom_est.pose.pose = pose_camera2map;
            odom_est.child_frame_id = "locCamera_link";
            v_odom.push_back(odom_est);

            // 发布递推后的位姿
            pubOdom(v_odom);
        }
    }

    // 发布定位结果
    void pubOdom(std::vector<nav_msgs::Odometry> v_odom)
    {
        // 主要需要输出的消息
        nav_msgs::Odometry odom_map_base = v_odom[0];
        nav_msgs::Odometry odom_map_camera = v_odom[1];

        // 更新由扫描二维码得到的位姿
        static nav_msgs::Odometry odom_hook;
        if (0 == odom_map_base.pose.covariance[2])
        {
            odom_hook = odom_map_base;
        }

        // 计算当前已经递推的距离
        float dis = sqrt(pow((odom_map.pose.pose.position.x - odom_hook.pose.pose.position.x), 2) +
                         pow((odom_map.pose.pose.position.y - odom_hook.pose.pose.position.y), 2));

        // 若递推的距离超限，则定位数据不可用
        if (dis > maxEstimationDis)
        {
            // map--->base_link
            odom_map_base.pose.covariance[0] = 0; // 此帧数据是否可用，0：不可用 1：可用
            odom_map_base.pose.covariance[1] = 1; // 此帧数据不可用的原因：0：离开运行区域 1：递推长度超出限制
            // map--->locCamera_link
            odom_map_camera.pose.covariance[0] = 0; // 此帧数据是否可用，0：不可用 1：可用
            odom_map_camera.pose.covariance[1] = 1; // 此帧数据不可用的原因：0：离开运行区域 1：递推长度超出限制
        }
        else // 递推的距离未超限
        {
            // 检测是否在运行区域内
            if (operating_area->isInArea(odom_map_base.pose.pose.position.x, odom_map_base.pose.pose.position.y))
            {
                // map--->base_link
                odom_map_base.pose.covariance[0] = 1; // 此帧数据是否可用，0：不可用 1：可用
                // map--->locCamera_link
                odom_map_camera.pose.covariance[0] = 1; // 此帧数据是否可用，0：不可用 1：可用
            }
            else
            {
                // map--->base_link
                odom_map_base.pose.covariance[0] = 0; // 此帧数据是否可用，0：不可用 1：可用
                odom_map_base.pose.covariance[1] = 0; // 此帧数据不可用的原因：0：离开运行区域 1：递推长度超出限制
                // map--->locCamera_link
                odom_map_camera.pose.covariance[0] = 0; // 此帧数据是否可用，0：不可用 1：可用
                odom_map_camera.pose.covariance[1] = 0; // 此帧数据不可用的原因：0：离开运行区域 1：递推长度超出限制
            }
        }

        // 发布odom消息
        pub_odom_map_base.publish(odom_map_base);
        pub_odom_map_camera.publish(odom_map_camera);

        // 输出的路线消息
        static nav_msgs::Path path_map_base;
        static nav_msgs::Path path_map_camera;

        if(1 == odom_map_base.pose.covariance[0]) // 此帧数据是否可用，不可用不计入路线消息
        {
            geometry_msgs::PoseStamped poseStamped;

            path_map_base.header = odom_map_base.header;
            poseStamped.header = odom_map_base.header;
            poseStamped.pose = odom_map_base.pose.pose;
            path_map_base.poses.push_back(poseStamped);
            pub_path_map_base.publish(path_map_base);

            path_map_camera.header = odom_map_camera.header;
            poseStamped.header = odom_map_camera.header;
            poseStamped.pose = odom_map_camera.pose.pose;
            path_map_camera.poses.push_back(poseStamped);
            pub_path_map_camera.publish(path_map_camera);
        }

        if(4 == v_odom.size()) // 4个odom，扫二维码得到
        {
            // 测试需要输出的消息
            nav_msgs::Odometry odom_qrmap_base = v_odom[2];
            nav_msgs::Odometry odom_qrmap_camera = v_odom[3];

            pub_odom_qrmap_base.publish(odom_qrmap_base);
            pub_odom_qrmap_camera.publish(odom_qrmap_camera);
        }

        // 保存到log
        save_log(odom_map, pic.code);
    }

    // 发布TF
    void pubTf(const nav_msgs::Odometry odom)
    {
        // 定义变换
        tf::Transform transform;
        static nav_msgs::Odometry odom_history;

        // 发布odom->map
        transform.setOrigin(tf::Vector3(0, 0, 0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "map", "odom"));

        // 发布base_link->odom
        transform.setOrigin(tf::Vector3(odom.pose.pose.position.x,
                                        odom.pose.pose.position.y,
                                        odom.pose.pose.position.z));
        transform.setRotation(tf::Quaternion(odom.pose.pose.orientation.x,
                                             odom.pose.pose.orientation.y,
                                             odom.pose.pose.orientation.z,
                                             odom.pose.pose.orientation.w));
        br.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "odom", "base_link"));

        double xerror = odom.pose.pose.position.x - odom_history.pose.pose.position.x;
        double yerror = odom.pose.pose.position.y - odom_history.pose.pose.position.y;
        std::stringstream stream;
        stream.str("");
        stream << format_time(odom.header.stamp)
               << std::fixed << std::setprecision(4)
               << "," << sqrt(pow(xerror, 2) + pow(yerror, 2))
               << "," << xerror
               << "," << yerror
               << "," << getYaw(odom.pose.pose) - getYaw(odom_history.pose.pose)
               << "," << odom.header.stamp.toSec()
               << "," << odom.pose.pose.position.x
               << "," << odom.pose.pose.position.y
               << "," << getYaw(odom.pose.pose);
        logger->log(stream.str());
        odom_history = odom;
    }

    // 保存error
    void save_error(const nav_msgs::Odometry msg, uint32_t code)
    {
        static std::list<nav_msgs::Odometry> odom_map_history;

        // 保存历史帧
        odom_map_history.push_back(msg);

        // 控制数量
        uint8_t frame_num = 40;
        if (odom_map_history.size() > frame_num)
        {
            odom_map_history.pop_front();
        }

        if (odom_map_history.size() < 2)
        {
            return;
        }
        // 筛选情况，记录跳变
        if (1 == odom_map_history.back().pose.covariance[0])
        {
            int sum = 0;
            for (std::list<nav_msgs::Odometry>::iterator it = odom_map_history.begin(); it != odom_map_history.end(); it++)
            {
                sum += it->pose.covariance[0];
            }
            if (frame_num * 2 - 1 == sum)
            {
                nav_msgs::Odometry BackofJump = odom_map_history.back();
                odom_map_history.pop_back();
                nav_msgs::Odometry BeforeofJump = odom_map_history.back();
                odom_map_history.push_back(BackofJump);

                double xerror = BackofJump.pose.pose.position.x - BeforeofJump.pose.pose.position.x;
                double yerror = BackofJump.pose.pose.position.y - BeforeofJump.pose.pose.position.y;

                std::stringstream stream;
                stream.str("");
                stream << format_time(BackofJump.header.stamp)
                       << "," << code << std::fixed << std::setprecision(4)
                       << "," << BackofJump.header.stamp.toSec()
                       << "," << sqrt(pow(xerror, 2) + pow(yerror, 2))
                       << "," << xerror
                       << "," << yerror
                       << "," << getYaw(BackofJump.pose.pose) - getYaw(BeforeofJump.pose.pose)
                       << "," << BackofJump.pose.pose.position.x
                       << "," << BackofJump.pose.pose.position.y
                       << "," << getYaw(BackofJump.pose.pose)
                       << "," << BeforeofJump.header.stamp.toSec()
                       << "," << BeforeofJump.pose.pose.position.x
                       << "," << BeforeofJump.pose.pose.position.y
                       << "," << getYaw(BeforeofJump.pose.pose);
                logger->log(stream.str());
            }
        }
    }

    // 保存log
    void save_log(const nav_msgs::Odometry msg, uint32_t code)
    {
        // 获取当前定位
        try
        {
            trans_base2map = buffer.lookupTransform("odom", "base_link", ros::Time(0));
        }
        catch (tf::TransformException &exception)
        {
            ROS_WARN("%s; retrying...", exception.what());
            stream.str("");
            stream << exception.what() << "; retrying...";
            logger->log(stream.str());
            return;
        }

        double xerror = msg.pose.pose.position.x - trans_base2map.transform.translation.x;
        double yerror = msg.pose.pose.position.y - trans_base2map.transform.translation.y;

        std::stringstream stream;
        stream.str("");
        stream << format_time(msg.header.stamp)
               << "," << code << std::fixed << std::setprecision(3)
               << "," << sqrt(pow(xerror, 2) + pow(yerror, 2))
               << "," << xerror
               << "," << yerror
               << "," << getYaw(msg.pose.pose) - getYaw(trans_base2map.transform.rotation)
               << "," << msg.header.stamp.toSec()
               << "," << msg.pose.pose.position.x
               << "," << msg.pose.pose.position.y
               << "," << getYaw(msg.pose.pose)
               << "," << trans_base2map.header.stamp.toSec()
               << "," << trans_base2map.transform.translation.x
               << "," << trans_base2map.transform.translation.y
               << "," << getYaw(trans_base2map.transform.rotation)
               << "," << msg.pose.covariance[0];
        logger->log(stream.str());
    }

    // 计算base_link在map坐标系下的坐标
    geometry_msgs::Pose get_pose_lidarmap(CameraFrame pic, QRcodeInfo code_info)
    {
        // 二维码到map变换关系
        geometry_msgs::Pose pose_qrcode2map;
        pose_qrcode2map.position.x = code_info.x;
        pose_qrcode2map.position.y = code_info.y;
        pose_qrcode2map.position.z = 0.0;
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, code_info.yaw * M_PI / 180);
        tf::quaternionTFToMsg(q, pose_qrcode2map.orientation);

        //  相机到二维码变换关系
        geometry_msgs::Pose pose_camera2qrcode;
        q.setRPY(0.0, 0.0, pic.error_yaw * M_PI / 180);
        tf::quaternionTFToMsg(q, pose_camera2qrcode.orientation);
        tf::Matrix3x3 m(q);
        tf::Matrix3x3 inv_m(m);
        tf::Vector3 v(pic.error_x / 1000.0, pic.error_y / -1000.0, 0.0);
        inv_m.inverse();                                         // 求逆
        pose_camera2qrcode.position.x = -inv_m.getRow(0).dot(v); //(row0(0)*v.getX() + inv_m(0,1)*v.getX() +inv_m(0,2)*v.getZ());
        pose_camera2qrcode.position.y = -inv_m.getRow(1).dot(v); //-(inv_m(1,0)*v.getX() + inv_m(1,1)*v.getX() +inv_m(1,2)*v.getZ());
        pose_camera2qrcode.position.z = 0.0;
        // 相机到map位姿
        geometry_msgs::Pose pose_camera2map;
        tf2::doTransform(pose_camera2qrcode, pose_camera2map, p2t(pose_qrcode2map));
        // double yaw_camera2map = getYaw(pose_camera2map);
        //  base到map位姿
        geometry_msgs::Pose pose_base2map;
        tf2::doTransform(t2p(trans_base2camera), pose_base2map, p2t(pose_camera2map));

        return pose_base2map;
    }

    // 计算base_link在qrmap和map坐标系下的坐标
    std::vector<geometry_msgs::Pose> get_pose(QRcodeInfo code_info)
    {
        // 二维码到QRmap
        geometry_msgs::Pose pose_qrcode2qrmap;
        pose_qrcode2qrmap.position.x = code_info.x;
        pose_qrcode2qrmap.position.y = code_info.y;
        pose_qrcode2qrmap.position.z = 0.0;
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, code_info.yaw * M_PI / 180);
        tf::quaternionTFToMsg(q, pose_qrcode2qrmap.orientation);

        //  相机到二维码
        geometry_msgs::Pose pose_camera2qrcode;
        q.setRPY(0.0, 0.0, code_info.frame.error_yaw * M_PI / 180);
        tf::quaternionTFToMsg(q, pose_camera2qrcode.orientation);
        tf::Matrix3x3 m(q);
        tf::Matrix3x3 inv_m(m);
        tf::Vector3 v(code_info.frame.error_x / 1000.0, code_info.frame.error_y / -1000.0, 0.0);
        inv_m.inverse();                                         // 求逆
        pose_camera2qrcode.position.x = -inv_m.getRow(0).dot(v); //(row0(0)*v.getX() + inv_m(0,1)*v.getX() +inv_m(0,2)*v.getZ());
        pose_camera2qrcode.position.y = -inv_m.getRow(1).dot(v); //-(inv_m(1,0)*v.getX() + inv_m(1,1)*v.getX() +inv_m(1,2)*v.getZ());
        pose_camera2qrcode.position.z = 0.0;

        // 相机到QRmap
        geometry_msgs::Pose pose_camera2qrmap;
        tf2::doTransform(pose_camera2qrcode, pose_camera2qrmap, p2t(pose_qrcode2qrmap));

        // 相机到map_copy
        geometry_msgs::Pose pose_camera2mapcopy;
        tf2::doTransform(pose_camera2qrmap, pose_camera2mapcopy, p2t(pose_qrmap2mapcopy));

        //  base_link到QRmap
        geometry_msgs::Pose pose_base2qrmap;
        tf2::doTransform(t2p(trans_base2camera), pose_base2qrmap, p2t(pose_camera2qrmap));

        //  base_link到map_copy
        geometry_msgs::Pose pose_base2mapcopy;
        tf2::doTransform(pose_base2qrmap, pose_base2mapcopy, p2t(pose_qrmap2mapcopy));

        std::vector<geometry_msgs::Pose> output;
        output.push_back(pose_base2mapcopy);
        output.push_back(pose_camera2mapcopy);
        output.push_back(pose_base2qrmap);
        output.push_back(pose_camera2qrmap);

        return output;
    }

    // 主循环
    void mainloopThread()
    {
        if (1 == mode) // 采集二维码位姿
        {
            ROS_INFO("mode: 1");
            ros::Rate loop_rate(100); // 主循环 100Hz
            while (ros::ok())
            {
                if (camera->getframe(&pic))
                {
                    // 查询二维码坐标
                    QRcodeInfo code_info;
                    if (Lidarmap_tab->find_add(pic, &code_info))
                    {
                        geometry_msgs::Pose pose_base2map = get_pose_lidarmap(pic, code_info);

                        // 发布 /qrCodeMsg
                        if (show_msg)
                        {
                            std::stringstream pub_ss;
                            pub_ss << format_time(pic.stamp) << " [" << pic.sender << "] " << pic.index << " " << pic.duration << "s " // ip
                                   << " " << pic.error_x << "mm " << pic.error_y << "mm " << pic.error_yaw << " yaw_base2map:" << getYaw(pose_base2map);
                            std_msgs::String msg;
                            msg.data = pub_ss.str();
                            pub_qrCodeMsg.publish(msg);
                        }
                    }
                }
                loop_rate.sleep();
                ros::spinOnce();
            }
        }
        else if (2 == mode) // 使用SVD计算qmap与lidarmap间的转换关系
        {
            ROS_INFO("mode: 2");
            std::vector<double> trans = calTrans_SVD();
            for (std::vector<double>::iterator it = trans.begin(); it != trans.end(); it++)
            {
                std::cout << *it << ", ";
            }
            std::cout << std::endl;
            return;
        }
        else if (3 == mode) // 使用论速计递推
        {
            ROS_INFO("mode: 3 ");
            QRcodeInfo code_info;     // 查询二维码坐标
            ros::Rate loop_rate(100); // 主循环 100Hz
            while (ros::ok())
            {
                if (camera->getframe(&pic))
                {
                    if (QRmap_tab->onlyfind(pic, &code_info))
                    {
                        std::lock_guard<std::mutex> locker(QRcodeLoc::mtx);

                        // 计算base_link在qrmap坐标和map坐标的坐标
                        std::vector<geometry_msgs::Pose> pose = get_pose(code_info);

                        // 定义变量
                        std::vector<nav_msgs::Odometry> v_odom;
                        nav_msgs::Odometry odom;

                        // 相同的项
                        odom.header.stamp = pic.stamp;
                        odom.header.seq = pic.index;
                        odom.pose.covariance[2] = 0; // 此帧数据来源，0：二维码 1：轮速计递推  

                        // map ---> base_link
                        odom.header.frame_id = "map";
                        odom.child_frame_id = "base_link";
                        odom.pose.pose = pose[0];
                        v_odom.push_back(odom);

                        // 设置轮速计递推的初始值
                        setEstimationInitialPose(odom);

                        // map ---> locCamera_link
                        odom.header.frame_id = "map";
                        odom.child_frame_id = "locCamera_link";
                        odom.pose.pose = pose[1];
                        v_odom.push_back(odom);

                        // qrmap ---> base_link
                        odom.header.frame_id = "qrmap";
                        odom.child_frame_id = "base_link";
                        odom.pose.pose = pose[2];
                        v_odom.push_back(odom);

                        // qrmap ---> locCamera_link
                        odom.header.frame_id = "qrmap";
                        odom.child_frame_id = "locCamera_link";
                        odom.pose.pose = pose[3];
                        v_odom.push_back(odom);

                        // 发布消息
                        pubOdom(v_odom);
                    }
                }

                loop_rate.sleep();
                ros::spinOnce();
            }
        }
        else
        {
            std::cout << "mode未识别" << std::endl;
        }
        return;
    }
};

// 主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "qrcode_loc");

    QRcodeLoc QLoc;

    ROS_INFO("\033[1;32m----> Localization with QRcode Started.\033[0m");

    std::thread loopthread(&QRcodeLoc::mainloopThread, &QLoc);

    ros::spin();

    loopthread.join();

    return 0;
}