
#include "utility_qloc.hpp"
#include "polygon.hpp"
#include "wheel_odom.hpp"
#include "camera.hpp"
#include "qrcode_table.hpp"
#include "qrcode_table_v2.hpp"
#include "div.hpp"
#include "err.hpp"

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
    ros::Subscriber sub_pos;

    QRcodeTable *QRmap_tab;
    QRcodeTable *Lidarmap_tab;
    MV_SC2005AM *camera;
    QRcodeTableV2 *qrcode_table;
    err *err_y;

    Polygon *operating_area;
    WheelSpeedOdometer *wheel_odom;

    std::ofstream log_err;
    Logger *logger;
    std::stringstream stream;

    geometry_msgs::TransformStamped trans_base2camera;
    geometry_msgs::TransformStamped trans_camera2base;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener *tfListener;
    geometry_msgs::TransformStamped trans_base2map;

    std::mutex mtx;
    
    
    
    nav_msgs::Odometry odom_map_base;
    nav_msgs::Odometry odom_map_camera;

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

    // 构造函数
    QRcodeLoc()
    {
        // 发布器
        logger = &Logger::getInstance();
        logger->info("QRcodeLoc Start");

        // 初始化发布器
        pub_odom_map_base = nh.advertise<nav_msgs::Odometry>(odomMapBase, 1);
        pub_odom_map_camera = nh.advertise<nav_msgs::Odometry>(odomMapCamera, 1);
        pub_path_map_base = nh.advertise<nav_msgs::Path>(pathMapBase, 1);
        pub_path_map_camera = nh.advertise<nav_msgs::Path>(pathMapCamera, 1);

        pub_odom_qrmap_base = nh.advertise<nav_msgs::Odometry>(odomQrmapBase, 1);
        pub_odom_qrmap_camera = nh.advertise<nav_msgs::Odometry>(odomQrmapCamera, 1);
        pub_qrCodeMsg = nh.advertise<std_msgs::String>(msgTopic, 1);

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

        // 实例化两个表格、定位相机、运行区域
        // QRmap_tab = new QRcodeTable(cfg_dir + "ep-qrcode-QRCodeMapTable.txt");
        qrcode_table = new QRcodeTableV2(cfg_dir, trans_camera2base);

        camera = new MV_SC2005AM();
        operating_area = new Polygon();
        wheel_odom = new WheelSpeedOdometer(trans_camera2base);
        err_y = new err(cfg_dir);

        stream.str("");
        stream << " init "
               << std::endl;
        logger->info(stream.str());
    }

    // 析构函数
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
        else
        {
            odom_map_base.pose.covariance[3] = 0; //递推得到，不是列首二维码
        }

        // 计算当前已经递推的距离
        float dis = sqrt(pow((odom_map_base.pose.pose.position.x - odom_hook.pose.pose.position.x), 2) +
                         pow((odom_map_base.pose.pose.position.y - odom_hook.pose.pose.position.y), 2));

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
            if (ignore_area || operating_area->isInArea(odom_map_base.pose.pose.position.x, odom_map_base.pose.pose.position.y))
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

        // 发布TF
        if(is_pub_tf)
        {
            pubTf(odom_map_base);
        }

        // 输出的路线消息
        if(1 == odom_map_base.pose.covariance[0]) // 此帧数据是否可用，不可用不计入路线消息
        {
            geometry_msgs::PoseStamped poseStamped;
            static nav_msgs::Path path_map_base;
            static nav_msgs::Path path_map_camera;

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
        //save_log(odom_map, pic.code);
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

        // double xerror = odom.pose.pose.position.x - odom_history.pose.pose.position.x;
        // double yerror = odom.pose.pose.position.y - odom_history.pose.pose.position.y;
        // std::stringstream stream;
        // stream.str("");
        // stream << format_time(odom.header.stamp)
        //        << std::fixed << std::setprecision(4)
        //        << "," << sqrt(pow(xerror, 2) + pow(yerror, 2))
        //        << "," << xerror
        //        << "," << yerror
        //        << "," << getYaw(odom.pose.pose) - getYaw(odom_history.pose.pose)
        //        << "," << odom.header.stamp.toSec()
        //        << "," << odom.pose.pose.position.x
        //        << "," << odom.pose.pose.position.y
        //        << "," << getYaw(odom.pose.pose);
        // logger->info(stream.str());
        odom_history = odom;
        //std::cout << stream.str() << std::endl;
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
                logger->info(stream.str());
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
            logger->info(stream.str());
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
        logger->info(stream.str());
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
        q.setRPY(0.0, 0.0, code_info.frame.error_yaw * M_PI / 180);
        geometry_msgs::Pose pose_camera2qrcode;
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

    // 打包需要输出的消息
    std::vector<nav_msgs::Odometry> packageMsg(std::vector<geometry_msgs::Pose> pose, QRcodeInfo code_info)
    {
        nav_msgs::Odometry odom;
        std::vector<nav_msgs::Odometry> v_odom;

        // 相同的项
        odom.header.stamp = pic.stamp;
        odom.header.seq = pic.index;
        if(true == code_info.is_head)
        {
            odom.pose.covariance[3] = 1; // 是否列首二维码，0：否 1：是
        }
        else
        {
            odom.pose.covariance[3] = 0; // 是否列首二维码，0：否 1：是
        }
        

        // map ---> base_link
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";
        odom.pose.pose = pose[0];
        //odom.pose.covariance[3] = code_info.frame.error_x;
        odom.pose.covariance[4] = code_info.frame.error_y;
        odom.pose.covariance[5] = code_info.frame.error_yaw;
        v_odom.push_back(odom);

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

        return v_odom;
    }

    // 采集二维码位姿模式
    void CollectQRCodePose_mode()
    {
        Lidarmap_tab = new QRcodeTable(cfg_dir + "CaptureTable.csv");
        ros::Rate loop_rate(100); // 主循环 100Hz
        while (ros::ok())
        {
            if (camera->getframe(&pic))
            {
                // 查询二维码坐标
                QRcodeInfo code_info;
                if (Lidarmap_tab->find_add(pic, &code_info))
                {
                    // 获取base_link位姿
                    geometry_msgs::Pose pose_base2map = get_pose_lidarmap(pic, code_info);

                    // 发布 /qrCodeMsg
                    if (show_original_msg)
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

    // 主循环
    void mainloopThread()
    {
        if (1 == operating_mode) // 采集二维码位姿
        {
            ROS_INFO("Mode: Collect QR-Code Pose");
            CollectQRCodePose_mode();
        }
        else if (2 == operating_mode) // 采集二维码编号
        {
            ROS_INFO("Mode: Collect QR-Code index");
            logger->info("Start Collect QR-Code");

            ros::Rate loop_rate(100); // 主循环 100Hz
            while (ros::ok())
            {
                static uint32_t code_last = 0;
                if (camera->getframe(&pic))
                {
                    if(code_last != pic.code)
                    {
                        std::stringstream stream;
                        stream.str("");
                        stream << pic.code;
                        logger->info(stream.str());

                        code_last = pic.code;
                    }
                }
            }
        }
        else if (3 == operating_mode) // 正常模式，使用论速计递推
        {
            ROS_INFO("Mode: Normal Run ");
            QRcodeInfo code_info;     // 查询二维码坐标
            bool is_inited = false;
            ros::Rate loop_rate(100); // 主循环 100Hz
            while (ros::ok())
            {
                logger->debug("new loop");

                // 处理二维码数据，设置轮速里程计初值
                if (camera->getframe(&pic))
                {
                    logger->debug("get pic: " + std::to_string(pic.code));

                    if (qrcode_table->onlyfind(pic, &code_info))
                    {
                        logger->debug("find pic: " 
                        + ' ' + std::to_string(pic.code)
                        + ' ' + std::to_string(code_info.x)
                        + ' ' + std::to_string(code_info.y)
                        + ' ' + std::to_string(code_info.yaw));

                        static CameraFrame pic_last;
                        static double min_dis_x0 = 1000;
                        static bool catch_zero = false;

                        //分析过程被打断的可能性与程序防护
                        
                        //未扫到同一个码,初始化
                        if(pic_last.code != pic.code)
                        {
                            logger->debug("is next code");

                            min_dis_x0 = 1000; //mm
                            catch_zero = false;

                            //logger->info(format_time(ros::Time::now()) + ",init");
                        }
                        pic_last = pic;

                        //速度不在低速范围
                        bool output_this_frame = true;
                        bool is_jump_point = false;
                        if(abs(wheel_odom->get_vel_x()) > low_speed_UL)
                        {
                            logger->debug("wheel_vel > " + std::to_string(low_speed_UL));
                            
                            double dis = sqrt(pic.error_x * pic.error_x + pic.error_y * pic.error_y);
                            if (dis < min_dis_x0) // 距离越来越近
                            {
                                logger->debug("closer");
                                min_dis_x0 = dis;
                                output_this_frame = false;
                            }
                            else if (false == catch_zero) // 刚刚越过0点
                            {
                                logger->debug("catch");
                                catch_zero = true;
                                output_this_frame = true;
                                is_jump_point = true;
                            }
                            else // 距离越来越远
                            {
                                logger->debug("further");
                                //不做处理，数据丢掉
                                output_this_frame = false;
                            }
                        }

                        // 发布该帧对应的位姿
                        if(output_this_frame)
                        {
                            logger->debug("output_this_frame");

                            // 计算base_link在qrmap坐标和map坐标的坐标
                            std::vector<geometry_msgs::Pose> pose = get_pose(code_info);

                            // 打包生成消息
                            std::vector<nav_msgs::Odometry> v_odom = packageMsg(pose, code_info);

                            // 设置轮速里程计初值
                            nav_msgs::Odometry last_odom = wheel_odom->getCurOdom();
                            wheel_odom->setEstimationInitialPose(v_odom[0]);
                            nav_msgs::Odometry cur_odom = wheel_odom->getCurOdom();
                            is_inited = true;

                            // 发布消息
                            pubOdom(v_odom);
                            logger->debug("pubOdom : QR-code");

                            // 跳跃点，计算二维码角度补偿值
                            if(is_jump_point)
                            {
                                logger->debug("is_jump_point");

                                if(save_y_err)
                                {
                                    err_y->add(ros::Time::now(),
                                               pic.code,
                                               wheel_odom->get_vel_x(),
                                               last_odom.pose.pose.position.y,
                                               cur_odom.pose.pose.position.y,
                                               pic.error_x,
                                               pic.error_y);
                                    logger->debug("save_y_err");

                                    if (wheel_odom->get_vel_x() > 0.2)
                                    {
                                        double y_err = cur_odom.pose.pose.position.y - last_odom.pose.pose.position.y;
                                        qrcode_table->jiaozheng(pic.code, y_err * err_ratio);
                                    }
                                }
                            }
                        }
                    }
                }

                // 轮速里程计递推
                if(is_inited)
                {
                    logger->debug("wheel odom is_inited");
                    wheel_odom->start();
                    std::vector<nav_msgs::Odometry> v_odom;
                    if(wheel_odom->run_odom(v_odom))
                    {
                        logger->debug("pubOdom : wheel");
                        // 发布递推后的位姿
                        pubOdom(v_odom);
                    }
                }

                // 循环控制延时函数
                loop_rate.sleep();
                ros::spinOnce();
            }
        }
        else if (4 == operating_mode) // 测试模式，只输出二维码得到的定位值，不使用论速计递推
        {
            ROS_INFO("Mode: Only QR-Code ");
            QRcodeInfo code_info;     // 查询二维码坐标
            ros::Rate loop_rate(100); // 主循环 100Hz
            while (ros::ok())
            {
                if (camera->getframe(&pic))
                {
                    if (qrcode_table->onlyfind(pic, &code_info))
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
                        //odom.pose.covariance[3] = code_info.frame.error_x;
                        odom.pose.covariance[4] = code_info.frame.error_y;
                        odom.pose.covariance[5] = code_info.frame.error_yaw;
                        v_odom.push_back(odom);

                        // 设置轮速计递推的初始值
                        wheel_odom->setEstimationInitialPose(odom);

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

                // // 轮速里程计递推
                // std::vector<nav_msgs::Odometry> v_odom;
                // if(wheel_odom->run_odom(v_odom))
                // {
                //     // 发布递推后的位姿
                //     pubOdom(v_odom);
                // }

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

    // Div div("/home/zl/work/ep-qrcode-loc/src/ep_qrcode_loc/config/data/");
    // ros::spinOnce();
    QRcodeLoc QLoc;

    std::thread loopthread(&QRcodeLoc::mainloopThread, &QLoc);

    ROS_INFO("\033[1;32m----> Localization with QRcode Started.\033[0m");

    ros::spin();

    loopthread.join();

    return 0;
}
