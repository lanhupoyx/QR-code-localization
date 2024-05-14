// ################################################
// #####        main function for ep_qrcode
// #####        yuanxun@ep-ep.com
// #####        updateTime:  2024.05.01
// ################################################
#include "hik_utility.hpp"
#include "MV_SC2005AM.hpp"

class vec  
{
public:
	double v[3];
	vec(){}
	vec(double x,double y ,double z)
	{
		 v[0] = x; v[1] = y; v[2] = z;
	}
	const double &operator [] (int i) const
	{
		return v[i];
	}
	double &operator [] (int i)
	{
		return v[i];
	}
};

std::vector<double> calTrans_SVD(){
    //读取Lidarmap_table.txt
    std::ifstream ifs;
    std::map<uint32_t, qrcode_info> map;
    std::string path = "/home/zl/work/ep-camera-hik/src/hik2005/config/Lidarmap_table.txt";
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
        qrcode_info info;
        line_ss >> info.code >> info.x >> info.y >> info.yaw;
        map.insert(std::pair<uint32_t, qrcode_info>(info.code, info));
    }
    ifs.close();
    std::cout << "Lidarmap_table的大小为: " << map.size() << std::endl;
    for (std::map<uint32_t, qrcode_info>::iterator it = map.begin(); it != map.end(); it++)
    {
        std::cout << (*it).second.code << " " << (*it).second.x << " " << (*it).second.y << " " << (*it).second.yaw << std::endl;
    }

    //读取QRmap_table.txt
    std::map<uint32_t, qrcode_info> qrmap;
    path = "/home/zl/work/ep-camera-hik/src/hik2005/config/QRmap_table.txt";
    ifs.open(path, std::ios::in);
    if (!ifs.is_open())
    {
        std::cout << path + "打开失败!" << std::endl;
    }
    while (std::getline(ifs, buf)) // 使用全局的getline()函数，其里面第一个参数代表输入流对象，第一个参数代表准备好的字符串，每次读取一行内容到buf
    {
        std::stringstream line_ss;
        line_ss << buf;
        qrcode_info info;
        line_ss >> info.code >> info.x >> info.y >> info.yaw;
        qrmap.insert(std::pair<uint32_t, qrcode_info>(info.code, info));
    }
    ifs.close();
    std::cout << "QRmap_table的大小为: " << qrmap.size() << std::endl;
    for (std::map<uint32_t, qrcode_info>::iterator it = qrmap.begin(); it != qrmap.end(); it++)
    {
        std::cout << (*it).second.code << " " << (*it).second.x << " " << (*it).second.y << " " << (*it).second.yaw << std::endl;
    }

    //提取点坐标
    int n = map.size();
    if (n < 3) 
    {
        std::cout << "less points" << std::endl;
        std::vector<double> result;
        return result;
    }
    std::vector<vec> qrmap_points;
    std::vector<vec> lidarmap_points;
    for (std::map<uint32_t, qrcode_info>::iterator lidar_it = map.begin(); lidar_it != map.end(); lidar_it++)
    {
        std::map<uint32_t, qrcode_info>::iterator qr_it = qrmap.find(lidar_it->first);
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

    //求质心
    n = qrmap_points.size();
    if (n < 3) 
    {
        std::cout << "less points" << std::endl;
        std::vector<double> result;
        return result;
    }
    vec al(0, 0, 0);//lidar map平均坐标
    vec ar(0, 0, 0);//qrcode map平均坐标
    for (int i = 0; i < n; ++i)
	{   
		al[0] += lidarmap_points[i][0] / n;
		al[1] += lidarmap_points[i][1] / n;
		al[2] = 0.0;

        ar[0] += qrmap_points[i][0] / n;
		ar[1] += qrmap_points[i][1] / n;
		ar[2] = 0.0;
	}

    //去质心坐标装填矩阵
    Eigen::Matrix3d mleft = Eigen::Matrix3d::Zero();
	for (int i = 0; i < n; ++i)
	{
        mleft += Eigen::Vector3d(lidarmap_points[i][0]-al[0], lidarmap_points[i][1]-al[1], 0.0)
                *Eigen::Vector3d(qrmap_points[i][0]-ar[0], qrmap_points[i][1]-ar[1], 0.0).transpose();
	}

    //奇异值分解求(R,t)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(mleft, Eigen::ComputeFullU | Eigen::ComputeFullV);//对矩阵分解
    Eigen::Matrix3d U_ = svd.matrixU();
    Eigen::Matrix3d V_ = svd.matrixV();
    Eigen::Matrix3d R_ = U_*(V_.transpose());
    // if(R_.determinant() < 0)
    // {
    //     R_ = -R_;
    // }
    Eigen::Matrix3d mR_ = -R_;
    Eigen::Vector3d t_ = Eigen::Vector3d(al[0], al[1], 0.0) - R_ * Eigen::Vector3d(ar[0], ar[1], 0.0);
    Eigen::Vector3d t_2 = Eigen::Vector3d(al[0], al[1], 0.0) - mR_ * Eigen::Vector3d(ar[0], ar[1], 0.0);

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

// 主函数
int main(int argc, char *argv[])
{
    // 初始化
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "hik2005");
    ros::NodeHandle nh;

    // 读取参数
    std::string odommapTopic;
    std::string odomqrmapTopic;
    std::string msgTopic;
    bool useLidar;
    bool calTrans;
    bool show_msg;
    bool collect_QRcode;
    std::string port;
    std::string log_dir;
    std::string cfg_dir;
    std::vector<double> qrmap2mapTrans;
    nh.param<std::string>("hik2005/odom4mapTopic", odommapTopic, "hik2005/odom_map");
    nh.param<std::string>("hik2005/odom4qrmapTopic", odomqrmapTopic, "hik2005/odom_qrmap");
    nh.param<std::string>("hik2005/msgTopic", msgTopic, "hik2005/msg");
    nh.param<bool>("hik2005/useLidar", useLidar, false);
    nh.param<bool>("hik2005/calTrans", calTrans, false);
    nh.param<bool>("hik2005/show_msg", show_msg, false);
    nh.param<bool>("hik2005/collect_QRcode", collect_QRcode, false);
    nh.param<std::string>("hik2005/port", port, "1024");
    nh.param<std::string>("hik2005/log_dir", log_dir, "/var/xmover/log/hik2005");
    nh.param<std::string>("hik2005/cfg_dir", cfg_dir, "/var/xmover/params");
    nh.param<std::vector<double>>("hik2005/qrmap2mapTrans", qrmap2mapTrans, std::vector<double>());
    geometry_msgs::Pose pose_qrmap2mapcopy;
    pose_qrmap2mapcopy.position.x = qrmap2mapTrans[0];
    pose_qrmap2mapcopy.position.y = qrmap2mapTrans[1];
    pose_qrmap2mapcopy.position.z = qrmap2mapTrans[2];
    pose_qrmap2mapcopy.orientation.x = qrmap2mapTrans[3];
    pose_qrmap2mapcopy.orientation.y = qrmap2mapTrans[4];
    pose_qrmap2mapcopy.orientation.z = qrmap2mapTrans[5];
    pose_qrmap2mapcopy.orientation.w = qrmap2mapTrans[6];

    // 发布器
    ros::Publisher pub_qrCodeMsg = nh.advertise<std_msgs::String>(msgTopic, 1000);
    ros::Publisher pub_odom_map = nh.advertise<nav_msgs::Odometry>(odommapTopic, 2000);
    ros::Publisher pub_odom_qrmap = nh.advertise<nav_msgs::Odometry>(odomqrmapTopic, 2000);

    //ros::ServiceServer srvSaveMap = nh.advertiseService("ep_mapping/save_map", &mapOptimization::saveMapService, this);

    // 日志文件初始化
    std::string log_name = log_dir + "/" + format_time(ros::Time::now()) + ".txt";
    std::ofstream log_file(log_name);
    if (!log_file.is_open())
    {
        std::cout << "can't open: " << log_name << std::endl;
    }
    else
    {
        std::cout << "open: " << log_name << std::endl;
    }

    // baselink---->camera的变换关系
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfListener(buffer);
    geometry_msgs::TransformStamped trans_base2camera;
    geometry_msgs::TransformStamped trans_camera2base;
    bool tferr = true;
    while (tferr)
    {
        tferr = false;
        try
        {
            trans_base2camera = buffer.lookupTransform("sc2005am_link", "base_link", ros::Time(0));
            trans_camera2base = buffer.lookupTransform("base_link", "sc2005am_link", ros::Time(0));
        }
        catch (tf::TransformException &exception)
        {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep();
            continue;
        }
    }

    QRcodeTable QRmap_tab(cfg_dir + "/QRmap_table.txt", &log_file, trans_camera2base);
    QRcodeTable Lidarmap_tab(cfg_dir + "/Lidarmap_table.txt", &log_file, trans_camera2base);
    MV_SC2005AM camera(port, &log_file);

    ros::Subscriber sub_pos = nh.subscribe<nav_msgs::Odometry>("/ep_localization/odometry/lidar", 1000, &QRcodeTable::tfCallback);

    if(calTrans)
    {
        std::vector<double> trans = calTrans_SVD();
        for (std::vector<double>::iterator it = trans.begin(); it != trans.end(); it++)
        {
            std::cout << *it << ", ";
        }
        std::cout << std::endl;
        return 0;
    }


    // 主循环 100Hz
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        frame pic; // 相机数据
        static nav_msgs::Odometry odom_map;
        static nav_msgs::Odometry odom_qrmap;
        if (camera.getframe(&pic))
        {
            // 查询二维码坐标
            qrcode_info code_info;
            if (useLidar && Lidarmap_tab.find_add(pic, &code_info))
            {
                // 二维码到map变换关系
                geometry_msgs::Pose pose_qrcode2map;
                pose_qrcode2map.position.x = code_info.x;
                pose_qrcode2map.position.y = code_info.y;
                pose_qrcode2map.position.z = 0.0;
                tf::Quaternion q;
                q.setRPY(0.0, 0.0, code_info.yaw*M_PI/180);
                tf::quaternionTFToMsg(q, pose_qrcode2map.orientation);
                double yaw_qrcode2map = getYaw(pose_qrcode2map);
                // 相机到二维码变换关系
                geometry_msgs::Pose pose_camera2qrcode;
                q.setRPY(0.0, 0.0, pic.error_yaw*M_PI/180);
                tf::quaternionTFToMsg(q, pose_camera2qrcode.orientation);
                double yaw_camera2qrcode = getYaw(pose_camera2qrcode);
                tf::Matrix3x3 m(q);
                tf::Matrix3x3 inv_m(m);
                tf::Vector3 v(pic.error_x/1000.0, pic.error_y/-1000.0, 0.0);
                inv_m.inverse();  // 求逆
                pose_camera2qrcode.position.x = -inv_m.getRow(0).dot(v);//(row0(0)*v.getX() + inv_m(0,1)*v.getX() +inv_m(0,2)*v.getZ());
                pose_camera2qrcode.position.y = -inv_m.getRow(1).dot(v);//-(inv_m(1,0)*v.getX() + inv_m(1,1)*v.getX() +inv_m(1,2)*v.getZ());
                pose_camera2qrcode.position.z = 0.0;
                // 相机到map位姿
                geometry_msgs::Pose pose_camera2map;
                tf2::doTransform(pose_camera2qrcode, pose_camera2map, p2t(pose_qrcode2map));
                double yaw_camera2map = getYaw(pose_camera2map);
                // base到map位姿
                geometry_msgs::Pose pose_base2map;
                tf2::doTransform(t2p(trans_base2camera), pose_base2map, p2t(pose_camera2map));
                double yaw_base2map = getYaw(pose_base2map);

                //发布base_link坐标
                odom_map.header.stamp = pic.stamp;
                odom_map.header.frame_id = "map";
                odom_map.child_frame_id = "base_link";
                odom_map.header.seq = pic.index;
                odom_map.pose.pose = pose_base2map;
                odom_map.pose.covariance[0] = 1;            // 此帧是否可用，1：可用，0：不可用
                odom_map.pose.covariance[1] = pic.duration; // 相机处理图像用时(s)
                pub_odom_map.publish(odom_map);

                // 发布 /qrCodeMsg
                if (show_msg)
                {
                    std::stringstream pub_ss;
                    pub_ss << format_time(pic.stamp) << " [" << pic.sender << "] " << pic.index << " " << pic.duration << "s " // ip
                        << " " << pic.error_x << "mm " << pic.error_y << "mm " << pic.error_yaw << " yaw_base2map:" << yaw_base2map;
                    std_msgs::String msg;
                    msg.data = pub_ss.str();
                    pub_qrCodeMsg.publish(msg);
                }
            }
            else if (!useLidar && QRmap_tab.onlyfind(pic, &code_info))
            {
                // 二维码到QRmap
                geometry_msgs::Pose pose_qrcode2qrmap;
                pose_qrcode2qrmap.position.x = code_info.x;
                pose_qrcode2qrmap.position.y = code_info.y;
                pose_qrcode2qrmap.position.z = 0.0;
                tf::Quaternion q;
                q.setRPY(0.0, 0.0, code_info.yaw*M_PI/180);
                tf::quaternionTFToMsg(q, pose_qrcode2qrmap.orientation);
                double yaw_qrcode2qrmap = getYaw(pose_qrcode2qrmap);
                // 相机到二维码
                geometry_msgs::Pose pose_camera2qrcode;
                q.setRPY(0.0, 0.0, pic.error_yaw*M_PI/180);
                tf::quaternionTFToMsg(q, pose_camera2qrcode.orientation);
                double yaw_camera2qrcode = getYaw(pose_camera2qrcode);
                tf::Matrix3x3 m(q);
                tf::Matrix3x3 inv_m(m);
                tf::Vector3 v(pic.error_x/1000.0, pic.error_y/-1000.0, 0.0);
                inv_m.inverse();  // 求逆
                pose_camera2qrcode.position.x = -inv_m.getRow(0).dot(v);//(row0(0)*v.getX() + inv_m(0,1)*v.getX() +inv_m(0,2)*v.getZ());
                pose_camera2qrcode.position.y = -inv_m.getRow(1).dot(v);//-(inv_m(1,0)*v.getX() + inv_m(1,1)*v.getX() +inv_m(1,2)*v.getZ());
                pose_camera2qrcode.position.z = 0.0;
                // 相机到QRmap
                geometry_msgs::Pose pose_camera2qrmap;
                tf2::doTransform(pose_camera2qrcode, pose_camera2qrmap, p2t(pose_qrcode2qrmap));
                double yaw_camera2qrmap = getYaw(pose_camera2qrmap);
                // base_link到QRmap
                geometry_msgs::Pose pose_base2qrmap;
                tf2::doTransform(t2p(trans_base2camera), pose_base2qrmap, p2t(pose_camera2qrmap));
                double yaw_base2qrmap = getYaw(pose_base2qrmap);
                // base_link到map_copy
                geometry_msgs::Pose pose_base2mapcopy;
                tf2::doTransform(pose_base2qrmap, pose_base2mapcopy, p2t(pose_qrmap2mapcopy));
                double yaw_base2mapcopy = getYaw(pose_base2mapcopy);
                
                //发布base_link到qrmap坐标
                odom_qrmap.header.stamp = pic.stamp;
                odom_qrmap.header.frame_id = "qrmap";
                odom_qrmap.child_frame_id = "base_link";
                odom_qrmap.header.seq = pic.index;
                odom_qrmap.pose.pose = pose_base2qrmap;
                odom_qrmap.pose.covariance[0] = 1;            // 此帧是否可用，1：可用，0：不可用
                odom_qrmap.pose.covariance[1] = pic.duration; // 相机处理图像用时(s)
                pub_odom_qrmap.publish(odom_qrmap);

                //发布base_link到map坐标
                odom_map.header.stamp = pic.stamp;
                odom_map.header.frame_id = "map";
                odom_map.child_frame_id = "base_link";
                odom_map.header.seq = pic.index;
                odom_map.pose.pose = pose_base2mapcopy;
                odom_map.pose.covariance[0] = 1;            // 此帧是否可用，1：可用，0：不可用
                odom_map.pose.covariance[1] = pic.duration; // 相机处理图像用时(s)
                pub_odom_map.publish(odom_map);
            }
            else
            {
                odom_qrmap.pose.covariance[0] = 0; // 此帧是否可用，1：可用，0：不可用
                pub_odom_qrmap.publish(odom_qrmap);
                odom_map.pose.covariance[0] = 0; // 此帧是否可用，1：可用，0：不可用
                pub_odom_map.publish(odom_map);
            }
        }
        else
        {
            odom_qrmap.pose.covariance[0] = 0; // 此帧是否可用，1：可用，0：不可用
            pub_odom_qrmap.publish(odom_qrmap);
            odom_map.pose.covariance[0] = 0; // 此帧是否可用，1：可用，0：不可用
            pub_odom_map.publish(odom_map);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
