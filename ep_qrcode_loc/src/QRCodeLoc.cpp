//////////////////////////////////////////////////////////////////////////////////
/////        main function for ep_qrcode_loc
/////              yuanxun@ep-ep.com
/////           updateTime:  2024.10.19
//////////////////////////////////////////////////////////////////////////////////
#include "utility_qloc.hpp"
#include "wheel_odom.hpp"
#include "camera.hpp"
#include "qrcode_table.hpp"
#include "qrcode_table_v2.hpp"
#include "div.hpp"

// 二维码定位
class QRcodeLoc : public ParamServer
{
public:

    // 互斥锁
    std::mutex mtx;

    // 记录器
    Logger *logger;

    // ros发布器、接收器
    ros::Publisher pub_odom_map_base;
    ros::Publisher pub_odom_map_camera;
    ros::Publisher pub_path_map_base;
    ros::Publisher pub_path_map_camera;
    ros::Publisher pub_qrCodeMsg;
    ros::Subscriber sub_pos;

    // tf转换相关
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener *tfListener;
    geometry_msgs::TransformStamped trans_base2camera;
    geometry_msgs::TransformStamped trans_camera2base;
    geometry_msgs::TransformStamped trans_base2map;
    tf::TransformBroadcaster br;

    // 功能对象
    MV_SC2005AM *camera;
    QRcodeTableV2 *qrcode_table;
    WheelSpeedOdometer *wheel_odom;
    
    //相机当前frame数据
    CameraFrame pic; // 相机数据

    // 构造函数
    QRcodeLoc()
    {
        // 记录器
        logger = &Logger::getInstance();
        logger->info("QRcodeLoc Start");

        // 记录参数文件内容
        std::string cfg_path = cfg_dir + "ep-qrcode-loc.yaml";
        logger->info("show cfg file: " + cfg_dir + "ep-qrcode-loc.yaml");
        std::ifstream ifs;
        ifs.open(cfg_path, std::ios::in);
        if (!ifs.is_open())
            logger->info(cfg_path + "打开失败!");
        else
            logger->info(cfg_path + "打开成功!");
        std::string buf; 
        while (std::getline(ifs, buf)) 
        {
            logger->info(buf);
        }

        // 记录读取的系统参数
        show_param();

        // 初始化发布器
        pub_odom_map_base = nh.advertise<nav_msgs::Odometry>(odomMapBase, 10);
        pub_odom_map_camera = nh.advertise<nav_msgs::Odometry>(odomMapCamera, 10);
        pub_path_map_base = nh.advertise<nav_msgs::Path>(pathMapBase, 10);
        pub_path_map_camera = nh.advertise<nav_msgs::Path>(pathMapCamera, 10);
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

        // 实例化功能对象
        qrcode_table = new QRcodeTableV2(cfg_dir, trans_camera2base);
        camera = new MV_SC2005AM();
        wheel_odom = new WheelSpeedOdometer(trans_camera2base);
    }

    // 析构函数
    ~QRcodeLoc()
    {
        delete camera;
        delete qrcode_table;
        delete wheel_odom;
    }

    void show_param()
    {
        logger->info("");
        logger->info("show params:");
        logger->info("ep_qrcode_loc/operating_mode: " + std::to_string(operating_mode));
        logger->info("ep_qrcode_loc/logLevel: " + logLevel);
        logger->info("ep_qrcode_loc/odomMapBase: " + odomMapBase);
        logger->info("ep_qrcode_loc/odomMapCamera: " + odomMapCamera);
        logger->info("ep_qrcode_loc/pathMapBase: " + pathMapBase);
        logger->info("ep_qrcode_loc/pathMapCamera: " + pathMapCamera);
        logger->info("ep_qrcode_loc/msgTopic: " + msgTopic);
        logger->info("ep_qrcode_loc/show_original_msg: " + std::to_string(show_original_msg));
        logger->info("ep_qrcode_loc/is_pub_tf: " + std::to_string(is_pub_tf));
        logger->info("ep_qrcode_loc/yaw_jump_UL: " + std::to_string(yaw_jump_UL));
        logger->info("ep_qrcode_loc/x_jump_UL: " + std::to_string(x_jump_UL));
        logger->info("ep_qrcode_loc/y_jump_UL: " + std::to_string(y_jump_UL));
        logger->info("ep_qrcode_loc/read_yaw_err: " + std::to_string(read_yaw_err));
        logger->info("ep_qrcode_loc/err_ratio_offline: " + std::to_string(err_ratio_offline));
        logger->info("ep_qrcode_loc/rec_p1: " + std::to_string(rec_p1));
        logger->info("ep_qrcode_loc/wheel_diameter: " + std::to_string(wheel_diameter));
        logger->info("ep_qrcode_loc/wheel_reduction_ratio: " + std::to_string(wheel_reduction_ratio));
        logger->info("ep_qrcode_loc/wheel_base_dis: " + std::to_string(wheel_base_dis));
        logger->info("ep_qrcode_loc/wheel_angular_offset: " + std::to_string(wheel_angular_offset));
        logger->info("ep_qrcode_loc/low_speed_UL: " + std::to_string(low_speed_UL));
        logger->info("ep_qrcode_loc/port: " + port);
        logger->info("ep_qrcode_loc/log_dir: " + log_dir);
        logger->info("ep_qrcode_loc/cfg_dir: " + cfg_dir);
        logger->info("ep_qrcode_loc/maxEstimationDis: " + std::to_string(maxEstimationDis));
        logger->info("ep_qrcode_loc/detect_site_dis: " + std::to_string(detect_site_dis));
        logger->info("ep_qrcode_loc/aux_site_dis: " + std::to_string(aux_site_dis));
        logger->info("ep_qrcode_loc/forkaction_site_dis: " + std::to_string(forkaction_site_dis));
        logger->info("ep_qrcode_loc/site_site_dis: " + std::to_string(site_site_dis));
    }

    // 发布定位结果
    void pubOdom(std::vector<nav_msgs::Odometry> v_odom)
    {
        // 主要需要输出的消息
        nav_msgs::Odometry odom_map_base = v_odom[0];
        nav_msgs::Odometry odom_map_camera = v_odom[1];

        // 发布odom消息
        pub_odom_map_base.publish(odom_map_base);
        pub_odom_map_camera.publish(odom_map_camera);

        // 发布TF
        if (is_pub_tf)
        {
            pubTf(odom_map_base);
        }

        // 输出的路线消息
        if (1 == odom_map_base.pose.covariance[0]) // 此帧数据是否可用，不可用不计入路线消息
        {
            static u_int32_t num = 0;
            num++;
            if (num % 5 == 0) // 降低发布频率
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
        }
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
        logger->debug("get_pose()");
        // 二维码到mapcopy
        geometry_msgs::Pose pose_qrcode2mapcopy;
        pose_qrcode2mapcopy.position.x = code_info.x;
        pose_qrcode2mapcopy.position.y = code_info.y;
        pose_qrcode2mapcopy.position.z = 0.0;
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, code_info.yaw * M_PI / 180);
        tf::quaternionTFToMsg(q, pose_qrcode2mapcopy.orientation);

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

        // 相机到mapcopy
        geometry_msgs::Pose pose_camera2mapcopy;
        tf2::doTransform(pose_camera2qrcode, pose_camera2mapcopy, p2t(pose_qrcode2mapcopy));

        // base_link到mapcopy
        geometry_msgs::Pose pose_base2mapcopy;
        tf2::doTransform(t2p(trans_base2camera), pose_base2mapcopy, p2t(pose_camera2mapcopy));

        std::vector<geometry_msgs::Pose> output;
        output.push_back(pose_base2mapcopy);
        output.push_back(pose_camera2mapcopy);

        logger->debug("get_pose() return");
        return output;
    }

    // pose1 预测值，pose2 观测值
    geometry_msgs::Pose kalman_f_my(geometry_msgs::Pose pose_recursion, double p1, 
                                    geometry_msgs::Pose pose_observe,   double p2, 
                                    double dis_U)
    {
        double dis;
        geometry_msgs::Pose pose_out;
        dis =   pow(pose_recursion.position.x - pose_observe.position.x, 2) + 
                pow(pose_recursion.position.y - pose_observe.position.y, 2);
        if (dis < dis_U * dis_U)
        {
            pose_out = pose_observe;
            pose_out.position.x = p1 * pose_recursion.position.x + p2 * pose_observe.position.x;
            pose_out.position.y = p1 * pose_recursion.position.y + p2 * pose_observe.position.y;
        }
        else
        {
            pose_out = pose_observe;
        }

        return pose_out;
    }

    // 打包需要输出的消息
    std::vector<nav_msgs::Odometry> packageMsg(std::vector<geometry_msgs::Pose> pose, QRcodeInfo code_info)
    {
        logger->debug("packageMsg()");
        nav_msgs::Odometry odom;
        std::vector<nav_msgs::Odometry> v_odom;

        // 相同的项
        odom.header.stamp = code_info.frame.stamp;// 时间戳
        odom.header.seq = code_info.frame.index; // 二维码编号

        // 标志信号
        odom.pose.covariance[0] = 1; // 可用
        odom.pose.covariance[2] = 0; // 数据源于二维码
        if(true == code_info.is_head)
        {
            odom.pose.covariance[3] = 1; // 是列首二维码
        }
        else
        {
            odom.pose.covariance[3] = 0; // 不是否列首二维码
        }

        odom.pose.covariance[7] = code_info.frame.error_x; // 地码与相机横向偏差
        odom.pose.covariance[8] = code_info.frame.error_y; // 地码与相机纵向偏差
        odom.pose.covariance[9] = code_info.frame.error_yaw; // 地码与相机角度偏差
        
        // map ---> base_link
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";
        odom.pose.pose = pose[0];
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

        logger->debug("packageMsg() return");
        return v_odom;
    }

    // 扫到码时，判断是否要需要输出这一帧
    bool need_output_this_frame(CameraFrame pic_new)
    {
        static CameraFrame pic_last;
        static double min_dis_x0 = 90000;
        static bool catch_zero = false;

        //分析过程被打断的可能性与程序防护
        logger->debug("need_output_this_frame()");
        
        //未扫到同一个码,初始化
        if(pic_last.code != pic_new.code)
        {
            logger->debug("is next code");

            min_dis_x0 = 90000; //mm,大于距离平方
            catch_zero = false;

            //logger->info(format_time(ros::Time::now()) + ",init");
        }
        pic_last = pic_new;

        //速度不在低速范围
        bool output_this_frame = true;
        if(abs(wheel_odom->get_vel_x()) > low_speed_UL)
        {
            logger->debug("wheel_vel > " + std::to_string(low_speed_UL));
            
            double disdis = pic_new.error_x * pic_new.error_x + pic_new.error_y * pic_new.error_y;
            if (disdis < min_dis_x0) // 距离越来越近
            {
                // logger->debug("closer");
                min_dis_x0 = disdis;
                output_this_frame = false;
            }
            else if (false == catch_zero) // 刚刚越过0点
            {
                // logger->debug("catch");
                catch_zero = true;
                output_this_frame = true;
            }
            else // 距离越来越远
            {
                // logger->debug("further");
                //不做处理，数据丢掉
                output_this_frame = false;
            }
        }
        logger->debug("need_output_this_frame() return: " + std::to_string(output_this_frame));
        return output_this_frame;
    }

    bool is_pose_jump(nav_msgs::Odometry pose_now)
    {
        // 定义变量
        bool result = false;
        static nav_msgs::Odometry pose_last = pose_now; 

        // 前一帧因里程计递推过长而不可用时，不认为跳变
        if( (0 == pose_last.pose.covariance[0]) && // pose_last不可用
            (1 == pose_last.pose.covariance[1]) && // pose_last递推过长
            (1 == pose_last.pose.covariance[2]) && // pose_last源于轮速计
            (1 == pose_now.pose.covariance[0]) &&  // pose_now可用
            (0 == pose_now.pose.covariance[2]))    // pose_now源于二维码
        {
            result = false;
        }
        else
        {
            // 检测车身yaw跳变
            double yaw_now = getYawRad(pose_now.pose.pose.orientation);
            double yaw_last = getYawRad(pose_last.pose.pose.orientation);
            double dis_yaw = abs(yaw_now - yaw_last);
            if(dis_yaw > M_PI) dis_yaw = abs(dis_yaw - 2*M_PI);
            if(dis_yaw > yaw_jump_UL) result = true;

            // 车身横向和纵向跳变
            double dis_x_map = pose_now.pose.pose.position.x - pose_last.pose.pose.position.x;
            double dis_y_map = pose_now.pose.pose.position.y - pose_last.pose.pose.position.y;
            double dis_map = sqrt(pow(dis_x_map,2) + pow(dis_y_map,2));
            double dis_x_base = abs(dis_map*cos(yaw_last));
            double dis_y_base = abs(dis_map*sin(yaw_last));
            if(dis_x_base > x_jump_UL) result = true; // 车身纵向
            if(dis_y_base > y_jump_UL) result = true; // 车身横向
        }

        // 保存此次数据
        pose_last = pose_now;

        // 返回结果
        return result;
    }
    
    // 采集二维码位姿模式
    void CollectQRCodePose_mode()
    {
        QRcodeTable Lidarmap_tab(cfg_dir + "CaptureTable.txt");
        ros::Rate loop_rate(100); // 主循环 100Hz
        while (ros::ok())
        {
            if (camera->getframe(&pic))
            {
                // 查询二维码坐标
                QRcodeInfo code_info;
                if (Lidarmap_tab.find_add(pic, &code_info))
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

    // 采集二维码编号模式
    void CollectQRCodeIndex_mode()
    {
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

    // 正常运行模式
    void NormalRun_mode()
    {
        logger->debug("NormalRun_mode()");
        QRcodeInfo code_info;     // 查询二维码坐标
        ros::Rate loop_rate(200); // 主循环 200Hz
        while (ros::ok())
        {
            static std::list<std::vector<nav_msgs::Odometry>> publist;

            logger->debug("new loop");
            // 处理二维码数据，设置轮速里程计初值
            if (camera->getframe(&pic))
            {
                logger->debug("getframe");
                if (qrcode_table->onlyfind(pic, &code_info))
                {
                    logger->debug("find pic: " + std::to_string(code_info.frame.code));
                    // 发布该帧对应的位姿
                    if(need_output_this_frame(pic))
                    {
                        logger->debug("need_output_this_frame: true");
                        // 计算base_link在qrmap坐标和map坐标的坐标
                        std::vector<geometry_msgs::Pose> v_pose_new = get_pose(code_info);
                        
                        // 观测值与预测值加权平均
                        geometry_msgs::Pose pose_observe = v_pose_new[0];
                        geometry_msgs::Pose pose_recursion = wheel_odom->getCurOdom().pose.pose;
                        if(code_info.is_head)
                        {
                            v_pose_new[0] = kalman_f_my(pose_recursion, 0.0, pose_observe, 1.0, 0.1);
                        }
                        else
                        {
                            v_pose_new[0] = kalman_f_my(pose_recursion, rec_p1, pose_observe, 1.0 - rec_p1, 0.1);
                        }

                        // 打包生成消息
                        std::vector<nav_msgs::Odometry> v_odom = packageMsg(v_pose_new, code_info);

                        // 放入发送队列
                        publist.push_back(v_odom);

                        // 设置轮速里程计初值
                        wheel_odom->setEstimationInitialPose(v_odom[0]);
                    }
                }
            }

            // 轮速里程计递推
            logger->debug("wheel_odom->is_start(): " + std::to_string(wheel_odom->is_start()));
            if(wheel_odom->is_start())
            {
                std::vector<nav_msgs::Odometry> v_odom;
                if(wheel_odom->run_odom(v_odom))
                {
                    v_odom[0].pose.covariance[7] = code_info.frame.error_x; // 地码与相机横向偏差
                    v_odom[0].pose.covariance[8] = code_info.frame.error_y; // 地码与相机纵向偏差
                    v_odom[0].pose.covariance[9] = code_info.frame.error_yaw; // 地码与相机角度偏差
                    // 发布递推后的位姿
                    logger->debug("wheel odom publist.push_back(v_odom)");
                    publist.push_back(v_odom);
                }
            }

            // 发布位姿数据
            logger->debug("publist.size(): " + std::to_string(publist.size()));
            while(publist.size()>0)
            {
                // 检测跳变
                if(is_pose_jump(publist.front()[0]))
                {
                    publist.front()[0].pose.covariance[4] = 1; // 跳变超过限制
                }
                else
                {
                    publist.front()[0].pose.covariance[4] = 0; // 跳变未超过限制
                }

                //发布消息
                pubOdom(publist.front());
                logger->debug("pubOdom(publist.front())");

                // 输出记录
                logger->pose( std::to_string(code_info.frame.code) + "," +                             // 二维码编号
                              std::to_string(code_info.is_head) + "," +                                // 是否是列首地码
                              std::to_string(code_info.frame.error_x) + "," +                          // 相机与地码偏移量x
                              std::to_string(code_info.frame.error_y) + "," +                          // 相机与地码偏移量y
                              std::to_string(code_info.frame.error_yaw) + "," +                        // 相机与地码偏移量yaw
                              std::to_string(wheel_odom->get_vel_msg().linear.x) + "," +               // base_link线速度
                              std::to_string(wheel_odom->get_vel_msg().angular.y) + "," +              // 轮速方向角
                              std::to_string(wheel_odom->get_vel_msg().angular.z) + "," +              // base_link角速度
                              std::to_string(publist.front()[0].pose.pose.position.x) + "," +          // base_link x
                              std::to_string(publist.front()[0].pose.pose.position.y) + "," +          // base_link y
                              std::to_string(getYaw(publist.front()[0].pose.pose.orientation)) + "," + // base_link yaw
                              std::to_string(publist.front()[1].pose.pose.position.x) + "," +          // locCamera_link x
                              std::to_string(publist.front()[1].pose.pose.position.y) + "," +          // locCamera_link y
                              std::to_string(getYaw(publist.front()[1].pose.pose.orientation)) + "," + // locCamera_link yaw
                              std::to_string(publist.front()[0].pose.covariance[0]) + "," +            // 数据是否可用
                              std::to_string(publist.front()[0].pose.covariance[1]) + "," +            // 递推长度是否超过限制
                              std::to_string(publist.front()[0].pose.covariance[2]) + "," +            // 数据来源，0：二维码，1：轮速计递推
                              std::to_string(publist.front()[0].pose.covariance[3]) + "," +            // 是否为列首二维码
                              std::to_string(publist.front()[0].pose.covariance[4]));                  // 是否跳变超过限制

                // 

                publist.pop_front();
                
            }
            
            // 循环控制延时函数
            loop_rate.sleep();
            ros::spinOnce();
        }
    }

    // 测试模式
    void TestRun_mode()
    {
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

                    // 打包生成消息
                    std::vector<nav_msgs::Odometry> v_odom = packageMsg(pose, code_info);

                    // 发布消息
                    pubOdom(v_odom);
                }
            }

            loop_rate.sleep();
            ros::spinOnce();
        }
    }

    // 补偿地码角度模式
    void GetYawErr_mode()
    {
        logger->debug("Mode: Get Yaw Error ");
        QRcodeInfo code_info;     // 查询二维码坐标
        ros::Rate loop_rate(100); // 主循环 100Hz
        while (ros::ok())
        {
            if (camera->getframe(&pic))
            {
                if (qrcode_table->onlyfind(pic, &code_info))
                {
                    logger->debug("find pic: " + std::to_string(pic.code));
                    // 获取最新雷达定位值
                    nav_msgs::Odometry base2map = qrcode_table->getCurLidarPose();

                    // lidar得到的base_link(map)
                    std::vector<geometry_msgs::Pose> v_pose_new = get_pose(code_info);
    
                    // lidar得到的二维码位姿(map)
                    geometry_msgs::Pose pose_camera2map;
                    tf2::doTransform(t2p(trans_camera2base), pose_camera2map, p2t(base2map.pose.pose));
                    geometry_msgs::Pose pose_qrcode2camera;
                    pose_qrcode2camera.position.x = pic.error_x / 1000.0;
                    pose_qrcode2camera.position.y = pic.error_y / -1000.0;
                    pose_qrcode2camera.position.z = 0.0;
                    tf::Quaternion q1;
                    q1.setRPY(0.0, 0.0, pic.error_yaw * M_PI / -180.0);
                    tf::quaternionTFToMsg(q1, pose_qrcode2camera.orientation);
                    geometry_msgs::Pose pose_qrcode2map;
                    tf2::doTransform(pose_qrcode2camera, pose_qrcode2map, p2t(pose_camera2map));
                    double yaw_qrcode2map = getYaw(pose_qrcode2map);
                    QRcodeInfo cur_qrcode;
                    cur_qrcode.x = pose_qrcode2map.position.x;
                    cur_qrcode.y = pose_qrcode2map.position.y;
                    cur_qrcode.yaw = yaw_qrcode2map;

                    // 输出
                    logger->other(  "," + std::to_string(pic.code) + "," +
                                    std::to_string(wheel_odom->get_vel_x()) + "," +
                                    std::to_string(wheel_odom->get_vel_msg().linear.z) + "," +
                                    std::to_string(wheel_odom->get_vel_msg().angular.y) + "," +
                                    std::to_string(base2map.pose.pose.position.x) + "," +
                                    std::to_string(base2map.pose.pose.position.y) + "," +
                                    std::to_string(getYaw(base2map.pose.pose.orientation)) + "," +
                                    std::to_string(pose_camera2map.position.x) + "," +
                                    std::to_string(pose_camera2map.position.y) + "," +
                                    std::to_string(getYaw(pose_camera2map.orientation)) + "," +
                                    std::to_string(v_pose_new[0].position.x) + "," +
                                    std::to_string(v_pose_new[0].position.y) + "," +
                                    std::to_string(getYaw(v_pose_new[0].orientation)) + "," +
                                    std::to_string(v_pose_new[1].position.x) + "," +
                                    std::to_string(v_pose_new[1].position.y) + "," +
                                    std::to_string(getYaw(v_pose_new[1].orientation)) + "," +
                                    std::to_string(cur_qrcode.x) + "," +
                                    std::to_string(cur_qrcode.y) + "," +
                                    std::to_string(cur_qrcode.yaw) + "," +
                                    std::to_string(code_info.x) + "," +
                                    std::to_string(code_info.y) + "," +
                                    std::to_string(code_info.yaw) + "," +
                                    std::to_string(pic.error_x) + "," +
                                    std::to_string(pic.error_y) + "," +
                                    std::to_string(pic.error_yaw)   );

                    // 计算yaw_err均值
                    if(wheel_odom->get_vel_x() > 0.0)
                    {
                        static double num = 0.0;
                        static double yaw_err_average = 0.0;
                        static uint32_t code_last = 0;

                        // 结算
                        if((code_last != pic.code) && (code_last != 0))
                        {
                            logger->yawerr(std::to_string(code_last) + " " +std::to_string(yaw_err_average));
                            num = 0.0;
                        }

                        // 差值取平均
                        double yaw_err_new = cur_qrcode.yaw - code_info.yaw;
                        yaw_err_average = yaw_err_average*num/(num+1.0) + yaw_err_new/(num+1.0);

                        // 更新记录
                        num += 1.0;
                        code_last = pic.code;
                    }
                }
                else
                {
                    logger->other("未识别index: " + std::to_string(pic.code));
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
            logger->info("Mode 1: Collect QR-Code Pose");
            CollectQRCodePose_mode();
        }
        else if (2 == operating_mode) // 采集二维码编号
        {
            logger->info("Mode 2: Collect QR-Code index");
            CollectQRCodeIndex_mode();
        }
        else if (3 == operating_mode) // 正常模式，使用轮速计递推
        {
            logger->info("Mode 3: Normal Run ");
            NormalRun_mode();
        }
        else if (4 == operating_mode) // 测试模式，只输出二维码得到的定位值，不使用轮速计递推
        {
            logger->info("Mode 4: Only QR-Code ");
            TestRun_mode();
        }
        else if (5 == operating_mode) // 采集角度模式
        {
            logger->info("Mode 5: Get Yaw Error ");
            GetYawErr_mode();
        }
        else
        {
            logger->info("Mode识别失败: " + std::to_string(operating_mode));
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
