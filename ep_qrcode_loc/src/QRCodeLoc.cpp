/*
 * @Author: YuanXun yuanxun@ep-ep.com
 * @Date: 2024-09-21 17:07:12
 * @LastEditors: YuanXun yuanxun@ep-ep.com
 * @LastEditTime: 2024-11-26 16:49:21
 * @FilePath: /ep-qrcode-loc/src/ep_qrcode_loc/src/QRCodeLoc.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "utility_qloc.hpp"
#include "wheel_odom.hpp"
#include "camera.hpp"
#include "qrcode_table.hpp"
#include "qrcode_table_v2.hpp"
#include "div.hpp"

// 偏差值信息
struct err_val
{
    double err_;
    uint32_t num_;

    err_val(double err, uint32_t num)
    {
        err_ = err;
        num_ = num;
    }
};

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
        logger->info("QRcodeLoc() Start");

        // // 删除过早的log文件
        // fs::path log_dir_path = log_dir; // 替换为你的目录路径
        // days keep_for = days(30);                  // 保留30天内的文件夹

        // try
        // {
        //     logger->delete_old_folders(log_dir_path, keep_for);
        // }
        // catch (const std::exception &e)
        // {
        //     std::cerr << "Error: " << e.what() << std::endl;
        //     logger->info("delete_old_folders() Error: " + e.what());
        // }

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

        if(is_debug)
        {
            trans_base2camera.transform.translation.x = -1.639;
            trans_base2camera.transform.translation.y = -0.27;
            trans_base2camera.transform.translation.z = 0;
            trans_base2camera.transform.rotation.x = 0;
            trans_base2camera.transform.rotation.y = 0;
            trans_base2camera.transform.rotation.z = 0;
            trans_base2camera.transform.rotation.w = 1;

            trans_camera2base.transform.translation.x = 1.639;
            trans_camera2base.transform.translation.y = 0.27;
            trans_camera2base.transform.translation.z = 0;
            trans_camera2base.transform.rotation.x = 0;
            trans_camera2base.transform.rotation.y = 0;
            trans_camera2base.transform.rotation.z = 0;
            trans_camera2base.transform.rotation.w = 1;
        }
        else
        {
            // 从TF获取baselink---->camera的变换关系
            tfListener = new tf2_ros::TransformListener(buffer);
            bool tferr = true;
            uint8_t loop_num = 0;
            while (tferr)
            {
                loop_num ++;
                if(loop_num > 10)
                {
                    logger->info("lookupTransform locCamera_link to base_link failed!");
                    break;
                }

                tferr = false;
                try
                {
                    trans_base2camera = buffer.lookupTransform("locCamera_link", "base_link", ros::Time(0));
                    trans_camera2base = buffer.lookupTransform("base_link", "locCamera_link", ros::Time(0));
                }
                catch (tf::TransformException &exception)
                {
                    logger->info(std::string(exception.what()) + "; retrying...");
                    tferr = true;
                    ros::Duration(0.5).sleep();
                    continue;
                }
            }
        }

        logger->info(  "trans_base2camera: " +
                        trans_base2camera.header.frame_id + ", " +
                        trans_base2camera.child_frame_id + ",  " +
                        std::to_string(trans_base2camera.transform.translation.x) + ", " +
                        std::to_string(trans_base2camera.transform.translation.y) + ", " +
                        std::to_string(trans_base2camera.transform.translation.z) + ",  " +
                        std::to_string(trans_base2camera.transform.rotation.x) + ", " +
                        std::to_string(trans_base2camera.transform.rotation.y) + ", " +
                        std::to_string(trans_base2camera.transform.rotation.z) + ", " +
                        std::to_string(trans_base2camera.transform.rotation.w)
        );

        logger->info(  "trans_camera2base: " +
                        trans_camera2base.header.frame_id + ", " +
                        trans_camera2base.child_frame_id + ",  " +
                        std::to_string(trans_camera2base.transform.translation.x) + ", " +
                        std::to_string(trans_camera2base.transform.translation.y) + ", " +
                        std::to_string(trans_camera2base.transform.translation.z) + ",  " +
                        std::to_string(trans_camera2base.transform.rotation.x) + ", " +
                        std::to_string(trans_camera2base.transform.rotation.y) + ", " +
                        std::to_string(trans_camera2base.transform.rotation.z) + ", " +
                        std::to_string(trans_camera2base.transform.rotation.w)
        );

        // 实例化功能对象
        qrcode_table = new QRcodeTableV2(cfg_dir, trans_camera2base);
        camera = new MV_SC2005AM();
        wheel_odom = new WheelSpeedOdometer(trans_camera2base);

        logger->info("QRcodeLoc() End");
    }

    // 析构函数
    ~QRcodeLoc()
    {
        delete camera;
        delete qrcode_table;
        delete wheel_odom;
    }

    // 记录参数
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
        logger->info("ep_qrcode_loc/avliable_yaw: " + std::to_string(avliable_yaw));
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
        logger->debug("pubOdom()");

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
        if (dis < dis_U * dis_U) // 比较平方和
        {
            pose_out = pose_observe;
            pose_out.position.x = p1 * pose_recursion.position.x + p2 * pose_observe.position.x;
            pose_out.position.y = p1 * pose_recursion.position.y + p2 * pose_observe.position.y;

            // 计算yaw偏差 
            double yaw_recursion = getYaw(pose_recursion.orientation);
            double yaw_observe = getYaw(pose_observe.orientation);
            double yaw_err = yaw_observe - yaw_recursion;
            logger->debug("yaw_recursion = " + del_n_end(std::to_string(yaw_recursion), 3) +
                          " yaw_observe = " + del_n_end(std::to_string(yaw_observe), 3) +
                          " yaw_err = " + del_n_end(std::to_string(yaw_err), 3));
            if (yaw_err < -180.0)
                yaw_err = yaw_err + 360.0;
            else if (yaw_err > 180.0)
                yaw_err = yaw_err - 360.0;
            
            // yaw加权
            double yaw_out = yaw_err * p2 + yaw_recursion;
            logger->debug("yaw_err = " + del_n_end(std::to_string(yaw_err), 3) +
                          " yaw_out = " + del_n_end(std::to_string(yaw_out), 3));
            tf::Quaternion q;
            q.setRPY(0.0, 0.0, yaw_out*M_PI/180.0);
            pose_out.orientation.w = q.getW();
            pose_out.orientation.x = q.getX();
            pose_out.orientation.y = q.getY();
            pose_out.orientation.z = q.getZ();
        }
        else
        {
            pose_out = pose_observe;
        }

        return pose_out;
    }

    // 车身方向角是否在允许识别二维码的范围内
    bool is_yaw_available(geometry_msgs::Quaternion q, double yaw_des, double range)
    {
        // 车身方向角
        double yaw_base = getYaw(q);

        // 数值偏差
        double diff = std::fmod(std::fabs(yaw_base - yaw_des), 360.0);

        // 处理零点问题
        diff = diff > 180.0 ? 360.0 - diff : diff;
        
        // 判断偏差是否在范围内
        if (diff < std::fabs(range / 2.0))
        {
            logger->debug("yaw in range, diff: "+std::to_string(diff));
            return true;
        }
        else
        {
            logger->info("角度超过限制! diff: "+std::to_string(diff));
            return false;
        }
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
        odom.pose.covariance[5] = 1; // 数据源：二维码

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

        logger->debug("packageMsg() return");
        return v_odom;
    }

    // 扫到码时，判断是否要需要输出这一帧
    bool do_not_jump_this_frame(CameraFrame pic_new, bool reset = false)
    {
        static CameraFrame pic_last;
        static double min_dis_x0 = 90000;
        static bool catch_zero = false;

        //分析过程被打断的可能性与程序防护
        logger->debug("do_not_jump_this_frame()");

        // 复位
        if(reset)
        {
            pic_last.code = 0;
            min_dis_x0 = 90000; //mm,大于距离平方
            catch_zero = false;
            return true;
        }
        
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
        logger->debug("do_not_jump_this_frame() return: " + std::to_string(output_this_frame));
        return output_this_frame;
    }

    // 判断是否跳变过大
    bool is_pose_jump(geometry_msgs::Pose pose_last, geometry_msgs::Pose pose_now)
    {
        // 定义变量
        bool result = false;

        // 检测车身yaw跳变
        double yaw_now = getYawRad(pose_now.orientation);
        double yaw_last = getYawRad(pose_last.orientation);
        double dis_yaw = abs(yaw_now - yaw_last);
        if (dis_yaw > M_PI)
            dis_yaw = abs(dis_yaw - 2 * M_PI);
        if (dis_yaw > (yaw_jump_UL*180/M_PI))
        {
            logger->info("yaw jump! dis_yaw = " + std::to_string(dis_yaw) + " > " + std::to_string(yaw_jump_UL));
            result = true;
        }

        // 车身横向和纵向跳变
        double dis_x_map = abs(pose_now.position.x - pose_last.position.x);// 跳变向量的x
        double dis_y_map = abs(pose_now.position.y - pose_last.position.y);// 跳变向量的y

        double dis_map = sqrt(pow(dis_x_map, 2) + pow(dis_y_map, 2));  // 跳变向量的长度
        // double dis_xy_yaw = asin(dis_y_map / dis_map);                 // 跳变向量的方向
        // double dis_x_base = abs(dis_map * cos(yaw_last - dis_xy_yaw)); // 跳变向量在车身横向分量
        // double dis_y_base = abs(dis_map * sin(yaw_last - dis_xy_yaw)); // 跳变向量在车身纵向分量

        double dis_jump_UL = sqrt(pow(x_jump_UL, 2) + pow(y_jump_UL, 2));

        if (dis_map > dis_jump_UL) // 跳变距离
        {
            logger->info("xy_base jump! dis_map = " + std::to_string(dis_map) + " > " + std::to_string(dis_jump_UL));
            result = true;
        }

        if (dis_x_map > x_jump_UL) // 车身纵向
        {
            logger->info("x_base jump! dis_x_map = " + std::to_string(dis_x_map) + " > " + std::to_string(x_jump_UL));
            result = true;
        }

        if (dis_y_map > y_jump_UL) // 车身横向
        {
            logger->info("y_base jump! dis_y_base = " + std::to_string(dis_y_map) + " > " + std::to_string(y_jump_UL));
            result = true;
        }

        if(result)
        {
            logger->info("dis_yaw = " + std::to_string(dis_yaw) +
                         "; dis_map = " + std::to_string(dis_map) +
                         "; dis_x_map = " + std::to_string(dis_x_map) +
                         "; dis_y_map = " + std::to_string(dis_y_map));
        }

        // 返回结果
        return result;
    }

    // 是否按顺序扫码
    bool is_code_in_order(uint32_t code_new, double vel_x, bool reset = false)
    {
        static uint32_t last_code = 0;

        if(!check_sequence)
        {
            return true;
        }

        if(reset)
        {
            last_code = 0;
        }
        else if (last_code == code_new)
        {
            logger->debug("is_code_in_order: same code, return true");
            return true;
        }
        else if (0 == last_code) // 首次扫码
        {
            if (qrcode_table->is_head(code_new))
            {
                last_code = code_new;
                logger->debug("is_code_in_order: is head, return true");
                return true;
            }
            else
            {
                logger->info("is_code_in_order: not head, return false  code_new=" + std::to_string(code_new));
                last_code = code_new;
                return false;
            }
        }
        else
        {
            std::vector<uint32_t> nbr = qrcode_table->get_neighbor(last_code);
            if ((vel_x > 0) && (code_new == nbr[0]))
            {
                if (qrcode_table->is_head(code_new))
                {
                    last_code = 0;
                    logger->debug("is_code_in_order: forward out, return true");
                }
                else
                {
                    last_code = code_new;
                    logger->debug("is_code_in_order: forward, return true");
                }
                return true;
            }
            else if ((vel_x < 0) && (code_new == nbr[1]))
            {
                last_code = code_new;
                logger->debug("is_code_in_order: retreat, return true");
                return true;
            }
            else
            {
                last_code = code_new;
                logger->info("is_code_in_order: other, return false  code_new=" + std::to_string(code_new) +
                             " vel_x=" + std::to_string(vel_x) +
                             " nbr[0]=" + std::to_string(nbr[0]) +
                             " nbr[1]=" + std::to_string(nbr[1]));
                return false;
            }
        }
    }

    // 输出记录
    void output_log(CameraFrame pic_latest, 
                    QRcodeInfo code_info, 
                    geometry_msgs::Twist wheel_msg,
                    std::vector<nav_msgs::Odometry> publist_front)
    {
        // 速度过小，不输出
        if (abs(wheel_odom->get_vel_msg().linear.x) < 0.001)
        {
            return;
        }

        std::string new_log =
            std::to_string(pic_latest.code) + "," +                                    // 二维码编号
            std::to_string(qrcode_table->is_head(pic_latest.code)) + "," +             // 是否是列首地码
            del_n_end(std::to_string(publist_front[0].pose.covariance[6]), 7) + ", " + // 是否用二维码结果设置里程计初值

            del_n_end(std::to_string(publist_front[0].pose.covariance[0]), 7) + "," +  // 数据是否可用
            del_n_end(std::to_string(publist_front[0].pose.covariance[1]), 7) + "," +  // 是否需要故障急停
            del_n_end(std::to_string(publist_front[0].pose.covariance[2]), 7) + "," +  // 故障编码
            del_n_end(std::to_string(publist_front[0].pose.covariance[3]), 7) + "," +  // 递推距离是否超过限制
            del_n_end(std::to_string(publist_front[0].pose.covariance[4]), 3) + "," +  // 递推距离
            del_n_end(std::to_string(publist_front[0].pose.covariance[5]), 7) + ", " + // 1:二维码，0：轮速计

            del_n_end(std::to_string(publist_front[0].pose.pose.position.x), 3) + "," +           // base_link x
            del_n_end(std::to_string(publist_front[0].pose.pose.position.y), 3) + "," +           // base_link y
            del_n_end(std::to_string(getYaw(publist_front[0].pose.pose.orientation)), 5) + ", " + // base_link yaw

            del_n_end(std::to_string(wheel_msg.angular.y), 3) + "," +  // 轮速方向角
            del_n_end(std::to_string(wheel_msg.linear.x), 3) + "," +   // base_link线速度
            del_n_end(std::to_string(wheel_msg.angular.z), 3) + ", " + // base_link角速度

            del_n_end(std::to_string(pic_latest.error_x / 100.0), 3) + "," + // 相机与地码偏移量x
            del_n_end(std::to_string(pic_latest.error_y / 100.0), 3) + "," + // 相机与地码偏移量y
            del_n_end(std::to_string(pic_latest.error_yaw), 3);              // 相机与地码偏移量yaw

        logger->pose(new_log);
        logger->debug(new_log);
    }

    /// @brief 输出扫码跳变数据到指定文件
    /// @param output
    /// @param output_last
    void output_jump_err(QRcodeInfo code_info,
                         std::vector<nav_msgs::Odometry> output,
                         std::vector<nav_msgs::Odometry> output_last)
    {
        // 速度过小，不输出
        if (abs(wheel_odom->get_vel_msg().linear.x) < 0.1)
        {
            return;
        }

        // 计算偏差 x
        double x_err = output[0].pose.pose.position.x - output_last[0].pose.pose.position.x;
        // 计算偏差 y
        double y_err = output[0].pose.pose.position.y - output_last[0].pose.pose.position.y;
        // 计算偏差 yaw
        double yaw_this = getYaw(output[0].pose.pose.orientation);
        double yaw_last = getYaw(output_last[0].pose.pose.orientation);
        double yaw_err = yaw_this - yaw_last;
        if (yaw_err < -180.0)
            yaw_err = yaw_err + 360.0;
        else if (yaw_err > 180.0)
            yaw_err = yaw_err - 360.0;

        // 记录
        logger->jumperr(std::to_string(code_info.code) + ", " + // 二维码编号

                        del_n_end(std::to_string(x_err * 1000), 5) + "," + // x_err
                        del_n_end(std::to_string(y_err * 1000), 5) + "," + // y_err
                        del_n_end(std::to_string(yaw_err), 4) + ", " +    // yaw_err

                        del_n_end(std::to_string(output[0].pose.pose.position.x), 3) + "," +           // base_link x
                        del_n_end(std::to_string(output[0].pose.pose.position.y), 3) + "," +           // base_link y
                        del_n_end(std::to_string(getYaw(output[0].pose.pose.orientation)), 4) + ", " + // base_link yaw

                        del_n_end(std::to_string(output_last[0].pose.pose.position.x), 3) + "," +  // base_link x
                        del_n_end(std::to_string(output_last[0].pose.pose.position.y), 3) + "," +  // base_link y
                        del_n_end(std::to_string(getYaw(output_last[0].pose.pose.orientation)), 4) // base_link yaw
        );
    }

//-------------------------------------------各种运行模式----------------------------------------------//


    // 采集二维码位姿模式
    void CollectQRCodePose_mode()
    {
        logger->info("CollectQRCodePose_mode()");
        QRcodeTable Lidarmap_tab(cfg_dir + "CaptureTable.txt");
        pub_qrCodeMsg = nh.advertise<std_msgs::String>(msgTopic, 1);

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
                    std::stringstream pub_ss;
                    pub_ss << format_time(pic.stamp) << " [" << pic.sender << "] " << pic.index << " " << pic.duration << "s " // ip
                            << " " << pic.error_x << "mm " << pic.error_y << "mm " << pic.error_yaw << " yaw_base2map:" << getYaw(pose_base2map);
                    std_msgs::String msg;
                    msg.data = pub_ss.str();
                    pub_qrCodeMsg.publish(msg);
                }
            }
            loop_rate.sleep();
            ros::spinOnce();
        }
    }

    // 采集二维码编号模式
    void CollectQRCodeIndex_mode()
    {
        logger->info("CollectQRCodeIndex_mode()");

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
    void NormalRun_mode_v1()
    {
        logger->info("NormalRun_mode()");
        QRcodeInfo code_info;            // 查询二维码坐标
        code_info.frame.code = 0;
        double loop_rate = 200.0;        // 主循环 200Hz
        ros::Rate loop_rate_ctrl(loop_rate); 
        while (ros::ok())
        {
            std::list<std::vector<nav_msgs::Odometry>> publist;

            // 处理二维码数据，解算baselink位姿
            if (camera->getframe(&pic))
            {
                if (qrcode_table->onlyfind(pic, &code_info))
                {
                    // 发布该帧对应的位姿
                    if(do_not_jump_this_frame(pic))
                    {
                        // 计算base_link在qrmap坐标和map坐标的坐标
                        std::vector<geometry_msgs::Pose> v_pose_new = get_pose(code_info);
                        // 打包生成消息
                        std::vector<nav_msgs::Odometry> v_odom = packageMsg(v_pose_new, code_info);

                        // 检测车身方向角度是否超过限制
                        if (is_yaw_available(v_odom[0].pose.pose.orientation, code_info.yaw, 20.0))
                        {
                            // 观测值与预测值加权平均
                            if(!code_info.is_head)
                            {
                                geometry_msgs::Pose pose_observe = v_odom[0].pose.pose;
                                geometry_msgs::Pose pose_recursion = wheel_odom->getCurOdom().pose.pose;
                                v_odom[0].pose.pose = kalman_f_my(pose_recursion, rec_p1, pose_observe, 1.0 - rec_p1, 0.1);
                            }


                            wheel_odom->setEstimationInitialPose(v_odom[0]); // 设置轮速里程计初值

                            // 放入发送队列
                            publist.push_back(v_odom);
                        }
                    }
                }
            }

            // 轮速里程计递推
            if((wheel_odom->is_start()))
            {
                logger->debug("wheel_odom is start");
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
            while (publist.size() > 0)
            {
                logger->debug("publist.size() > 0");

                // 发布消息
                pubOdom(publist.front());
                logger->debug("pubOdom(publist.front())");

                // 输出记录
                output_log(pic, code_info, wheel_odom->get_vel_msg(), publist.front());

                // 删除已发送的消息
                publist.pop_front();
            }

            // 循环控制延时函数
            loop_rate_ctrl.sleep();
            ros::spinOnce();
        }
    }

    // 正常运行模式v3
    void NormalRun_mode_v3()
    {
        logger->info("NormalRun_mode()");

        QRcodeInfo code_info;     // 存放查询到的地码信息
        code_info.frame.code = 0; // 初始化地码编号
        double loop_rate = 200.0; // 控制主循环频率200Hz
        ros::Rate loop_rate_ctrl(loop_rate);
        while (ros::ok())
        {
            std::vector<geometry_msgs::Pose> v_pose_new;        // 存放地码解算出的位姿
            std::vector<nav_msgs::Odometry> v_odom;             // 存放打包好可以发出的地码解算数据
            std::list<std::vector<nav_msgs::Odometry>> publist; // 最终数据发送队列
            static uint8_t err_type = 0x00;                     // 16进制数据，存放故障类型
            static bool is_output_available = false;            // 记录是否可以输出数据

            logger->debug_endl(); // 新循环，log空一行

            // 一、获取、解算相机原始数据
            if (camera->getframe(&pic))
            {
                logger->debug("getframe");
                if (do_not_jump_this_frame(pic)) // 检查是否需要跳过该帧数据
                {
                    if (qrcode_table->onlyfind(pic, &code_info)) // 查询地码信息
                    {
                        v_pose_new = get_pose(code_info); // 计算base_link在qrmap坐标和map坐标的坐标
                    }
                }
            }

            // 二、处理扫码解算结果
            nav_msgs::Odometry base2map = qrcode_table->getCurLidarPose(); // 获取tf输出的baselink
            logger->debug("base2map: " +
                          base2map.header.frame_id + ", " +
                          base2map.child_frame_id + ",  " +
                          std::to_string(base2map.pose.pose.position.x) + ", " +
                          std::to_string(base2map.pose.pose.position.y) + ", " +
                          std::to_string(base2map.pose.pose.position.z) + ",  ");
            if (v_pose_new.size() > 0)                                     // 本次循环解算出相机数据
            {
                logger->debug("v_pose_new.size() > 0");
                if (qrcode_table->is_in_queue(base2map, -0.3)) // baselink在列范围内
                {
                    logger->debug("qrcode_table->is_in_queue(base2map, -0.3)");

                    // 监测车身方向角度是否超过限制
                    if (!is_yaw_available(v_pose_new[0].orientation, code_info.yaw, 10.0))
                        err_type = err_type | 0x01; // 角度超过限制

                    // 监测是否顺序扫码
                    if (!is_code_in_order(pic.code, wheel_odom->get_vel_msg().linear.x))
                        err_type = err_type | 0x02; // 未按顺序扫码

                    // 监测是否在二维码处发生跳变
                    if (is_output_available)
                    {
                        if (is_pose_jump(wheel_odom->getCurOdom().pose.pose, v_pose_new[0]))
                            err_type = err_type | 0x04; // 发生跳变
                    }

                    // 卡尔曼滤波
                    if (!qrcode_table->is_head(pic.code)) // 不是列首码
                    {
                        geometry_msgs::Pose pose_observe = v_pose_new[0];
                        geometry_msgs::Pose pose_recursion = wheel_odom->getCurOdom().pose.pose;
                        v_pose_new[0] = kalman_f_my(pose_recursion, rec_p1, pose_observe, 1.0 - rec_p1, 0.1);
                    }

                    // 打包生成消息
                    v_odom = packageMsg(v_pose_new, code_info);

                    // 扫码无异常后设置轮速里程计初值
                    if ((0x00 == err_type) || (!check_sequence))
                    {
                        wheel_odom->setEstimationInitialPose(v_odom[0]); // 设置递推初值
                        v_odom[0].pose.covariance[6] = 1;                // 扫码正常，已根据二维码结果设置递推初值”
                    }
                    else
                    {
                        v_odom[0].pose.covariance[6] = 2; // 扫码异常，未根据二维码结果设置递推初值”
                    }

                    publist.push_back(v_odom);  // 放入发送队列
                    is_output_available = true; // 入列并扫码后输出可用
                }
                else
                {
                    // 在列外道路上扫到地码
                }
            }

            // 三、轮速里程计递推
            if ((wheel_odom->is_start()))
            {
                logger->debug("wheel_odom is start");
                std::vector<nav_msgs::Odometry> v_odom_wheel;
                if (wheel_odom->run_odom(v_odom_wheel))
                {
                    // 用于展示曲线
                    v_odom_wheel[0].pose.covariance[7] = code_info.frame.error_x;   // 地码与相机横向偏差
                    v_odom_wheel[0].pose.covariance[8] = code_info.frame.error_y;   // 地码与相机纵向偏差
                    v_odom_wheel[0].pose.covariance[9] = code_info.frame.error_yaw; // 地码与相机角度偏差

                    publist.push_back(v_odom_wheel); // 发布递推后的位姿
                    logger->debug("wheel odom publist.push_back(v_odom_wheel)");

                    // 递推距离清零、判断递推是否过远
                    if (qrcode_table->is_in_queue(base2map, -0.5)) // 在列内
                    {
                        if (!qrcode_table->is_in_queue(base2map, -0.4))
                        {
                            wheel_odom->reset_path_dis(); // 进入列首，递推距离清零
                        }

                        if (wheel_odom->is_path_dis_overflow()) // 递推过远
                        {
                            logger->debug("在列内递推过远");
                            err_type = err_type | 0x08; // 在列内递推过远
                        }
                    }
                }
            }

            // 四、发布位姿数据
            while (publist.size() > 0)
            {
                // 取出数据
                std::vector<nav_msgs::Odometry> output = publist.front();    // 取出即将发送的数据
                static std::vector<nav_msgs::Odometry> output_last = output; // 上个输出
                publist.pop_front();                                         // 删除取出的数据

                // 状态判断
                if (qrcode_table->is_in_queue(base2map, -0.4)) // 在列内
                {
                    // 判断数据是否可用
                    if (is_output_available)
                        output[0].pose.covariance[0] = 1; // 数据可用
                    else
                        output[0].pose.covariance[0] = 0; // 数据不可用

                    // 二次判断跳变
                    if (1 == output_last[0].pose.covariance[0]) // 数据可用
                    {
                        if (1 == output[0].pose.covariance[0]) // 数据可用
                        {
                            if (is_pose_jump(output_last[0].pose.pose, output[0].pose.pose)) // 发生
                            {
                                logger->info("二次监测跳变");
                                err_type = err_type | 0x04; // 发生跳变
                            }
                        }
                    }

                    // 判断是否处于故障状态
                    if (0x00 != err_type)
                        output[0].pose.covariance[1] = 1; // 故障急停：是
                    else
                        output[0].pose.covariance[1] = 0; // 故障急停：否
                }
                else // 在列外
                {
                    is_output_available = false;       // 数据不可用
                    err_type = 0x00;                   // 清除故障急停状态
                    output[0].pose.covariance[0] = 0;  // 数据不可用
                    output[0].pose.covariance[1] = 0;  // 故障急停：否
                    pic.code = 0;                      // 列外码值清零
                    is_code_in_order(0, 0, true);      // 列外初始化二维码顺序判定
                    do_not_jump_this_frame(pic, true); // 复位该功能
                }
                output[0].pose.covariance[2] = err_type; // 故障类型

                // 记录正常扫码的跳变值，用于地码方向角调试
                if (1 == output[0].pose.covariance[6]) // 扫码正常，已根据二维码结果设置递推初值
                {
                    output_jump_err(code_info, output, output_last);
                }

                // 记录本帧数据
                output_log(pic, code_info, wheel_odom->get_vel_msg(), output); 

                // 扫码无异常，则发布消息
                if (2 != output[0].pose.covariance[6])
                {
                    pubOdom(output);      // 发布消息
                    output_last = output; // 保存用于下个循环
                }
            }

            // 五、循环控制延时函数
            loop_rate_ctrl.sleep();
            ros::spinOnce();
        }
    }

    // 测试模式
    void TestRun_mode()
    {
        logger->info("TestRun_mode()");
        QRcodeInfo code_info;     // 查询二维码坐标
        ros::Rate loop_rate(200); // 主循环 200Hz
        while (ros::ok())
        {
            if (camera->getframe(&pic))
            {
                code_info.frame = pic;
                std::vector<geometry_msgs::Pose> pose;

                // 打包生成消息
                std::vector<nav_msgs::Odometry> v_odom = packageMsg(pose, code_info);

                // 发布消息
                pubOdom(v_odom);
            }

            loop_rate.sleep();
            ros::spinOnce();
        }
    }

    // 补偿地码角度模式
    void GetYawErr_mode_v1()
    {
        logger->info("GetYawErr_mode_v1()");
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

    // 采集地码角度模式
    void GetYaw_mode()
    {
        logger->debug("GetYaw_mode()");
        QRcodeInfo code_info;     // 查询二维码坐标
        ros::Rate loop_rate(200); // 主循环 100Hz
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

                    // 记录扫码次数，码变化算一次

                    // 输出
                    logger->yawerr(  "," + std::to_string(pic.code) + "," +
                                    std::to_string(wheel_odom->get_vel_x()) + "," +
                                    std::to_string(cur_qrcode.yaw) + "," +
                                    std::to_string(code_info.yaw));
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

    // 补偿地码角度模式
    void CalYawErr_mode()
    {
        logger->debug("CalYawErr_mode()");

        // 关闭logger中的yawerr.txt文件
        logger->close_yawerr();

        // 打开yawerr.txt
        std::string yaw_err_path = log_dir + "yawerr.txt";
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

        // 读取yawerr.txt
        std::map<uint32_t, err_val> yawerr_database;
        std::string buf;
        while (std::getline(ifs, buf)) 
        {
            buf = replaceChar(buf, ',', ' '); // ','替换为' '
            std::stringstream line_ss;
            line_ss << buf;

            // 跳过空行
            if(("" == buf) || (' ' == buf[0]))
            {
                continue;
            }

            logger->info(buf);

            // 定义变量
            std::string time;
            uint32_t index;
            double vel_x, curYaw, refYaw;

            // 提取信息
            line_ss >> time >> index >> vel_x >> curYaw >> refYaw ;

            // 前进时车头摆动幅度小
            if(vel_x < 0.0) continue;

            // 数据存入数据库
            std::map<uint32_t, err_val>::iterator it = yawerr_database.find(index);
            if (it != yawerr_database.end())
            {
                it->second.err_ = (it->second.err_ * it->second.num_ + (curYaw - refYaw)) / (it->second.num_ + 1.0);
                it->second.num_++;
            }
            else
            {
                err_val new_err(curYaw - refYaw, 1);
                yawerr_database.insert(std::pair<uint32_t, err_val>(index, new_err));
            }
        }
        ifs.close();
        logger->info(yaw_err_path + "读取完毕!");


        // 打开初始库位信息文件
        std::string site_info_path = cfg_dir + "SiteTable.csv";
        ifs.open(site_info_path, std::ios::in);
        if (!ifs.is_open())
        {
            logger->info(site_info_path + "打开失败!");
        }
        else
        {
            logger->info(site_info_path + "打开成功!");
        }

        // 打开输出库位信息文件
        std::string output_path = cfg_dir + "SiteTable_output.csv";
        std::ofstream ofs;
        ofs.open(output_path, std::ios::out);
        if (!ofs.is_open())
        {
            logger->info(output_path + "打开失败!");
            return;
        }
        else
        {
            logger->info(output_path + "打开成功!");
        }

        // 读取库位及其绑定的二维码信息
        while (std::getline(ifs, buf))
        {
            std::string new_buf = replaceChar(buf, ',', ' '); // ','替换为' '
            std::stringstream line_ss;
            line_ss << new_buf;

            // 跳过空行
            if(("" == new_buf) || (' ' == new_buf[0]))
            {
                ofs << buf << std::endl;
                continue;
            }

            // 定义变量
            uint32_t list_index, site_index, code_index, index_;
            double x_err, y_err, yaw_err;

            // 提取信息
            line_ss >> list_index >> site_index >> code_index;
            
            // 忽略库位位姿信息
            if((0 == site_index) && (0 == code_index))
            {
                ofs << buf << std::endl;
                continue;
            }

            // 提取信息
            line_ss >> index_ >>  x_err >> y_err >> yaw_err;

            // 已经补偿过的跳过
            if(0.0 != yaw_err)
            {
                ofs << buf << std::endl;
                continue;
            }

            // 查找数据库
            std::map<uint32_t, err_val>::iterator it = yawerr_database.find(index_);
            if (it != yawerr_database.end())
            {
                std::string outstring = buf.substr(0, buf.size() - 1);
                outstring = outstring.substr(0, outstring.find_last_of(',') + 1);
                ofs << outstring << it->second.err_ << std::endl;
            }
            else
            {
                ofs << buf << std::endl;
            }
        }

        ifs.close();
        logger->info(site_info_path + "读取完毕!");

        ofs.close();
        logger->info(output_path + "写入完毕!");
    }

    // 检查相机水平模式
    void CheckCameraHorizon_mode()
    {
        logger->info("CheckCameraHorizon_mode()");
        QRcodeInfo code_info;     // 查询二维码坐标
        ros::Rate loop_rate(200); // 主循环 200Hz
        while (ros::ok())
        {
            if (camera->getframe(&pic))
            {
                nav_msgs::Odometry odom;
                std::vector<nav_msgs::Odometry> v_odom;

                odom.pose.covariance[7] = pic.error_x; // 地码与相机横向偏差
                odom.pose.covariance[8] = pic.error_y; // 地码与相机纵向偏差
                odom.pose.covariance[9] = pic.error_yaw; // 地码与相机角度偏差
                
                // map ---> base_link
                odom.header.frame_id = "map";
                odom.child_frame_id = "base_link";
                v_odom.push_back(odom);
                v_odom.push_back(odom);

                // 发布消息
                pubOdom(v_odom);
            }

            loop_rate.sleep();
            ros::spinOnce();
        }
    }

    // 代码测试
    void Code_Debug()
    {
        nav_msgs::Odometry base2map;

        base2map.pose.pose.position.x = 10;
        base2map.pose.pose.position.y = 10;

        qrcode_table->is_in_queue(base2map, 0.0);
    }

    // 主循环
    void mainloopThread()
    {
        logger->info("mainloopThread()");
        
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
        else if (3 == operating_mode) // 正常模式v1，使用轮速计递推
        {
            logger->info("Mode 3: Normal Run v1");
            NormalRun_mode_v3();
        }
        else if (4 == operating_mode) // 测试模式，只输出二维码得到的定位值，不使用轮速计递推
        {
            logger->info("Mode 4: Only QR-Code ");
            TestRun_mode();
        }
        else if (5 == operating_mode) // 采集角度模式
        {
            logger->info("Mode 5: Get Yaw Error ");
            GetYaw_mode();
        }
        else if (6 == operating_mode) // 计算角度模式
        {
            logger->info("Mode 6: Calculate Yaw Error ");
            CalYawErr_mode();
        }
        else if (7 == operating_mode) // 检查相机安装是否水平
        {
            logger->info("Mode 7: Check Camera Horizon ");
            CheckCameraHorizon_mode();
        }
        else if (8 == operating_mode) // 正常模式v3，使用轮速计递推
        {
            logger->info("Mode 8: Normal Run v1");
            NormalRun_mode_v1();
        }
        else if (9 == operating_mode) // 正常模式v3，使用轮速计递推
        {
            logger->info("Mode 9: Code Debug");
            Code_Debug();
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
    ros::init(argc, argv, "qrcode_loc");                                  // ros初始化
    QRcodeLoc QLoc;                                                       // 实例化二维码定位对象
    std::thread loopthread(&QRcodeLoc::mainloopThread, &QLoc);            // 创建主循环线程
    ROS_INFO("\033[1;32m----> Localization with QRcode Started.\033[0m"); // 输出提示
    ros::spin();                                                          // spin
    loopthread.join();                                                    // 运行主循环

    return 0;
}
