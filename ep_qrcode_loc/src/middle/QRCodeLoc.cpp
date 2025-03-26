#include "QRCodeLoc.hpp"

// 偏差值信息
err_val::err_val(double err, uint32_t num)
{
    err_ = err;
    num_ = num;
}

// 构造函数
QRcodeLoc::QRcodeLoc(ParamServer &param, MV_SC2005AM* camera) : param(param),camera(camera)
{
    // 记录器
    logger = &epLogger::getInstance();
    logger->info(std::string(__FUNCTION__) + "() start");

    // 初始化发布器
    pub_odom_map_base = param.nh.advertise<nav_msgs::Odometry>(param.odomMapBase, 10);
    pub_odom_map_camera = param.nh.advertise<nav_msgs::Odometry>(param.odomMapCamera, 10);
    pub_path_map_base = param.nh.advertise<nav_msgs::Path>(param.pathMapBase, 10);
    pub_path_map_camera = param.nh.advertise<nav_msgs::Path>(param.pathMapCamera, 10);

    // 订阅手自动状态
    is_handle = false;
    sub_BasicState = param.nh.subscribe<xmover_msgs::BasicState>("/xmover_basic_state", 1, &QRcodeLoc::BasicStateCallback,
                                                                 this, ros::TransportHints().tcpNoDelay());
    
    // 获取tf值，base和camera
    getTrans_BaseToCamera();

    logger->info(std::string(__FUNCTION__) + "() return");
}

// 析构函数
QRcodeLoc::~QRcodeLoc(){}

// 获取tf值，base和camera
void QRcodeLoc::getTrans_BaseToCamera()
{
    if (param.is_debug)
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

        logger->info("param.is_debug = true, Transform locCamera_link to base_link is default!");
    }
    else
    {
        // 从TF获取baselink---->camera的变换关系
        tfListener = new tf2_ros::TransformListener(buffer);
        bool tferr = true;
        uint8_t loop_num = 0;
        while (tferr)
        {
            loop_num++;
            if (loop_num > 10)
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

    // 保存进参数服务器
    param.trans_base2camera = trans_base2camera;
    param.trans_camera2base = trans_camera2base;

    // 输出到log
    logger->info("trans_base2camera: " +
                 trans_base2camera.header.frame_id + ", " +
                 trans_base2camera.child_frame_id + ",  " +
                 std::to_string(trans_base2camera.transform.translation.x) + ", " +
                 std::to_string(trans_base2camera.transform.translation.y) + ", " +
                 std::to_string(trans_base2camera.transform.translation.z) + ",  " +
                 std::to_string(trans_base2camera.transform.rotation.x) + ", " +
                 std::to_string(trans_base2camera.transform.rotation.y) + ", " +
                 std::to_string(trans_base2camera.transform.rotation.z) + ", " +
                 std::to_string(trans_base2camera.transform.rotation.w));

    logger->info("trans_camera2base: " +
                 trans_camera2base.header.frame_id + ", " +
                 trans_camera2base.child_frame_id + ",  " +
                 std::to_string(trans_camera2base.transform.translation.x) + ", " +
                 std::to_string(trans_camera2base.transform.translation.y) + ", " +
                 std::to_string(trans_camera2base.transform.translation.z) + ",  " +
                 std::to_string(trans_camera2base.transform.rotation.x) + ", " +
                 std::to_string(trans_camera2base.transform.rotation.y) + ", " +
                 std::to_string(trans_camera2base.transform.rotation.z) + ", " +
                 std::to_string(trans_camera2base.transform.rotation.w));
}

// 获取/xmover_basic_state的回调函数
void QRcodeLoc::BasicStateCallback(const xmover_msgs::BasicState::ConstPtr &p_base_state_msg)
{
    static bool last_is_handle = is_handle;
    logger->debug(std::string(__FUNCTION__) + "() start");
    std::lock_guard<std::mutex> locker(mtx);
    is_handle = p_base_state_msg->handle;
    if (last_is_handle != is_handle)
    {
        last_is_handle = is_handle;
        logger->info(std::string(__FUNCTION__) + " change is_handle to: " + std::to_string(is_handle));
    }

    logger->debug(std::string(__FUNCTION__) + "() return");
    return;
}

// 发布定位结果
void QRcodeLoc::pubOdom(std::vector<nav_msgs::Odometry> v_odom)
{
    logger->debug(std::string(__FUNCTION__) + "() start");

    // 主要需要输出的消息
    nav_msgs::Odometry odom_map_base = v_odom[0];
    nav_msgs::Odometry odom_map_camera = v_odom[1];

    // 发布odom消息
    pub_odom_map_base.publish(odom_map_base);
    pub_odom_map_camera.publish(odom_map_camera);

    // 发布TF
    if (param.is_pub_tf)
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
    
    logger->debug(std::string(__FUNCTION__) + "() return");
    return;
}

// 发布TF
void QRcodeLoc::pubTf(const nav_msgs::Odometry odom)
{
    logger->debug(std::string(__FUNCTION__) + "() start");

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
    // std::cout << stream.str() << std::endl;
    
    logger->debug(std::string(__FUNCTION__) + "() return");
    return;
}

// 计算base_link在map坐标系下的坐标
geometry_msgs::Pose QRcodeLoc::get_pose_lidarmap(CameraFrame pic, QRcodeInfo code_info)
{
    logger->debug(std::string(__FUNCTION__) + "() start");

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

    logger->debug(std::string(__FUNCTION__) + "() return");
    return pose_base2map;
}

// 计算base_link在qrmap和map坐标系下的坐标
std::vector<geometry_msgs::Pose> QRcodeLoc::get_pose(QRcodeInfo code_info)
{
    logger->debug(std::string(__FUNCTION__) + "() start");
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

    logger->debug(std::string(__FUNCTION__) + "() return");
    return output;
}

// pose1 预测值，pose2 观测值
geometry_msgs::Pose QRcodeLoc::kalman_f_my(geometry_msgs::Pose pose_recursion, double p1,
                                           geometry_msgs::Pose pose_observe, double p2,
                                           double dis_U)
{
    logger->debug(std::string(__FUNCTION__) + "() start");

    double dis;
    geometry_msgs::Pose pose_out;
    dis = pow(pose_recursion.position.x - pose_observe.position.x, 2) +
          pow(pose_recursion.position.y - pose_observe.position.y, 2);
    if (dis < dis_U * dis_U) // 比较平方和
    {
        pose_out = pose_observe;
        pose_out.position.x = p1 * pose_recursion.position.x + p2 * pose_observe.position.x;
        pose_out.position.y = p1 * pose_recursion.position.y + p2 * pose_observe.position.y;

        // 计算yaw偏差
        if (param.cal_yaw && (wheel_odom->get_vel_x() > param.low_speed_UL))
        {
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
            q.setRPY(0.0, 0.0, yaw_out * M_PI / 180.0);
            pose_out.orientation.w = q.getW();
            pose_out.orientation.x = q.getX();
            pose_out.orientation.y = q.getY();
            pose_out.orientation.z = q.getZ();
        }
    }
    else
    {
        pose_out = pose_observe;
    }

    logger->debug(std::string(__FUNCTION__) + "() return");
    return pose_out;
}

// 车身方向角是否在允许识别二维码的范围内
bool QRcodeLoc::check_is_yaw_available(geometry_msgs::Quaternion q, double yaw_des, double range)
{
    logger->debug(std::string(__FUNCTION__) + "() start");
    // 车身方向角
    double yaw_base = getYaw(q);

    // 数值偏差
    double diff = std::fmod(std::fabs(yaw_base - yaw_des), 360.0);

    // 处理零点问题
    diff = diff > 180.0 ? 360.0 - diff : diff;

    // 判断偏差是否在范围内
    if (diff < std::fabs(range / 2.0))
    {
        logger->debug("yaw in range, diff: " + std::to_string(diff));
        logger->debug(std::string(__FUNCTION__) + "() return true");
        return true;
    }
    else
    {
        logger->info("角度超过限制! diff: " + std::to_string(diff));
        logger->debug(std::string(__FUNCTION__) + "() return false");
        return false;
    }
}

// 车身方向角是否在允许识别二维码的范围内(双向)
bool QRcodeLoc::check_is_yaw_available_dual(geometry_msgs::Quaternion q, double yaw_des, double range)
{
    logger->debug(std::string(__FUNCTION__) + "() start");
    if (check_is_yaw_available(q, yaw_des, range) || check_is_yaw_available(q, yaw_des + 180.0, range))
    {
        logger->debug(std::string(__FUNCTION__) + "() return true");
        return true;
    }
    else
    {
        logger->debug(std::string(__FUNCTION__) + "() return false");
        return false;
    }
}

// 打包需要输出的消息
std::vector<nav_msgs::Odometry> QRcodeLoc::packageMsg(std::vector<geometry_msgs::Pose> pose, QRcodeInfo code_info)
{
    logger->debug(std::string(__FUNCTION__) + "() start");
    nav_msgs::Odometry odom;
    std::vector<nav_msgs::Odometry> v_odom;

    // 相同的项
    odom.header.stamp = code_info.frame.stamp; // 时间戳
    odom.header.seq = code_info.frame.index;   // 二维码编号

    // 标志信号
    odom.pose.covariance[5] = 1; // 数据源：二维码

    odom.pose.covariance[7] = code_info.frame.error_x;   // 地码与相机横向偏差
    odom.pose.covariance[8] = code_info.frame.error_y;   // 地码与相机纵向偏差
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

    logger->debug(std::string(__FUNCTION__) + "() return");
    return v_odom;
}

// 扫到码时，判断是否要需要输出这一帧
bool QRcodeLoc::do_not_jump_this_frame(CameraFrame pic_new, bool reset)
{
    static CameraFrame pic_last;
    static double min_dis_x0 = 90000;
    static bool catch_zero = false;

    // 分析过程被打断的可能性与程序防护
    logger->debug(std::string(__FUNCTION__) + "() start");

    // 复位
    if (reset)
    {
        pic_last.code = 0;
        min_dis_x0 = 90000; // mm,大于距离平方
        catch_zero = false;
        logger->debug(std::string(__FUNCTION__) + "() return: reset");
        return true;
    }

    // 未扫到同一个码,初始化
    if (pic_last.code != pic_new.code)
    {
        logger->debug("is next code");

        min_dis_x0 = 90000; // mm,大于距离平方
        catch_zero = false;

        // logger->info(format_time(ros::Time::now()) + ",init");
    }
    pic_last = pic_new;

    // 速度不在低速范围
    bool output_this_frame = true;
    if (abs(wheel_odom->get_vel_x()) > param.low_speed_UL)
    {
        logger->debug("wheel_vel > " + std::to_string(param.low_speed_UL));

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
            // 不做处理，数据丢掉
            output_this_frame = false;
        }
    }
    logger->debug(std::string(__FUNCTION__) + "() return" + std::to_string(output_this_frame));
    return output_this_frame;
}

// 判断是否跳变过大
bool QRcodeLoc::check_is_pose_jump(geometry_msgs::Pose pose_last, geometry_msgs::Pose pose_now)
{
    logger->debug(std::string(__FUNCTION__) + "() start");
    // 定义变量
    bool result = false;

    // 检测车身yaw跳变
    double yaw_now = getYawRad(pose_now.orientation);
    double yaw_last = getYawRad(pose_last.orientation);
    double dis_yaw = abs(yaw_now - yaw_last);
    if (dis_yaw > M_PI)
        dis_yaw = abs(dis_yaw - 2 * M_PI);
    if (dis_yaw > (param.yaw_jump_UL * 180 / M_PI))
    {
        logger->info("yaw jump! dis_yaw = " + std::to_string(dis_yaw) + " > " + std::to_string(param.yaw_jump_UL));
        result = true;
    }

    // 车身横向和纵向跳变
    double dis_x_map = abs(pose_now.position.x - pose_last.position.x); // 跳变向量的x
    double dis_y_map = abs(pose_now.position.y - pose_last.position.y); // 跳变向量的y

    double dis_map = sqrt(pow(dis_x_map, 2) + pow(dis_y_map, 2)); // 跳变向量的长度
    // double dis_xy_yaw = asin(dis_y_map / dis_map);                 // 跳变向量的方向
    // double dis_x_base = abs(dis_map * cos(yaw_last - dis_xy_yaw)); // 跳变向量在车身横向分量
    // double dis_y_base = abs(dis_map * sin(yaw_last - dis_xy_yaw)); // 跳变向量在车身纵向分量

    double dis_jump_UL = sqrt(pow(param.x_jump_UL, 2) + pow(param.y_jump_UL, 2));

    if (dis_map > dis_jump_UL) // 跳变距离
    {
        logger->info("xy_base jump! dis_map = " + std::to_string(dis_map) + " > " + std::to_string(dis_jump_UL));
        result = true;
    }

    if (dis_x_map > param.x_jump_UL) // 车身纵向
    {
        logger->info("x_base jump! dis_x_map = " + std::to_string(dis_x_map) + " > " + std::to_string(param.x_jump_UL));
        result = true;
    }

    if (dis_y_map > param.y_jump_UL) // 车身横向
    {
        logger->info("y_base jump! dis_y_base = " + std::to_string(dis_y_map) + " > " + std::to_string(param.y_jump_UL));
        result = true;
    }

    if (result)
    {
        logger->info("dis_yaw = " + std::to_string(dis_yaw) +
                     "; dis_map = " + std::to_string(dis_map) +
                     "; dis_x_map = " + std::to_string(dis_x_map) +
                     "; dis_y_map = " + std::to_string(dis_y_map));
    }

    // 返回结果
    logger->debug(std::string(__FUNCTION__) + "() return: " + std::to_string(result));
    return result;
}

// 输出记录
void QRcodeLoc::output_log(CameraFrame pic_latest,
                           QRcodeInfo code_info,
                           geometry_msgs::Twist wheel_msg,
                           std::vector<nav_msgs::Odometry> publist_front,
                           bool is_head)
{
    logger->debug(std::string(__FUNCTION__) + "() start");
    // 速度过小，不输出
    if (abs(wheel_odom->get_vel_msg().linear.x) < 0.001)
    {
        logger->debug(std::string(__FUNCTION__) + "() return: abs(wheel_odom->get_vel_msg().linear.x) < 0.001");
        return;
    }

    if(publist_front.size() == 0)
    {
        logger->debug(std::string(__FUNCTION__) + "() return: publist_front.size() == 0");
        return;
    }

    std::string new_log =
        std::to_string(pic_latest.code) + "," +                                    // 二维码编号
        std::to_string(is_head) + "," +                                            // 0:不是列首地码，1:是列首地码
        del_n_end(std::to_string(publist_front[0].pose.covariance[6]), 7) + ", " + // 0:轮速计结果，1:扫码正常，已根据二维码结果设置递推初值，2:扫码异常，未根据二维码结果设置递推初值

        del_n_end(std::to_string(publist_front[0].pose.covariance[0]), 7) + "," +  // 0:数据不可用，1:数据可用
        del_n_end(std::to_string(publist_front[0].pose.covariance[1]), 7) + "," +  // 0:无故障急停，1:故障急停
        del_n_end(std::to_string(publist_front[0].pose.covariance[2]), 7) + "," +  // 故障编码相加：1:角度超过限制，2:未按顺序扫码，4:发生跳变，8:在列内递推过远
        del_n_end(std::to_string(publist_front[0].pose.covariance[3]), 7) + "," +  // 0:递推距离未超过限制，1:递推距离超过限制
        del_n_end(std::to_string(publist_front[0].pose.covariance[4]), 3) + ", " + // 递推距离(m)

        del_n_end(std::to_string(publist_front[0].pose.pose.position.x), 3) + "," +           // base_link x(m)
        del_n_end(std::to_string(publist_front[0].pose.pose.position.y), 3) + "," +           // base_link y(m)
        del_n_end(std::to_string(getYaw(publist_front[0].pose.pose.orientation)), 5) + ", " + // base_link yaw(度)

        del_n_end(std::to_string(wheel_msg.angular.y), 3) + "," +  // 轮速方向角(度)
        del_n_end(std::to_string(wheel_msg.linear.x), 3) + "," +   // base_link线速度(m/s)
        del_n_end(std::to_string(wheel_msg.angular.z), 3) + ", " + // base_link角速度(度/s)

        del_n_end(std::to_string(pic_latest.error_x / 10.0), 3) + "," + // 相机与地码偏移量x(cm)
        del_n_end(std::to_string(pic_latest.error_y / 10.0), 3) + "," + // 相机与地码偏移量y(cm)
        del_n_end(std::to_string(pic_latest.error_yaw), 3);             // 相机与地码偏移量yaw(度)

    logger->pose(new_log);

    static std::string last_log = new_log;
    uint32_t new_code = pic_latest.code;
    static uint32_t last_code = new_code;
    double is_stop = publist_front[0].pose.covariance[1];
    static double is_stop_last = is_stop;
    
    if(0 == new_code)
    {
        last_code = new_code;
        last_log = new_log;
        is_stop_last = is_stop;
        logger->debug(std::string(__FUNCTION__) + "() return: 0 == new_code");
        return;
    }
    else
    {
        if(0 == last_code)
        {
            logger->info(last_log);
        }

        if(0 == is_stop_last)
        {
            if(1 == is_stop)
            {
                logger->info("stop");
            }
        }

        logger->info(new_log);

        last_code = new_code;
        last_log = new_log;
        is_stop_last = is_stop;
        
        logger->debug(std::string(__FUNCTION__) + "() return");
        return;
    }


}

/// @brief 输出扫码跳变数据到指定文件
/// @param output
/// @param output_last
void QRcodeLoc::output_jump_err(QRcodeInfo code_info,
                                std::vector<nav_msgs::Odometry> output,
                                std::vector<nav_msgs::Odometry> output_last)
{
    logger->debug(std::string(__FUNCTION__) + "() start");
    // 速度过小，不输出
    if (abs(wheel_odom->get_vel_msg().linear.x) < 0.1)
    {
        logger->debug(std::string(__FUNCTION__) + "() return: abs(wheel_odom->get_vel_msg().linear.x) < 0.1 ");
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
    logger->jumperr(+"," + std::to_string(code_info.code) + ", " + // 二维码编号

                    del_n_end(std::to_string(x_err * 100), 5) + "," + // x_err
                    del_n_end(std::to_string(y_err * 100), 5) + "," + // y_err
                    del_n_end(std::to_string(yaw_err), 4) + ", " +    // yaw_err

                    del_n_end(std::to_string(output[0].pose.pose.position.x), 3) + "," +           // base_link x
                    del_n_end(std::to_string(output[0].pose.pose.position.y), 3) + "," +           // base_link y
                    del_n_end(std::to_string(getYaw(output[0].pose.pose.orientation)), 4) + ", " + // base_link yaw

                    del_n_end(std::to_string(output_last[0].pose.pose.position.x), 3) + "," +           // base_link x
                    del_n_end(std::to_string(output_last[0].pose.pose.position.y), 3) + "," +           // base_link y
                    del_n_end(std::to_string(getYaw(output_last[0].pose.pose.orientation)), 4) + ", " + // base_link yaw

                    del_n_end(std::to_string(code_info.frame.error_x / 10.0), 3) + "," + // 相机与地码偏移量x
                    del_n_end(std::to_string(code_info.frame.error_y / 10.0), 3) + "," + // 相机与地码偏移量y
                    del_n_end(std::to_string(code_info.frame.error_yaw), 3)              // 相机与地码偏移量yaw
    );
    logger->debug(std::string(__FUNCTION__) + "() return");
}
