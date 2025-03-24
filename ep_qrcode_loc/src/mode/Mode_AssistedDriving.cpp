
#include "Mode.hpp"

Mode_AssistedDriving::Mode_AssistedDriving(ParamServer &param, MV_SC2005AM *camera) : QRcodeLoc(param, camera)
{
    logger->info("Mode_AssistedDriving() start");

    qrcode_table_v3 = new QRcodeTableV3(param);
    wheel_odom = new WheelSpeedOdometer(trans_camera2base, param);

    code_info.frame.code = 0;
    err_type = 0x00;
    is_output_available = false;

    logger->info("Mode_AssistedDriving() return");
}

Mode_AssistedDriving::~Mode_AssistedDriving() {}

void Mode_AssistedDriving::loop()
{
    logger->info("Mode_AssistedDriving::loop()");

    double loop_rate = 200.0; // 控制主循环频率200Hz
    ros::Rate loop_rate_ctrl(loop_rate);
    while (ros::ok())
    {
        std::vector<geometry_msgs::Pose> v_pose_new;        // 存放地码解算出的位姿

        logger->debug_endl(); // 新循环，log空一行

        // 一、获取、解算相机原始数据
        if (camera->getframe(&pic))
        {
            logger->debug("getframe");
            if (do_not_jump_this_frame(pic)) // 检查是否需要跳过该帧数据
            {
                if (qrcode_table_v3->onlyfind(pic, &code_info)) // 查询地码信息
                {
                    v_pose_new = get_pose(code_info); // 计算base_link在qrmap坐标和map坐标的坐标
                }
            }
        }

        // 二、处理扫码解算结果
        if (v_pose_new.size() > 0) // 本次循环解算出相机数据
        {
            cameraFrameProcess(v_pose_new);
        }

        // 三、轮速里程计递推
        if (wheel_odom->is_start())
        {
            wheelOdomProcess();
        }

        // 四、发布位姿数据
        while (publist.size() > 0)
        {
            // 取出数据
            std::vector<nav_msgs::Odometry> output = publist.front();    // 取出即将发送的数据
            publist.pop_front();                                         // 删除取出的数据

            // 处理并发布
            publishProcess(output);
        }

        loop_rate_ctrl.sleep();
        ros::spinOnce();
    }
}

void Mode_AssistedDriving::cameraFrameProcess(std::vector<geometry_msgs::Pose> &v_pose_new)
{
    std::vector<nav_msgs::Odometry> v_odom;             // 存放打包好可以发出的地码解算数据

    logger->debug("v_pose_new.size() > 0");

    // 监测车身方向角度是否超过限制
    if (!is_yaw_available(v_pose_new[0].orientation, code_info.yaw, 10.0))
        err_type = err_type | 0x01; // 角度超过限制

    // 监测是否顺序扫码
    if(param.is_check_code_in_order)
    {
        if (!qrcode_table_v3->is_code_in_order(pic.code, wheel_odom->get_vel_msg().linear.x))
            err_type = err_type | 0x02; // 未按顺序扫码
    }

    // 监测是否在二维码处发生跳变
    if (is_output_available)
    {
        if (is_pose_jump(wheel_odom->getCurOdom().pose.pose, v_pose_new[0]))
            err_type = err_type | 0x04; // 发生跳变
    }

    // 卡尔曼滤波
    if (!qrcode_table_v3->is_head(pic.code)) // 不是列首码
    {
        geometry_msgs::Pose pose_observe = v_pose_new[0];
        geometry_msgs::Pose pose_recursion = wheel_odom->getCurOdom().pose.pose;
        v_pose_new[0] = kalman_f_my(pose_recursion, param.rec_p1, pose_observe, 1.0 - param.rec_p1, 0.1);
    }

    // 打包生成消息
    v_odom = packageMsg(v_pose_new, code_info);

    // 扫码无异常后设置轮速里程计初值
    if (0x00 == err_type)
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

void Mode_AssistedDriving::wheelOdomProcess()
{
    logger->debug("wheel_odom is start");
    std::vector<nav_msgs::Odometry> v_odom_wheel;
    if (wheel_odom->run_odom(v_odom_wheel))
    {
        // 用于展示曲线
        v_odom_wheel[0].pose.covariance[7] = code_info.frame.error_x;   // 地码与相机横向偏差
        v_odom_wheel[0].pose.covariance[8] = code_info.frame.error_y;   // 地码与相机纵向偏差
        v_odom_wheel[0].pose.covariance[9] = code_info.frame.error_yaw; // 地码与相机角度偏差

        publist.push_back(v_odom_wheel); // 预发布递推后的位姿
        logger->debug("wheel odom publist.push_back(v_odom_wheel)");

        // 递推距离清零、判断递推是否过远
        if (is_output_available) // 可用
        {
            if (wheel_odom->is_path_dis_overflow()) // 递推过远
            {
                logger->debug("在列内递推过远");
                err_type = err_type | 0x08; // 在列内递推过远
            }
        }
    }
}

void Mode_AssistedDriving::publishProcess(std::vector<nav_msgs::Odometry> &output)
{
    static std::vector<nav_msgs::Odometry> output_last = output; // 上个输出
    // 状态判断
    mtx.lock();
    if (!is_handle) // 自动
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
    else // 手动
    {
        is_output_available = false;       // 数据不可用
        err_type = 0x00;                   // 清除故障急停状态
        output[0].pose.covariance[0] = 0;  // 数据不可用
        output[0].pose.covariance[1] = 0;  // 故障急停：否
        pic.code = 0;                      // 列外码值清零
        is_code_in_order(0, 0, true);      // 列外初始化二维码顺序判定
        do_not_jump_this_frame(pic, true); // 复位该功能
        wheel_odom->reset_path_dis();      // 进入列首，递推距离清零
    }
    output[0].pose.covariance[2] = err_type; // 故障类型
    mtx.unlock();

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
