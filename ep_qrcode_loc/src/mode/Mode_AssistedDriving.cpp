
#include "Mode.hpp"

Mode_AssistedDriving::Mode_AssistedDriving(ParamServer &param, MV_SC2005AM *camera) : QRcodeLoc(param, camera)
{
    logger->info(std::string(__FUNCTION__) + "() start");

    qrcode_table = new QRcodeTableV3(param);
    wheel_odom = new WheelSpeedOdometer(trans_camera2base, param);

    code_info.frame.code = 0;
    is_output_available = false;

    logger->info(std::string(__FUNCTION__) + "() return");
}

Mode_AssistedDriving::~Mode_AssistedDriving() {}

void Mode_AssistedDriving::loop()
{
    logger->info(std::string(__FUNCTION__) + "() start");

    double loop_rate = 200.0; // 控制主循环频率200Hz
    ros::Rate loop_rate_ctrl(loop_rate);
    while (ros::ok())
    {
        logger->debug_endl(); // 新循环，log空一行

        std::list<std::vector<nav_msgs::Odometry>> publist; // 最终数据发送队列

        // 一、获取、解算、处理相机原始数据
        cameraFrameProcess(publist);

        // 二、轮速里程计递推
        wheelOdomProcess(publist);

        // 三、处理并发布位姿数据
        publishProcess(publist);

        loop_rate_ctrl.sleep();
        ros::spinOnce();
    }
    logger->info(std::string(__FUNCTION__) + "() return");
}

bool Mode_AssistedDriving::cameraFrameProcess(std::list<std::vector<nav_msgs::Odometry>> &publist)
{
    logger->debug(std::string(__FUNCTION__) + "() start");

    std::vector<geometry_msgs::Pose> v_pose_new;

    if (camera->getframe(&pic))
    {
        if (do_not_jump_this_frame(pic)) // 检查是否需要跳过该帧数据
        {
            if (qrcode_table->onlyfind(pic, &code_info)) // 查询地码信息
            {
                v_pose_new = get_pose(code_info); // 计算base_link在qrmap坐标和map坐标的坐标
            }
            else
            {
                logger->debug(std::string(__FUNCTION__) + "() end: 未查询到地码信息" + std::to_string(pic.code));
                return false;
            }
        }
        else
        {
            logger->debug(std::string(__FUNCTION__) + "() end: 跳过该帧" + std::to_string(pic.code));
            return false;
        }
    }
    else
    {
        logger->debug(std::string(__FUNCTION__) + "() end: 无新的帧");
        return false;
    }

    if (v_pose_new.size() > 0) // 本次循环解算出相机数据
    {
        // 监测车身方向角度是否超过限制
        if (check_is_yaw_available(v_pose_new[0].orientation, code_info.yaw, 10.0))
        {
            err.yaw_out_range = false;
        }
        else
        {
            err.yaw_out_range = true; // 角度超过限制
            logger->info(std::string(__FUNCTION__) + "() 角度超过限制");
        }

        // 监测是否顺序扫码
        if (param.is_check_code_in_order)
        {
            if (qrcode_table->check_is_code_in_order(pic.code, wheel_odom->get_vel_msg().linear.x))
            {
                err.code_jump = false;
            }
            else
            {
                err.code_jump = true; // 未按顺序扫码
                logger->info(std::string(__FUNCTION__) + "() 未按顺序扫码");
            }
        }

        // 监测是否在二维码处发生跳变
        if (is_output_available)
        {
            if (check_is_pose_jump(wheel_odom->getCurOdom().pose.pose, v_pose_new[0]))
            {
                err.pose_jump = true; // 发生跳变
                logger->info(std::string(__FUNCTION__) + "() 发生跳变");
            }
            else
            {
                err.is_pose_jump = false;
            }
        }

        // 卡尔曼滤波
        if (!qrcode_table->is_head(pic.code)) // 不是列首码
        {
            geometry_msgs::Pose pose_observe = v_pose_new[0];
            geometry_msgs::Pose pose_recursion = wheel_odom->getCurOdom().pose.pose;
            v_pose_new[0] = kalman_f_my(pose_recursion, param.rec_p1, pose_observe, 1.0 - param.rec_p1, 0.1);
        }

        // 打包生成消息
        std::vector<nav_msgs::Odometry> v_odom = packageMsg(v_pose_new, code_info);

        // 扫码无异常后设置轮速里程计初值
        if (err.is_noErr())
        {
            wheel_odom->setEstimationInitialPose(v_odom[0]); // 设置递推初值
            v_odom[0].pose.covariance[6] = 1;                // 扫码正常，已根据二维码结果设置递推初值”
        }
        else
        {
            v_odom[0].pose.covariance[6] = 2; // 扫码异常，未根据二维码结果设置递推初值”
            logger->debug(std::string(__FUNCTION__) + "() 扫码异常，未根据二维码结果设置递推初值");
        }

        publist.push_back(v_odom);  // 放入发送队列
        is_output_available = true; // 入列并扫码后输出可用

        logger->debug(std::string(__FUNCTION__) + "() end: 输出帧" + std::to_string(pic.code));
        return true;
    }
    else
    {
        logger->debug(std::string(__FUNCTION__) + "() end: v_pose_new无数据 " + std::to_string(pic.code));
        return false;
    }
}

bool Mode_AssistedDriving::wheelOdomProcess(std::list<std::vector<nav_msgs::Odometry>> &publist)
{
    logger->debug(std::string(__FUNCTION__) + "() start");
    std::vector<nav_msgs::Odometry> v_odom_wheel;
    if (wheel_odom->is_start())
    {
        if (wheel_odom->run_odom(v_odom_wheel))
        {
            // 用于展示曲线
            v_odom_wheel[0].pose.covariance[7] = code_info.frame.error_x;   // 地码与相机横向偏差
            v_odom_wheel[0].pose.covariance[8] = code_info.frame.error_y;   // 地码与相机纵向偏差
            v_odom_wheel[0].pose.covariance[9] = code_info.frame.error_yaw; // 地码与相机角度偏差

            publist.push_back(v_odom_wheel); // 预发布递推后的位姿

            // 递推距离清零、判断递推是否过远
            if (is_output_available) // 可用
            {
                if (wheel_odom->is_path_dis_overflow()) // 递推过远
                {
                    logger->debug(std::string(__FUNCTION__) + "() 在列内递推过远");
                    err.path_dis_overflow = true; // 在列内递推过远
                }
                else
                {
                    err.path_dis_overflow = false;
                }
            }

            logger->debug(std::string(__FUNCTION__) + "() end");
            return true;
        }
        else
        {
            logger->debug(std::string(__FUNCTION__) + "() end: run_odom() return false");
            return false;
        }
    }
    else
    {
        logger->debug(std::string(__FUNCTION__) + "() end: wheel_odom未开始运行");
        return false;
    }
}

void Mode_AssistedDriving::publishProcess(std::list<std::vector<nav_msgs::Odometry>> &publist)
{
    logger->debug(std::string(__FUNCTION__) + "() start");

    while (publist.size() > 0)
    {
        // 取出数据
        std::vector<nav_msgs::Odometry> output = publist.front(); // 取出即将发送的数据
        publist.pop_front();                                      // 删除取出的数据

        static std::vector<nav_msgs::Odometry> output_last = output; // 上个输出

        // 状态判断
        mtx.lock();
        if (is_handle) // 手动
        {
            is_output_available = false; // 数据不可用
            err.reset();
            output[0].pose.covariance[0] = 0;   // 数据不可用
            output[0].pose.covariance[1] = 0;   // 故障急停：否
            pic.code = 0;                       // 列外码值清零
            check_is_code_in_order(0, 0, true); // 列外初始化二维码顺序判定
            do_not_jump_this_frame(pic, true);  // 复位该功能
            wheel_odom->reset_path_dis();       // 进入列首，递推距离清零
        }
        else // 自动
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
                    if (check_is_pose_jump(output_last[0].pose.pose, output[0].pose.pose)) // 发生
                    {
                        err.pose_jump = true;
                        logger->info(std::string(__FUNCTION__) + "() 发生跳变");
                    }
                    else
                    {
                        err.pose_jump = false;
                    }
                }
            }

            // 判断是否处于故障状态
            if (err.is_noErr())
            {
                output[0].pose.covariance[1] = 0; // 故障急停：否
            }
            else
            {
                output[0].pose.covariance[1] = 1; // 故障急停：是
                logger->debug(std::string(__FUNCTION__) + "() 紧急停车！！！");
            }
        }
        output[0].pose.covariance[2] = err.errCode(); // 故障代码
        mtx.unlock();

        // 记录正常扫码的跳变值，用于地码方向角调试
        if (1 == output[0].pose.covariance[6]) // 扫码正常，已根据二维码结果设置递推初值
        {
            output_jump_err(code_info, output, output_last);
        }

        // 记录本帧数据
        output_log(pic, code_info, wheel_odom->get_vel_msg(), output, qrcode_table->is_head(pic.code));

        // 扫码无异常，则发布消息
        if (2 == output[0].pose.covariance[6])
        {
            logger->debug(std::string(__FUNCTION__) + "() 扫码异常，本帧消息不发布");
        }
        else
        {
            pubOdom(output);      // 发布消息
            output_last = output; // 保存用于下个循环
        }
    }
    logger->debug(std::string(__FUNCTION__) + "() end");
    return;
}

// void a()
// {
//     if("递推距离在允许范围内" && "车身角度在允许范围内")
//     {
//         "定位可用"
//     }
//     else
//     {
//         "定位不可用"
//     }

//     if("自动状态")
//     {
//         if("定位不可用" && "车在列内" && "车在移动")
//         {
//             "停车，不可急停"
//         }

//         if("定位可用" && "跳变超过阈值")
//         {
//             "停车，不可急停"
//             "报警"
//         }
//     }
//     else
//     {
//         "手动控制方向"
//     }
// }
