#include "Mode.hpp"

// 构造函数
Mode_GetYaw::Mode_GetYaw(ParamServer &param, MV_SC2005AM *camera) : QRcodeLoc(param, camera)
{
    // 实例化功能对象
    qrcode_table = new QRcodeTableV2(param.cfg_dir, trans_camera2base, param);
    wheel_odom = new WheelSpeedOdometer(trans_camera2base, param);
}

Mode_GetYaw::~Mode_GetYaw() {}

// 采集地码角度模式
void Mode_GetYaw::loop()
{
    logger->info("Mode_GetYaw::loop()");
    
    param.read_yaw_err = false;
    logger->info("param.read_yaw_err = false;");

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
                logger->yawerr("," + std::to_string(pic.code) + "," +
                               std::to_string(wheel_odom->get_vel_x()) + "," +
                               std::to_string(cur_qrcode.yaw) + "," +
                               std::to_string(code_info.yaw));
            }
            else
            {
                logger->info("未识别index: " + std::to_string(pic.code));
            }
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
}
