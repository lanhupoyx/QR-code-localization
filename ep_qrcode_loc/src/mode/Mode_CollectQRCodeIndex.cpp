#include "Mode.hpp"

// 构造函数
Mode_CollectQRCodeIndex::Mode_CollectQRCodeIndex(ParamServer &param, MV_SC2005AM *camera) : QRcodeLoc(param, camera)
{
    logger->info(std::string(__FUNCTION__) + "() start");

    // 实例化功能对象
    qrcode_table = new QRcodeTableV2(param.cfg_dir, trans_camera2base, param);
    wheel_odom = new WheelSpeedOdometer(trans_camera2base, param);

    logger->info(std::string(__FUNCTION__) + "() return");
}

Mode_CollectQRCodeIndex::~Mode_CollectQRCodeIndex() {}

// 采集二维码编号模式
void Mode_CollectQRCodeIndex::loop()
{
    logger->info(std::string(__FUNCTION__) + "() start");

    ros::Rate loop_rate(100); // 主循环 100Hz
    while (ros::ok())
    {
        static uint32_t code_last = 0;
        if (camera->getframe(&pic))
        {
            if (code_last != pic.code)
            {
                std::stringstream stream;
                stream.str("");
                stream << pic.code;
                logger->info(stream.str());

                code_last = pic.code;
            }
        }
    }
    logger->info(std::string(__FUNCTION__) + "() return");
}
