#include "Mode.hpp"

// 构造函数
Mode_Trash::Mode_Trash(ParamServer &param, MV_SC2005AM *camera) : QRcodeLoc(param, camera)
{
    logger->info(std::string(__FUNCTION__) + "() start");

    // 实例化功能对象
    //qrcode_table = new QRcodeTableV2(param.cfg_dir, trans_camera2base, param);
    //wheel_odom = new WheelSpeedOdometer(trans_camera2base, param);

    logger->info(std::string(__FUNCTION__) + "() return");
}

Mode_Trash::~Mode_Trash() {}

// 采集二维码编号模式
void Mode_Trash::loop()
{
    logger->info(std::string(__FUNCTION__) + "() start");
    logger->info(std::string(__FUNCTION__) + "() return");
    return;
}
