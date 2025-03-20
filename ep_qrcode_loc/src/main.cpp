#include "utility_qloc.hpp"
#include "Mode.hpp"

bool chooseMode(QRcodeLoc *qloc, ParamServer &param, MV_SC2005AM &Camera)
{
    if ("1" == param.operating_mode)
    {
        qloc = new Mode_CollectQRCodePose(param, &Camera);
        return true;
    }
    else if ("2" == param.operating_mode)
    {
        qloc = new Mode_CollectQRCodeIndex(param, &Camera);
        return true;
    }
    else if ("3" == param.operating_mode)
    {
        qloc = new Mode_ShiHua(param, &Camera);
        return true;
    }
    else if ("4" == param.operating_mode)
    {
        qloc = new Mode_TestRun(param, &Camera);
        return true;
    }
    else if ("5" == param.operating_mode)
    {
        qloc = new Mode_GetYaw(param, &Camera);
        return true;
    }
    else if ("6" == param.operating_mode)
    {
        qloc = new Mode_CalYawErr(param, &Camera);
        return true;
    }
    else if ("7" == param.operating_mode)
    {
        qloc = new Mode_CheckCameraHorizon(param, &Camera);
        return true;
    }
    else if ("8" == param.operating_mode)
    {
        qloc = new Mode_AssistedDriving(param, &Camera);
        return true;
    }
    else if ("Mode_CollectQRCodePose" == param.operating_mode)
    {
        qloc = new Mode_CollectQRCodePose(param, &Camera);
        return true;
    }
    else if ("Mode_CollectQRCodeIndex" == param.operating_mode)
    {
        qloc = new Mode_CollectQRCodeIndex(param, &Camera);
        return true;
    }
    else if ("Mode_ShiHua" == param.operating_mode)
    {
        qloc = new Mode_ShiHua(param, &Camera);
        return true;
    }
    else if ("Mode_TestRun" == param.operating_mode)
    {
        qloc = new Mode_TestRun(param, &Camera);
        return true;
    }
    else if ("Mode_GetYaw" == param.operating_mode)
    {
        qloc = new Mode_GetYaw(param, &Camera);
        return true;
    }
    else if ("Mode_CalYawErr" == param.operating_mode)
    {
        qloc = new Mode_CalYawErr(param, &Camera);
        return true;
    }
    else if ("Mode_CheckCameraHorizon" == param.operating_mode)
    {
        qloc = new Mode_CheckCameraHorizon(param, &Camera);
        return true;
    }
    else if ("Mode_AssistedDriving" == param.operating_mode)
    {
        qloc = new Mode_AssistedDriving(param, &Camera);
        return true;
    }
    else if ("Mode_North" == param.operating_mode)
    {
        qloc = new Mode_North(param, &Camera);
        return true;
    }
    else
    {
        return false;
    }
}

// 主函数
int main(int argc, char **argv)
{
    // ros初始化
    ros::init(argc, argv, "qrcode_loc");
    ros::NodeHandle nh;

    // 参数管理器
    ParamServer param(nh);

    // 记录管理器
    epLogger *logger;
    logger = &epLogger::getInstance();
    logger->init(&param);
    param.saveLog(logger);
    logger->roll_delete_old_folders();                     // 滚动删除历史文件夹
    std::thread logLoopThread(&epLogger::logLoop, logger); // 相机循环线程

    // 相机对象
    MV_SC2005AM Camera(param);
    std::thread cameraLoopThread(&MV_SC2005AM::cameraLoop, &Camera); // 相机循环线程

    // 运行模式对象
    QRcodeLoc *QLoc;
    bool res = chooseMode(QLoc, param, Camera);
    if (res)
    {
        std::thread mainLoopThread(&QRcodeLoc::loop, QLoc); // 主循环线程
    }
    else
    {
        logger->info("not found Mode!");
    }

    // 输出提示
    ROS_INFO("\033[1;32m----> Localization with QR-code Started.\033[0m");
    ros::spin(); // spin
    return 0;
}

/*
TODO:


DONE:
1、相机数据作为topic发出，订阅该topic进行使用，可通过录包形式保存数据
4、考虑车辆一直运行带来的log保存问题

DELAY:
2、二维码序号参数文件放在vcs进行管理，需配合单码调试工具进行升级
3、监控vcs中参数文件的变动，自动重启程序


*/
