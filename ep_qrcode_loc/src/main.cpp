#include "utility_qloc.hpp"
#include "Mode.hpp"

// 主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "qrcode_loc"); // ros初始化
    ros::NodeHandle nh;

    ParamServer param(nh); // 参数管理器
    Logger *logger;        // 记录管理器
    logger = &Logger::getInstance();
    logger->init(&param);
    param.saveLog(logger);

    MV_SC2005AM Camera(param);                                       // 相机对象
    std::thread cameraLoopThread(&MV_SC2005AM::cameraLoop, &Camera); // 相机循环线程

    // QRcodeLoc QLoc(param, &Camera);                           // 实例化二维码定位对象
    // std::thread mainLoopThread(&QRcodeLoc::mainloop, &QLoc); // 主循环线程
    QRcodeLoc* QLoc;

    if ("1" == param.operating_mode)
        QLoc = new Mode_CollectQRCodePose(param, &Camera);  
    else if ("2" == param.operating_mode)
        QLoc = new Mode_CollectQRCodeIndex(param, &Camera);  
    else if ("3" == param.operating_mode)
        QLoc = new Mode_ShiHua(param, &Camera);  
    else if ("4" == param.operating_mode)
        QLoc = new Mode_TestRun(param, &Camera);  
    else if ("5" == param.operating_mode)
        QLoc = new Mode_GetYaw(param, &Camera);  
    else if ("6" == param.operating_mode)
        QLoc = new Mode_CalYawErr(param, &Camera);  
    else if ("7" == param.operating_mode)
        QLoc = new Mode_CheckCameraHorizon(param, &Camera);  
    else if ("8" == param.operating_mode)
        QLoc = new Mode_AssistedDriving(param, &Camera);  
    else if ("Mode_CollectQRCodePose" == param.operating_mode)
        QLoc = new Mode_CollectQRCodePose(param, &Camera);  
    else if ("Mode_CollectQRCodeIndex" == param.operating_mode)
        QLoc = new Mode_CollectQRCodeIndex(param, &Camera);  
    else if ("Mode_ShiHua" == param.operating_mode)
        QLoc = new Mode_ShiHua(param, &Camera);  
    else if ("Mode_TestRun" == param.operating_mode)
        QLoc = new Mode_TestRun(param, &Camera);  
    else if ("Mode_GetYaw" == param.operating_mode)
        QLoc = new Mode_GetYaw(param, &Camera);  
    else if ("Mode_CalYawErr" == param.operating_mode)
        QLoc = new Mode_CalYawErr(param, &Camera);  
    else if ("Mode_CheckCameraHorizon" == param.operating_mode)
        QLoc = new Mode_CheckCameraHorizon(param, &Camera);  
    else if ("Mode_AssistedDriving" == param.operating_mode)
        QLoc = new Mode_AssistedDriving(param, &Camera);  
    else if ("Mode_North" == param.operating_mode)
        QLoc = new Mode_North(param, &Camera);  
    else
    {
        logger->info("not found Mode!");
        return 0;
    }

    std::thread mainLoopThread(&QRcodeLoc::loop, QLoc); // 主循环线程

    ROS_INFO("\033[1;32m----> Localization with QR-code Started.\033[0m"); // 输出提示
    ros::spin();                                                          // spin

    return 0;
}

/*
TODO：
1、相机数据作为topic发出，订阅该topic进行使用，可通过录包形式保存数据
2、二维码序号参数文件放在vcs进行管理，需配合单码调试工具进行升级
3、监控vcs中参数文件的变动，自动重启程序

4、考虑车辆一直运行带来的log保存问题


*/
