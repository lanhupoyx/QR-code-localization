#include "utility_qloc.hpp"
#include "QRCodeLoc.hpp"

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

    QRcodeLoc QLoc(param, &Camera);                           // 实例化二维码定位对象
    std::thread mainLoopThread(&QRcodeLoc::mainloop, &QLoc); // 主循环线程

    ROS_INFO("\033[1;32m----> Localization with QRcode Started.\033[0m"); // 输出提示
    ros::spin();                                                          // spin

    return 0;
}

/*
TODO：
1、相机数据作为topic发出，订阅该topic进行使用，可通过录包形式保存数据

2、二维码序号参数文件放在vcs进行管理，需配合单码调试工具进行升级

3、监控vcs中参数文件的变动，自动重启程序




*/
