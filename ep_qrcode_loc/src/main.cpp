#include "utility_qloc.hpp"
#include "QRCodeLoc.hpp"


// 主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "qrcode_loc");                                  // ros初始化
    ros::NodeHandle nh;

    ParamServer param(nh);
    Logger *logger;
    logger = &Logger::getInstance();
    logger->init(&param);
    param.saveLog(logger);

    QRcodeLoc QLoc(param);                                                       // 实例化二维码定位对象
    std::thread loopthread(&QRcodeLoc::mainloop, &QLoc);            // 创建主循环线程
    ROS_INFO("\033[1;32m----> Localization with QRcode Started.\033[0m"); // 输出提示
    ros::spin();                                                          // spin
    loopthread.join();                                                    // 运行主循环

    return 0;
}


/*
TODO：
1、相机数据作为topic发出，订阅该topic进行使用，可通过录包形式保存数据

2、二维码序号参数文件放在vcs进行管理，需配合单码调试工具进行升级

3、监控vcs中参数文件的变动，自动重启程序




*/
