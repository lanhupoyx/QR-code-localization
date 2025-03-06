#include "utility_qloc.hpp"
#include "logger.hpp"
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
