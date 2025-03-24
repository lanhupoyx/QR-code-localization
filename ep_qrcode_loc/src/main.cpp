#include "utilityQloc.hpp"
#include "ParamServer.hpp"
#include "logger.hpp"
#include "Mode.hpp"
#include "camera.hpp"
#include "QRCodeLoc.hpp"

std::shared_ptr<QRcodeLoc> chooseMode(ParamServer &param, MV_SC2005AM &camera)
{
    std::shared_ptr<QRcodeLoc> QRcodeLocPtr;

    if ("1" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_CollectQRCodePose>(param, &camera);
    }
    else if ("2" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_CollectQRCodeIndex>(param, &camera);
    }
    else if ("3" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_ShiHua>(param, &camera);
    }
    else if ("4" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_TestRun>(param, &camera);
    }
    else if ("5" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_GetYaw>(param, &camera);
    }
    else if ("6" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_CalYawErr>(param, &camera);
    }
    else if ("7" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_CheckCameraHorizon>(param, &camera);
    }
    else if ("8" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_AssistedDriving>(param, &camera);
    }
    else if ("Mode_CollectQRCodePose" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_CollectQRCodePose>(param, &camera);
    }
    else if ("Mode_CollectQRCodeIndex" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_CollectQRCodeIndex>(param, &camera);
    }
    else if ("Mode_ShiHua" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_ShiHua>(param, &camera);
    }
    else if ("Mode_TestRun" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_TestRun>(param, &camera);
    }
    else if ("Mode_GetYaw" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_GetYaw>(param, &camera);
    }
    else if ("Mode_CalYawErr" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_CalYawErr>(param, &camera);
    }
    else if ("Mode_CheckCameraHorizon" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_CheckCameraHorizon>(param, &camera);
    }
    else if ("Mode_AssistedDriving" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_AssistedDriving>(param, &camera);
    }
    else if ("Mode_North" == param.operating_mode)
    {
        QRcodeLocPtr = std::make_shared<Mode_North>(param, &camera);
    }
    else
    {
        QRcodeLocPtr = std::make_shared<Mode_North>(param, &camera);
    }

    return QRcodeLocPtr;
}

void runLoop(std::shared_ptr<QRcodeLoc> qloc)
{
    qloc->loop();
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
    logger->init(param.logLevel, param.log_dir, param.logKeepDays);
    logger->info("\n" + param.yamlData);                   // 参数原始文本
    logger->info("\n" + param.logData);                    // 读取记录
    logger->roll_delete_old_folders();                     // 滚动删除历史文件夹
    std::thread logLoopThread(&epLogger::logLoop, logger); // 相机循环线程

    // 相机对象
    MV_SC2005AM camera(param);
    std::thread cameraLoopThread(&MV_SC2005AM::cameraLoop, &camera); // 相机循环线程

    // 运行模式对象
    std::shared_ptr<QRcodeLoc> QRcodeLocPtr = chooseMode(param, camera);
    std::thread mainLoopThread(runLoop, QRcodeLocPtr); // 相机循环线程

    // 输出提示
    ROS_INFO("\033[1;32m----> Localization with QR-code Started.\033[0m");
    ros::spin(); // spin
    return 0;
}

/*
TODO:
6、自动读取版本号等基础信息到log
7、qrcode_table多态问题
9、sub相关的功能集合成一个模块
10、重启后log文件接着用
11、所有函数都添加开始和结束的debug信息


DONE:
1、相机数据作为topic发出，订阅该topic进行使用，可通过录包形式保存数据
4、考虑车辆一直运行带来的log保存问题
5、每天运行一次log循环删除
8、log文件问题，pose分小时，其它分天


DELAY:
2、二维码序号参数文件放在vcs进行管理，需配合单码调试工具进行升级
3、监控vcs中参数文件的变动，自动重启程序


*/
