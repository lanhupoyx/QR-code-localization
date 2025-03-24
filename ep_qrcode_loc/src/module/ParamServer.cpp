#include "ParamServer.hpp"

using namespace vcs;

// 参数服务器
ParamServer::ParamServer(ros::NodeHandle &nh) : nh(nh)
{
    std::cout << "ParamServer() start" << std::endl;

    //获取vcs参数文件，或者默认参数文件路径
    mainParamPath = loadMainParamPath();
    siteTablePath = loadSiteTableParamPath();
    GroundCodeTablePath = loadGroundCodeTableParamPath();

    // 读取主参数文件
    yamlData = "";
    cppc::File file(mainParamPath);
    if (file.exists())
    {
        if (file.open(cppc::File::ReadOnly))
        {
            yamlData = file.readAll();
            file.close();
            std::cout << "参数文件：" << mainParamPath << " 读取成功！" << std::endl;
        }
        else
        {
            std::cout << "参数文件：" << mainParamPath << " 读取失败！" << std::endl;
        }
    }
    else
    {
        std::cout << "参数文件：" << mainParamPath << " 不存在！" << std::endl;
    }

    // 读取参数
    try
    {
        // 从字符串加载 YAML 数据
        YAML::Node config = YAML::Load(yamlData);

        importItem<std::string>(config, "ep_qrcode_loc", "logLevel", logLevel, "INFO");
        importItem<std::string>(config, "ep_qrcode_loc", "operating_mode", operating_mode, "0");
        importItem<std::string>(config, "ep_qrcode_loc", "odomMapBase", odomMapBase, "ep_qrcode_loc/odometry/base");
        importItem<std::string>(config, "ep_qrcode_loc", "odomMapCamera", odomMapCamera, "ep_qrcode_loc/odometry/locCamera");
        importItem<std::string>(config, "ep_qrcode_loc", "pathMapBase", pathMapBase, "ep_qrcode_loc/path/base");
        importItem<std::string>(config, "ep_qrcode_loc", "pathMapCamera", pathMapCamera, "ep_qrcode_loc/path/locCamera");
        importItem<std::string>(config, "ep_qrcode_loc", "msgTopic", msgTopic, "ep_qrcode_loc/msg");
        importItem<bool>(config, "ep_qrcode_loc", "show_original_msg", show_original_msg, false);
        importItem<bool>(config, "ep_qrcode_loc", "is_pub_tf", is_pub_tf, false);
        importItem<double>(config, "ep_qrcode_loc", "yaw_jump_UL", yaw_jump_UL, 2.0);
        importItem<double>(config, "ep_qrcode_loc", "x_jump_UL", x_jump_UL, 0.05);
        importItem<double>(config, "ep_qrcode_loc", "y_jump_UL", y_jump_UL, 0.05);
        importItem<bool>(config, "ep_qrcode_loc", "read_yaw_err", read_yaw_err, false);
        importItem<double>(config, "ep_qrcode_loc", "err_ratio_offline", err_ratio_offline, 1.0);
        importItem<double>(config, "ep_qrcode_loc", "rec_p1", rec_p1, 0.0);
        importItem<double>(config, "ep_qrcode_loc", "avliable_yaw", avliable_yaw, 0.0);
        importItem<double>(config, "ep_qrcode_loc", "wheel_diameter", wheel_diameter, 0.0);
        importItem<double>(config, "ep_qrcode_loc", "wheel_reduction_ratio", wheel_reduction_ratio, 1.0);
        importItem<double>(config, "ep_qrcode_loc", "wheel_base_dis", wheel_base_dis, 0.0);
        importItem<double>(config, "ep_qrcode_loc", "wheel_angular_forward", wheel_angular_forward, 0.0);
        importItem<double>(config, "ep_qrcode_loc", "wheel_angular_backward", wheel_angular_backward, 0.0);
        importItem<double>(config, "ep_qrcode_loc", "low_speed_UL", low_speed_UL, 0.2);
        importItem<std::string>(config, "ep_qrcode_loc", "port", port, "1024");
        importItem<std::string>(config, "ep_qrcode_loc", "log_dir", log_dir, "/var/xmover/log/QR_code_loc/");
        importItem<std::string>(config, "ep_qrcode_loc", "cfg_dir", cfg_dir, "/var/xmover/params/ep-qrcode-loc/");
        importItem<float>(config, "ep_qrcode_loc", "maxEstimationDis", maxEstimationDis, 1.0);
        importItem<double>(config, "ep_qrcode_loc", "detect_site_dis", detect_site_dis, 1.8);
        importItem<double>(config, "ep_qrcode_loc", "aux_site_dis", aux_site_dis, 1.365);
        importItem<double>(config, "ep_qrcode_loc", "forkaction_site_dis", forkaction_site_dis, 0.93);
        importItem<double>(config, "ep_qrcode_loc", "site_site_dis", site_site_dis, 1.36);
        importItem<bool>(config, "ep_qrcode_loc", "is_debug", is_debug, false);
        importItem<bool>(config, "ep_qrcode_loc", "check_sequence", check_sequence, true);
        importItem<bool>(config, "ep_qrcode_loc", "cal_yaw", cal_yaw, true);
        importItem<double>(config, "ep_qrcode_loc", "ground_code_yaw_offset", ground_code_yaw_offset, 0.0);
        importItem<std::size_t>(config, "ep_qrcode_loc", "logKeepDays", logKeepDays, 90);
        importItem<bool>(config, "ep_qrcode_loc", "is_mainloop_query_camera", is_mainloop_query_camera, true);
        importItem<bool>(config, "ep_qrcode_loc", "is_check_code_in_order", is_check_code_in_order, false);
        importItem<bool>(config, "ep_qrcode_loc", "is_moter_speed_reverse", is_moter_speed_reverse, false);
        importItem<bool>(config, "ep_qrcode_loc", "is_wheel_angular_reverse", is_wheel_angular_reverse, false);
    }
    catch (YAML::Exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    std::cout << "ParamServer() return" << std::endl;
}

// 加载主参数
std::string ParamServer::loadMainParamPath()
{
    std::string DefaultParamFilePath = "/opt/xmover/ros/melodic/ep_qrcode_loc/share/ep_qrcode_loc/config/ep-qrcode-loc.yaml";

    ConfigInfo configInfo;
    configInfo.setDataId("ep_qrcode_loc");
    configInfo.setGroupId("SLAM");
    configInfo.setType("yaml"); // yaml/toml/json/text

    vcs::VcsManager vm;
    vcs::VcsParams params;
    vm.initParams(params);

    bool configValid = false;
    ros::Time startTime = ros::Time::now();

    ros::Rate loop_rate(1); // 1Hz
    while (ros::ok())
    {
        ros::Time nowTime = ros::Time::now();
        if (nowTime - startTime > ros::Duration(120.0))
        {
            break; // 最多等2分钟，如果2分钟后还没取到配置,则向后执行，比如上报故障，注意不能乱动作避免安全问题
        }

        // 一、拉取配置
        configValid = vm.configService().getConfig(configInfo); // 先获取一下配置，如VCS无法访问，会在30秒内一直重试
        if (configValid)
        {
            // 可以进行参数检查，如果有需要自动追加的新参数，则可以追加后强制推送
            // bool forcePublicFlag = vm.configService().forcePublishConfig(configInfo, 30000); // 强制覆盖,建议不要强制覆盖，除非明确知道影响。
            break;
        }

        // 二、平台不存在配置，且本地无缓存，默认配置文件上传平台一份
        std::cout << "平台不存在配置，且本地无缓存，默认配置文件上传平台一份" << std::endl;
        bool defaultConfigFlag = configInfo.setContentWithFile(DefaultParamFilePath);
        if (!defaultConfigFlag)
        {
            std::cout << "加载默认配置失败，磁盘可能有问题" << std::endl;
            continue;
        }
        std::cout << "加载打包的默认配置成功，准备上传平台一份，方便后续维护" << std::endl;

        configInfo.setMaxBackupCount(-1); // -2：不修改现状，-1：无限备份，0：无备份，正数为具体备份个数
        bool publicFlag = vm.configService().publishConfig(configInfo, 30000);
        if (publicFlag)
        {
            std::cout << "默认配置上传平台成功" << std::endl;
            configValid = true;
            continue;
        }
        else
        {
            if (configInfo.state() == vcs::ConfigInfo::CS_ONLINE_OK)
            {
                std::cout << "默认配置上传平台失败,已有相同配置,禁止覆盖" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(3));
                continue; // 既然VCS存在了相同配置，那么等一下接着重新拉取配置
            }
            else
            {
                std::cout << "默认配置上传平台失败,发生未知错误,重走获取流程" << std::endl;
            }
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    if (configValid)
    {
        std::cout << "文件路径：" 
            << configInfo.localFilePath() << std::endl;
        return configInfo.localFilePath();
    }
    else
    {
        std::cout << "文件路径：" 
            << DefaultParamFilePath << std::endl;
        return DefaultParamFilePath;
    }
}

// 加载库位参数
std::string ParamServer::loadSiteTableParamPath()
{
    std::string DefaultParamFilePath = "/var/xmover/params/ep-qrcode-loc/SiteTable.txt";

    bool configValid = false;

    ConfigInfo configInfo;

    if (false) //是否使用VCS管理地码信息
    {
        DefaultParamFilePath = "/opt/xmover/ros/melodic/ep_qrcode_loc/share/ep_qrcode_loc/config/SiteTable.txt";

        configInfo.setDataId("ep_qrcode_SiteTable");
        configInfo.setGroupId("SLAM");
        configInfo.setType("text"); // yaml/toml/json/text

        vcs::VcsManager vm;
        vcs::VcsParams params;
        vm.initParams(params);

        ros::Time startTime = ros::Time::now();

        ros::Rate loop_rate(1); // 1Hz
        while (ros::ok())
        {
            ros::Time nowTime = ros::Time::now();
            if (nowTime - startTime > ros::Duration(120.0))
            {
                break; // 最多等2分钟，如果2分钟后还没取到配置,则向后执行，比如上报故障，注意不能乱动作避免安全问题
            }

            // 一、拉取配置
            configValid = vm.configService().getConfig(configInfo); // 先获取一下配置，如VCS无法访问，会在30秒内一直重试
            if (configValid)
            {
                // 可以进行参数检查，如果有需要自动追加的新参数，则可以追加后强制推送
                // bool forcePublicFlag = vm.configService().forcePublishConfig(configInfo, 30000); // 强制覆盖,建议不要强制覆盖，除非明确知道影响。
                break;
            }

            // 二、平台不存在配置，且本地无缓存，默认配置文件上传平台一份
            std::cout << "平台不存在配置，且本地无缓存，默认配置文件上传平台一份" << std::endl;
            bool defaultConfigFlag = configInfo.setContentWithFile(DefaultParamFilePath);
            if (!defaultConfigFlag)
            {
                std::cout << "加载默认配置失败，磁盘可能有问题" << std::endl;
                continue;
            }
            std::cout << "加载打包的默认配置成功，准备上传平台一份，方便后续维护" << std::endl;

            configInfo.setMaxBackupCount(-1); // -2：不修改现状，-1：无限备份，0：无备份，正数为具体备份个数
            bool publicFlag = vm.configService().publishConfig(configInfo, 30000);
            if (publicFlag)
            {
                std::cout << "默认配置上传平台成功" << std::endl;
                configValid = true;
                continue;
            }
            else
            {
                if (configInfo.state() == vcs::ConfigInfo::CS_ONLINE_OK)
                {
                    std::cout << "默认配置上传平台失败,已有相同配置,禁止覆盖" << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    continue; // 既然VCS存在了相同配置，那么等一下接着重新拉取配置
                }
                else
                {
                    std::cout << "默认配置上传平台失败,发生未知错误,重走获取流程" << std::endl;
                }
            }

            loop_rate.sleep();
            ros::spinOnce();
        }
    }

    if (configValid)
    {
        std::cout << "文件路径："
                  << configInfo.localFilePath() << std::endl;
        return configInfo.localFilePath();
    }
    else
    {
        std::cout << "文件路径："
                  << DefaultParamFilePath << std::endl;
        return DefaultParamFilePath;
    }
}

// 加载地码参数
std::string ParamServer::loadGroundCodeTableParamPath()
{
    std::string DefaultParamFilePath = "/var/xmover/params/ep-qrcode-loc/GroundCodeTable.txt";

    bool configValid = false;

    ConfigInfo configInfo;

    if (false) //是否使用VCS管理地码信息
    {
        DefaultParamFilePath = "/opt/xmover/ros/melodic/ep_qrcode_loc/share/ep_qrcode_loc/config/GroundCodeTable.txt";

        configInfo.setDataId("ep_qrcode_GroundCodeTable");
        configInfo.setGroupId("SLAM");
        configInfo.setType("text"); // yaml/toml/json/text

        vcs::VcsManager vm;
        vcs::VcsParams params;
        vm.initParams(params);

        ros::Time startTime = ros::Time::now();

        ros::Rate loop_rate(1); // 1Hz
        while (ros::ok())
        {
            ros::Time nowTime = ros::Time::now();
            if (nowTime - startTime > ros::Duration(120.0))
            {
                break; // 最多等2分钟，如果2分钟后还没取到配置,则向后执行，比如上报故障，注意不能乱动作避免安全问题
            }

            // 一、拉取配置
            configValid = vm.configService().getConfig(configInfo); // 先获取一下配置，如VCS无法访问，会在30秒内一直重试
            if (configValid)
            {
                // 可以进行参数检查，如果有需要自动追加的新参数，则可以追加后强制推送
                // bool forcePublicFlag = vm.configService().forcePublishConfig(configInfo, 30000); // 强制覆盖,建议不要强制覆盖，除非明确知道影响。
                break;
            }

            // 二、平台不存在配置，且本地无缓存，默认配置文件上传平台一份
            std::cout << "平台不存在配置，且本地无缓存，默认配置文件上传平台一份" << std::endl;
            bool defaultConfigFlag = configInfo.setContentWithFile(DefaultParamFilePath);
            if (!defaultConfigFlag)
            {
                std::cout << "加载默认配置失败，磁盘可能有问题" << std::endl;
                continue;
            }
            std::cout << "加载打包的默认配置成功，准备上传平台一份，方便后续维护" << std::endl;

            configInfo.setMaxBackupCount(-1); // -2：不修改现状，-1：无限备份，0：无备份，正数为具体备份个数
            bool publicFlag = vm.configService().publishConfig(configInfo, 30000);
            if (publicFlag)
            {
                std::cout << "默认配置上传平台成功" << std::endl;
                configValid = true;
                continue;
            }
            else
            {
                if (configInfo.state() == vcs::ConfigInfo::CS_ONLINE_OK)
                {
                    std::cout << "默认配置上传平台失败,已有相同配置,禁止覆盖" << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    continue; // 既然VCS存在了相同配置，那么等一下接着重新拉取配置
                }
                else
                {
                    std::cout << "默认配置上传平台失败,发生未知错误,重走获取流程" << std::endl;
                }
            }

            loop_rate.sleep();
            ros::spinOnce();
        }
    }

    if (configValid)
    {
        std::cout << "文件路径："
                  << configInfo.localFilePath() << std::endl;
        return configInfo.localFilePath();
    }
    else
    {
        std::cout << "文件路径："
                  << DefaultParamFilePath << std::endl;
        return DefaultParamFilePath;
    }
}

// 加载单个参数条目
template <typename T>
void ParamServer::importItem(YAML::Node &config, std::string FirstName, std::string LastName, T &TargetParam, T DefaultVal)
{
    std::stringstream buffer; // 创建一个字符串流
    if (config[FirstName][LastName].IsDefined())
    {
        TargetParam = config[FirstName][LastName].as<T>();
        buffer << "[已读取] ";
    }
    else
    {
        TargetParam = DefaultVal;
        buffer << "[默认值] ";
    }
    buffer << LastName + ": " << TargetParam << std::endl;
    std::cout << buffer.str();
    logData += buffer.str();
}
