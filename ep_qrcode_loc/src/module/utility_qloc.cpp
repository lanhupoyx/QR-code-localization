#include "utility_qloc.hpp"

using namespace vcs;

namespace fs = boost::filesystem;

// 数据帧转换到16进制
std::string charArrayToHex(std::array<char, 1024> array, size_t size)
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < size; ++i)
    {
        ss << std::setw(2) << static_cast<int>(static_cast<unsigned char>(array[i]));
    }
    return ss.str();
}

// 十六进制转换为十进制
uint32_t convert_16_to_10(std::string str2)
{

    double sum = 0, times;
    double m;
    std::string::size_type sz = str2.size();
    for (std::string::size_type index = 0; index != sz; ++index)
    {
        // 变为小写，这个思路很好
        str2[index] = tolower(str2[index]);
        if (str2[index] >= 'a' && str2[index] <= 'f')
        {
            // 这里让a~f进行转换为数字字符，很奇妙
            m = str2[index] - 'a' + 10;
            // 求幂次方
            times = pow(16, (sz - 1 - index));
            sum += m * times;
        }
        else if (isdigit(str2[index]))
        {
            // 需要将字符类型转换为数字类型
            // 因为0的ASCII码是48，所以转换为相应的数字，减去48即可
            m = str2[index] - 48;
            times = pow(16, (sz - 1 - index));
            sum += m * times;
        }
        else
        {
            std::cout << "无法识别的十六进制!";
            break;
        }
    }
    return uint32_t(sum);
}

// 十六进制转换为ASCII
std::string hexToAscii(const std::string &hex)
{
    std::string ascii;
    std::stringstream ss;

    // 将16进制字符串转换为整数
    for (size_t i = 0; i < hex.length(); i += 2)
    {
        std::string byte = hex.substr(i, 2);
        unsigned int value = 0;
        ss << std::hex << byte;
        ss >> value;
        ss.clear();

        // 将整数转换为对应的ASCII字符
        ascii += static_cast<char>(value);
    }

    return ascii;
}

// 时间格式化输出
std::string format_time(ros::Time t)
{
    std::stringstream ss;
    ros::WallTime wall_time = ros::WallTime(t.toSec() + 28800.0); // 移动8个时区
    ss << wall_time.toBoost().date().year() << '-';
    ss << wall_time.toBoost().date().month() << '-';
    ss << wall_time.toBoost().date().day() << '_';
    ss << wall_time.toBoost().time_of_day().hours() << ':';
    ss << wall_time.toBoost().time_of_day().minutes() << ':';
    int second = wall_time.toBoost().time_of_day().seconds();
    ss << std::fixed << std::setprecision(6) << (double(second) * 1e3 + double(t.nsec) / 1000000.0) / 1000.0; // 毫秒
    return ss.str();
}

// 日期格式化输出
std::string format_date(ros::Time t)
{
    std::stringstream ss;
    ros::WallTime wall_time = ros::WallTime(t.toSec() + 28800.0); // 移动8个时区
    ss << wall_time.toBoost().date().year() << '-';
    ss << wall_time.toBoost().date().month() << '-';
    ss << wall_time.toBoost().date().day();
    return ss.str();
}

// string 替换字符
std::string replaceChar(std::string str, char toReplace, char replacement)
{
    size_t start_pos = 0;
    while ((start_pos = str.find(toReplace, start_pos)) != std::string::npos)
    {
        str.replace(start_pos, 1, 1, replacement);
        ++start_pos;
    }
    return str;
}

// 删除末尾若干个字符
std::string del_n_end(std::string str, uint16_t n)
{
    if (str.size() <= n)
    {
        return str;
    }
    else
    {
        return str.substr(0, str.size() - n);
    }
}

// pose to transform
geometry_msgs::TransformStamped p2t(geometry_msgs::Pose pose)
{
    geometry_msgs::TransformStamped trans;
    trans.transform.translation.x = pose.position.x;
    trans.transform.translation.y = pose.position.y;
    trans.transform.translation.z = pose.position.z;
    trans.transform.rotation = pose.orientation;
    return trans;
}

// transform to pose
geometry_msgs::Pose t2p(geometry_msgs::TransformStamped trans)
{
    geometry_msgs::Pose pose;
    pose.position.x = trans.transform.translation.x;
    pose.position.y = trans.transform.translation.y;
    pose.position.z = trans.transform.translation.z;
    pose.orientation = trans.transform.rotation;
    return pose;
}

// get yaw frome pose
double getYaw(geometry_msgs::Pose pose)
{
    tf::Quaternion q(pose.orientation.x,
                     pose.orientation.y,
                     pose.orientation.z,
                     pose.orientation.w);      // 初始化四元数
    double roll = 0.0, pitch = 0.0, yaw = 0.0; // 初始化欧拉角
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw); // 四元数转欧拉角
    return yaw * 180 / M_PI;
}

// get yaw frome pose
double getYaw(geometry_msgs::Quaternion q)
{
    tf::Quaternion quaternion(q.x, q.y, q.z, q.w);      // 初始化四元数
    double roll = 0.0, pitch = 0.0, yaw = 0.0;          // 初始化欧拉角
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw); // 四元数转欧拉角
    return yaw * 180 / M_PI;
}

// get yaw frome TransformStamped
double getYaw(geometry_msgs::TransformStamped trans)
{
    tf::Quaternion q(trans.transform.rotation.x,
                     trans.transform.rotation.y,
                     trans.transform.rotation.z,
                     trans.transform.rotation.w); // 初始化四元数
    double roll = 0.0, pitch = 0.0, yaw = 0.0;    // 初始化欧拉角
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);    // 四元数转欧拉角
    return yaw * 180 / M_PI;
}

// get yaw(rad) frome pose
double getYawRad(geometry_msgs::Pose pose)
{
    tf::Quaternion quaternion(pose.orientation.x,
                              pose.orientation.y,
                              pose.orientation.z,
                              pose.orientation.w);      // 初始化四元数
    double roll = 0.0, pitch = 0.0, yaw = 0.0;          // 初始化欧拉角
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw); // 四元数转欧拉角
    return yaw;
}

// get yaw(rad) frome pose
double getYawRad(geometry_msgs::Quaternion q)
{
    tf::Quaternion quaternion(q.x, q.y, q.z, q.w);      // 初始化四元数
    double roll = 0.0, pitch = 0.0, yaw = 0.0;          // 初始化欧拉角
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw); // 四元数转欧拉角
    return yaw;
}

// get yaw frome pose
geometry_msgs::Pose poseInverse(geometry_msgs::Pose source)
{
    geometry_msgs::Pose result;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, -getYawRad(source.orientation));
    tf::quaternionTFToMsg(q, result.orientation);
    tf::Matrix3x3 m(q);
    tf::Matrix3x3 inv_m(m);
    tf::Vector3 v(source.position.x, source.position.y, 0.0);
    inv_m.inverse();                             // 求逆
    result.position.x = -inv_m.getRow(0).dot(v); //(row0(0)*v.getX() + inv_m(0,1)*v.getX() +inv_m(0,2)*v.getZ());
    result.position.y = -inv_m.getRow(1).dot(v); //-(inv_m(1,0)*v.getX() + inv_m(1,1)*v.getX() +inv_m(1,2)*v.getZ());
    result.position.z = 0.0;
    return result;
}

QRcodeInfo::QRcodeInfo(uint32_t code_, double x_, double y_, double yaw_, bool is_head_)
{
    code = code_;
    x = x_;
    y = y_;
    yaw = yaw_;
    is_head = is_head_;
    type = 0;
}

// 参数服务器v2

ParamServer::ParamServer(ros::NodeHandle &nh) : nh(nh)
{
    //获取vcs参数文件，或者默认参数文件路径
    mainParamPath = loadMainParamPath();
    siteTablePath = loadSitetableParamPath();

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
        if ("5" == operating_mode)
            read_yaw_err = false;
        importItem<double>(config, "ep_qrcode_loc", "err_ratio_offline", err_ratio_offline, 1.0);
        importItem<double>(config, "ep_qrcode_loc", "rec_p1", rec_p1, 0.0);
        importItem<double>(config, "ep_qrcode_loc", "avliable_yaw", avliable_yaw, 0.0);
        importItem<double>(config, "ep_qrcode_loc", "wheel_diameter", wheel_diameter, 0.0);
        importItem<double>(config, "ep_qrcode_loc", "wheel_reduction_ratio", wheel_reduction_ratio, 1.0);
        importItem<double>(config, "ep_qrcode_loc", "wheel_base_dis", wheel_base_dis, 0.0);
        importItem<double>(config, "ep_qrcode_loc", "wheel_angular_offset", wheel_angular_offset, 0.0);
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
        importItem<size_t>(config, "ep_qrcode_loc", "logKeepDays", logKeepDays, 90);
        importItem<bool>(config, "ep_qrcode_loc", "is_mainloop_query_camera", is_mainloop_query_camera, true);
    }
    catch (YAML::Exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

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

std::string ParamServer::loadSitetableParamPath()
{
    std::string DefaultParamFilePath = "/var/xmover/params/ep-qrcode-loc//SiteTable.txt";

    bool configValid = false;

    ConfigInfo configInfo;

    if (false) //是否使用VCS管理地码信息
    {
        DefaultParamFilePath = "/opt/xmover/ros/melodic/ep_qrcode_loc/share/ep_qrcode_loc/config/SiteTable.txt";

        configInfo.setDataId("ep_qrcode_sitetable");
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

// 保存log
void ParamServer::saveLog(Logger *logger)
{
    logger->info("\n" + yamlData); // 原始文本
    logger->info("\n" + logData);  // 读取记录
}

