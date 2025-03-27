#include "utilityQloc.hpp"

// 数据帧转换到16进制
std::string charArrayToHex(std::array<char, 1024> array, std::size_t size)
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < size; ++i)
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
    for (std::size_t i = 0; i < hex.length(); i += 2)
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

// 时间格式化输出
std::string format_time_sec(ros::Time t)
{
    std::stringstream ss;
    ros::WallTime wall_time = ros::WallTime(t.toSec() + 28800.0); // 移动8个时区
    ss << wall_time.toBoost().date().year() << '-';
    ss << wall_time.toBoost().date().month() << '-';
    ss << wall_time.toBoost().date().day() << '_';
    ss << wall_time.toBoost().time_of_day().hours() << ':';
    ss << wall_time.toBoost().time_of_day().minutes() << ':';
    ss << wall_time.toBoost().time_of_day().seconds();
    return ss.str();
}

// 小时数输出
std::string getHour(ros::Time t)
{
    std::stringstream ss;
    ros::WallTime wall_time = ros::WallTime(t.toSec() + 28800.0); // 移动8个时区
    ss << wall_time.toBoost().time_of_day().hours();
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

// 日期数输出
std::string getDay(ros::Time t)
{
    std::stringstream ss;
    ros::WallTime wall_time = ros::WallTime(t.toSec() + 28800.0); // 移动8个时区
    ss << wall_time.toBoost().date().day();
    return ss.str();
}

// string 替换字符
std::string replaceChar(std::string str, char toReplace, char replacement)
{
    std::size_t start_pos = 0;
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
    return yaw * 180.0 / M_PI;
}

// get yaw frome pose
double getYaw(geometry_msgs::Quaternion q)
{
    tf::Quaternion quaternion(q.x, q.y, q.z, q.w);      // 初始化四元数
    double roll = 0.0, pitch = 0.0, yaw = 0.0;          // 初始化欧拉角
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw); // 四元数转欧拉角
    return yaw * 180.0 / M_PI;
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
    return yaw * 180.0 / M_PI;
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

/// @brief 得到pose，yaw单位deg
/// @param x 
/// @param y 
/// @param yaw 
/// @return pose
geometry_msgs::Pose toPoseDeg(double x, double y, double yawDeg)
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yawDeg * M_PI / 180.0);
    tf::quaternionTFToMsg(q, pose.orientation);
    return pose;
}

/// @brief 得到pose，yaw单位rad
/// @param x 
/// @param y 
/// @param yaw 
/// @return pose
geometry_msgs::Pose toPoseRad(double x, double y, double yawRad)
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yawRad);
    tf::quaternionTFToMsg(q, pose.orientation);
    return pose;
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

// 函数：执行系统命令并捕获输出
std::string exec(std::string scmd)
{
    const char *cmd = scmd.c_str();
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);

    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }

    return result;
}

// 构造函数
QRcodeInfo::QRcodeInfo(uint32_t code_, double x_, double y_, double yaw_, bool is_head_)
{
    code = code_;
    x = x_;
    y = y_;
    yaw = yaw_;
    is_head = is_head_;
    type = 0;
}

// 转动角度
void QRcodeGround::turn(double d_yaw)
{
    double yaw = getYawRad(pose_.orientation);
    // yaw += d_yaw;
    yaw += d_yaw * M_PI / 180.0;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(q, pose_.orientation);
}

