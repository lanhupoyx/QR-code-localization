#include "Mode.hpp"

// 构造函数
Mode_CalYawErr::Mode_CalYawErr(ParamServer &param, MV_SC2005AM *camera) : QRcodeLoc(param, camera)
{
    logger->info("Mode_CalYawErr() start");

    // 实例化功能对象
    qrcode_table = new QRcodeTableV2(param.cfg_dir, trans_camera2base, param);
    wheel_odom = new WheelSpeedOdometer(trans_camera2base, param);

    logger->info("Mode_CalYawErr() return");
}

Mode_CalYawErr::~Mode_CalYawErr() {}

// 补偿地码角度模式
void Mode_CalYawErr::loop()
{
    logger->info("Mode_CalYawErr::loop()");

    // 关闭logger中的yawerr.txt文件
    logger->close_yawerr();

    // 打开yawerr.txt
    std::string yaw_err_path = param.log_dir + "yawerr.txt";
    std::ifstream ifs;
    ifs.open(yaw_err_path, std::ios::in);
    if (!ifs.is_open())
    {
        logger->info(yaw_err_path + "打开失败!");
    }
    else
    {
        logger->info(yaw_err_path + "打开成功!");
    }

    // 读取yawerr.txt
    std::map<uint32_t, err_val> yawerr_database;
    std::string buf;
    while (std::getline(ifs, buf))
    {
        buf = replaceChar(buf, ',', ' '); // ','替换为' '
        std::stringstream line_ss;
        line_ss << buf;

        // 跳过空行
        if (("" == buf) || (' ' == buf[0]))
        {
            continue;
        }

        logger->info(buf);

        // 定义变量
        std::string time;
        uint32_t index;
        double vel_x, curYaw, refYaw;

        // 提取信息
        line_ss >> time >> index >> vel_x >> curYaw >> refYaw;

        // 前进时车头摆动幅度小
        if (vel_x < 0.0)
            continue;

        // 数据存入数据库
        std::map<uint32_t, err_val>::iterator it = yawerr_database.find(index);
        if (it != yawerr_database.end())
        {
            it->second.err_ = (it->second.err_ * it->second.num_ + (curYaw - refYaw)) / (it->second.num_ + 1.0);
            it->second.num_++;
        }
        else
        {
            err_val new_err(curYaw - refYaw, 1);
            yawerr_database.insert(std::pair<uint32_t, err_val>(index, new_err));
        }
    }
    ifs.close();
    logger->info(yaw_err_path + "读取完毕!");

    // 打开初始库位信息文件
    // std::string site_info_path = param.cfg_dir + "SiteTable.txt";
    std::string site_info_path = param.siteTablePath;
    ifs.open(site_info_path, std::ios::in);
    if (!ifs.is_open())
    {
        logger->info(site_info_path + "打开失败!");
    }
    else
    {
        logger->info(site_info_path + "打开成功!");
    }

    // 打开输出库位信息文件
    std::string output_path = param.cfg_dir + "SiteTable_output.txt";
    std::ofstream ofs;
    ofs.open(output_path, std::ios::out);
    if (!ofs.is_open())
    {
        logger->info(output_path + "打开失败!");
        return;
    }
    else
    {
        logger->info(output_path + "打开成功!");
    }

    // 读取库位及其绑定的二维码信息
    while (std::getline(ifs, buf))
    {
        std::string new_buf = replaceChar(buf, ',', ' '); // ','替换为' '
        std::stringstream line_ss;
        line_ss << new_buf;

        // 跳过空行
        if (("" == new_buf) || (' ' == new_buf[0]))
        {
            ofs << buf << std::endl;
            continue;
        }

        // 定义变量
        uint32_t list_index, site_index, code_index, index_;
        double x_err, y_err, yaw_err;

        // 提取信息
        line_ss >> list_index >> site_index >> code_index;

        // 忽略库位位姿信息
        if ((0 == site_index) && (0 == code_index))
        {
            ofs << buf << std::endl;
            continue;
        }

        // 提取信息
        line_ss >> index_ >> x_err >> y_err >> yaw_err;

        // 已经补偿过的跳过
        if (0.0 != yaw_err)
        {
            ofs << buf << std::endl;
            continue;
        }

        // 查找数据库
        std::map<uint32_t, err_val>::iterator it = yawerr_database.find(index_);
        if (it != yawerr_database.end())
        {
            std::string outstring = buf.substr(0, buf.size() - 1);
            outstring = outstring.substr(0, outstring.find_last_of(',') + 1);
            ofs << outstring << it->second.err_ << std::endl;
        }
        else
        {
            ofs << buf << std::endl;
        }
    }

    ifs.close();
    logger->info(site_info_path + "读取完毕!");

    ofs.close();
    logger->info(output_path + "写入完毕!");
}
