#pragma once

#include "utility_qloc.hpp"

// 数据分割
class err : public ParamServer
{
private:
    std::mutex mtx;

    Logger *logger;
    std::stringstream stream;

    std::ofstream data_ofs;

public:
    err(std::string path)
    {
        // 读取文本文件中库位相关信息
        logger = &Logger::getInstance();
        logger->info("Err Start");

        std::string csv_path = path + "y_err.txt";
        data_ofs.open(csv_path, std::ios::app);

        logger->info(csv_path + "打开完毕!");
    }

    ~err(){}

    void add(ros::Time time, uint32_t index, double vel_x, double start, double end, double camera_error_x, double camera_error_y)
    {
        std::lock_guard<std::mutex> locker(mtx);
        double des_err = end - start;
        data_ofs <<  format_time(time) 
                + ' ' + std::to_string(index)
                + ' ' + std::to_string(vel_x)
                + ' ' + std::to_string(des_err)
                + ' ' + std::to_string(camera_error_x)
                + ' ' + std::to_string(camera_error_y)
                << std::endl;
        logger->debug("err add");
    }

    void add_string(std::string buf)
    {
        std::lock_guard<std::mutex> locker(mtx);
        data_ofs <<  format_time(ros::Time::now()) + ' ' + buf << std::endl;
        logger->debug("err add_string");
    }
};
