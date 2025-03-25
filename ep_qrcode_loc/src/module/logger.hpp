#pragma once
#include "utilityQloc.hpp"

#include <iostream>
#include <array>
#include <memory>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/wait.h>

// 记录服务器
class epLogger
{
private:
    std::ofstream logFile_;
    std::mutex mutex;
    std::ofstream poseFile_;
    std::mutex mutex_pose;
    std::ofstream otherFile_;
    std::mutex mutex_other;
    std::ofstream yawerrFile_;
    std::mutex mutex_yawerr;
    std::ofstream jumperrFile_;
    std::mutex mutex_jumperr;

    std::string logLevel_;
    std::string log_dir_;
    std::size_t logKeepDays_;

    std::string log_dir_today_;

    // 私有构造函数确保不能直接创建Logger实例
    epLogger() {}

    // 防止拷贝构造和赋值
    epLogger(const epLogger &) = delete;
    epLogger &operator=(const epLogger &) = delete;

public:
    // 用于初始化
    void init(std::string logLevel, std::string log_dir, std::size_t logKeepDays);

    // 获取单例对象
    static epLogger &getInstance();

    // 记录日志的方法
    void log(const std::string &message);
    void fatal(const std::string &message);
    void error(const std::string &message);
    void warn(const std::string &message);
    void info(const std::string &message);
    void debug(const std::string &message);
    void debug_endl();

    // 记录数据的方法
    void pose(const std::string &message);
    void other(const std::string &message);
    void yawerr(const std::string &message);
    void close_yawerr();
    void jumperr(const std::string &message);

    // 滚动删除历史文件夹
    void roll_delete_old_folders();

    // 创建日期文件夹
    bool createDateFolder(std::string logdir);

    // 打开文件
    bool openFile(std::ofstream &file, std::mutex &mtx, std::string path);

    // 监测循环
    void logLoop();

    void saveBasicInfo();
    std::string exec(const char *cmd);
};
