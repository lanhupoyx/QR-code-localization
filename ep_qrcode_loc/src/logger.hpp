#pragma once
#include "utility_qloc.hpp"

using namespace vcs;

class ParamServer;

namespace fs = boost::filesystem;
// 记录服务器
class Logger
{
private:
    static std::ofstream logFile_;
    static std::mutex mutex;

    static std::ofstream poseFile_;
    static std::mutex mutex_pose;

    static std::ofstream otherFile_;
    static std::mutex mutex_other;

    static std::ofstream yawerrFile_;
    static std::mutex mutex_yawerr;

    static std::ofstream jumperrFile_;
    static std::mutex mutex_jumperr;

    std::string loglevel_;
    std::string log_dir_;
    ParamServer *param_;

    // 私有构造函数确保不能直接创建Logger实例
    Logger() {}

    // 防止拷贝构造和赋值
    Logger(const Logger &) = delete;
    Logger &operator=(const Logger &) = delete;

public:
    // 用于初始化
    void init(ParamServer *param);

    // 获取单例对象
    static Logger &getInstance();

    // 记录日志的方法
    void log(const std::string &message);
    void fatal(const std::string &message);
    void error(const std::string &message);
    void warn(const std::string &message);
    void info(const std::string &message);
    void debug_endl();
    void debug(const std::string &message);
    void pose(const std::string &message);
    void other(const std::string &message);
    void yawerr(const std::string &message);
    void close_yawerr();
    void jumperr(const std::string &message);
    // 滚动删除历史文件夹
    void roll_delete_old_folders(size_t keep_count);
};

