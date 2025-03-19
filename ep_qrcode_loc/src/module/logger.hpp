#pragma once
#include "utility_qloc.hpp"

using namespace vcs;

class ParamServer;

namespace fs = boost::filesystem;
// 记录服务器
class epLogger
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
    epLogger() {}

    // 防止拷贝构造和赋值
    epLogger(const epLogger &) = delete;
    epLogger &operator=(const epLogger &) = delete;

public:
    // 用于初始化
    void init(ParamServer *param);

    // 获取单例对象
    static epLogger &getInstance();

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

