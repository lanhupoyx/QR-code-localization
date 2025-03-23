#include "logger.hpp"

namespace fs = boost::filesystem;

// 获取单例对象
epLogger& epLogger::getInstance()
{
    static epLogger instance; // 懒汉式，在第一次调用时实例化
    return instance;
}

// 用于初始化
void epLogger::init(std::string logLevel, std::string log_dir, std::size_t logKeepDays)
{
    // 初始化参数
    logLevel_ = logLevel;
    log_dir_ = log_dir;
    logKeepDays_ = logKeepDays;

    // 根据日期创建文件夹
    createDateFolder(log_dir_);    
    
    // 创建日志文件
    openFile(logFile_, mutex, log_dir_today_ + "qrcode_log_" + format_time(ros::Time::now()) + ".txt"); // log文件

    // 创建数据文件
    openFile(poseFile_, mutex_pose, log_dir_today_ + "pose_" + format_time(ros::Time::now()) + ".txt"); // pose文件
    openFile(yawerrFile_, mutex_yawerr, log_dir_ + "yawerr.txt");        // yawerr文件
    openFile(jumperrFile_, mutex_jumperr, log_dir_today_ + "jumperr.txt"); // jumperr文件
}

// 记录日志的方法
void epLogger::log(const std::string &message)
{
    std::lock_guard<std::mutex> lock(mutex); // 线程安全
    logFile_ << message << std::endl;
}

void epLogger::fatal(const std::string &message)
{
    std::lock_guard<std::mutex> lock(mutex); // 线程安全
    logFile_ << format_time(ros::Time::now()) << "[FATAL]" << message << std::endl;
}

void epLogger::error(const std::string &message)
{
    std::lock_guard<std::mutex> lock(mutex); // 线程安全
    logFile_ << format_time(ros::Time::now()) << " [ERROR] " << message << std::endl;
}

void epLogger::warn(const std::string &message)
{
    std::lock_guard<std::mutex> lock(mutex); // 线程安全
    logFile_ << format_time(ros::Time::now()) << " [WARN] " << message << std::endl;
}

void epLogger::info(const std::string &message)
{
    std::lock_guard<std::mutex> lock(mutex); // 线程安全
    logFile_ << format_time(ros::Time::now()) << " [INFO] " << message << std::endl;
}

void epLogger::debug(const std::string &message)
{
    if ("DEBUG" == logLevel_)
    {
        std::lock_guard<std::mutex> lock(mutex); // 线程安全
        logFile_ << format_time(ros::Time::now()) << " [DEBUG] " << message << std::endl;
    }
}

void epLogger::debug_endl()
{
    if ("DEBUG" == logLevel_)
    {
        std::lock_guard<std::mutex> lock(mutex); // 线程安全
        logFile_ << std::endl;
    }
}

// 记录数据的方法
void epLogger::pose(const std::string &message)
{
    std::lock_guard<std::mutex> lock(mutex_pose); // 线程安全
    poseFile_ << del_n_end(format_time(ros::Time::now()), 3) << "," << message << std::endl;
}

void epLogger::other(const std::string &message)
{
    std::lock_guard<std::mutex> lock(mutex_other); // 线程安全
    otherFile_ << format_time(ros::Time::now()) << " " << message << std::endl;
}

void epLogger::yawerr(const std::string &message)
{
    std::lock_guard<std::mutex> lock(mutex_yawerr); // 线程安全
    yawerrFile_ << format_time(ros::Time::now()) << " " << message << std::endl;
}

void epLogger::close_yawerr()
{
    yawerrFile_.close();
}

void epLogger::jumperr(const std::string &message)
{
    std::lock_guard<std::mutex> lock(mutex_jumperr); // 线程安全
    jumperrFile_ << format_time(ros::Time::now()) << " " << message << std::endl;
}

// 滚动删除历史文件夹
void epLogger::roll_delete_old_folders()
{
    fs::path dir_path(log_dir_);

    std::vector<fs::directory_entry> directories;
    for (const auto &entry : fs::directory_iterator(dir_path))
    {
        if (fs::is_directory(entry))
        {
            directories.push_back(entry);
        }
    }

    if (directories.size() <= logKeepDays_)
    {
        this->info("No old log folders to delete. Total folders: " + std::to_string(directories.size()));
        return;
    }

    std::sort(directories.begin(), directories.end(), [](const fs::directory_entry &a, const fs::directory_entry &b)
              { return fs::last_write_time(a.path()) < fs::last_write_time(b.path()); });

    for (std::size_t i = 0; i < directories.size() - logKeepDays_; ++i)
    {
        fs::remove_all(directories[i].path());
        this->info("Deleted: " + directories[i].path().string());
    }
}

bool epLogger::createDateFolder(std::string logdir)
{
    // 创建文件夹
    log_dir_today_ = logdir + format_date(ros::Time::now()) + "/";
    fs::path fsdir(log_dir_today_);
    if (fs::create_directory(fsdir))
    {
        std::cout << log_dir_today_ + " created successfully." << std::endl;
        return true;
    }
    else
    {
        std::cout << log_dir_today_ + " already exists, cannot be created." << std::endl;
        return false;
    }
}

bool epLogger::openFile(std::ofstream &file, std::mutex &mtx, std::string path)
{
    std::lock_guard<std::mutex> lock(mtx); // 线程安全
    if (file.is_open())
    {
        file.close();
    }

    file.open(path, std::ios::app);
    if (!file.is_open())
    {
        std::cout << "can't open: " << path << std::endl;
        return false;
    }
    else
    {
        std::cout << "open: " << path << std::endl;
        return true;
    }
}

void epLogger::logLoop()
{
    info("epLogger::logLoop() start");

    double loop_rate = 1.0; // 控制主循环频率1Hz
    ros::Rate loop_rate_ctrl(loop_rate);
    while (ros::ok())
    {
        debug("epLogger::logLoop() next loop");

        // 随日期的文件
        std::string dayNum = getDay(ros::Time::now()); // 日期数
        static std::string dayNumLast = dayNum;
        if (dayNumLast != dayNum) // 检查是否更新日期
        {
            createDateFolder(log_dir_); // 根据日期创建文件夹
            roll_delete_old_folders();  // 删除过早log

            openFile(jumperrFile_, mutex_jumperr, log_dir_today_ + "jumperr.txt"); // jumperr文件
            info("reopen jumperr.txt in :" + log_dir_today_ + "jumperr.txt");
        }
        dayNumLast = dayNum;

        // 随小时的文件
        std::string hourNum = getHour(ros::Time::now()); // 小时数
        static std::string hourNumLast = hourNum;
        if (hourNumLast != hourNum) // 检查是否更新小时
        {
            openFile(poseFile_, mutex_pose, log_dir_today_ + "pose_" + format_time(ros::Time::now()) + ".txt"); // pose文件
            info("reopen pose.txt in :" + log_dir_today_ + "pose.txt");

            std::string logPath = log_dir_today_ + "qrcode_log_" + format_time(ros::Time::now()) + ".txt";
            info("is reopening qrcode_log.txt in :" + logPath);
            openFile(logFile_, mutex, logPath); // log文件
        }
        hourNumLast = hourNum;

        // 频率控制延时
        loop_rate_ctrl.sleep();
        ros::spinOnce();
    }
}
