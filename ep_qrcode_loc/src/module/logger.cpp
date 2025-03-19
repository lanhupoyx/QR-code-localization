#include "logger.hpp"

std::ofstream epLogger::logFile_;
std::mutex epLogger::mutex;
std::ofstream epLogger::poseFile_;
std::mutex epLogger::mutex_pose;
std::ofstream epLogger::otherFile_;
std::mutex epLogger::mutex_other;
std::ofstream epLogger::yawerrFile_;
std::mutex epLogger::mutex_yawerr;
std::ofstream epLogger::jumperrFile_;
std::mutex epLogger::mutex_jumperr;

// 用于初始化
void epLogger::init(ParamServer *param)
{
    // 初始化参数
    param_ = param;
    log_dir_ = param_->log_dir;
    loglevel_ = param_->logLevel;

    // 创建文件夹
    std::string log_dir_today = log_dir_ + format_date(ros::Time::now()) + "/";
    fs::path dir(log_dir_today);
    if (fs::create_directory(dir))
    {
        std::cout << log_dir_today + " created successfully." << std::endl;
    }
    else
    {
        std::cout << log_dir_today + " already exists, cannot be created." << std::endl;
    }

    // log文件
    std::string log_path = log_dir_today + format_time(ros::Time::now()) + "_qrcode_log.txt";
    logFile_.open(log_path, std::ios::app);
    if (!logFile_.is_open())
    {
        std::cout << "can't open: " << log_path << std::endl;
    }
    else
    {
        std::cout << "open: " << log_path << std::endl;
    }

    // pose文件
    std::string pose_path = log_dir_today + "pose.txt";
    poseFile_.open(pose_path, std::ios::app);
    if (!poseFile_.is_open())
    {
        std::cout << "can't open: " << pose_path << std::endl;
    }
    else
    {
        std::cout << "open: " << pose_path << std::endl;
    }

    // other文件
    std::string other_path = log_dir_today + "other.csv";
    otherFile_.open(other_path, std::ios::app);
    if (!otherFile_.is_open())
    {
        std::cout << "can't open: " << other_path << std::endl;
    }
    else
    {
        std::cout << "open: " << other_path << std::endl;
    }

    // yawerr文件
    std::string yawerr_path = log_dir_ + "yawerr.txt";
    yawerrFile_.open(yawerr_path, std::ios::app);
    if (!yawerrFile_.is_open())
    {
        std::cout << "can't open: " << yawerr_path << std::endl;
    }
    else
    {
        std::cout << "open: " << yawerr_path << std::endl;
    }

    // jumperr文件
    std::string jumperr_path = log_dir_today + "jumperr.txt";
    jumperrFile_.open(jumperr_path, std::ios::app);
    if (!jumperrFile_.is_open())
    {
        std::cout << "can't open: " << jumperr_path << std::endl;
    }
    else
    {
        std::cout << "open: " << jumperr_path << std::endl;
    }
}

// 获取单例对象
epLogger& epLogger::getInstance()
{
    static epLogger instance; // 懒汉式，在第一次调用时实例化
    return instance;
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

void epLogger::debug_endl()
{
    if ("DEBUG" == loglevel_)
    {
        std::lock_guard<std::mutex> lock(mutex); // 线程安全
        logFile_ << std::endl;
    }
}

void epLogger::debug(const std::string &message)
{
    if ("DEBUG" == loglevel_)
    {
        std::lock_guard<std::mutex> lock(mutex); // 线程安全
        logFile_ << format_time(ros::Time::now()) << " [DEBUG] " << message << std::endl;
    }
}

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
void epLogger::roll_delete_old_folders(size_t keep_count)
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

    if (directories.size() <= keep_count)
    {
        this->info("No old log folders to delete. Total folders: " + std::to_string(directories.size()));
        return;
    }

    std::sort(directories.begin(), directories.end(), [](const fs::directory_entry &a, const fs::directory_entry &b)
              { return fs::last_write_time(a.path()) < fs::last_write_time(b.path()); });

    for (size_t i = 0; i < directories.size() - keep_count; ++i)
    {
        fs::remove_all(directories[i].path());
        this->info("Deleted: " + directories[i].path().string());
    }
}


