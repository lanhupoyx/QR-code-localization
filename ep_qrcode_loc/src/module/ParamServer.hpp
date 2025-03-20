#pragma once
#include "utility_qloc.hpp"

// 参数服务器
class ParamServer
{
public:
    ros::NodeHandle nh;
    std::string yamlData;
    std::string logData;

    std::string logLevel;

    std::string odomMapBase;
    std::string odomMapCamera;
    std::string pathMapBase;
    std::string pathMapCamera;
    std::string msgTopic;

    std::string operating_mode;
    bool show_original_msg;
    bool is_pub_tf;
    double low_speed_UL;
    std::string port;
    std::string log_dir;
    std::string cfg_dir;
    float maxEstimationDis;

    bool read_yaw_err;

    double yaw_jump_UL;
    double x_jump_UL;
    double y_jump_UL;

    double rec_p1;
    double wheel_diameter;
    double wheel_reduction_ratio;
    double wheel_base_dis;
    double wheel_angular_forward;
    double wheel_angular_backward;

    double err_ratio_offline;

    double detect_site_dis;
    double aux_site_dis;
    double forkaction_site_dis;
    double site_site_dis;

    double avliable_yaw;

    bool is_debug;
    bool check_sequence;
    bool cal_yaw;
    double ground_code_yaw_offset;

    std::string mainParamPath;
    std::string siteTablePath;
    std::string GroundCodeTablePath;

    std::size_t logKeepDays;

    bool is_mainloop_query_camera;

    geometry_msgs::TransformStamped trans_base2camera;
    geometry_msgs::TransformStamped trans_camera2base;

    ParamServer(ros::NodeHandle &nh);

    std::string loadMainParamPath();
    std::string loadSiteTableParamPath();
    std::string loadGroundCodeTableParamPath();
    
    // 加载单个参数条目
    template <typename T>
    void importItem(YAML::Node &config, std::string FirstName, std::string LastName, T &TargetParam, T DefaultVal);

};
