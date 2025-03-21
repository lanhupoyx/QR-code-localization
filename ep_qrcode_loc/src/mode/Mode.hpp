#pragma once

#include "utilityQloc.hpp"
#include "ParamServer.hpp"
#include "logger.hpp"
#include "QRCodeLoc.hpp"

/// @brief 辅助驾驶三向车模式，巷道内使用
class Mode_AssistedDriving : public QRcodeLoc
{
public:
    // 构造函数
    Mode_AssistedDriving(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_AssistedDriving();

    // 循环
    void loop();
};

/// @brief 通过lidar获取地码方向角
class Mode_GetYaw : public QRcodeLoc
{
public:
    // 构造函数
    Mode_GetYaw(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_GetYaw();

    // 循环
    void loop();
};

/// @brief 计算地码方向角偏差
class Mode_CalYawErr : public QRcodeLoc
{
public:
    // 构造函数
    Mode_CalYawErr(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_CalYawErr();

    // 循环
    void loop();
};

/// @brief 检查相机水平模式
class Mode_CheckCameraHorizon : public QRcodeLoc
{
public:
    // 构造函数
    Mode_CheckCameraHorizon(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_CheckCameraHorizon();

    // 循环
    void loop();
};

/// @brief 获取地码编号模式
class Mode_CollectQRCodeIndex : public QRcodeLoc
{
public:
    // 构造函数
    Mode_CollectQRCodeIndex(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_CollectQRCodeIndex();

    // 循环
    void loop();
};

/// @brief 获取地码pose模式
class Mode_CollectQRCodePose : public QRcodeLoc
{
public:
    // 构造函数
    Mode_CollectQRCodePose(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_CollectQRCodePose();

    // 循环
    void loop();
};

/// @brief 北区二维码定位改造，地码间距固定
class Mode_North : public QRcodeLoc
{
private:
    QRcodeTableV3 *qrcode_table_v3;

    QRcodeInfo code_info;     // 存放查询到的地码信息
    uint8_t err_type;         // 16进制数据，存放故障类型
    bool is_output_available; // 记录是否可以输出数据

    std::list<std::vector<nav_msgs::Odometry>> publist; // 最终数据发送队列

public:
    /// @brief 构造函数
    /// @param param 参数服务对象
    /// @param camera 相机对象
    Mode_North(ParamServer &param, MV_SC2005AM *camera);

    /// @brief 析构函数
    ~Mode_North();

    /// @brief 工作循环
    void loop();

    /// @brief 相机处理流程
    /// @param v_pose_new 有相机得到的最新pose
    void cameraFrameProcess(std::vector<geometry_msgs::Pose> &v_pose_new);

    /// @brief 轮速计第推处理流程
    void wheelOdomProcess();

    /// @brief 结果发布处理流程
    /// @param output 待发布数据
    void publishProcess(std::vector<nav_msgs::Odometry> &output);
};

/// @brief 石花项目，地码间距与库位间距、动作点位置等紧密相关
class Mode_ShiHua : public QRcodeLoc
{
public:
    // 构造函数
    Mode_ShiHua(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_ShiHua();

    // 循环
    void loop();
};

/// @brief 测试模式
class Mode_TestRun : public QRcodeLoc
{
public:
    // 构造函数
    Mode_TestRun(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_TestRun();

    // 循环
    void loop();
};

/// @brief 废弃模式
class Mode_Trash : public QRcodeLoc
{
public:
    // 构造函数
    Mode_Trash(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_Trash();

    // 循环
    void loop();
};
