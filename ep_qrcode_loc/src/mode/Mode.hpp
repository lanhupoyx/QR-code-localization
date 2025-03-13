#pragma once

#include "utility_qloc.hpp"
#include "QRCodeLoc.hpp"

class Mode_AssistedDriving : public QRcodeLoc
{
public:
    // 构造函数
    Mode_AssistedDriving(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_AssistedDriving();

    // 循环
    void loop();
};

class Mode_CalYawErr : public QRcodeLoc
{
public:
    // 构造函数
    Mode_CalYawErr(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_CalYawErr();

    // 循环
    void loop();
};

class Mode_CheckCameraHorizon : public QRcodeLoc
{
public:
    // 构造函数
    Mode_CheckCameraHorizon(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_CheckCameraHorizon();

    // 循环
    void loop();
};

class Mode_CollectQRCodeIndex : public QRcodeLoc
{
public:
    // 构造函数
    Mode_CollectQRCodeIndex(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_CollectQRCodeIndex();

    // 循环
    void loop();
};

class Mode_CollectQRCodePose : public QRcodeLoc
{
public:
    // 构造函数
    Mode_CollectQRCodePose(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_CollectQRCodePose();

    // 循环
    void loop();
};

class Mode_GetYaw : public QRcodeLoc
{
public:
    // 构造函数
    Mode_GetYaw(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_GetYaw();

    // 循环
    void loop();
};

// 北区二维码定位改造
class Mode_North : public QRcodeLoc
{
public:
    // 构造函数
    Mode_North(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_North();

    // 循环
    void loop();
};

class Mode_ShiHua : public QRcodeLoc
{
public:
    // 构造函数
    Mode_ShiHua(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_ShiHua();

    // 循环
    void loop();
};

class Mode_TestRun : public QRcodeLoc
{
public:
    // 构造函数
    Mode_TestRun(ParamServer &param, MV_SC2005AM *camera);
    ~Mode_TestRun();

    // 循环
    void loop();
};
