#pragma once

#include <list>
#include <string>

#include <thread/lock/mutex_lock.h>

#include "config/config_info.h"

namespace vcs {
/// 配置变更监听器基类，需业务人员继承此类实现configChanged
class ConfigListener {
public:
    /// 配置变更类型
    enum ConfigOpType {
        COPT_NULL,
        COPT_ADD,
        COPT_DELETE,
        COPT_UPDATE,
        COPT_RESTART,
        COPT_CUSTOM_OPERATE
    };

    /// 配置变更类型字符串转为枚举
    static ConfigListener::ConfigOpType getOpTypeByStr(const std::string &opType);

public:
    ConfigListener();
    virtual ~ConfigListener();

    /// 获取监听的配置列表
    std::list<ConfigInfo> listenConfigList();

    /// 本地现有配置的MD5，设置后在本地配置与服务器配置不一致时也能监听到变化事件；也可以设置为空，则只能收到注册后的变更事件。
    void addListenConfig(const std::string &contentMd5, const std::string &dataId, const std::string &groupId = "", const std::string &tenantId = "");

    // 移除配置监听
    void removeListenConfig(const std::string &dataId, const std::string &groupId = "", const std::string &tenantId = "");

    /// 注意本地通过setMd5()设置现有配置的MD5，设置后在本地配置与服务器配置不一致时也能监听到变化事件；也可以设置为空，则只能收到注册后的变更事件。
    void addListenConfig(ConfigInfo &configListenInfo);

    // 移除配置监听
    void removeListenConfig(ConfigInfo &configListenInfo);

    /// 检查是否是自己监听的配置，如是则触发configChanged()
    bool handleConfigChangedEvent(ConfigInfo &config, ConfigListener::ConfigOpType opType);

protected:
    /// 配置变化后的回调函数
    virtual void configChanged(const ConfigInfo &config, ConfigListener::ConfigOpType opType);

protected:
    cppc::MutexLock m_lock;
    /// 监听的配置信息
    std::list<ConfigInfo> m_listenConfigList;
};
} // namespace vcs
