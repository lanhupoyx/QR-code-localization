#include "config_listener.h"

#include <log/log.h>
#include <thread/lock/auto_locker.h>

namespace vcs {
ConfigListener::ConfigOpType ConfigListener::getOpTypeByStr(const std::string &opType) {
    if (opType == "U") {
        return COPT_UPDATE;
    } else if (opType == "A") {
        return COPT_ADD;
    } else if (opType == "D") {
        return COPT_DELETE;
    } else if (opType == "RESTART") {
        return COPT_RESTART;
    } else {
        return COPT_CUSTOM_OPERATE;
    }
}

ConfigListener::ConfigListener() {
}

ConfigListener::~ConfigListener() {
}

std::list<ConfigInfo> ConfigListener::listenConfigList() {
    cppc::AutoLocker locker(m_lock);
    return m_listenConfigList;
}

void ConfigListener::addListenConfig(const std::string &contentMd5, const std::string &dataId, const std::string &groupId, const std::string &tenantId) {
    ConfigInfo configListenInfo;
    configListenInfo.setDataId(dataId);
    configListenInfo.setGroupId(groupId);
    configListenInfo.setTenantId(tenantId);
    configListenInfo.setMd5(contentMd5);
    addListenConfig(configListenInfo);
}

void ConfigListener::removeListenConfig(const std::string &dataId, const std::string &groupId, const std::string &tenantId) {
    ConfigInfo configListenInfo;
    configListenInfo.setDataId(dataId);
    configListenInfo.setGroupId(groupId);
    configListenInfo.setTenantId(tenantId);

    removeListenConfig(configListenInfo);
}

void ConfigListener::addListenConfig(ConfigInfo &configListenInfo) {
    cppc::AutoLocker locker(m_lock);
    configListenInfo.checkAndSetDefault();
    if (configListenInfo.dataId().size() <= 0) {
        return;
    }

    std::string configTopic = configListenInfo.topic();
    for (auto it = m_listenConfigList.begin(); it != m_listenConfigList.end();) {
        ConfigInfo &config = *it;
        if (configTopic == config.topic()) {
            return; // 已经添加过了
        }
        ++it;
    }
    m_listenConfigList.push_back(configListenInfo);
}

void ConfigListener::removeListenConfig(ConfigInfo &configListenInfo) {
    cppc::AutoLocker locker(m_lock);
    configListenInfo.checkAndSetDefault();
    if (configListenInfo.dataId().size() <= 0) {
        return;
    }

    std::string configTopic = configListenInfo.topic();
    for (auto it = m_listenConfigList.begin(); it != m_listenConfigList.end();) {
        ConfigInfo &config = *it;
        if (configTopic == config.topic()) {
            it = m_listenConfigList.erase(it);
            break;
        }
        ++it;
    }
}
bool ConfigListener::handleConfigChangedEvent(ConfigInfo &config, ConfigListener::ConfigOpType opType) {
    cppc::AutoLocker locker(m_lock);

    std::string configTopic = config.topic();
    for (auto it = m_listenConfigList.begin(); it != m_listenConfigList.end();) {
        ConfigInfo &listenerConfig = *it;
        if (configTopic == listenerConfig.topic()) {
            listenerConfig.setMd5(config.md5()); // 变更为新的配置参数
            listenerConfig.setContent(config.content());
            listenerConfig.setType(config.type());
            if (config.saveLocalFile()) {
                configChanged(config, opType);
            }
            return true;
        }
        ++it;
    }
    return false;
}
void ConfigListener::configChanged(const ConfigInfo &config, ConfigListener::ConfigOpType opType) {
}
} // namespace vcs