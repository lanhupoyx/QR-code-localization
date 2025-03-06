
#include "config_monitor.h"

#include <map>

#include <format/json.h>
#include <httplib.h>
#include <log/log.h>
#include <thread/lock/auto_locker.h>

#include "common/config_constant.h"

namespace vcs {
ConfigMonitor::ConfigMonitor() :
    m_autoReleaseListener(true) {
    this->setThreadName("VcsConfigMonitor");
}
ConfigMonitor::~ConfigMonitor() {
    cppc::AutoLocker locker(m_lock);
    if (m_autoReleaseListener) {
        for (auto it = m_listenerList.begin(); it != m_listenerList.end(); ++it) {
            delete *it;
        }
    }
    m_listenerList.clear();
}
void ConfigMonitor::initParams(std::shared_ptr<VcsParams> params) {
    m_lock.lock();
    m_params = params;
    m_lock.unlock();
}

void ConfigMonitor::setSecurity(std::shared_ptr<SecurityManager> security) {
    m_security = security;
}

void ConfigMonitor::addListener(ConfigListener *listener) {
    if (!listener) {
        return;
    }
    cppc::AutoLocker locker(m_lock);
    m_listenerList.push_back(listener);
}
void ConfigMonitor::removeListener(ConfigListener *listener) {
    if (!listener) {
        return;
    }
    cppc::AutoLocker locker(m_lock);
    m_listenerList.push_back(listener);
    // 遍历并移除元素
    for (auto it = m_listenerList.begin(); it != m_listenerList.end();) {
        ConfigListener *t_listener = *it;
        if (t_listener == listener) {
            if (m_autoReleaseListener) {
                delete listener;
            }
            it = m_listenerList.erase(it);
        } else {
            ++it;
        }
    }
}

bool ConfigMonitor::autoReleaseListener() {
    cppc::AutoLocker locker(m_lock);
    return m_autoReleaseListener;
}

void ConfigMonitor::setAutoReleaseListener(bool autoReleaseListener) {
    cppc::AutoLocker locker(m_lock);
    m_autoReleaseListener = autoReleaseListener;
}

void ConfigMonitor::run() {
    int requestTimeout = m_params->getParam(ConfigConstant::P_LONGPULLLING_TIMEOUT, ConfigConstant::P_DEFAULT_LONGPULLLING_TIMEOUT);
    int requestInterval = m_params->getParam(ConfigConstant::P_REQUEST_TIMEOUT, ConfigConstant::P_DEFAULT_REQUEST_TIMEOUT);

    std::unique_ptr<httplib::Client> m_http(static_cast<httplib::Client *>(m_params->createHttpClient()));
    m_http->set_read_timeout(requestTimeout);
    m_http->set_write_timeout(requestTimeout);
    m_http->set_connection_timeout(requestTimeout);

    while (isRunning()) {
        std::string t_sessionid;
        bool loginFlag = m_security->login(t_sessionid);
        if (!loginFlag) {
            if (isRunning()) {
                sleepMS(requestInterval);
                continue;
            } else {
                return;
            }
        }

        std::string listeningConfigs;
        {
            cppc::AutoLocker locker(m_lock);

            // 汇总要监听的配置列表
            std::map<std::string, ConfigInfo> configMap;
            for (auto it = m_listenerList.begin(); it != m_listenerList.end();) {
                ConfigListener *t_listener = *it;
                std::list<ConfigInfo> t_list = t_listener->listenConfigList();
                for (auto it2 = t_list.begin(); it2 != t_list.end();) {
                    ConfigInfo &config = *it2;
                    std::string topic = config.topic();
                    if (configMap.count(topic) <= 0) {
                        configMap[topic] = config;
                    }
                    ++it2;
                }

                ++it;
            }
            if (configMap.size() <= 0) {
                continue;
            }

            // 组织请求参数
            cppc::Json::array configList;
            for (auto it = configMap.begin(); it != configMap.end();) {
                ConfigInfo config = it->second;
                cppc::Json::object configObj;
                configObj["tenantId"] = config.tenantId();
                configObj["groupId"] = config.groupId();
                configObj["dataId"] = config.dataId();
                configObj["md5"] = config.md5();

                configList.push_back(configObj);
                it++;
            }
            listeningConfigs = cppc::Json(configList).dump();
        }

        std::unique_ptr<httplib::Headers> header(static_cast<httplib::Headers *>(m_params->createLongPullingHeader(t_sessionid)));
        httplib::Params params = {{ConfigConstant::HEADER_VCS_LISTENING_CONFIGS, listeningConfigs}};
        if (auto res = m_http->Post(ConfigConstant::VCS_URL_LISTEN_POST_PATH, *header, params)) {
            if (res->status == ConfigConstant::HTTP_STATUS_OK) {
                std::string errComment;

                cppc::Json jsonObj;
                try {
                    jsonObj = cppc::Json::parse(res->body, errComment, cppc::Json::ParseStandard);
                } catch (...) {
                    cppc::log::debug(errComment);
                }

                if (errComment.empty()) {
                    if (jsonObj.hasKey("dataId")) {
                        ConfigInfo changedConfig;
                        changedConfig.setTenantId(jsonObj["tenantId"].stringValue(""));
                        changedConfig.setGroupId(jsonObj["groupId"].stringValue(""));
                        changedConfig.setDataId(jsonObj["dataId"].stringValue(""));
                        changedConfig.setMd5(jsonObj["md5"].stringValue(""));
                        changedConfig.setType(jsonObj["type"].stringValue(""));
                        changedConfig.setContent(jsonObj["content"].stringValue(""));
                        changedConfig.setMaxBackupCount(jsonObj["maxBackupCount"].intValue(-2));
                        changedConfig.setCustomOpType(jsonObj["customOpType"].stringValue(""));

                        ConfigListener::ConfigOpType opType = ConfigListener::getOpTypeByStr(jsonObj["opType"].stringValue());

                        {
                            // 找到相应的监听器，触发其回调函数
                            cppc::AutoLocker locker(m_lock);
                            for (auto it = m_listenerList.begin(); it != m_listenerList.end();) {
                                ConfigListener *t_listener = *it;
                                t_listener->handleConfigChangedEvent(changedConfig, opType);
                                ++it;
                            }
                        }
                        continue;
                    }
                }
            } else {
                auto err = res.error();
                if (res->status == ConfigConstant::HTTP_STATUS_REDIRECT) {
                    cppc::log::error("[vcs]config service,http error:{}", httplib::to_string(err));
                    m_security->cleanSession();
                }
            }
        } else {
            LOG_EVERY_N(DEBUG, 30, "[vcs]config monitor,http timeout");
        }
        sleepMS(2000);
    }
}

} // namespace vcs