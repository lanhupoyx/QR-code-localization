#include "config_service.h"

#include <datetime/datetime.h>
#include <format/json.h>
#include <format/string_format.h>
#include <httplib.h>
#include <log/log.h>
#include <thread/lock/auto_locker.h>
#include <thread/thread.h>
#include <util/string_util.h>

#include "common/config_constant.h"
#include "common/vcs_utils.h"

using namespace cppc;
namespace vcs {
ConfigService::ConfigService() {
}

ConfigService::~ConfigService() {
}

void ConfigService::initParams(std::shared_ptr<VcsParams> params) {
    m_lock.lock();
    m_params = params;
    m_lock.unlock();
}

void ConfigService::setSecurity(std::shared_ptr<SecurityManager> security) {
    m_security = security;
}

std::tuple<bool, std::list<ConfigInfo>> ConfigService::getConfigList(const std::string &dataIdMatchType, const std::string &dataId, const std::string &groupId, const std::string &tenantId, long timeoutMs) {
    ConfigInfo configInfo;
    configInfo.setDataId(dataId);
    configInfo.setGroupId(groupId);
    configInfo.setTenantId(tenantId);
    return getConfigList(dataIdMatchType, configInfo, timeoutMs);
}

std::tuple<bool, std::list<ConfigInfo>> ConfigService::getConfigList(const std::string &dataIdMatchType, ConfigInfo &configInfo, long timeoutMs) {
    configInfo.checkAndSetDefault();

    int requestTimeout = m_params->getParam(ConfigConstant::P_REQUEST_TIMEOUT, ConfigConstant::P_DEFAULT_REQUEST_TIMEOUT);

    std::unique_ptr<httplib::Client> m_http(static_cast<httplib::Client *>(m_params->createHttpClient()));
    m_http->set_read_timeout(requestTimeout);
    m_http->set_write_timeout(requestTimeout);
    m_http->set_connection_timeout(requestTimeout);

    bool getConfigListFlag = false;
    std::list<ConfigInfo> configInfoList;
    int64_t startTime = cppc::DateTime::currentTickCount();
    do {
        int64_t nowTime = cppc::DateTime::currentTickCount();
        if (nowTime - startTime > timeoutMs) {
            break;
        }
        std::string t_sessionid;
        bool loginFlag = m_security->login(t_sessionid);
        if (loginFlag) {
            std::unique_ptr<httplib::Headers> header(static_cast<httplib::Headers *>(m_params->createCommonHeader(t_sessionid)));
            httplib::Params params = {{"tenantId", configInfo.tenantId()}, {"groupId", configInfo.groupId()}, {"dataId", configInfo.dataId()}, {"dataIdMatchType", dataIdMatchType}};
            if (auto res = m_http->Get(ConfigConstant::VCS_URL_CONFIG_LIST_GET_PATH, params, *header)) {
                if (res->status == ConfigConstant::HTTP_STATUS_OK) {
                    std::string errComment;

                    cppc::Json jsonRootObj;
                    try {
                        jsonRootObj = cppc::Json::parse(res->body, errComment, cppc::Json::ParseStandard);
                    } catch (...) {
                        cppc::log::debug(errComment);
                    }

                    if (errComment.empty()) {
                        configInfoList.clear();
                        auto &configList = jsonRootObj.arrayItems();
                        for (uint i = 0; i < configList.size(); i++) {
                            auto &jsonObj = jsonRootObj[i];
                            if (jsonObj.hasKey("dataId")) {
                                ConfigInfo config;
                                config.setTenantId(jsonObj["tenantId"].stringValue(""));
                                config.setGroupId(jsonObj["groupId"].stringValue(""));
                                config.setDataId(jsonObj["dataId"].stringValue(""));
                                config.setMd5(jsonObj["md5"].stringValue(""));
                                config.setType(jsonObj["type"].stringValue(""));
                                config.setContent(jsonObj["content"].stringValue(""));
                                config.setMaxBackupCount(jsonObj["maxBackupCount"].intValue(-2));

                                configInfoList.push_back(config);
                            }
                        }
                        getConfigListFlag = true;
                    }
                } else if (res->status == ConfigConstant::HTTP_STATUS_NOT_FOUND) {
                    getConfigListFlag = false;
                } else {
                    getConfigListFlag = false;
                    auto err = res.error();
                    cppc::log::debug("[vcs]config service,http error:{}", httplib::to_string(err));
                }
                break;
            }
        }

        cppc::Thread::sleepMS(2000);
    } while (true);

    return std::tuple<bool, std::list<ConfigInfo>>(getConfigListFlag, configInfoList);
}

ConfigInfo ConfigService::getConfig(const std::string &dataId, const std::string &groupId, const std::string &tenantId, long timeoutMs) {
    ConfigInfo configInfo;
    configInfo.setDataId(dataId);
    configInfo.setGroupId(groupId);
    configInfo.setTenantId(tenantId);
    getConfig(configInfo, timeoutMs);
    return configInfo;
}

bool ConfigService::getConfig(ConfigInfo &configInfo, long timeoutMs) {
    configInfo.checkAndSetDefault();
    configInfo.setLocalCachePath(m_params->getParam(ConfigConstant::P_LOCAL_CACHE_PATH, ConfigConstant::P_DEFAULT_LOCAL_CACHE_PATH));

    int requestTimeout = m_params->getParam(ConfigConstant::P_REQUEST_TIMEOUT, ConfigConstant::P_DEFAULT_REQUEST_TIMEOUT);

    std::unique_ptr<httplib::Client> m_http(static_cast<httplib::Client *>(m_params->createHttpClient()));
    m_http->set_read_timeout(requestTimeout);
    m_http->set_write_timeout(requestTimeout);
    m_http->set_connection_timeout(requestTimeout);

    int64_t startTime = cppc::DateTime::currentTickCount();
    do {
        int64_t nowTime = cppc::DateTime::currentTickCount();
        if (nowTime - startTime > timeoutMs) {
            break;
        }
        std::string t_sessionid;
        bool loginFlag = m_security->login(t_sessionid);
        if (loginFlag) {
            std::unique_ptr<httplib::Headers> header(static_cast<httplib::Headers *>(m_params->createCommonHeader(t_sessionid)));
            httplib::Params params = {{"tenantId", configInfo.tenantId()}, {"groupId", configInfo.groupId()}, {"dataId", configInfo.dataId()}, {"md5", configInfo.md5()}};
            if (auto res = m_http->Get(ConfigConstant::VCS_URL_CONFIG_GET_PATH, params, *header)) {
                if (res->status == ConfigConstant::HTTP_STATUS_OK) {
                    configInfo.setType(getHeader(res->headers, ConfigConstant::HEADER_VCS_CONTENT_TYPE, ConfigConstant::VCS_DEFAULT_CONTENT_TYPE));
                    configInfo.setMd5(getHeader(res->headers, ConfigConstant::HEADER_VCS_CONTENT_MD5, ""));
                    configInfo.setMaxBackupCount(getHeader(res->headers, ConfigConstant::HEADER_VCS_MAX_BACKUP_COUNT, -2));
                    configInfo.setContent(res->body);
                    configInfo.setState(ConfigInfo::CS_ONLINE_OK);
                    bool saveFileFlag = configInfo.saveLocalFile(); // 保存到本地缓存文件

                    cppc::log::info("[vcs]online config ok");
                    return true;
                } else if (res->status == ConfigConstant::HTTP_STATUS_NOT_FOUND) {
                    cppc::log::error("[vcs]online config not exist");
                    configInfo.setState(ConfigInfo::CS_ONLINE_NOT_EXIST);
                    return false;
                } else {
                    configInfo.setState(ConfigInfo::CS_INVALID);
                    auto err = res.error();
                    cppc::log::error("[vcs]config service,http error:{}", httplib::to_string(err));
                    if (res->status == ConfigConstant::HTTP_STATUS_REDIRECT) {
                        m_security->cleanSession();
                    }
                }
            }
        }

        cppc::Thread::sleepMS(2000);
    } while (true);

    // 加载本地缓存文件
    return configInfo.loadLocalFile();
}
bool ConfigService::publishConfig(const std::string &content, const std::string contentType, const std::string &dataId, const std::string &groupId, const std::string &tenantId, long timeoutMs) {
    ConfigInfo configInfo;
    configInfo.setDataId(dataId);
    configInfo.setGroupId(groupId);
    configInfo.setTenantId(tenantId);
    configInfo.setType(contentType);
    configInfo.setContent(content);

    return tryPublishConfig(configInfo, timeoutMs, false);
}

bool ConfigService::publishConfig(ConfigInfo &configInfo, long timeoutMs) {
    return tryPublishConfig(configInfo, timeoutMs, false);
}

bool ConfigService::forcePublishConfig(ConfigInfo &configInfo, long timeoutMs) {
    return tryPublishConfig(configInfo, timeoutMs, true);
}

bool ConfigService::tryPublishConfig(ConfigInfo &configInfo, long timeoutMs, bool forceUpdate) {
    configInfo.checkAndSetDefault();
    configInfo.setLocalCachePath(m_params->getParam(ConfigConstant::P_LOCAL_CACHE_PATH, ConfigConstant::P_DEFAULT_LOCAL_CACHE_PATH));

    int requestTimeout = m_params->getParam(ConfigConstant::P_REQUEST_TIMEOUT, ConfigConstant::P_DEFAULT_REQUEST_TIMEOUT);

    std::unique_ptr<httplib::Client> m_http((httplib::Client *)m_params->createHttpClient());
    m_http->set_read_timeout(requestTimeout);
    m_http->set_write_timeout(requestTimeout);
    m_http->set_connection_timeout(requestTimeout);
    int64_t startTime = cppc::DateTime::currentTickCount();
    do {
        int64_t nowTime = cppc::DateTime::currentTickCount();
        if (nowTime - startTime > timeoutMs) {
            break;
        }
        configInfo.setState(ConfigInfo::CS_INVALID);
        std::string t_sessionid;
        bool loginFlag = m_security->login(t_sessionid);
        if (loginFlag) {
            std::unique_ptr<httplib::Headers> header((httplib::Headers *)m_params->createCommonHeader(t_sessionid));
            httplib::Params params = {{"tenantId", configInfo.tenantId()},
                                      {"groupId", configInfo.groupId()},
                                      {"dataId", configInfo.dataId()},
                                      {"content", configInfo.content()},
                                      {"maxBackupCount", std::to_string(configInfo.maxBackupCount())},
                                      {"type", configInfo.type()},
                                      {"forceUpdate", forceUpdate ? "1" : " 0"},
                                      {"appName", configInfo.appName()},
                                      {"configDesc", configInfo.configDesc()}};
            if (auto res = m_http->Post(ConfigConstant::VCS_URL_CONFIG_POST_PATH, *header, params)) {
                if (res->status == ConfigConstant::HTTP_STATUS_OK) {
                    std::string errComment;
                    cppc::Json jsonObj;
                    try {
                        jsonObj = cppc::Json::parse(res->body, errComment, cppc::Json::ParseStandard);
                    } catch (...) {
                        cppc::log::debug(errComment);
                    }
                    if (errComment.empty()) {
                        int code = -1;
                        if (jsonObj.hasKey("code")) {
                            code = jsonObj["code"].intValue(code);
                        }
                        if (code == ConfigConstant::API_RESULT_SUCCESS) {
                            configInfo.setState(ConfigInfo::CS_ONLINE_OK);
                            bool saveFileFlag = configInfo.saveLocalFile(); // 保存到本地缓存文件
                            return saveFileFlag;
                        } else if (code == ConfigConstant::API_RESULT_WARN) {
                            configInfo.setState(ConfigInfo::CS_ONLINE_OK);
                            return false;
                        }
                    }
                } else {
                    auto err = res.error();
                    cppc::log::error("[vcs]config service,http error:{}", httplib::to_string(err));
                    if (res->status == ConfigConstant::HTTP_STATUS_REDIRECT) {
                        m_security->cleanSession();
                    }
                }
            }
        }
        cppc::Thread::sleepMS(2000);
    } while (true);
    return false;
}
} // namespace vcs