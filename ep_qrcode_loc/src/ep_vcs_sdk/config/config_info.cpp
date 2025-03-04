#include "config_info.h"

#include <file/file.h>
#include <file/path.h>
#include <format/string_format.h>
#include <log/log.h>
#include <secret/md5.h>
#include <util/string_util.h>

#include "common/config_constant.h"
#include "common/vcs_utils.h"

namespace vcs {
ConfigInfo::ConfigInfo() :
    m_type(ConfigConstant::VCS_DEFAULT_CONTENT_TYPE),
    m_maxBackupCount(-1),
    m_localCachePath(ConfigConstant::P_DEFAULT_LOCAL_CACHE_PATH),
    m_state(ConfigInfo::CS_INVALID) {
}

std::string ConfigInfo::topic() const {
    return cppc::format("%s@%s@%s", m_dataId.c_str(), m_groupId.c_str(), m_tenantId.c_str());
}

std::string ConfigInfo::dataId() const {
    return m_dataId;
}

void ConfigInfo::setDataId(const std::string &value) {
    m_dataId = value;
    cppc::trim(m_dataId);
}

std::string ConfigInfo::groupId() const {
    return m_groupId;
}

void ConfigInfo::setGroupId(const std::string &value) {
    m_groupId = value;
    cppc::trim(m_groupId);
}

std::string ConfigInfo::tenantId() const {
    return m_tenantId;
}

void ConfigInfo::setTenantId(const std::string &value) {
    m_tenantId = value;
    cppc::trim(m_tenantId);
}

std::string ConfigInfo::type() const {
    return m_type;
}

void ConfigInfo::setType(const std::string &value) {
    m_type = value;
    cppc::trim(m_type);
}

std::string ConfigInfo::content() const {
    return m_content;
}

void ConfigInfo::setContent(const std::string &value) {
    m_content = value;
}

bool ConfigInfo::setContentWithFile(const std::string &filePath) {
    cppc::File file(filePath);
    if (file.exists()) {
        if (file.open(cppc::File::ReadOnly)) {
            m_content = file.readAll();

            cppc::MD5 contentMd5(m_content);
            contentMd5.finalize();
            m_md5 = contentMd5.hexdigest();

            file.close();
            return true;
        } else {
            cppc::log::error("[vcs]local config read fail{}", filePath);
        }
    } else {
        cppc::log::error("[vcs]local config not exist{}", filePath);
    }
    m_content = "";
    m_md5 = "";
    return false;
}

std::string ConfigInfo::md5() const {
    return m_md5;
}

void ConfigInfo::setMd5(const std::string &value) {
    m_md5 = value;
    cppc::trim(m_md5);
}

std::string ConfigInfo::appName() const {
    return m_appName;
}

void ConfigInfo::setAppName(const std::string &value) {
    m_appName = value;
}

std::string ConfigInfo::configDesc() const {
    return m_configDesc;
}

void ConfigInfo::setConfigDesc(const std::string &value) {
    m_configDesc = value;
}

int ConfigInfo::maxBackupCount() const {
    return m_maxBackupCount;
}

void ConfigInfo::setMaxBackupCount(int value) {
    m_maxBackupCount = value;
}

std::string ConfigInfo::customOpType() const {
    return m_customOpType;
}

void ConfigInfo::setCustomOpType(const std::string &value) {
    m_customOpType = value;
}

std::string ConfigInfo::localCachePath() const {
    return m_localCachePath;
}

void ConfigInfo::setLocalCachePath(const std::string &localCachePath) {
    m_localCachePath = localCachePath;
}

std::string ConfigInfo::localFilePath() const {
    return cppc::format("%s/%s/%s/%s.txt", m_localCachePath.c_str(), m_tenantId.c_str(), m_groupId.c_str(), m_dataId.c_str());
}

bool ConfigInfo::saveLocalFile() {
    cppc::log::info("[vcs]save local config");
    // 先检查目录是否存在，不存在则创建
    std::string filePath = localFilePath();
    cppc::Path path(filePath);
    cppc::Path dirPath = path.parentPath();

    bool saveFlag = false;
    if (!dirPath.exists() && !dirPath.createDirectory()) {
        cppc::log::error("[vcs]config save fail");
    } else {
        saveFlag = cppc::File::saveTextTo(filePath, m_content);
    }
    return saveFlag;
}

bool ConfigInfo::loadLocalFile() {
    cppc::log::info("[vcs]load local config");
    std::string filePath = localFilePath();
    if (setContentWithFile(filePath)) {
        m_state = ConfigInfo::CS_LOCAL_OK;
        return true;
    } else {
        cppc::log::error("[vcs]local config not exist");
        m_state = ConfigInfo::CS_NOT_EXIST;
        return false;
    }
}

void ConfigInfo::checkAndSetDefault() {
    trimAndSetDefault(m_dataId, "");
    trimAndSetDefault(m_groupId, ConfigConstant::VCS_DEFAULT_GROUP);
    trimAndSetDefault(m_tenantId, ConfigConstant::VCS_DEFAULT_TENANT);
    trimAndSetDefault(m_md5, "");
}

ConfigInfo::ConfigState ConfigInfo ::state() const {
    return m_state;
}

void ConfigInfo ::setState(ConfigInfo::ConfigState state) {
    m_state = state;
}
} // namespace vcs