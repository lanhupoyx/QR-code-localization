#pragma once

#include <memory>
#include <string>

#include <thread/lock/mutex_lock.h>

#include "common/vcs_params.h"
#include "config_info.h"
#include "listen/config_listener.h"
#include "security/security_manager.h"

namespace cppc {
class Json;
}
namespace vcs {
class ConfigService {
public:
    ConfigService();
    ~ConfigService();

    /// 设置服务参数
    void initParams(std::shared_ptr<VcsParams> params);

    /// 设置权限管理对象
    void setSecurity(std::shared_ptr<SecurityManager> security);

    /**
     * 获取配置列表
     *
    /// \param dataIdMatchType dataId的匹配方式，"contain"/空：包含，"start"：以其开头，"end"：以其结尾
     * \param dataId 配置文件名称，正则匹配。包含：[.*关键字.*]，开头：[^关键字.*]，结尾：[.*关键字$]
     * \param groupId 分组名称，要完全匹配
     * \param tenantId 命名空间，要完全匹配
     * \param timeoutMs 读取timeout，在此时间前会重试
     * \return <是否获取成功,配置列表>
     */
    std::tuple<bool, std::list<ConfigInfo>> getConfigList(const std::string &dataIdMatchType, const std::string &dataId, const std::string &groupId, const std::string &tenantId = "", long timeoutMs = 30000);

    /**
     * 获取配置列表
     *
    /// \param dataIdMatchType dataId的匹配方式，"contain"/空：包含，"start"：以其开头，"end"：以其结尾
     * \param configInfo 配置信息，dataId 配置文件名称，正则匹配。开头：[^关键字.*]，结尾：[.*关键字$]，包含：[.*关键字.*]
     *                                                              groupId 分组名称，要完全匹配
     *                                                             tenantId 命名空间，要完全匹配
     * \param timeoutMs 读取timeout，在此时间前会重试
     * \return <是否获取成功,配置列表>
     */
    std::tuple<bool, std::list<ConfigInfo>> getConfigList(const std::string &dataIdMatchType, ConfigInfo &configInfo, long timeoutMs = 30000);

    /**
     * 获取配置，先从服务器，获取成功则更新本地缓存，获取失败则尝试从本地缓存读取
     *
     * \param timeoutMs 读取timeout，在此时间前会重试
     * \return 配置内容
     */
    ConfigInfo getConfig(const std::string &dataId, const std::string &groupId = "", const std::string &tenantId = "", long timeoutMs = 30000);

    /**
     * 获取配置，先从服务器，获取成功则更新本地缓存，获取失败则尝试从本地缓存读取
     *
     * \param configInfo 必填：dataId，选填：groupId、tenantId
     * \param timeoutMs 读取timeout，在此时间前会重试
     * \return 是否获取到配置，平台或本地缓存
     */
    bool getConfig(ConfigInfo &configInfo, long timeoutMs = 30000);

    /**
     * 更新或新建配置，并保存到本地缓存，如VCS已有则失败。
     *\param content 配置内容
     *\param contentType 配置类型，新建时需要(yaml/json/text)，更新时如为空则保留现有类型
     * \param timeoutMs 更新timeout，在此时间前会重试
     * \return 是否发布成功
     */
    bool publishConfig(const std::string &content, const std::string contentType,
                       const std::string &dataId, const std::string &groupId = "", const std::string &tenantId = "", long timeoutMs = 30000);
    /**
     * 更新或新建配置，并保存到本地缓存，如VCS已有则失败。
     *\param configInfo 必填：dataId、content，其他选填，如：groupId、tenantId、type、appName、configDesc
     * \param timeoutMs 更新timeout，在此时间前会重试
     * \return 是否发布成功
     */
    bool publishConfig(ConfigInfo &configInfo, long timeoutMs = 30000);

    /**
     * 强制更新或新建配置，并保存到本地缓存。如VCS已有则覆盖。
     *\param configInfo 必填：dataId、content，其他选填，如：groupId、tenantId、type、appName、configDesc
     * \param timeoutMs 更新timeout，在此时间前会重试
     * \return 是否发布成功
     */
    bool forcePublishConfig(ConfigInfo &configInfo, long timeoutMs = 30000);

private:
    bool tryPublishConfig(ConfigInfo &configInfo, long timeoutMs, bool forceUpdate);

private:
    cppc::MutexLock m_lock;
    std::shared_ptr<VcsParams> m_params;
    std::shared_ptr<SecurityManager> m_security;
};
} // namespace vcs