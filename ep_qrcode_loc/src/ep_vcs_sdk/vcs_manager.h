#pragma once

#include <thread/lock/mutex_lock.h>

#include "common/config_constant.h"
#include "common/vcs_params.h"
#include "config/config_service.h"
#include "listen/config_monitor.h"
#include "security/security_manager.h"

namespace vcs {
///  VCS配置服务的顶层管理对象，由此创建具体的子功能接口
class VcsManager {
public:
    VcsManager();
    ~VcsManager() = default;

public:
    void initParams();
    void initParams(const VcsParams &params);
    ConfigService &configService();
    ConfigMonitor &configMonitor();

private:
    std::shared_ptr<VcsParams> m_params;
    std::shared_ptr<SecurityManager> m_security;
    ConfigService m_configService;
    ConfigMonitor m_configMonitor;
};
} // namespace vcs
