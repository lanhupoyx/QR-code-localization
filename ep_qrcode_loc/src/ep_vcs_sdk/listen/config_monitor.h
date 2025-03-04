#pragma once

#include <list>
#include <memory>

#include <thread/lock/mutex_lock.h>
#include <thread/thread.h>

#include "common/vcs_params.h"
#include "listen/config_listener.h"
#include "security/security_manager.h"

namespace vcs {
/// 定时监听配置信息变更事件
class ConfigMonitor : public cppc::Thread {
public:
    ConfigMonitor();
    ~ConfigMonitor();

    /// 设置服务参数
    void initParams(std::shared_ptr<VcsParams> params);

    /// 设置权限管理对象
    void setSecurity(std::shared_ptr<SecurityManager> security);

    /// 注册一个事件监听器
    void addListener(ConfigListener *listener);

    /// 移除一个事件监听器
    void removeListener(ConfigListener *listener);

    /// 获取是否自动释放监听器
    bool autoReleaseListener();

    /// 设置是否自动释放监听器
    void setAutoReleaseListener(bool autoReleaseListener);

protected:
    virtual void run();

private:
    cppc::MutexLock m_lock;
    std::shared_ptr<VcsParams> m_params;
    std::list<ConfigListener *> m_listenerList;
    std::shared_ptr<SecurityManager> m_security;
    bool m_autoReleaseListener; /// 是否自动释放m_listenerList中的对象
};
} // namespace vcs