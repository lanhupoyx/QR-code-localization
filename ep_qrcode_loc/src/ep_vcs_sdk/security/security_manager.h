#pragma once

#include <memory>
#include <string>

#include <thread/lock/mutex_lock.h>

#include "common/vcs_params.h"

namespace vcs {
/// 身份认证管理
class SecurityManager {
public:
    SecurityManager();

    /// 执行登录
    /// \param sessionid[out] 获取到的SESSIONID
    /// \return 是否登录成功
    bool login(std::string &sessionid);

    /// 清除SESSION，重新登录
    void cleanSession();

    /// 设置参数
    void initParams(std::shared_ptr<VcsParams> params);

private:
    cppc::MutexLock m_lock;
    std::shared_ptr<VcsParams> m_params;
    std::string m_sessionid;
    int64_t m_lastLoginTime;
};

} // namespace vcs
