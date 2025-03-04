#include "security_manager.h"

#include <format/json.h>
#include <httplib.h>
#include <log/log.h>
#include <thread/lock/auto_locker.h>

#include "common/config_constant.h"
#include "common/vcs_utils.h"
#include <datetime/datetime.h>

namespace vcs {
SecurityManager::SecurityManager() {
    m_lastLoginTime = 0;
}

void SecurityManager::initParams(std::shared_ptr<VcsParams> params) {
    m_lock.lock();
    m_params = params;
    m_lock.unlock();
}

bool SecurityManager::login(std::string &sessionid) {
    int64_t nowTime = cppc::DateTime::currentTickCount();

    sessionid = "";
    cppc::AutoLocker locker(m_lock);
    if (m_sessionid.size() > 0) {
        // if (nowTime - m_lastLoginTime >= 30 * 60 * 1000) {
        //     cleanSession();
        // } else {
        sessionid = m_sessionid;
        return true;
        // }
    }

    std::unique_ptr<httplib::Headers> header(static_cast<httplib::Headers *>(m_params->createBaseHeader()));
    std::unique_ptr<httplib::Client> m_http(static_cast<httplib::Client *>(m_params->createHttpClient()));

    std::string username = m_params->getParam(ConfigConstant::P_AUTH_USERNAME, ConfigConstant::P_DEFAULT_USERNAME);
    std::string password = m_params->getParam(ConfigConstant::P_AUTH_PASSWORD, "");
    httplib::Params params = {{"username", username}, {"password", password}};

    if (auto res = m_http->Post(ConfigConstant::VCS_URL_LOGIN_POST_PATH, *header, params)) {
        if (res->status != ConfigConstant::HTTP_STATUS_OK) {
            return false;
        }
        std::string errComment;
        cppc::Json jsonObj = cppc::Json::parse(res->body, errComment, cppc::Json::ParseComments);

        if (!errComment.empty()) {
            return false;
        }
        int resultCode = getApiResultCode(jsonObj, ConfigConstant::API_RESULT_CODE_NAME, ConfigConstant::API_RESULT_ERROR);
        if (resultCode == ConfigConstant::API_RESULT_SUCCESS) {
            /// 取出SESSIONID，<Set-Cookie,JSESSIONID=***>
            httplib::Headers responseHeader = res->headers;
            auto it = responseHeader.find(ConfigConstant::HEADER_SET_COOKIE);
            if (it != responseHeader.end()) {
                httplib::Params params;
                httplib::detail::parse_disposition_params(it->second, params);
                if (params.count(ConfigConstant::HEADER_JSESSIONID) > 0) {
                    auto it = params.find(ConfigConstant::HEADER_JSESSIONID);
                    if (it != params.end()) {
                        m_sessionid = it->second;
                        sessionid = m_sessionid;
                        m_lastLoginTime = cppc::DateTime::currentTickCount();
                        return true;
                    }
                }
            }
        }
    }
    return false;
}
void SecurityManager::cleanSession() {
    cppc::AutoLocker locker(m_lock);
    m_sessionid = "";
}
} // namespace vcs
