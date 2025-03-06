#include "vcs_params.h"

#include <format/string_format.h>
#include <httplib.h>

#include "common/config_constant.h"

namespace vcs {
std::string VcsParams::getParam(const std::string &paramName, const std::string &defaultValue) const {
    if (count(paramName) > 0) {
        return at(paramName);
    }
    return defaultValue;
}

int VcsParams::getParam(const std::string &paramName, const int defaultValue) const {
    if (count(paramName) > 0) {
        try {
            return std::stoi(at(paramName));
        } catch (...) {
        }
    }
    return defaultValue;
}

void *VcsParams::createHttpClient() {
    std::string serverAddr = getParam(ConfigConstant::P_SERVER_ADDR, ConfigConstant::P_DEFAULT_SERVER_ADDR);
    return new httplib::Client(serverAddr);
}

void *VcsParams::createBaseHeader() {
    httplib::Headers *headers = new httplib::Headers({{"accept", "application/json"}, {"X-Requested-With", "XMLHttpRequest"}});
    return static_cast<void *>(headers);
}

void *VcsParams::createCommonHeader(const std::string &sessionid) {
    httplib::Headers *headers = (httplib::Headers *)createBaseHeader();
    headers->insert({ConfigConstant::HEADER_COOKIE, cppc::format("%s=%s", ConfigConstant::HEADER_JSESSIONID.c_str(), sessionid.c_str())});
    return static_cast<void *>(headers);
}

void *VcsParams::createLongPullingHeader(const std::string &sessionid) {
    int longPullingTimeout = getParam(ConfigConstant::P_LONGPULLLING_TIMEOUT, 60000);
    int longPullingTimeoutSecond = longPullingTimeout / 1000;

    httplib::Headers *headers = (httplib::Headers *)createCommonHeader(sessionid);
    headers->insert({ConfigConstant::HEADER_VCS_LONGPULLLING_TIMEOUT, std::to_string(longPullingTimeout)});

    return static_cast<void *>(headers);
}
} // namespace vcs