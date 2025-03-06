#include "config_constant.h"

/// 服务器配置参数
namespace vcs {
const std::string ConfigConstant::P_AUTH_USERNAME = "vcs.auth.username";
const std::string ConfigConstant::P_AUTH_PASSWORD = "vcs.auth.password";
const std::string ConfigConstant::P_SERVER_ADDR = "vcs.server.addr";
const std::string ConfigConstant::P_REQUEST_TIMEOUT = "vcs.server.request.timeout";
const std::string ConfigConstant::P_LONGPULLLING_TIMEOUT = "vcs.server.longpulling.timeout";
const std::string ConfigConstant::P_LOCAL_CACHE_PATH = "vcs.local.cache.path";
const std::string ConfigConstant::P_REQUEST_RETRY_COUNT = "vcs.server.retry.count";

const std::string ConfigConstant::P_DEFAULT_SERVER_ADDR = "127.0.0.1:8888";
const std::string ConfigConstant::P_DEFAULT_USERNAME = "ep";
const int ConfigConstant::P_DEFAULT_REQUEST_RETRY_COUNT = 10;
const int ConfigConstant::P_DEFAULT_REQUEST_TIMEOUT = 3000;
const int ConfigConstant::P_DEFAULT_LONGPULLLING_TIMEOUT = 120000;
const std::string ConfigConstant::P_DEFAULT_LOCAL_CACHE_PATH = "/var/xmover/params/vcs";
} // namespace vcs

/// VCS业务参数
namespace vcs {
const std::string ConfigConstant::VCS_DEFAULT_GROUP = "public";
const std::string ConfigConstant::VCS_DEFAULT_TENANT = "public";
const std::string ConfigConstant::VCS_DEFAULT_CONTENT_TYPE = "text";

const std::string ConfigConstant::VCS_URL_BASE = "/vcs/vcsApi/v1";
const std::string ConfigConstant::VCS_URL_LOGIN_POST_PATH = "/login";
const std::string ConfigConstant::VCS_URL_CONFIG_LIST_GET_PATH = VCS_URL_BASE + "/getConfigList";
const std::string ConfigConstant::VCS_URL_CONFIG_GET_PATH = VCS_URL_BASE + "/getConfig";
const std::string ConfigConstant::VCS_URL_CONFIG_POST_PATH = VCS_URL_BASE + "/publishConfig";
const std::string ConfigConstant::VCS_URL_LISTEN_POST_PATH = VCS_URL_BASE + "/listen";
} // namespace vcs

/// HTTP请求的一些参数
namespace vcs {
const std::string ConfigConstant::HTTP_ENCODE = "UTF-8";
const int ConfigConstant::HTTP_STATUS_OK = 200;
const int ConfigConstant::HTTP_STATUS_REDIRECT = 302;
const int ConfigConstant::HTTP_STATUS_NOT_FOUND = 404;

const std::string ConfigConstant::API_RESULT_CODE_NAME = "code";
const int ConfigConstant::API_RESULT_SUCCESS = 0;
const int ConfigConstant::API_RESULT_WARN = 301;
const int ConfigConstant::API_RESULT_ERROR = 500;

const std::string ConfigConstant::HEADER_VCS_LISTENING_CONFIGS = "vcs-listening-configs";
const std::string ConfigConstant::HEADER_VCS_CONTENT_TYPE = "vcs-content-type";
const std::string ConfigConstant::HEADER_VCS_CONTENT_MD5 = "vcs-content-md5";
const std::string ConfigConstant::HEADER_VCS_MAX_BACKUP_COUNT = "vcs-max-backup-count";
const std::string ConfigConstant::HEADER_VCS_LONGPULLLING_TIMEOUT = "vcs-longpulling-timeout";
const std::string ConfigConstant::HEADER_COOKIE = "Cookie";
const std::string ConfigConstant::HEADER_SET_COOKIE = "Set-Cookie";
const std::string ConfigConstant::HEADER_JSESSIONID = "JSESSIONID";

} // namespace vcs
