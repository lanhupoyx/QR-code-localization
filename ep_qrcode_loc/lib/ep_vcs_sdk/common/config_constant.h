#pragma once

#include <string>
namespace vcs {
/// HTTP请求相关的一些常量
class ConfigConstant {
public:
    /// 服务器配置参数
    static const std::string P_AUTH_USERNAME;
    static const std::string P_AUTH_PASSWORD;
    static const std::string P_SERVER_ADDR;
    static const std::string P_REQUEST_TIMEOUT;
    static const std::string P_LONGPULLLING_TIMEOUT;
    static const std::string P_LOCAL_CACHE_PATH;
    static const std::string P_REQUEST_RETRY_COUNT;

    static const std::string P_DEFAULT_SERVER_ADDR;
    static const std::string P_DEFAULT_USERNAME;
    static const int P_DEFAULT_REQUEST_TIMEOUT;
    static const int P_DEFAULT_LONGPULLLING_TIMEOUT;
    static const int P_DEFAULT_REQUEST_RETRY_COUNT;
    static const std::string P_DEFAULT_LOCAL_CACHE_PATH;

public:
    /// VCS业务参数
    static const std::string VCS_DEFAULT_GROUP;
    static const std::string VCS_DEFAULT_TENANT;
    static const std::string VCS_DEFAULT_CONTENT_TYPE;

    static const std::string VCS_URL_BASE;
    static const std::string VCS_URL_CONFIG_LIST_GET_PATH;
    static const std::string VCS_URL_LOGIN_POST_PATH;
    static const std::string VCS_URL_CONFIG_GET_PATH;
    static const std::string VCS_URL_CONFIG_POST_PATH;
    static const std::string VCS_URL_LISTEN_POST_PATH;

public:
    /// HTTP请求的一些参数
    static const std::string HTTP_ENCODE;
    static const int HTTP_STATUS_OK;
    static const int HTTP_STATUS_REDIRECT;
    static const int HTTP_STATUS_NOT_FOUND;

    static const std::string API_RESULT_CODE_NAME;
    static const int API_RESULT_SUCCESS;
    static const int API_RESULT_WARN;
    static const int API_RESULT_ERROR;

    static const std::string HEADER_VCS_LISTENING_CONFIGS;
    static const std::string HEADER_VCS_CONTENT_TYPE;
    static const std::string HEADER_VCS_CONTENT_MD5;
    static const std::string HEADER_VCS_MAX_BACKUP_COUNT;
    static const std::string HEADER_VCS_LONGPULLLING_TIMEOUT;
    static const std::string HEADER_COOKIE;
    static const std::string HEADER_SET_COOKIE;
    static const std::string HEADER_JSESSIONID;
};
} // namespace vcs