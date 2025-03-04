#pragma once

#include <map>
#include <string>

namespace vcs {
/// 管理客户端配置参数
class VcsParams : public std::map<std::string, std::string> {
public:
    /// 获取指定参数
    /// \param paramName 参数名称
    /// \param defaultValue 如果参数不存在，返回该默认值
    /// \return 参数值
    std::string getParam(const std::string &paramName, const std::string &defaultValue) const;

    /// 获取指定参数
    /// \param paramName 参数名称
    /// \param defaultValue 如果参数不存在，返回该默认值
    /// \return 参数值
    int getParam(const std::string &paramName, const int defaultValue) const;

    /// 创建一个HTTP连接客户端
    void *createHttpClient();

    /// 创建HTTP请求用的头信息
    void *createBaseHeader();
    void *createCommonHeader(const std::string &sessionid);
    void *createLongPullingHeader(const std::string &sessionid);
};

} // namespace  vcs