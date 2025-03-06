#include "vcs_utils.h"

#include <util/string_util.h>
namespace vcs {

void trimAndSetDefault(std::string &param, const std::string &defaultValue) {
    cppc::trim(param);
    if (param.size() <= 0) {
        param = defaultValue;
    }
}
int getApiResultCode(const cppc::Json &jsonObj, const std::string &key, const int defaultValue) {
    if (jsonObj.hasKey(key)) {
        return jsonObj[key].intValue();
    } else {
        return defaultValue;
    }
}
std::string getHeader(const httplib::Headers &headers, const std::string &name, std::string defaultValue) {
    // 使用范围查找
    auto range = headers.equal_range(name);

    // 遍历范围并输出值
    for (auto it = range.first; it != range.second; ++it) {
        return it->second;
    }
    return defaultValue;
}

int getHeader(const httplib::Headers &headers, const std::string &name, int defaultValue) {
    // 使用范围查找
    auto range = headers.equal_range(name);

    // 遍历范围并输出值
    for (auto it = range.first; it != range.second; ++it) {
        try {
            return std::stoi(it->second);
        } catch (...) {
        }
        break;
    }
    return defaultValue;
}
} // namespace vcs