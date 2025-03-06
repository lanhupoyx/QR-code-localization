#pragma once
#include <string>

#include <format/json.h>
#include <httplib.h>
namespace vcs {

extern void trimAndSetDefault(std::string &param, const std::string &defaultValue);

extern int getApiResultCode(const cppc::Json &jsonObj, const std::string &key, const int defaultValue);

extern std::string getHeader(const httplib::Headers &headers, const std::string &name, std::string defaultValue);

extern int getHeader(const httplib::Headers &headers, const std::string &name, int defaultValue);

} // namespace vcs