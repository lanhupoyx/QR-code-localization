#pragma once

#include <string>

namespace vcs {
class ConfigInfo {
public:
    /// 配置文件状态
    enum ConfigState {
        CS_INVALID,          /// 配置对象无效，未进行任何参数设置
        CS_NOT_EXIST,        /// 平台不存在且本地缓存不存在
        CS_ONLINE_NOT_EXIST, /// 平台访问成功，但配置不存在。平台不存在时也不会取本地缓存
        CS_ONLINE_OK,        /// 平台拉取成功，或平台已存在配置
        CS_LOCAL_OK          /// 平台拉取失败，但取本地缓存成功
    };

public:
    ConfigInfo();
    ~ConfigInfo() = default;

    /// 配置变更监测用的配置KEY
    std::string topic() const;

    std::string dataId() const;
    void setDataId(const std::string &value);

    std::string groupId() const;
    void setGroupId(const std::string &value);

    std::string tenantId() const;
    void setTenantId(const std::string &value);

    std::string type() const;
    void setType(const std::string &value);

    std::string content() const;
    void setContent(const std::string &value);

    /// 加载一个配置文件内容，再设置setContent()
    /// \param filePath 绝对路径
    /// \return 是否加载成功，失败则表示文件不存在或读取失败
    bool setContentWithFile(const std::string &filePath);

    std::string md5() const;
    void setMd5(const std::string &value);

    std::string appName() const;
    void setAppName(const std::string &value);

    std::string configDesc() const;
    void setConfigDesc(const std::string &value);

    int maxBackupCount() const;
    void setMaxBackupCount(int value);

    std::string customOpType() const;
    void setCustomOpType(const std::string &value);

public:
    std::string localCachePath() const;
    void setLocalCachePath(const std::string &localCachePath);

    ConfigInfo::ConfigState state() const;
    void setState(ConfigInfo::ConfigState state);

public:
    /// 本地缓存文件地址
    std::string localFilePath() const;

    /// 保存配置信息到本地缓存文件
    bool saveLocalFile();

    ///  加载本地缓存
    /// \return 是否成功
    bool loadLocalFile();

    /// 检查groupId,tenantId是否为空，是空则设置为默认值
    /// 并对数据进行trim:dataId,groupId,tenantId,md5
    void checkAndSetDefault();

protected:
    std::string m_dataId;     /// 配置ID，区分同一应用的多个配置
    std::string m_groupId;    /// 分组ID，区分不同应用
    std::string m_tenantId;   /// 租户ID，区分车辆
    std::string m_type;       /// 配置类型，text/yaml/json/toml
    std::string m_content;    /// 配置内容
    std::string m_md5;        /// 配置内容的MD5
    std::string m_appName;    /// 程序名称
    std::string m_configDesc; /// 配置描述
    int m_maxBackupCount;     /// 最大备份数量

    std::string m_customOpType; /// 自定义的平台操作指令，用于将来扩展

protected:
    /// 本地缓存目录绝对路径
    std::string m_localCachePath;

    /// 配置文件状态
    ConfigState m_state;
};
} // namespace vcs
