#include "vcs_manager.h"
namespace vcs {

VcsManager::VcsManager() {
    m_security = std::make_shared<SecurityManager>();
    m_configService.setSecurity(m_security);
    m_configMonitor.setSecurity(m_security);

    VcsParams defParam;
    initParams(defParam);
}
void VcsManager::initParams() {
    VcsParams params;
    initParams(params);
}
void VcsManager::initParams(const VcsParams &params) {
    m_params = std::make_shared<VcsParams>(params);
    m_security->initParams(m_params);

    m_configService.initParams(m_params);
    m_configMonitor.initParams(m_params);
}
ConfigService &VcsManager::configService() {
    return m_configService;
}
ConfigMonitor &VcsManager::configMonitor() {
    return m_configMonitor;
}
} // namespace vcs
