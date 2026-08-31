#pragma once

#include <functional>
#include <string>

#include <sys/types.h>

namespace vdsuit_wireless {

struct HotspotAddressing {
    std::string apAddress;
    std::string apCidr;
    std::string clientAddress;
    std::string networkCidr;
    std::string broadcastAddress;
    std::string netmask;
};

struct HotspotConfig {
    std::string interfaceName = "wlan0";
    std::string ssid;
    std::string passphrase = "12345678";
    std::string countryCode = "CN";
    int channel = 165;
    HotspotAddressing addressing;
};

struct ClientLease {
    std::string macAddress;
    std::string ipAddress;
};

bool deriveHotspotAddressing(const std::string& apAddress,
                             HotspotAddressing& addressing,
                             std::string& error);
bool validateInterfaceName(const std::string& interfaceName);
bool validateSerialSsid(const std::string& ssid);

std::string makeHostapdConfig(const HotspotConfig& config,
                              const std::string& controlDirectory);
std::string makeDnsmasqConfig(const HotspotConfig& config,
                              const std::string& leaseFile,
                              const std::string& pidFile);

class HotspotManager {
public:
    explicit HotspotManager(const HotspotConfig& config);
    ~HotspotManager();

    HotspotManager(const HotspotManager&) = delete;
    HotspotManager& operator=(const HotspotManager&) = delete;

    bool start(std::string& error);
    bool waitForClient(const std::function<bool()>& shouldStop,
                       ClientLease& lease,
                       std::string& error);
    void stop();

    bool active() const { return active_; }
    const std::string& runtimeDirectory() const { return runtimeDirectory_; }
    const std::string& hostapdLogPath() const { return hostapdLogPath_; }
    const std::string& dnsmasqLogPath() const { return dnsmasqLogPath_; }

private:
    bool captureNetworkManagerState(std::string& error);
    bool acquireInstanceLock(std::string& error);
    void releaseInstanceLock();
    bool recoverStaleState(std::string& error);
    bool stopStaleHelperProcesses(std::string& error);
    bool releaseInterface(std::string& error);
    bool writeRuntimeFiles(std::string& error);
    bool startHostapd(std::string& error);
    bool configureAddress(std::string& error);
    bool configureFirewall(std::string& error);
    bool startDnsmasq(std::string& error);
    bool startRecoveryWatchdog(std::string& error);
    void finishRecoveryWatchdog();
    void emergencyRestoreFromWatchdog();
    void preserveDiagnosticLogs();
    void removeFirewallRules();
    void restoreInterface();
    void removeRuntimeFiles();

    HotspotConfig config_;
    bool active_ = false;
    bool interfaceReleased_ = false;
    bool wasManaged_ = false;
    std::string previousConnectionUuid_;
    std::string runtimeDirectory_;
    std::string hostapdConfigPath_;
    std::string dnsmasqConfigPath_;
    std::string leaseFilePath_;
    std::string dnsmasqPidPath_;
    std::string hostapdLogPath_;
    std::string dnsmasqLogPath_;
    std::string hostapdExecutable_;
    std::string dnsmasqExecutable_;
    std::string iptablesExecutable_;
    std::string firewallComment_;
    bool dhcpFirewallRule_ = false;
    bool protocolFirewallRule_ = false;
    pid_t hostapdPid_ = -1;
    pid_t dnsmasqPid_ = -1;
    pid_t recoveryWatchdogPid_ = -1;
    int recoveryWatchdogWriteFd_ = -1;
    int instanceLockFd_ = -1;
    std::string instanceLockPath_;
};

} // namespace vdsuit_wireless
