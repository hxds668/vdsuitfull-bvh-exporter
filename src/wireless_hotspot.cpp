#include "wireless_hotspot.h"
#include "vdsuit_wireless_protocol.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <net/if.h>
#include <grp.h>
#include <pwd.h>
#include <signal.h>
#include <dirent.h>
#include <sys/prctl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

namespace vdsuit_wireless {
namespace {

struct CommandResult {
    int exitCode = -1;
    std::string output;
};

std::string trim(const std::string& value)
{
    const std::string whitespace = " \t\r\n";
    const std::size_t first = value.find_first_not_of(whitespace);
    if (first == std::string::npos) return std::string();
    const std::size_t last = value.find_last_not_of(whitespace);
    return value.substr(first, last - first + 1);
}

std::vector<char*> makeArgv(const std::vector<std::string>& arguments)
{
    std::vector<char*> argv;
    argv.reserve(arguments.size() + 1);
    for (const std::string& argument : arguments) {
        argv.push_back(const_cast<char*>(argument.c_str()));
    }
    argv.push_back(nullptr);
    return argv;
}

CommandResult runCommand(const std::vector<std::string>& arguments)
{
    CommandResult result;
    if (arguments.empty()) {
        result.output = "empty command";
        return result;
    }

    int outputPipe[2];
    if (pipe(outputPipe) != 0) {
        result.output = std::string("pipe failed: ") + std::strerror(errno);
        return result;
    }

    pid_t pid = fork();
    if (pid < 0) {
        close(outputPipe[0]);
        close(outputPipe[1]);
        result.output = std::string("fork failed: ") + std::strerror(errno);
        return result;
    }
    if (pid == 0) {
        close(outputPipe[0]);
        dup2(outputPipe[1], STDOUT_FILENO);
        dup2(outputPipe[1], STDERR_FILENO);
        close(outputPipe[1]);
        setenv("PATH", "/usr/sbin:/usr/bin:/sbin:/bin", 1);
        setenv("LC_ALL", "C", 1);
        std::vector<char*> argv = makeArgv(arguments);
        execvp(argv[0], argv.data());
        _exit(127);
    }

    close(outputPipe[1]);
    char buffer[4096];
    while (true) {
        const ssize_t count = read(outputPipe[0], buffer, sizeof(buffer));
        if (count > 0) {
            result.output.append(buffer, static_cast<std::size_t>(count));
            continue;
        }
        if (count < 0 && errno == EINTR) continue;
        break;
    }
    close(outputPipe[0]);

    int status = 0;
    while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
    }
    if (WIFEXITED(status)) result.exitCode = WEXITSTATUS(status);
    else if (WIFSIGNALED(status)) result.exitCode = 128 + WTERMSIG(status);
    return result;
}

bool runChecked(const std::vector<std::string>& arguments, std::string& error)
{
    const CommandResult result = runCommand(arguments);
    if (result.exitCode == 0) return true;
    std::ostringstream message;
    message << arguments.front() << " failed with exit code " << result.exitCode;
    const std::string output = trim(result.output);
    if (!output.empty()) message << ": " << output;
    error = message.str();
    return false;
}

std::string findExecutable(const std::string& name)
{
    const std::string paths = "/usr/sbin:/usr/bin:/sbin:/bin";
    std::istringstream pathStream(paths);
    std::string directory;
    while (std::getline(pathStream, directory, ':')) {
        if (directory.empty()) continue;
        const std::string candidate = directory + '/' + name;
        if (access(candidate.c_str(), X_OK) == 0) return candidate;
    }
    return std::string();
}

bool writeFile(const std::string& path, const std::string& contents, std::string& error)
{
    const int descriptor = open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (descriptor < 0) {
        error = "cannot create " + path + ": " + std::strerror(errno);
        return false;
    }
    const char* cursor = contents.data();
    std::size_t remaining = contents.size();
    while (remaining > 0) {
        const ssize_t count = write(descriptor, cursor, remaining);
        if (count > 0) {
            cursor += count;
            remaining -= static_cast<std::size_t>(count);
            continue;
        }
        if (count < 0 && errno == EINTR) continue;
        error = "cannot write " + path + ": " + std::strerror(errno);
        close(descriptor);
        return false;
    }
    if (close(descriptor) != 0) {
        error = "cannot close " + path + ": " + std::strerror(errno);
        return false;
    }
    return true;
}

bool prepareDnsmasqLeaseFile(const std::string& path, std::string& error)
{
    const passwd* account = getpwnam("nobody");
    const group* accountGroup = getgrnam("nogroup");
    if (!account || !accountGroup) {
        error = "cannot resolve the nobody:nogroup account for dnsmasq";
        return false;
    }

    struct stat directoryStatus {};
    if (stat("/run/dnsmasq", &directoryStatus) != 0) {
        if (mkdir("/run/dnsmasq", 0755) != 0 && errno != EEXIST) {
            error = std::string("cannot create /run/dnsmasq: ") + std::strerror(errno);
            return false;
        }
        if (chown("/run/dnsmasq", account->pw_uid, accountGroup->gr_gid) != 0) {
            error = std::string("cannot assign /run/dnsmasq ownership: ") +
                    std::strerror(errno);
            return false;
        }
    }

    const int descriptor = open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0640);
    if (descriptor < 0) {
        error = "cannot create dnsmasq lease file " + path + ": " + std::strerror(errno);
        return false;
    }
    close(descriptor);
    if (chown(path.c_str(), account->pw_uid, accountGroup->gr_gid) != 0 ||
        chmod(path.c_str(), 0640) != 0) {
        error = "cannot assign dnsmasq lease file ownership: " +
                std::string(std::strerror(errno));
        return false;
    }
    return true;
}

std::string readFile(const std::string& path)
{
    std::ifstream input(path.c_str());
    if (!input) return std::string();
    std::ostringstream contents;
    contents << input.rdbuf();
    return contents.str();
}

std::string tailText(const std::string& text, std::size_t maximum)
{
    if (text.size() <= maximum) return trim(text);
    return trim(text.substr(text.size() - maximum));
}

pid_t spawnLoggedProcess(const std::vector<std::string>& arguments,
                         const std::string& logPath,
                         std::string& error)
{
    const int logDescriptor = open(logPath.c_str(),
                                   O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC,
                                   0600);
    if (logDescriptor < 0) {
        error = "cannot create log " + logPath + ": " + std::strerror(errno);
        return -1;
    }

    const pid_t pid = fork();
    if (pid < 0) {
        error = std::string("fork failed: ") + std::strerror(errno);
        close(logDescriptor);
        return -1;
    }
    if (pid == 0) {
        // Do not leave hotspot helpers alive if the controlling program is
        // killed by SIGABRT or another unhandled fatal signal.
        prctl(PR_SET_PDEATHSIG, SIGTERM);
        if (getppid() == 1) _exit(125);
        dup2(logDescriptor, STDOUT_FILENO);
        dup2(logDescriptor, STDERR_FILENO);
        close(logDescriptor);
        setenv("PATH", "/usr/sbin:/usr/bin:/sbin:/bin", 1);
        setenv("LC_ALL", "C", 1);
        std::vector<char*> argv = makeArgv(arguments);
        execv(argv[0], argv.data());
        _exit(127);
    }
    close(logDescriptor);
    return pid;
}

bool processRunning(pid_t pid, int* status = nullptr)
{
    if (pid <= 0) return false;
    siginfo_t information {};
    if (waitid(P_PID, static_cast<id_t>(pid), &information,
               WEXITED | WNOHANG | WNOWAIT) != 0) {
        return errno == ECHILD ? false : kill(pid, 0) == 0;
    }
    if (information.si_pid == 0) return true;
    if (status) *status = information.si_status;
    return false;
}

void stopProcess(pid_t& pid)
{
    if (pid <= 0) return;
    if (kill(pid, SIGTERM) == 0) {
        for (int i = 0; i < 20; ++i) {
            if (!processRunning(pid)) {
                int status = 0;
                while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
                }
                pid = -1;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        kill(pid, SIGKILL);
    }
    int status = 0;
    while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
    }
    pid = -1;
}

bool waitForLogMarker(pid_t pid,
                      const std::string& logPath,
                      const std::string& marker,
                      int timeoutMilliseconds,
                      std::string& error)
{
    const int iterations = timeoutMilliseconds / 100;
    for (int i = 0; i < iterations; ++i) {
        if (readFile(logPath).find(marker) != std::string::npos) return true;
        int status = 0;
        if (!processRunning(pid, &status)) {
            std::ostringstream message;
            message << "process exited before " << marker;
            const std::string log = tailText(readFile(logPath), 2000);
            if (!log.empty()) message << ": " << log;
            error = message.str();
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    error = "timed out waiting for " + marker + "; log: " +
            tailText(readFile(logPath), 2000);
    return false;
}

bool isPrivateIpv4(uint32_t address)
{
    const uint8_t first = static_cast<uint8_t>((address >> 24) & 0xff);
    const uint8_t second = static_cast<uint8_t>((address >> 16) & 0xff);
    return first == 10 ||
           (first == 172 && second >= 16 && second <= 31) ||
           (first == 192 && second == 168);
}

bool startsWith(const std::string& value, const std::string& prefix)
{
    return value.size() >= prefix.size() &&
           value.compare(0, prefix.size(), prefix) == 0;
}

std::string configValue(const std::string& contents, const std::string& key)
{
    std::istringstream lines(contents);
    std::string line;
    const std::string prefix = key + '=';
    while (std::getline(lines, line)) {
        if (startsWith(line, prefix)) return trim(line.substr(prefix.size()));
    }
    return std::string();
}

std::string processCommandLine(pid_t pid)
{
    const std::string path = "/proc/" + std::to_string(static_cast<long long>(pid)) +
                             "/cmdline";
    const int descriptor = open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (descriptor < 0) return std::string();
    std::string result;
    char buffer[4096];
    while (true) {
        const ssize_t count = read(descriptor, buffer, sizeof(buffer));
        if (count > 0) {
            for (ssize_t i = 0; i < count; ++i) {
                result.push_back(buffer[i] == '\0' ? ' ' : buffer[i]);
            }
            continue;
        }
        if (count < 0 && errno == EINTR) continue;
        break;
    }
    close(descriptor);
    return result;
}

bool runtimeDirectoryIsInUse(const std::string& runtimeDirectory)
{
    DIR* proc = opendir("/proc");
    if (!proc) return false;
    bool found = false;
    while (dirent* entry = readdir(proc)) {
        char* end = nullptr;
        errno = 0;
        const long parsed = std::strtol(entry->d_name, &end, 10);
        if (errno != 0 || !end || *end != '\0' || parsed <= 0 ||
            parsed == static_cast<long>(getpid())) {
            continue;
        }
        const pid_t pid = static_cast<pid_t>(parsed);
        const std::string processName = trim(readFile(
            "/proc/" + std::to_string(static_cast<long long>(pid)) + "/comm"));
        if ((processName == "hostapd" || processName == "dnsmasq") &&
            processCommandLine(pid).find(runtimeDirectory) != std::string::npos) {
            found = true;
            break;
        }
    }
    closedir(proc);
    return found;
}

void removeKnownRuntimeFiles(const std::string& runtimeDirectory,
                             const std::string& interfaceName)
{
    const std::size_t slash = runtimeDirectory.find_last_of('/');
    const std::string runtimeName = slash == std::string::npos
        ? runtimeDirectory
        : runtimeDirectory.substr(slash + 1);
    unlink((runtimeDirectory + "/hostapd.conf").c_str());
    unlink((runtimeDirectory + "/dnsmasq.conf").c_str());
    unlink((runtimeDirectory + "/hostapd.log").c_str());
    unlink((runtimeDirectory + "/dnsmasq.log").c_str());
    unlink((runtimeDirectory + "/dnsmasq.leases").c_str());
    unlink((runtimeDirectory + "/dnsmasq.pid").c_str());
    unlink((runtimeDirectory + "/recovery.state").c_str());
    unlink(("/run/dnsmasq/" + runtimeName + ".leases").c_str());
    unlink(("/run/dnsmasq/" + runtimeName + ".pid").c_str());
    const std::string controlDirectory = runtimeDirectory + "/hostapd_ctrl";
    unlink((controlDirectory + '/' + interfaceName).c_str());
    rmdir(controlDirectory.c_str());
    rmdir(runtimeDirectory.c_str());
}

std::string shellRuleValue(const std::string& line, const std::string& option)
{
    const std::size_t optionPosition = line.find(option);
    if (optionPosition == std::string::npos) return std::string();
    std::size_t start = optionPosition + option.size();
    while (start < line.size() && line[start] == ' ') ++start;
    if (start >= line.size()) return std::string();
    const bool quoted = line[start] == '"';
    if (quoted) ++start;
    std::size_t end = start;
    while (end < line.size() &&
           (quoted ? line[end] != '"' : line[end] != ' ')) {
        ++end;
    }
    return line.substr(start, end - start);
}

void removeStaleFirewallRules(const std::string& iptablesExecutable,
                              const std::string& interfaceName)
{
    if (iptablesExecutable.empty()) return;
    const CommandResult listed = runCommand({iptablesExecutable, "-w", "5", "-S", "INPUT"});
    if (listed.exitCode != 0) return;
    std::istringstream lines(listed.output);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.find("-i " + interfaceName + " ") == std::string::npos ||
            line.find("vdsuit-wireless-") == std::string::npos) {
            continue;
        }
        const std::string port = shellRuleValue(line, "--dport");
        const std::string comment = shellRuleValue(line, "--comment");
        if ((port != "67" && port != std::to_string(kProtocolPort)) ||
            !startsWith(comment, "vdsuit-wireless-")) {
            continue;
        }
        runCommand({
            iptablesExecutable, "-w", "5", "-D", "INPUT",
            "-i", interfaceName, "-p", "udp", "--dport", port,
            "-m", "comment", "--comment", comment, "-j", "ACCEPT"
        });
    }
}

std::string addressFromHostOrder(uint32_t address)
{
    in_addr networkAddress {};
    networkAddress.s_addr = htonl(address);
    char text[INET_ADDRSTRLEN];
    if (!inet_ntop(AF_INET, &networkAddress, text, sizeof(text))) return std::string();
    return text;
}

} // namespace

bool deriveHotspotAddressing(const std::string& apAddress,
                             HotspotAddressing& addressing,
                             std::string& error)
{
    error.clear();
    addressing = HotspotAddressing();
    in_addr parsed {};
    if (inet_pton(AF_INET, apAddress.c_str(), &parsed) != 1) {
        error = "invalid IPv4 address";
        return false;
    }

    const uint32_t hostAddress = ntohl(parsed.s_addr);
    if (!isPrivateIpv4(hostAddress)) {
        error = "AP address must be an RFC1918 private IPv4 address";
        return false;
    }
    const uint32_t networkAddress = hostAddress & 0xffffff00U;
    const uint8_t lastOctet = static_cast<uint8_t>(hostAddress & 0xffU);
    if (lastOctet == 0 || lastOctet == 3 || lastOctet == 255) {
        error = "AP address cannot use host octet 0, 3, or 255";
        return false;
    }

    addressing.apAddress = addressFromHostOrder(hostAddress);
    addressing.apCidr = addressing.apAddress + "/24";
    addressing.clientAddress = addressFromHostOrder(networkAddress + 3);
    addressing.networkCidr = addressFromHostOrder(networkAddress) + "/24";
    addressing.broadcastAddress = addressFromHostOrder(networkAddress + 255);
    addressing.netmask = "255.255.255.0";
    return true;
}

bool validateInterfaceName(const std::string& interfaceName)
{
    if (interfaceName.empty() || interfaceName.size() >= IFNAMSIZ) return false;
    for (char value : interfaceName) {
        const bool valid = (value >= 'a' && value <= 'z') ||
                           (value >= 'A' && value <= 'Z') ||
                           (value >= '0' && value <= '9') ||
                           value == '_' || value == '-' || value == '.';
        if (!valid) return false;
    }
    return true;
}

bool validateSerialSsid(const std::string& ssid)
{
    if (ssid.empty() || ssid.size() > 32) return false;
    for (char value : ssid) {
        if (value < '0' || value > '9') return false;
    }
    return true;
}

std::string makeHostapdConfig(const HotspotConfig& config,
                              const std::string& controlDirectory)
{
    std::ostringstream output;
    output
        << "interface=" << config.interfaceName << '\n'
        << "driver=nl80211\n"
        << "ctrl_interface=" << controlDirectory << '\n'
        << "ssid=" << config.ssid << '\n'
        << "country_code=" << config.countryCode << '\n'
        << "ieee80211d=1\n"
        << "hw_mode=a\n"
        << "channel=" << config.channel << '\n'
        << "ieee80211n=1\n"
        << "wmm_enabled=1\n"
        << "auth_algs=1\n"
        << "ignore_broadcast_ssid=0\n"
        << "max_num_sta=1\n"
        << "wpa=2\n"
        << "wpa_passphrase=" << config.passphrase << '\n'
        << "wpa_key_mgmt=WPA-PSK\n"
        << "rsn_pairwise=CCMP\n";
    return output.str();
}

std::string makeDnsmasqConfig(const HotspotConfig& config,
                              const std::string& leaseFile,
                              const std::string& pidFile)
{
    std::ostringstream output;
    output
        << "interface=" << config.interfaceName << '\n'
        << "bind-dynamic\n"
        << "port=0\n"
        << "no-hosts\n"
        << "no-resolv\n"
        << "dhcp-authoritative\n"
        << "dhcp-range=" << config.addressing.clientAddress << ','
        << config.addressing.clientAddress << ','
        << config.addressing.netmask << ",18h\n"
        << "dhcp-option=1," << config.addressing.netmask << '\n'
        << "dhcp-option=3," << config.addressing.apAddress << '\n'
        << "dhcp-option=6,0.0.0.0\n"
        << "dhcp-option=28,255.255.255.255\n"
        << "dhcp-leasefile=" << leaseFile << '\n'
        << "pid-file=" << pidFile << '\n'
        << "log-dhcp\n"
        << "log-facility=-\n";
    return output.str();
}

HotspotManager::HotspotManager(const HotspotConfig& config)
    : config_(config)
{
}

HotspotManager::~HotspotManager()
{
    stop();
}

bool HotspotManager::acquireInstanceLock(std::string& error)
{
    error.clear();
    instanceLockPath_ = "/run/vdsuit-hotspot-" + config_.interfaceName + ".lock";
    instanceLockFd_ = open(instanceLockPath_.c_str(),
                           O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (instanceLockFd_ < 0) {
        error = "cannot open hotspot instance lock " + instanceLockPath_ + ": " +
                std::strerror(errno);
        return false;
    }
    if (flock(instanceLockFd_, LOCK_EX | LOCK_NB) != 0) {
        error = "another VDSuit hotspot process is already using " +
                config_.interfaceName;
        close(instanceLockFd_);
        instanceLockFd_ = -1;
        return false;
    }
    return true;
}

void HotspotManager::releaseInstanceLock()
{
    if (instanceLockFd_ < 0) return;
    unlink(instanceLockPath_.c_str());
    flock(instanceLockFd_, LOCK_UN);
    close(instanceLockFd_);
    instanceLockFd_ = -1;
    instanceLockPath_.clear();
}

bool HotspotManager::recoverStaleState(std::string& error)
{
    error.clear();
    DIR* runDirectory = opendir("/run");
    if (!runDirectory) {
        error = std::string("cannot inspect /run for stale hotspots: ") +
                std::strerror(errno);
        return false;
    }

    std::vector<std::string> staleDirectories;
    while (dirent* entry = readdir(runDirectory)) {
        const std::string name = entry->d_name;
        if (!startsWith(name, "vdsuit-hotspot-")) continue;
        const std::string path = "/run/" + name;
        const std::string configuredInterface =
            configValue(readFile(path + "/hostapd.conf"), "interface");
        if (configuredInterface != config_.interfaceName) continue;
        if (runtimeDirectoryIsInUse(path)) {
            closedir(runDirectory);
            error = "another VDSuit hotspot still appears active at " + path;
            return false;
        }
        staleDirectories.push_back(path);
    }
    closedir(runDirectory);
    if (staleDirectories.empty()) return true;

    const CommandResult managed = runCommand({
        "nmcli", "-g", "GENERAL.NM-MANAGED", "device", "show", config_.interfaceName
    });
    if (managed.exitCode != 0) {
        error = "cannot inspect NetworkManager during stale recovery: " +
                trim(managed.output);
        return false;
    }

    removeStaleFirewallRules(iptablesExecutable_, config_.interfaceName);
    for (const std::string& directory : staleDirectories) {
        removeKnownRuntimeFiles(directory, config_.interfaceName);
    }

    if (trim(managed.output) != "yes") {
        runCommand({"ip", "address", "flush", "dev", config_.interfaceName});
        runCommand({"ip", "link", "set", "dev", config_.interfaceName, "down"});
        const CommandResult restored = runCommand({
            "nmcli", "device", "set", config_.interfaceName, "managed", "yes"
        });
        if (restored.exitCode != 0) {
            error = "could not recover " + config_.interfaceName +
                    " after the previous abnormal exit: " + trim(restored.output);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    std::cout << "Recovered " << staleDirectories.size()
              << " stale VDSuit hotspot instance(s) on "
              << config_.interfaceName << ".\n";
    return true;
}

bool HotspotManager::stopStaleHelperProcesses(std::string& error)
{
    error.clear();
    DIR* proc = opendir("/proc");
    if (!proc) {
        error = std::string("cannot inspect /proc for stale hotspot helpers: ") +
                std::strerror(errno);
        return false;
    }

    std::vector<pid_t> stalePids;
    while (dirent* entry = readdir(proc)) {
        char* end = nullptr;
        errno = 0;
        const long parsed = std::strtol(entry->d_name, &end, 10);
        if (errno != 0 || !end || *end != '\0' || parsed <= 0 ||
            parsed == static_cast<long>(getpid())) {
            continue;
        }
        const pid_t pid = static_cast<pid_t>(parsed);
        const std::string processName = trim(readFile(
            "/proc/" + std::to_string(static_cast<long long>(pid)) + "/comm"));
        if (processName != "hostapd" && processName != "dnsmasq") continue;
        if (processCommandLine(pid).find("/run/vdsuit-hotspot-") ==
            std::string::npos) {
            continue;
        }
        stalePids.push_back(pid);
    }
    closedir(proc);

    for (pid_t pid : stalePids) {
        if (kill(pid, SIGTERM) != 0 && errno != ESRCH) {
            error = "cannot terminate stale VDSuit hotspot helper PID " +
                    std::to_string(static_cast<long long>(pid)) + ": " +
                    std::strerror(errno);
            return false;
        }
    }
    for (pid_t pid : stalePids) {
        bool stopped = false;
        for (int attempt = 0; attempt < 20; ++attempt) {
            if (kill(pid, 0) != 0 && errno == ESRCH) {
                stopped = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (!stopped) kill(pid, SIGKILL);
    }
    if (!stalePids.empty()) {
        std::cout << "Stopped " << stalePids.size()
                  << " stale VDSuit hotspot helper process(es).\n";
    }
    return true;
}

bool HotspotManager::captureNetworkManagerState(std::string& error)
{
    const CommandResult managed = runCommand({
        "nmcli", "-g", "GENERAL.NM-MANAGED", "device", "show", config_.interfaceName
    });
    if (managed.exitCode != 0) {
        error = "NetworkManager cannot inspect " + config_.interfaceName + ": " +
                trim(managed.output);
        return false;
    }
    wasManaged_ = trim(managed.output) == "yes";
    if (!wasManaged_) {
        error = config_.interfaceName + " must be managed by NetworkManager before startup";
        return false;
    }

    const CommandResult active = runCommand({
        "nmcli", "-t", "-f", "UUID,DEVICE", "connection", "show", "--active"
    });
    if (active.exitCode != 0) {
        error = "NetworkManager cannot list active connections: " + trim(active.output);
        return false;
    }
    std::istringstream lines(active.output);
    std::string line;
    while (std::getline(lines, line)) {
        const std::size_t separator = line.rfind(':');
        if (separator == std::string::npos) continue;
        if (line.substr(separator + 1) == config_.interfaceName) {
            previousConnectionUuid_ = line.substr(0, separator);
            break;
        }
    }
    return true;
}

bool HotspotManager::startRecoveryWatchdog(std::string& error)
{
    error.clear();
    int descriptors[2];
    if (pipe2(descriptors, O_CLOEXEC) != 0) {
        error = std::string("cannot create hotspot recovery watchdog pipe: ") +
                std::strerror(errno);
        return false;
    }

    const pid_t pid = fork();
    if (pid < 0) {
        error = std::string("cannot start hotspot recovery watchdog: ") +
                std::strerror(errno);
        close(descriptors[0]);
        close(descriptors[1]);
        return false;
    }
    if (pid == 0) {
        close(descriptors[1]);
        char marker = 0;
        ssize_t count = -1;
        do {
            count = read(descriptors[0], &marker, 1);
        } while (count < 0 && errno == EINTR);
        close(descriptors[0]);
        if (count == 1 && marker == 'N') _exit(0);
        emergencyRestoreFromWatchdog();
        _exit(0);
    }

    close(descriptors[0]);
    recoveryWatchdogPid_ = pid;
    recoveryWatchdogWriteFd_ = descriptors[1];
    return true;
}

void HotspotManager::finishRecoveryWatchdog()
{
    if (recoveryWatchdogWriteFd_ >= 0) {
        const char marker = 'N';
        ssize_t result = -1;
        do {
            result = write(recoveryWatchdogWriteFd_, &marker, 1);
        } while (result < 0 && errno == EINTR);
        close(recoveryWatchdogWriteFd_);
        recoveryWatchdogWriteFd_ = -1;
    }
    if (recoveryWatchdogPid_ > 0) {
        int status = 0;
        while (waitpid(recoveryWatchdogPid_, &status, 0) < 0 && errno == EINTR) {
        }
        recoveryWatchdogPid_ = -1;
    }
}

void HotspotManager::emergencyRestoreFromWatchdog()
{
    // The hotspot children receive SIGTERM through PR_SET_PDEATHSIG. Give them
    // a short head start before removing their files and network state.
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    const auto removeRule = [this](const std::string& port) {
        if (iptablesExecutable_.empty()) return;
        runCommand({
            iptablesExecutable_, "-w", "5", "-D", "INPUT",
            "-i", config_.interfaceName, "-p", "udp", "--dport", port,
            "-m", "comment", "--comment", firewallComment_, "-j", "ACCEPT"
        });
    };
    removeRule(std::to_string(kProtocolPort));
    removeRule("67");

    runCommand({"ip", "address", "flush", "dev", config_.interfaceName});
    runCommand({"ip", "link", "set", "dev", config_.interfaceName, "down"});
    if (wasManaged_) {
        runCommand({"nmcli", "device", "set", config_.interfaceName, "managed", "yes"});
        if (!previousConnectionUuid_.empty()) {
            runCommand({
                "nmcli", "--wait", "15", "connection", "up", "uuid",
                previousConnectionUuid_, "ifname", config_.interfaceName
            });
        }
    }
    preserveDiagnosticLogs();
    removeKnownRuntimeFiles(runtimeDirectory_, config_.interfaceName);
    if (!instanceLockPath_.empty()) unlink(instanceLockPath_.c_str());
}

void HotspotManager::preserveDiagnosticLogs()
{
    std::string ignoredError;
    const std::string hostapdLog = readFile(hostapdLogPath_);
    if (!hostapdLog.empty()) {
        writeFile("/run/vdsuit-hotspot-last-hostapd.log", hostapdLog, ignoredError);
    }
    const std::string dnsmasqLog = readFile(dnsmasqLogPath_);
    if (!dnsmasqLog.empty()) {
        writeFile("/run/vdsuit-hotspot-last-dnsmasq.log", dnsmasqLog, ignoredError);
    }
}

bool HotspotManager::releaseInterface(std::string& error)
{
    if (!previousConnectionUuid_.empty()) {
        if (!runChecked({"nmcli", "--wait", "15", "connection", "down", "uuid",
                         previousConnectionUuid_}, error)) {
            return false;
        }
    }
    // From this point onward cleanup must restore the previous profile even if
    // one of the remaining interface operations fails.
    interfaceReleased_ = true;
    if (!runChecked({"nmcli", "device", "set", config_.interfaceName,
                     "managed", "no"}, error)) {
        return false;
    }
    if (!runChecked({"ip", "link", "set", "dev", config_.interfaceName, "down"}, error) ||
        !runChecked({"ip", "address", "flush", "dev", config_.interfaceName}, error) ||
        !runChecked({"ip", "link", "set", "dev", config_.interfaceName, "up"}, error)) {
        return false;
    }
    return true;
}

bool HotspotManager::writeRuntimeFiles(std::string& error)
{
    char directoryTemplate[] = "/run/vdsuit-hotspot-XXXXXX";
    char* directory = mkdtemp(directoryTemplate);
    if (!directory) {
        error = std::string("cannot create runtime directory: ") + std::strerror(errno);
        return false;
    }
    runtimeDirectory_ = directory;
    chmod(runtimeDirectory_.c_str(), 0700);

    hostapdConfigPath_ = runtimeDirectory_ + "/hostapd.conf";
    dnsmasqConfigPath_ = runtimeDirectory_ + "/dnsmasq.conf";
    const std::size_t finalSlash = runtimeDirectory_.find_last_of('/');
    const std::string runtimeName = runtimeDirectory_.substr(finalSlash + 1);
    leaseFilePath_ = "/run/dnsmasq/" + runtimeName + ".leases";
    dnsmasqPidPath_ = "/run/dnsmasq/" + runtimeName + ".pid";
    hostapdLogPath_ = runtimeDirectory_ + "/hostapd.log";
    dnsmasqLogPath_ = runtimeDirectory_ + "/dnsmasq.log";
    const std::string controlDirectory = runtimeDirectory_ + "/hostapd_ctrl";
    if (mkdir(controlDirectory.c_str(), 0700) != 0) {
        error = "cannot create hostapd control directory: " + std::string(std::strerror(errno));
        return false;
    }

    return prepareDnsmasqLeaseFile(leaseFilePath_, error) &&
           writeFile(hostapdConfigPath_, makeHostapdConfig(config_, controlDirectory), error) &&
           writeFile(dnsmasqConfigPath_,
                     makeDnsmasqConfig(config_, leaseFilePath_, dnsmasqPidPath_),
                     error);
}

bool HotspotManager::startHostapd(std::string& error)
{
    hostapdPid_ = spawnLoggedProcess({hostapdExecutable_, hostapdConfigPath_},
                                     hostapdLogPath_, error);
    if (hostapdPid_ <= 0) return false;
    if (!waitForLogMarker(hostapdPid_, hostapdLogPath_, "AP-ENABLED", 10000, error)) {
        stopProcess(hostapdPid_);
        return false;
    }
    return true;
}

bool HotspotManager::configureAddress(std::string& error)
{
    return runChecked({"ip", "address", "add", config_.addressing.apCidr,
                       "dev", config_.interfaceName}, error) &&
           runChecked({"ip", "link", "set", "dev", config_.interfaceName, "up"}, error);
}

bool HotspotManager::configureFirewall(std::string& error)
{
    const std::vector<std::string> base = {
        iptablesExecutable_, "-w", "5", "-I", "INPUT", "1",
        "-i", config_.interfaceName, "-p", "udp"
    };
    std::vector<std::string> dhcpRule = base;
    dhcpRule.insert(dhcpRule.end(), {
        "--dport", "67", "-m", "comment", "--comment", firewallComment_,
        "-j", "ACCEPT"
    });
    if (!runChecked(dhcpRule, error)) return false;
    dhcpFirewallRule_ = true;

    std::vector<std::string> protocolRule = base;
    protocolRule.insert(protocolRule.end(), {
        "--dport", std::to_string(kProtocolPort),
        "-m", "comment", "--comment", firewallComment_, "-j", "ACCEPT"
    });
    if (!runChecked(protocolRule, error)) return false;
    protocolFirewallRule_ = true;
    return true;
}

void HotspotManager::removeFirewallRules()
{
    if (iptablesExecutable_.empty()) return;
    const auto removeRule = [this](const std::string& port) {
        runCommand({
            iptablesExecutable_, "-w", "5", "-D", "INPUT",
            "-i", config_.interfaceName, "-p", "udp", "--dport", port,
            "-m", "comment", "--comment", firewallComment_, "-j", "ACCEPT"
        });
    };
    if (protocolFirewallRule_) removeRule(std::to_string(kProtocolPort));
    if (dhcpFirewallRule_) removeRule("67");
    protocolFirewallRule_ = false;
    dhcpFirewallRule_ = false;
}

bool HotspotManager::startDnsmasq(std::string& error)
{
    dnsmasqPid_ = spawnLoggedProcess({dnsmasqExecutable_, "--keep-in-foreground",
                                      "--conf-file=" + dnsmasqConfigPath_},
                                     dnsmasqLogPath_, error);
    if (dnsmasqPid_ <= 0) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    int status = 0;
    if (!processRunning(dnsmasqPid_, &status)) {
        error = "dnsmasq exited during startup: " + tailText(readFile(dnsmasqLogPath_), 2000);
        stopProcess(dnsmasqPid_);
        return false;
    }
    return true;
}

bool HotspotManager::start(std::string& error)
{
    error.clear();
    if (active_) return true;
    if (geteuid() != 0) {
        error = "run this program as root (sudo)";
        return false;
    }
    if (!validateInterfaceName(config_.interfaceName) ||
        if_nametoindex(config_.interfaceName.c_str()) == 0) {
        error = "wireless interface does not exist or has an invalid name";
        return false;
    }
    if (!validateSerialSsid(config_.ssid)) {
        error = "SSID/serial must contain 1-32 decimal digits";
        return false;
    }

    hostapdExecutable_ = findExecutable("hostapd");
    dnsmasqExecutable_ = findExecutable("dnsmasq");
    iptablesExecutable_ = findExecutable("iptables");
    if (hostapdExecutable_.empty()) {
        error = "hostapd is not installed or executable";
        return false;
    }
    if (dnsmasqExecutable_.empty()) {
        error = "dnsmasq is not installed or executable";
        return false;
    }
    if (iptablesExecutable_.empty()) {
        error = "iptables is not installed or executable";
        return false;
    }
    firewallComment_ = "vdsuit-wireless-" + std::to_string(static_cast<long long>(getpid()));

    if (!acquireInstanceLock(error)) return false;
    if (!stopStaleHelperProcesses(error)) return false;
    if (!recoverStaleState(error)) return false;

    const CommandResult conflictingRoute = runCommand({
        "ip", "-4", "route", "show", config_.addressing.networkCidr
    });
    if (conflictingRoute.exitCode != 0) {
        error = "cannot inspect IPv4 routes: " + trim(conflictingRoute.output);
        return false;
    }
    if (!trim(conflictingRoute.output).empty() &&
        conflictingRoute.output.find("dev " + config_.interfaceName) == std::string::npos) {
        error = "another interface already routes " + config_.addressing.networkCidr;
        return false;
    }

    if (!captureNetworkManagerState(error) ||
        !writeRuntimeFiles(error) ||
        !startRecoveryWatchdog(error) ||
        !runChecked({dnsmasqExecutable_, "--test",
                     "--conf-file=" + dnsmasqConfigPath_}, error) ||
        !releaseInterface(error) ||
        !startHostapd(error) ||
        !configureAddress(error) ||
        !configureFirewall(error) ||
        !startDnsmasq(error)) {
        stop();
        return false;
    }
    active_ = true;
    return true;
}

bool HotspotManager::waitForClient(const std::function<bool()>& shouldStop,
                                   ClientLease& lease,
                                   std::string& error)
{
    error.clear();
    lease = ClientLease();
    while (!shouldStop()) {
        if (!processRunning(hostapdPid_)) {
            error = "hostapd stopped: " + tailText(readFile(hostapdLogPath_), 2000);
            return false;
        }
        if (!processRunning(dnsmasqPid_)) {
            error = "dnsmasq stopped: " + tailText(readFile(dnsmasqLogPath_), 2000);
            return false;
        }

        std::istringstream lines(readFile(leaseFilePath_));
        std::string line;
        while (std::getline(lines, line)) {
            std::istringstream fields(line);
            std::string expiry;
            ClientLease candidate;
            if (!(fields >> expiry >> candidate.macAddress >> candidate.ipAddress)) continue;
            if (candidate.ipAddress == config_.addressing.clientAddress) {
                lease = candidate;
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    error = "stopped while waiting for transmitter DHCP lease";
    return false;
}

void HotspotManager::restoreInterface()
{
    if (!interfaceReleased_) return;
    runCommand({"ip", "address", "flush", "dev", config_.interfaceName});
    runCommand({"ip", "link", "set", "dev", config_.interfaceName, "down"});
    if (wasManaged_) {
        const CommandResult managed = runCommand({
            "nmcli", "device", "set", config_.interfaceName, "managed", "yes"
        });
        if (managed.exitCode != 0) {
            std::cerr << "Warning: could not return " << config_.interfaceName
                      << " to NetworkManager: " << trim(managed.output) << '\n';
        } else if (!previousConnectionUuid_.empty()) {
            const CommandResult restored = runCommand({
                "nmcli", "--wait", "15", "connection", "up", "uuid",
                previousConnectionUuid_, "ifname", config_.interfaceName
            });
            if (restored.exitCode != 0) {
                std::cerr << "Warning: could not restore the previous "
                          << config_.interfaceName << " connection: "
                          << trim(restored.output) << '\n';
            }
        }
    }
    interfaceReleased_ = false;
}

void HotspotManager::removeRuntimeFiles()
{
    if (runtimeDirectory_.empty()) return;
    unlink(hostapdConfigPath_.c_str());
    unlink(dnsmasqConfigPath_.c_str());
    unlink(leaseFilePath_.c_str());
    unlink(dnsmasqPidPath_.c_str());
    unlink(hostapdLogPath_.c_str());
    unlink(dnsmasqLogPath_.c_str());
    const std::string controlDirectory = runtimeDirectory_ + "/hostapd_ctrl";
    const std::string controlSocket = controlDirectory + '/' + config_.interfaceName;
    unlink(controlSocket.c_str());
    rmdir(controlDirectory.c_str());
    rmdir(runtimeDirectory_.c_str());
    runtimeDirectory_.clear();
}

void HotspotManager::stop()
{
    active_ = false;
    stopProcess(dnsmasqPid_);
    stopProcess(hostapdPid_);
    removeFirewallRules();
    restoreInterface();
    preserveDiagnosticLogs();
    removeRuntimeFiles();
    finishRecoveryWatchdog();
    releaseInstanceLock();
}

} // namespace vdsuit_wireless
