#include "bvh_exporter.h"
#include "calibration_backup.h"
#include "mag_calibration.h"
#include "network_streamer.h"
#include "sdk_virtual_serial.h"
#include "wireless_hotspot.h"
#include "wireless_sdk_bridge.h"
#include "vdsuit_wireless_protocol.h"

#include <atomic>
#include <cmath>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <dlfcn.h>
#include <execinfo.h>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <memory>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/select.h>
#include <termios.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace VDSuitMiniDevice;

// Keep our console UI visible even when vendor SDK writes noisy printf/stdout logs.
// After this macro, existing std::cout calls become std::cerr calls.
#define cout cerr

namespace {

void redirectVendorStdout()
{
    const char* keep = std::getenv("VDSUIT_KEEP_SDK_STDOUT");
    if (keep && std::strcmp(keep, "0") != 0) return;

    std::fflush(stdout);
    if (std::freopen("/tmp/vdsuit_sdk_stdout.log", "a", stdout)) {
        std::setvbuf(stdout, nullptr, _IOLBF, 0);
    }
}


struct AppOptions {
    std::string libPath;
    std::string outDir = "records";
    int frequency = 60;
    int durationSeconds = 0;
    BvhExportMode mode = BvhExportMode::FullHands;
    bool prependStudioRestFrame = true;
    std::string sendIp;
    uint16_t sendPort = 0;
    std::string sendHandIp;
    uint16_t sendHandPort = 0;
    std::string wirelessSsid;
    std::string wirelessInterface = "wlan0";
    std::string wirelessApAddress = "192.168.18.1";
    bool wirelessOptionSeen = false;
};

class RawTerminalGuard {
public:
    RawTerminalGuard()
    {
        enabled_ = isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &oldTerm_) == 0;
        if (!enabled_) return;

        termios raw = oldTerm_;
        raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

    ~RawTerminalGuard()
    {
        if (enabled_) tcsetattr(STDIN_FILENO, TCSANOW, &oldTerm_);
    }

private:
    bool enabled_ = false;
    termios oldTerm_ {};
};

class SdkApi {
public:
    using MocapDataCallback = void (*)(_MocapData_ md);
    using MocapDataWithVirtualCallback = void (*)(_MocapDataWithVirtual_ md);
    using DeviceBreakCallback = void (*)();

    using SetVDMocapDataCallBackFunc = void (*)(MocapDataCallback);
    using SetVDMocapDataWithVirtualCallBackFunc = void (*)(MocapDataWithVirtualCallback);
    using SetDeviceBreakCallBackFunc = void (*)(DeviceBreakCallback);
    using InitialFunc = void (*)(_WorldSpace_, float[NODES_BODY][3], float[NODES_HAND][3], float[NODES_HAND][3], int);
    using ConnectFunc = bool (*)();
    using DisConnectFunc = void (*)();
    using RecvMocapDataFunc = void (*)(_MocapData_*);
    using GetConnectStateFunc = bool (*)();
    using GetDeviceTypeFunc = _DeviceType_ (*)();
    using GetFrequencyFunc = int (*)();
    using SetFrequencyFunc = bool (*)(_Frequency_);
    using GetDevicePowerFunc = float (*)();
    using GetGestureFunc = void (*)(_Gesture_*, _Gesture_*);
    using StartCalibrationFunc = bool (*)(_CalibrationMode_, float[4]);
    using StartCalibrationFastFunc = bool (*)(_CalibrationMode_, float[4]);
    using CancelCalibrationFunc = void (*)();
    using GetCalibrationProgressFunc = _CalibrationProgress_ (*)();
    using StartMagCorrectFunc = bool (*)();
    using EndMagCorrectFunc = bool (*)();
    using CancelMagCorrectFunc = void (*)();
    using GetMagCorrectResultFunc = bool (*)(_MagCorrectResult_*);

    ~SdkApi()
    {
        if (handle_) {
            cleanupConnection();
            clearCallbacks();
            vdsuit_wireless::setSdkLibraryHandle(nullptr);
            dlclose(handle_);
        }
    }

    bool load(const std::string& path)
    {
        // Use lazy binding because some vendor SDK builds contain optional
        // gesture/XML helper symbols that are not needed by the mocap recorder.
        // RTLD_NOW may fail immediately with unresolved XmlCQHand/XmlCQ* symbols.
        handle_ = dlopen(path.c_str(), RTLD_LAZY);
        if (!handle_) {
            std::cerr << "dlopen failed: " << dlerror() << "\n";
            return false;
        }
        vdsuit_wireless::setSdkLibraryHandle(handle_);

        bool ok = true;
        ok &= loadRequired(setVDMocapDataCallBackFunc, "SetVDMocapDataCallBackFunc");
        ok &= loadRequired(setVDMocapDataWithVirtualCallBackFunc, "SetVDMocapDataWithVirtualCallBackFunc");
        ok &= loadRequired(setDeviceBreakCallBackFunc, "SetDeviceBreakCallBackFunc");
        ok &= loadRequired(initial, "Initial");
        ok &= loadRequired(connectDevice, "Connect");
        ok &= loadRequired(disConnect, "DisConnect");
        ok &= loadRequired(recvMocapData, "RecvMocapData");
        ok &= loadRequired(getConnectState, "GetConnectState");
        ok &= loadRequired(getDeviceType, "GetDeviceType");
        ok &= loadRequired(getFrequency, "GetFrequency");
        ok &= loadRequired(setFrequency, "SetFrequency");
        ok &= loadRequired(getDevicePower, "GetDevicePower");

        loadOptional(getGesture, "GetGesture");
        loadOptional(startCalibration, "StartCalibration");
        loadOptional(startCalibrationFast, "StartCalibrationFast");
        loadOptional(cancelCalibration, "CancelCalibration");
        loadOptional(getCalibrationProgress, "GetCalibrationProgress");
        loadOptional(startMagCorrect, "StartMagCorrect");
        loadOptional(endMagCorrect, "EndMagCorrect");
        loadOptional(cancelMagCorrect, "CancelMagCorrect");
        loadOptional(getMagCorrectResult, "GetMagCorrectResult");

        if (!ok) {
            vdsuit_wireless::setSdkLibraryHandle(nullptr);
            dlclose(handle_);
            handle_ = nullptr;
        }
        return ok;
    }

    void markConnectionTouched()
    {
        connectionTouched_ = true;
    }

    bool connectionTouched() const
    {
        return connectionTouched_;
    }

    bool cleanupConnection()
    {
        if (!handle_ || !connectionTouched_ || !disConnect) return false;
        // After a transport loss the SDK runs its own break cleanup. Calling
        // DisConnect again in that state races the vendor read thread and has
        // been observed to trip glibc's fortified-buffer abort path.
        const bool sdkAlreadyDisconnected =
            successfulConnection_ && getConnectState && !getConnectState();
        if (!sdkAlreadyDisconnected) {
            disConnect();
        }
        connectionTouched_ = false;
        successfulConnection_ = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        return true;
    }

    void markConnected()
    {
        successfulConnection_ = true;
    }

    SetVDMocapDataCallBackFunc setVDMocapDataCallBackFunc = nullptr;
    SetVDMocapDataWithVirtualCallBackFunc setVDMocapDataWithVirtualCallBackFunc = nullptr;
    SetDeviceBreakCallBackFunc setDeviceBreakCallBackFunc = nullptr;
    InitialFunc initial = nullptr;
    ConnectFunc connectDevice = nullptr;
    DisConnectFunc disConnect = nullptr;
    RecvMocapDataFunc recvMocapData = nullptr;
    GetConnectStateFunc getConnectState = nullptr;
    GetDeviceTypeFunc getDeviceType = nullptr;
    GetFrequencyFunc getFrequency = nullptr;
    SetFrequencyFunc setFrequency = nullptr;
    GetDevicePowerFunc getDevicePower = nullptr;
    GetGestureFunc getGesture = nullptr;
    StartCalibrationFunc startCalibration = nullptr;
    StartCalibrationFastFunc startCalibrationFast = nullptr;
    CancelCalibrationFunc cancelCalibration = nullptr;
    GetCalibrationProgressFunc getCalibrationProgress = nullptr;
    StartMagCorrectFunc startMagCorrect = nullptr;
    EndMagCorrectFunc endMagCorrect = nullptr;
    CancelMagCorrectFunc cancelMagCorrect = nullptr;
    GetMagCorrectResultFunc getMagCorrectResult = nullptr;

private:
    void clearCallbacks()
    {
        if (!handle_) return;
        if (setVDMocapDataCallBackFunc) setVDMocapDataCallBackFunc(nullptr);
        if (setVDMocapDataWithVirtualCallBackFunc) setVDMocapDataWithVirtualCallBackFunc(nullptr);
        if (setDeviceBreakCallBackFunc) setDeviceBreakCallBackFunc(nullptr);
    }

    template <typename T>
    bool loadRequired(T& out, const char* name)
    {
        dlerror();
        out = reinterpret_cast<T>(dlsym(handle_, name));
        const char* error = dlerror();
        if (error || !out) {
            std::cerr << "missing required SDK symbol: " << name << "\n";
            return false;
        }
        return true;
    }

    template <typename T>
    void loadOptional(T& out, const char* name)
    {
        dlerror();
        out = reinterpret_cast<T>(dlsym(handle_, name));
        dlerror();
    }

    void* handle_ = nullptr;
    bool connectionTouched_ = false;
    bool successfulConnection_ = false;
};

BvhExporter g_exporter;
std::atomic<NetworkStreamer*> g_networkStreamer(nullptr);
std::atomic<NetworkStreamer*> g_networkHandStreamer(nullptr);
std::atomic<bool> g_connected(false);
std::atomic<bool> g_showData(false);
std::atomic<uint64_t> g_sdkFrameCount(0);
std::atomic<bool> g_disconnectInProgress(false);
volatile std::sig_atomic_t g_shutdownSignal = 0;
volatile std::sig_atomic_t g_fatalSignalHandling = 0;
std::mutex g_latestMocapMutex;
_MocapDataWithVirtual_ g_latestMocap {};
std::chrono::steady_clock::time_point g_latestMocapTime {};
bool g_haveLatestMocap = false;

bool stopNetworkForwardingIfActive()
{
    NetworkStreamer* streamer = g_networkStreamer.exchange(nullptr, std::memory_order_acq_rel);
    if (!streamer) return false;
    streamer->stop();
    return true;
}

bool stopHandForwardingIfActive()
{
    NetworkStreamer* streamer = g_networkHandStreamer.exchange(nullptr, std::memory_order_acq_rel);
    if (!streamer) return false;
    streamer->stop();
    return true;
}

float g_initialBody[NODES_BODY][3] = {
    {0, 0, 1.11f}, {0.105f, 0, 1.022f}, {0.105f, 0, 0.566f}, {0.105f, 0, 0.099f}, {0.105f, 0.121f, 0.019f},
    {-0.105f, 0, 1.022f}, {-0.105f, 0, 0.566f}, {-0.105f, 0, 0.099f}, {-0.105f, 0.121f, 0.019f},
    {0, 0, 1.207f}, {0, 0, 1.319f}, {0, 0, 1.44f}, {0, 0.012f, 1.557f}, {0, 0, 1.677f}, {0, 0, 1.777f},
    {0.049f, 0, 1.597f}, {0.205f, 0, 1.597f}, {0.464f, 0, 1.597f}, {0.748f, 0, 1.597f},
    {-0.049f, 0, 1.597f}, {-0.205f, 0, 1.597f}, {-0.464f, 0, 1.597f}, {-0.748f, 0, 1.597f}
};

float g_initialRightHand[NODES_HAND][3] = {
    {0.748f, 0, 1.597f}, {0.782f, 0.042f, 1.6f}, {0.817f, 0.077f, 1.6f}, {0.842f, 0.101f, 1.6f},
    {0.792f, 0.026f, 1.604f}, {0.862f, 0.04f, 1.603f}, {0.912f, 0.04f, 1.601f}, {0.939f, 0.04f, 1.599f},
    {0.794f, 0.01f, 1.604f}, {0.864f, 0.014f, 1.603f}, {0.917f, 0.014f, 1.6f}, {0.951f, 0.014f, 1.597f},
    {0.793f, -0.001f, 1.605f}, {0.856f, -0.008f, 1.604f}, {0.903f, -0.008f, 1.6f}, {0.935f, -0.008f, 1.597f},
    {0.791f, -0.016f, 1.604f}, {0.847f, -0.031f, 1.603f}, {0.884f, -0.031f, 1.601f}, {0.908f, -0.031f, 1.6f}
};

float g_initialLeftHand[NODES_HAND][3] = {
    {-0.748f, 0, 1.597f}, {-0.782f, 0.042f, 1.6f}, {-0.817f, 0.077f, 1.6f}, {-0.842f, 0.102f, 1.6f},
    {-0.792f, 0.026f, 1.604f}, {-0.862f, 0.04f, 1.603f}, {-0.912f, 0.04f, 1.601f}, {-0.939f, 0.04f, 1.599f},
    {-0.794f, 0.01f, 1.604f}, {-0.864f, 0.014f, 1.603f}, {-0.917f, 0.014f, 1.6f}, {-0.951f, 0.014f, 1.597f},
    {-0.793f, -0.001f, 1.605f}, {-0.856f, -0.008f, 1.604f}, {-0.903f, -0.008f, 1.601f}, {-0.935f, -0.008f, 1.599f},
    {-0.791f, -0.016f, 1.604f}, {-0.847f, -0.031f, 1.603f}, {-0.884f, -0.03f, 1.601f}, {-0.908f, -0.031f, 1.6f}
};

void onMocapData(_MocapData_ md)
{
    if (!g_showData || !md.isUpdate) return;
    std::cout << "\rframe=" << md.frameIndex
              << " battery=" << static_cast<int>(md.devicePower * 100.0f + 0.5f) << "%   " << std::flush;
}

void onMocapDataWithVirtual(_MocapDataWithVirtual_ md)
{
    if (md.isUpdate) {
        g_sdkFrameCount.fetch_add(1, std::memory_order_relaxed);
        std::lock_guard<std::mutex> lock(g_latestMocapMutex);
        g_latestMocap = md;
        g_latestMocapTime = std::chrono::steady_clock::now();
        g_haveLatestMocap = true;
    }

    NetworkStreamer* streamer = g_networkStreamer.load(std::memory_order_acquire);
    if (streamer) streamer->enqueueFrame(md);
    NetworkStreamer* handStreamer = g_networkHandStreamer.load(std::memory_order_acquire);
    if (handStreamer) handStreamer->enqueueFrame(md);
    g_exporter.addFrame(md);
    if (!g_showData || !md.isUpdate) return;
    std::cout << "\rframe=" << md.frameIndex
              << " battery=" << static_cast<int>(md.devicePower * 100.0f + 0.5f) << "%   " << std::flush;
}

void onDeviceBreak()
{
    g_connected = false;
    if (g_disconnectInProgress.load(std::memory_order_acquire)) return;
    stopNetworkForwardingIfActive();
    stopHandForwardingIfActive();
    std::cout << "\nDevice disconnected unexpectedly.\n";
}

void handleShutdownSignal(int signal)
{
    if (g_shutdownSignal == 0) g_shutdownSignal = signal;
}

void handleFatalSignal(int signalNumber)
{
    if (g_fatalSignalHandling) _exit(128 + signalNumber);
    g_fatalSignalHandling = 1;
    const int descriptor = open(
        "/run/vdsuit-bvh-exporter-crash.log",
        O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC | O_NOFOLLOW,
        0644);
    if (descriptor >= 0) {
        const char header[] = "\n=== fatal signal in vdsuit_bvh_exporter ===\n";
        const ssize_t headerResult = write(descriptor, header, sizeof(header) - 1);
        (void)headerResult;
        void* frames[64];
        const int count = backtrace(frames, 64);
        backtrace_symbols_fd(frames, count, descriptor);
        close(descriptor);
    }
    signal(signalNumber, SIG_DFL);
    kill(getpid(), signalNumber);
    _exit(128 + signalNumber);
}

void installSignalHandlers()
{
    struct sigaction action {};
    action.sa_handler = handleShutdownSignal;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
    sigaction(SIGHUP, &action, nullptr);

    struct sigaction fatalAction {};
    fatalAction.sa_handler = handleFatalSignal;
    sigemptyset(&fatalAction.sa_mask);
    fatalAction.sa_flags = SA_RESETHAND;
    sigaction(SIGABRT, &fatalAction, nullptr);
}

bool shutdownRequested()
{
    return g_shutdownSignal != 0;
}

int shutdownExitCode()
{
    return g_shutdownSignal == 0 ? 0 : 128 + g_shutdownSignal;
}

void resetLatestMocap()
{
    std::lock_guard<std::mutex> lock(g_latestMocapMutex);
    g_latestMocap = _MocapDataWithVirtual_ {};
    g_latestMocapTime = std::chrono::steady_clock::time_point {};
    g_haveLatestMocap = false;
    g_sdkFrameCount.store(0, std::memory_order_relaxed);
}

class WirelessRuntime {
public:
    ~WirelessRuntime()
    {
        vdsuit_wireless::clearSdkVirtualSerialPath();
        if (bridge_) bridge_->close();
        if (hotspot_) hotspot_->stop();
    }

    bool start(const AppOptions& options, std::string& error)
    {
        if (options.wirelessSsid.empty()) return true;
        if (geteuid() != 0) {
            error = "wireless SDK mode must run as root (use sudo)";
            return false;
        }
        if (!vdsuit_wireless::validateInterfaceName(options.wirelessInterface)) {
            error = "invalid wireless interface name";
            return false;
        }
        if (!vdsuit_wireless::validateSerialSsid(options.wirelessSsid)) {
            error = "wireless SSID/serial must contain 1-32 decimal digits";
            return false;
        }

        vdsuit_wireless::HotspotConfig hotspotConfig;
        hotspotConfig.interfaceName = options.wirelessInterface;
        hotspotConfig.ssid = options.wirelessSsid;
        if (!vdsuit_wireless::deriveHotspotAddressing(
                options.wirelessApAddress, hotspotConfig.addressing, error)) {
            error = "invalid wireless AP address: " + error;
            return false;
        }

        std::cout << "Starting isolated SDK hotspot on "
                  << hotspotConfig.interfaceName << "...\n";
        hotspot_.reset(new vdsuit_wireless::HotspotManager(hotspotConfig));
        if (!hotspot_->start(error)) {
            error = "hotspot startup failed: " + error;
            return false;
        }
        std::cout << "Hotspot ready: SSID=" << hotspotConfig.ssid
                  << ", password=" << hotspotConfig.passphrase
                  << ", channel=" << hotspotConfig.channel
                  << ", AP=" << hotspotConfig.addressing.apCidr
                  << ", DHCP=" << hotspotConfig.addressing.clientAddress << "\n"
                  << "Waiting for the transmitter to associate and obtain DHCP...\n";

        if (!hotspot_->waitForClient(shutdownRequested, lease_, error)) {
            if (shutdownRequested()) error = "shutdown requested while waiting for transmitter";
            else error = "transmitter wait failed: " + error;
            return false;
        }
        std::cout << "Transmitter ready: MAC=" << lease_.macAddress
                  << ", IP=" << lease_.ipAddress << "\n";

        vdsuit_wireless::WirelessSdkBridgeConfig bridgeConfig;
        bridgeConfig.interfaceName = hotspotConfig.interfaceName;
        bridgeConfig.localAddress = hotspotConfig.addressing.apAddress;
        bridgeConfig.clientAddress = hotspotConfig.addressing.clientAddress;
        // The physical receiver uses the limited broadcast address exactly;
        // preserve that behavior instead of relying on subnet-broadcast rules.
        bridgeConfig.destinationAddress = "255.255.255.255";
        bridgeConfig.port = vdsuit_wireless::kProtocolPort;
        bridge_.reset(new vdsuit_wireless::WirelessSdkBridge(bridgeConfig));
        if (!bridge_->open(error)) {
            error = "wireless SDK bridge startup failed: " + error;
            return false;
        }
        vdsuit_wireless::setSdkVirtualSerialPath(bridge_->slavePath());
        std::cout << "SDK wireless bridge ready: UDP "
                  << hotspotConfig.addressing.apAddress << ':' << bridge_->localPort()
                  << " <-> " << bridge_->slavePath() << "\n";
        return true;
    }

    bool transmitterDataStale(int64_t& ageMilliseconds) const
    {
        if (!bridge_ || !bridge_->active()) {
            ageMilliseconds = -1;
            return false;
        }
        ageMilliseconds = bridge_->millisecondsSinceLastNetworkFrame();
        return ageMilliseconds < 0 || ageMilliseconds > 500;
    }

private:
    std::unique_ptr<vdsuit_wireless::HotspotManager> hotspot_;
    std::unique_ptr<vdsuit_wireless::WirelessSdkBridge> bridge_;
    vdsuit_wireless::ClientLease lease_;
};

WirelessRuntime* g_wirelessRuntime = nullptr;

bool fileExists(const std::string& path)
{
    return access(path.c_str(), R_OK) == 0;
}

std::vector<std::string> defaultLibCandidates()
{
#if defined(__aarch64__)
    return {
        "./lib/arm64/libVDMocapSDK_miniArm64.so",
        "./lib/arm64/libVDMocapSDK_VDSuitMiniArm64.so",
        "../vdsuitfull-sdktest/lib/arm64/libVDMocapSDK_miniArm64.so",
        "../vdsuitfull-sdktest/lib/ubuntu22.04_arm64/libVDMocapSDK_VDSuitMiniArm64.so",
        "../vdsuitfull-sdktest/lib/ubuntu20.04_arm64/libVDMocapSDK_VDSuitMiniArm64.so",
        "../so/ubuntu22.04_arm64/libVDMocapSDK_miniArm64.so",
        "../so/ubuntu20.04_arm64/libVDMocapSDK_miniArm64.so"
    };
#else
    return {
        "./lib/x64/libVDMocapSDK_mini.so",
        "./lib/x64/libVDMocapSDK_VDSuitMini.so",
        "../vdsuitfull-sdktest/lib/x64/libVDMocapSDK_mini.so",
        "../vdsuitfull-sdktest/lib/ubuntu22.04_x64/libVDMocapSDK_VDSuitMini.so",
        "../vdsuitfull-sdktest/lib/ubuntu20.04_x64/libVDMocapSDK_VDSuitMini.so",
        "../so/ubuntu22.04_x64/libVDMocapSDK_mini.so",
        "../so/ubuntu20.04_x64/libVDMocapSDK_mini.so"
    };
#endif
}

std::string chooseLibPath(const std::string& explicitPath)
{
    if (!explicitPath.empty()) return explicitPath;
    for (const std::string& path : defaultLibCandidates()) {
        if (fileExists(path)) return path;
    }
    return defaultLibCandidates().front();
}

std::string timestamp()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm localTime{};
    localtime_r(&t, &localTime);
    std::ostringstream oss;
    oss << std::put_time(&localTime, "%Y%m%d_%H%M%S");
    return oss.str();
}

bool ensureDirectory(const std::string& path)
{
    if (path.empty()) return false;
    std::string current;
    for (char ch : path) {
        current += ch;
        if (ch == '/') {
            if (current.size() > 1) mkdir(current.c_str(), 0755);
        }
    }
    if (mkdir(path.c_str(), 0755) == 0) return true;
    struct stat st {};
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

std::string makeRecordPath(const std::string& outDir, BvhExportMode mode)
{
    ensureDirectory(outDir);
    std::string prefix = mode == BvhExportMode::FullHands ? "fullhands" : "bodyonly";
    std::string separator = (!outDir.empty() && outDir.back() == '/') ? "" : "/";
    return outDir + separator + prefix + "_record_" + timestamp() + ".bvh";
}

const char* deviceTypeName(_DeviceType_ type)
{
    switch (type) {
    case DT_VDSuitMini: return "VDSuitMini";
    case DT_VDSuitFull: return "VDSuitFull";
    default: return "Unknown";
    }
}

const char* calibrationStateName(_CalibrationState_ state)
{
    switch (state) {
    case CS_UnStart: return "not-started";
    case CS_InPose: return "in-pose";
    case CS_Successed: return "success";
    case CS_Failed: return "failed";
    default: return "unknown";
    }
}

bool hasMagCalibrationApi(const SdkApi& sdk)
{
    return vdsuit::magCalibrationFunctionsAvailable(
        sdk.startMagCorrect != nullptr,
        sdk.endMagCorrect != nullptr,
        sdk.cancelMagCorrect != nullptr,
        sdk.getMagCorrectResult != nullptr);
}

void printFailedMagNodes(const char* label, const std::vector<std::string>& nodes)
{
    if (nodes.empty()) return;
    std::cout << "  " << label << ": ";
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << nodes[i];
    }
    std::cout << "\n";
}

void reportCalibrationBackups(const vdsuit::CalibrationBackupResult& result,
                              const std::string& directory)
{
    for (const std::string& path : result.savedPaths) {
        std::cout << "Calibration parameter snapshot saved: " << path << "\n";
    }
    for (const std::string& error : result.errors) {
        std::cout << "Warning: calibration parameter snapshot failed: " << error << "\n";
    }
    if (!result.anyFileChanged && result.errors.empty()) {
        std::cout << "Warning: calibration succeeded, but no CQ XML changed in "
                  << directory << "; no timestamped snapshot was created.\n";
    }
}

struct Vec3f {
    float x;
    float y;
    float z;
};

Vec3f bodyPosition(const _MocapDataWithVirtual_& md, _BodyNodes_ node)
{
    return {
        md.position_body[node][0],
        md.position_body[node][1],
        md.position_body[node][2]
    };
}

Vec3f sub(Vec3f a, Vec3f b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

bool isFinite(Vec3f v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool hasBodyTracking(const _MocapDataWithVirtual_& md, _BodyNodes_ node)
{
    _SensorState_ state = md.sensorState_body[node];
    return state != SS_NONE && state != SS_NoData;
}

float xyLength(Vec3f v)
{
    return std::sqrt(v.x * v.x + v.y * v.y);
}

bool getLatestMocapSnapshot(_MocapDataWithVirtual_& out, std::chrono::milliseconds maxAge)
{
    std::lock_guard<std::mutex> lock(g_latestMocapMutex);
    if (!g_haveLatestMocap) return false;
    if (std::chrono::steady_clock::now() - g_latestMocapTime > maxAge) return false;
    out = g_latestMocap;
    return true;
}

bool isPposeGuardDisabled()
{
    const char* value = std::getenv("VDSUIT_SKIP_PPOSE_GUARD");
    return value && std::strcmp(value, "0") != 0;
}

bool looksLikePpose(const _MocapDataWithVirtual_& md, std::string& reason)
{
    if (!md.isUpdate) {
        reason = "no updated mocap frame";
        return false;
    }

    if (!hasBodyTracking(md, BN_RightUpperArm) ||
        !hasBodyTracking(md, BN_RightLowerArm) ||
        !hasBodyTracking(md, BN_LeftUpperArm) ||
        !hasBodyTracking(md, BN_LeftLowerArm)) {
        reason = "arm sensor data unavailable";
        return false;
    }

    Vec3f rightShoulder = bodyPosition(md, BN_RightShoulder);
    Vec3f leftShoulder = bodyPosition(md, BN_LeftShoulder);
    Vec3f rightHand = bodyPosition(md, BN_RightHand);
    Vec3f leftHand = bodyPosition(md, BN_LeftHand);
    if (!isFinite(rightShoulder) || !isFinite(leftShoulder) ||
        !isFinite(rightHand) || !isFinite(leftHand)) {
        reason = "invalid body positions";
        return false;
    }

    Vec3f shoulderLine = sub(rightShoulder, leftShoulder);
    float shoulderSpan = xyLength(shoulderLine);
    if (shoulderSpan < 0.05f) {
        reason = "shoulder direction unavailable";
        return false;
    }

    float forwardX = -shoulderLine.y / shoulderSpan;
    float forwardY = shoulderLine.x / shoulderSpan;

    auto sidePasses = [&](Vec3f shoulder, Vec3f hand,
                          float& forwardMeters, float& verticalDelta, float& xyReach) {
        Vec3f rel = sub(hand, shoulder);
        forwardMeters = rel.x * forwardX + rel.y * forwardY;
        verticalDelta = std::fabs(rel.z);
        xyReach = xyLength(rel);
        return forwardMeters >= 0.22f &&
               verticalDelta <= 0.40f &&
               xyReach >= 0.28f;
    };

    float rightForward = 0.0f;
    float rightVertical = 0.0f;
    float rightReach = 0.0f;
    float leftForward = 0.0f;
    float leftVertical = 0.0f;
    float leftReach = 0.0f;
    bool rightOk = sidePasses(rightShoulder, rightHand, rightForward, rightVertical, rightReach);
    bool leftOk = sidePasses(leftShoulder, leftHand, leftForward, leftVertical, leftReach);
    if (!rightOk || !leftOk) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "R forward=" << rightForward << "m dz=" << rightVertical << "m"
            << ", L forward=" << leftForward << "m dz=" << leftVertical << "m";
        reason = oss.str();
        return false;
    }

    reason.clear();
    return true;
}

int readKeyNonBlocking()
{
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    timeval timeout {};
    int result = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout);
    if (result <= 0) return -1;

    unsigned char ch = 0;
    if (read(STDIN_FILENO, &ch, 1) == 1) return ch;
    return -1;
}

bool waitForLocalPposeReady()
{
    if (isPposeGuardDisabled()) {
        std::cout << "Local P-pose guard disabled by VDSUIT_SKIP_PPOSE_GUARD.\n";
        return true;
    }

    constexpr int requiredStableSamples = 8;
    int stableSamples = 0;
    std::cout << "Waiting for real P-pose before starting SDK calibration.\n";

    while (true) {
        if (shutdownRequested()) {
            std::cout << "\nCalibration canceled by shutdown.\n";
            return false;
        }

        int key = readKeyNonBlocking();
        if (key == 'q' || key == 'Q') {
            std::cout << "\nCalibration canceled before SDK start.\n";
            return false;
        }

        _MocapDataWithVirtual_ md {};
        std::string reason = "no recent mocap frame";
        bool ok = getLatestMocapSnapshot(md, std::chrono::milliseconds(800)) &&
                  looksLikePpose(md, reason);
        stableSamples = ok ? stableSamples + 1 : 0;
        int guardProgress = stableSamples * 100 / requiredStableSamples;
        if (guardProgress > 100) guardProgress = 100;

        std::cout << "\rP-pose guard="
                  << (ok ? "ready" : "waiting")
                  << " stable=" << guardProgress << "%";
        if (!ok) std::cout << " (" << reason << ")";
        std::cout << "          " << std::flush;

        if (stableSamples >= requiredStableSamples) {
            std::cout << "\nP-pose guard passed.\n";
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void printUsage()
{
    std::cout
        << "Usage: ./bin/arm64/vdsuit_bvh_exporter [options]\n"
        << "Options:\n"
        << "  --lib PATH             SDK .so path\n"
        << "  --out DIR              output directory, default: records\n"
        << "  --freq N               set device frequency: 60, 72, 80, 96\n"
        << "  --duration SEC         auto record for SEC seconds then save and exit\n"
        << "  --send IP:PORT         configure UDP body target; press S to start\n"
        << "  --send-hand IP:PORT    configure UDP hand target; press H to start\n"
        << "  --wireless-ssid SERIAL receive through a local Wi-Fi hotspot\n"
        << "  --wireless-ip ADDRESS  wireless AP address, default: 192.168.18.1\n"
        << "  --wireless-interface N wireless AP interface, default: wlan0\n"
        << "  --body-only            export body-only BVH\n"
        << "  --no-rest-frame        do not prepend Studio rest frame\n"
        << "  --help                 show this help\n";
}

bool parseArgs(int argc, char** argv, AppOptions& options)
{
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") {
            printUsage();
            return false;
        } else if (arg == "--lib" && i + 1 < argc) {
            options.libPath = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            options.outDir = argv[++i];
        } else if (arg == "--freq" && i + 1 < argc) {
            options.frequency = std::stoi(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            options.durationSeconds = std::stoi(argv[++i]);
        } else if (arg == "--send" && i + 1 < argc) {
            std::string error;
            if (!parseNetworkEndpoint(argv[++i], options.sendIp, options.sendPort, error)) {
                std::cerr << "Invalid --send endpoint: " << error << "\n";
                return false;
            }
        } else if (arg == "--send-hand" && i + 1 < argc) {
            std::string error;
            if (!parseNetworkEndpoint(argv[++i], options.sendHandIp, options.sendHandPort, error)) {
                std::cerr << "Invalid --send-hand endpoint: " << error << "\n";
                return false;
            }
        } else if (arg == "--wireless-ssid" && i + 1 < argc) {
            options.wirelessSsid = argv[++i];
            options.wirelessOptionSeen = true;
        } else if (arg == "--wireless-ip" && i + 1 < argc) {
            options.wirelessApAddress = argv[++i];
            options.wirelessOptionSeen = true;
        } else if (arg == "--wireless-interface" && i + 1 < argc) {
            options.wirelessInterface = argv[++i];
            options.wirelessOptionSeen = true;
        } else if (arg == "--body-only") {
            options.mode = BvhExportMode::BodyOnly;
        } else if (arg == "--no-rest-frame") {
            options.prependStudioRestFrame = false;
        } else {
            std::cerr << "Unknown or incomplete argument: " << arg << "\n";
            printUsage();
            return false;
        }
    }
    if (options.wirelessOptionSeen && options.wirelessSsid.empty()) {
        std::cerr << "--wireless-ssid is required when wireless options are used.\n";
        return false;
    }
    return true;
}

_Frequency_ toSdkFrequency(int frequency)
{
    switch (frequency) {
    case 72: return HZ_72;
    case 80: return HZ_80;
    case 96: return HZ_96;
    case 60:
    default: return HZ_60;
    }
}

bool tryConnectDeviceOnce(SdkApi& sdk)
{
    resetLatestMocap();
    sdk.initial(WS_Geo, g_initialBody, g_initialRightHand, g_initialLeftHand, 0);
    sdk.markConnectionTouched();
    return sdk.connectDevice();
}

bool connectDevice(SdkApi& sdk, const AppOptions& options)
{
    g_connected = false;
    g_exporter.cancel();

    if (sdk.connectionTouched()) {
        sdk.cleanupConnection();
    }

    bool connected = tryConnectDeviceOnce(sdk) && sdk.getConnectState();
    if (!connected && !shutdownRequested()) {
        std::cerr << "Connect failed once. Resetting SDK connection state and retrying...\n";
        sdk.cleanupConnection();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        connected = tryConnectDeviceOnce(sdk) && sdk.getConnectState();
    }

    if (shutdownRequested() || !connected) {
        sdk.cleanupConnection();
        if (options.wirelessSsid.empty()) {
            std::cerr << "Connect failed. Check USB, serial permission, and driver.\n";
        } else {
            std::cerr << "Connect failed. Check the transmitter, hotspot, and UDP bridge.\n";
        }
        return false;
    }

    g_connected = true;
    sdk.markConnected();
    const int negotiatedFrequency = sdk.getFrequency();
    if (negotiatedFrequency != options.frequency &&
        !sdk.setFrequency(toSdkFrequency(options.frequency))) {
        std::cerr << "Connected, but changing frequency from "
                  << negotiatedFrequency << "Hz to " << options.frequency
                  << "Hz failed. Disconnecting this incomplete session.\n";
        g_connected = false;
        sdk.cleanupConnection();
        return false;
    }
    std::cout << "Connected: " << deviceTypeName(sdk.getDeviceType())
              << ", frequency=" << sdk.getFrequency()
              << "Hz, battery=" << static_cast<int>(sdk.getDevicePower() * 100.0f + 0.5f) << "%\n";
    return true;
}

void disconnectDevice(SdkApi& sdk, bool quiet = false)
{
    g_disconnectInProgress.store(true, std::memory_order_release);
    bool wasConnected = g_connected.exchange(false);
    bool wasForwarding = stopNetworkForwardingIfActive();
    bool wasHandForwarding = stopHandForwardingIfActive();
    g_exporter.cancel();

    int64_t wirelessFrameAge = 0;
    if (sdk.connectionTouched() && g_wirelessRuntime &&
        g_wirelessRuntime->transmitterDataStale(wirelessFrameAge)) {
        std::cerr << "Wireless transmitter data stopped";
        if (wirelessFrameAge >= 0) {
            std::cerr << " " << wirelessFrameAge << " ms ago";
        }
        std::cerr << "; exiting without the vendor SDK's unsafe DisConnect path.\n"
                  << "The recovery watchdog is restoring the hotspot and wlan interface.\n"
                  << std::flush;
        _exit(0);
    }

    bool cleaned = sdk.cleanupConnection();
    g_disconnectInProgress.store(false, std::memory_order_release);
    resetLatestMocap();
    if (!quiet && (wasConnected || cleaned || wasForwarding || wasHandForwarding)) {
        std::cout << "Disconnected.\n";
    }
}

bool startRecording(SdkApi& sdk)
{
    if (!g_connected) {
        std::cout << "Please connect first.\n";
        return false;
    }
    int frequency = sdk.getFrequency();
    if (frequency <= 0) frequency = 60;
    g_exporter.begin(frequency);
    std::cout << "Recording started. frequency=" << frequency << "Hz\n";
    return true;
}

bool stopAndSave(const AppOptions& options)
{
    if (!g_exporter.isRecording()) {
        std::cout << "Not recording.\n";
        return false;
    }
    BvhExportMode mode = g_exporter.mode();
    std::string path = makeRecordPath(options.outDir, mode);
    int framesBeforeSave = g_exporter.outputFrameCount();
    if (!g_exporter.saveToFile(path)) {
        std::cerr << "Save failed: " << path << "\n";
        return false;
    }
    std::cout << "Saved " << framesBeforeSave << " frames -> " << path << "\n";
    return true;
}

void runAutoRecord(SdkApi& sdk, const AppOptions& options)
{
    if (!connectDevice(sdk, options)) return;
    startRecording(sdk);
    std::cout << "Auto recording for " << options.durationSeconds << " seconds...\n";
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(options.durationSeconds);
    while (!shutdownRequested() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (shutdownRequested()) {
        g_exporter.cancel();
    } else {
        stopAndSave(options);
    }
    disconnectDevice(sdk);
}

void toggleNetworkForwarding(NetworkStreamer* streamer)
{
    if (!streamer) {
        std::cout << "Configure a target first with --send IP:PORT.\n";
        return;
    }

    if (streamer->isRunning()) {
        stopNetworkForwardingIfActive();
        std::cout << "UDP forwarding stopped.\n";
        return;
    }

    if (!g_connected) {
        std::cout << "Please connect the device before starting UDP forwarding.\n";
        return;
    }

    streamer->start();
    g_networkStreamer.store(streamer, std::memory_order_release);
    std::cout << "UDP forwarding started.\n";
}

void toggleHandForwarding(NetworkStreamer* streamer)
{
    if (!streamer) {
        std::cout << "Configure a hand target first with --send-hand IP:PORT.\n";
        return;
    }

    if (streamer->isRunning()) {
        stopHandForwardingIfActive();
        std::cout << "Hand UDP forwarding stopped.\n";
        return;
    }

    if (!g_connected) {
        std::cout << "Please connect the device before starting hand UDP forwarding.\n";
        return;
    }

    streamer->start();
    g_networkHandStreamer.store(streamer, std::memory_order_release);
    std::cout << "Hand UDP forwarding started.\n";
}

void showMenu(const SdkApi& sdk,
              const AppOptions& options,
              const NetworkStreamer* streamer,
              const NetworkStreamer* handStreamer)
{
    std::cout << "\n==== VDSuitFull Linux BVH Exporter ====\n";
    if (!options.wirelessSsid.empty()) {
        std::cout << "Input: wireless, SSID=" << options.wirelessSsid
                  << ", AP=" << options.wirelessApAddress
                  << ", interface=" << options.wirelessInterface << "\n";
    } else {
        std::cout << "Input: physical USB receiver\n";
    }
    if (g_connected) {
        std::cout << "Status: connected, " << sdk.getFrequency() << "Hz, battery="
                  << static_cast<int>(sdk.getDevicePower() * 100.0f + 0.5f) << "%\n";
        std::cout << "SDK output: "
                  << g_sdkFrameCount.load(std::memory_order_relaxed)
                  << " processed frames\n";
    } else {
        std::cout << "Status: disconnected\n";
    }
    std::cout << "BVH: " << (g_exporter.mode() == BvhExportMode::FullHands ? "FullHands" : "BodyOnly")
              << ", rest frame=" << (g_exporter.prependStudioRestFrame() ? "on" : "off")
              << ", recording=" << (g_exporter.isRecording() ? "yes" : "no") << "\n";
    if (streamer) {
        std::cout << "Body UDP: target=" << options.sendIp << ':' << options.sendPort
                  << ", forwarding=" << (streamer->isRunning() ? "yes" : "no") << "\n";
    } else {
        std::cout << "Body UDP: target not configured\n";
    }
    if (handStreamer) {
        std::cout << "Hand UDP: target=" << options.sendHandIp << ':' << options.sendHandPort
                  << ", forwarding=" << (handStreamer->isRunning() ? "yes" : "no") << "\n";
    } else {
        std::cout << "Hand UDP: target not configured\n";
    }
    std::cout << "1. Connect\n";
    std::cout << "2. Disconnect\n";
    std::cout << "3. Start/stop BVH recording\n";
    std::cout << "4. Cancel recording\n";
    std::cout << "5. Toggle BVH mode\n";
    std::cout << "6. Toggle Studio rest frame\n";
    std::cout << "7. Toggle data print\n";
    std::cout << "8. Show gesture\n";
    std::cout << "9. A-pose calibration\n";
    std::cout << "P. P-pose calibration\n";
    std::cout << "M. Magnetometer calibration\n";
    std::cout << "S. Start/stop body UDP forwarding\n";
    std::cout << "H. Start/stop hand UDP forwarding\n";
    std::cout << "0. Exit\n";
    std::cout << "Select: ";
}

void showGesture(SdkApi& sdk)
{
    if (!g_connected) {
        std::cout << "Please connect first.\n";
        return;
    }
    if (!sdk.getGesture) {
        std::cout << "GetGesture is not exported by this SDK.\n";
        return;
    }
    _Gesture_ right = GESTURE_NONE;
    _Gesture_ left = GESTURE_NONE;
    sdk.getGesture(&right, &left);
    std::cout << "Gesture right=" << static_cast<int>(right)
              << " left=" << static_cast<int>(left) << "\n";
}

void doCalibration(SdkApi& sdk, _CalibrationMode_ mode)
{
    if (!g_connected) {
        std::cout << "Please connect first.\n";
        return;
    }
    if ((!sdk.startCalibrationFast && !sdk.startCalibration) ||
        !sdk.getCalibrationProgress ||
        !sdk.cancelCalibration) {
        std::cout << "Calibration symbols are not exported by this SDK library.\n";
        return;
    }

    const char* modeName = mode == CM_Apose ? "A-pose" : "P-pose";
    float quatEndCalibrationRoot[4] = {0, 0, 0, 0};

    std::cout << "\nStarting " << modeName << " calibration.\n";
    std::cout << "Hold the pose steady. Press Q to cancel.\n";

    RawTerminalGuard rawTerminal;
    if (mode == CM_Ppose && !waitForLocalPposeReady()) return;

    const std::string calibrationDirectory =
        vdsuit::calibrationDirectoryForCurrentExecutable();
    const std::vector<vdsuit::CalibrationFileState> calibrationFilesBefore =
        calibrationDirectory.empty()
            ? std::vector<vdsuit::CalibrationFileState>()
            : vdsuit::captureCalibrationFileStates(calibrationDirectory);

    bool started = false;
    if (sdk.startCalibration) {
        started = sdk.startCalibration(mode, quatEndCalibrationRoot);
    } else {
        std::cout << "StartCalibration is unavailable; falling back to fast calibration.\n";
        started = sdk.startCalibrationFast(mode, quatEndCalibrationRoot);
    }

    if (!started) {
        std::cout << "Calibration start failed.\n";
        return;
    }

    while (true) {
        if (shutdownRequested()) {
            sdk.cancelCalibration();
            std::cout << "\nCalibration canceled by shutdown.\n";
            return;
        }

        int key = readKeyNonBlocking();
        if (key == 'q' || key == 'Q') {
            sdk.cancelCalibration();
            std::cout << "\nCalibration canceled.\n";
            return;
        }

        _CalibrationProgress_ progress = sdk.getCalibrationProgress();
        std::cout << "\rCalibration " << modeName
                  << " state=" << calibrationStateName(progress.state)
                  << " progress=" << static_cast<int>(progress.progress * 100.0f + 0.5f)
                  << "%   " << std::flush;

        if (progress.state == CS_Successed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            vdsuit::CalibrationBackupResult backupResult =
                vdsuit::backupChangedCalibrationFiles(
                    calibrationDirectory,
                    calibrationFilesBefore,
                    vdsuit::calibrationSnapshotTimestamp());
            reportCalibrationBackups(backupResult, calibrationDirectory);

            if (mode == CM_Ppose && !isPposeGuardDisabled()) {
                _MocapDataWithVirtual_ md {};
                std::string reason = "no recent mocap frame";
                if (!getLatestMocapSnapshot(md, std::chrono::milliseconds(800)) ||
                    !looksLikePpose(md, reason)) {
                    sdk.cancelCalibration();
                    std::cout << "\nSDK reported success, but local P-pose guard rejected the final pose: "
                              << reason << "\n";
                    std::cout << "Hold both arms forward and horizontal, then run P-pose calibration again.\n";
                    return;
                }
            }
            std::cout << "\nCalibration succeeded.\n";
            return;
        }
        if (progress.state == CS_Failed) {
            std::cout << "\nCalibration failed. Try again after holding the pose steadily.\n";
            return;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void doMagCalibration(SdkApi& sdk)
{
    if (!g_connected) {
        std::cout << "Please connect first.\n";
        return;
    }
    if (!hasMagCalibrationApi(sdk)) {
        std::cout << "Magnetometer calibration symbols are not exported by this SDK library.\n";
        return;
    }
    if (g_exporter.isRecording()) {
        std::cout << "Stop BVH recording before magnetometer calibration.\n";
        return;
    }

    std::cout << "\nStarting magnetometer calibration.\n";
    std::cout << "Move away from magnets, motors, speakers, and large steel objects.\n";
    std::cout << "Slowly rotate the body, arms, hands, and fingers through varied 3D orientations.\n";
    std::cout << "Press E when every sensor has been moved sufficiently; press Q to cancel.\n";

    RawTerminalGuard rawTerminal;
    if (!sdk.startMagCorrect()) {
        std::cout << "Magnetometer calibration start failed.\n";
        return;
    }

    auto collectionStart = std::chrono::steady_clock::now();
    while (true) {
        if (shutdownRequested()) {
            sdk.cancelMagCorrect();
            std::cout << "\nMagnetometer calibration canceled by shutdown.\n";
            return;
        }
        if (!g_connected) {
            sdk.cancelMagCorrect();
            std::cout << "\nMagnetometer calibration canceled because the device disconnected.\n";
            return;
        }

        int key = readKeyNonBlocking();
        if (key == 'q' || key == 'Q') {
            sdk.cancelMagCorrect();
            std::cout << "\nMagnetometer calibration canceled.\n";
            return;
        }
        if (key == 'e' || key == 'E') break;

        long elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - collectionStart).count();
        std::cout << "\rCollecting magnetic samples: " << elapsed
                  << "s (E=end, Q=cancel)   " << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nFinishing sample collection and calculating corrections.\n";
    if (!sdk.endMagCorrect()) {
        sdk.cancelMagCorrect();
        std::cout << "Magnetometer calibration could not start calculation.\n";
        return;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
    while (true) {
        if (shutdownRequested()) {
            sdk.cancelMagCorrect();
            std::cout << "\nMagnetometer calibration canceled by shutdown.\n";
            return;
        }
        if (!g_connected) {
            sdk.cancelMagCorrect();
            std::cout << "\nMagnetometer calibration canceled because the device disconnected.\n";
            return;
        }

        int key = readKeyNonBlocking();
        if (key == 'q' || key == 'Q') {
            sdk.cancelMagCorrect();
            std::cout << "\nMagnetometer calibration canceled.\n";
            return;
        }

        _MagCorrectResult_ result {};
        bool finished = sdk.getMagCorrectResult(&result);
        std::cout << "\rCalculating magnetometer corrections: "
                  << vdsuit::magCalibrationProgressPercent(result.progress)
                  << "% (Q=cancel)   " << std::flush;

        if (finished != result.isFinished) {
            sdk.cancelMagCorrect();
            std::cout << "\nMagnetometer calibration returned an inconsistent SDK result.\n";
            return;
        }
        if (finished) {
            std::cout << "\n";
            vdsuit::MagCalibrationSummary summary =
                vdsuit::summarizeMagCalibrationResult(result);
            switch (summary.outcome) {
            case vdsuit::MagCalibrationOutcome::Success:
                std::cout << "Magnetometer calibration succeeded for all detected sensors.\n";
                return;
            case vdsuit::MagCalibrationOutcome::PartialSuccess:
                std::cout << "Magnetometer calibration partially succeeded. Failed sensors:\n";
                printFailedMagNodes("Body", summary.failedBodyNodes);
                printFailedMagNodes("Right hand", summary.failedRightHandNodes);
                printFailedMagNodes("Left hand", summary.failedLeftHandNodes);
                std::cout << "Move the failed sensors through more orientations, then run M again.\n";
                return;
            case vdsuit::MagCalibrationOutcome::Failed:
                std::cout << "Magnetometer calibration failed: no sensor was calibrated successfully.\n";
                return;
            case vdsuit::MagCalibrationOutcome::InvalidResult:
                sdk.cancelMagCorrect();
                std::cout << "Magnetometer calibration failed: " << summary.error << ".\n";
                return;
            case vdsuit::MagCalibrationOutcome::InProgress:
                sdk.cancelMagCorrect();
                std::cout << "Magnetometer calibration returned an incomplete SDK result.\n";
                return;
            }
        }

        if (std::chrono::steady_clock::now() >= deadline) {
            sdk.cancelMagCorrect();
            std::cout << "\nMagnetometer calibration timed out after 120 seconds.\n";
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void runInteractive(SdkApi& sdk,
                    const AppOptions& options,
                    NetworkStreamer* streamer,
                    NetworkStreamer* handStreamer)
{
    std::string line;
    while (!shutdownRequested()) {
        showMenu(sdk, options, streamer, handStreamer);
        if (!std::getline(std::cin, line)) break;
        if (shutdownRequested()) break;
        if (line.empty()) continue;
        switch (line[0]) {
        case '1':
            if (g_connected) std::cout << "Already connected.\n";
            else connectDevice(sdk, options);
            break;
        case '2':
            disconnectDevice(sdk);
            break;
        case '3':
            if (g_exporter.isRecording()) stopAndSave(options);
            else startRecording(sdk);
            break;
        case '4':
            g_exporter.cancel();
            std::cout << "Recording canceled.\n";
            break;
        case '5':
            if (g_exporter.isRecording()) {
                std::cout << "Stop recording before switching mode.\n";
            } else {
                g_exporter.setMode(g_exporter.mode() == BvhExportMode::FullHands
                    ? BvhExportMode::BodyOnly
                    : BvhExportMode::FullHands);
            }
            break;
        case '6':
            if (g_exporter.isRecording()) {
                std::cout << "Stop recording before switching rest frame.\n";
            } else {
                g_exporter.setPrependStudioRestFrame(!g_exporter.prependStudioRestFrame());
            }
            break;
        case '7':
            g_showData = !g_showData;
            std::cout << "Data print " << (g_showData ? "on" : "off") << "\n";
            break;
        case '8':
            showGesture(sdk);
            break;
        case '9':
        case 'A':
        case 'a':
            doCalibration(sdk, CM_Apose);
            break;
        case 'P':
        case 'p':
            doCalibration(sdk, CM_Ppose);
            break;
        case 'M':
        case 'm':
            doMagCalibration(sdk);
            break;
        case 'S':
        case 's':
            toggleNetworkForwarding(streamer);
            break;
        case 'H':
        case 'h':
            toggleHandForwarding(handStreamer);
            break;
        case '0':
            disconnectDevice(sdk);
            return;
        default:
            std::cout << "Unknown command.\n";
            break;
        }
    }
}

} // namespace

int main(int argc, char** argv)
{
    AppOptions options;
    if (!parseArgs(argc, argv, options)) return 1;
    installSignalHandlers();

    g_exporter.setMode(options.mode);
    g_exporter.setPrependStudioRestFrame(options.prependStudioRestFrame);

    WirelessRuntime wirelessRuntime;
    std::string wirelessError;
    if (!wirelessRuntime.start(options, wirelessError)) {
        if (!shutdownRequested()) std::cerr << wirelessError << "\n";
        return shutdownRequested() ? shutdownExitCode() : 2;
    }
    if (!options.wirelessSsid.empty()) g_wirelessRuntime = &wirelessRuntime;

    std::string libPath = chooseLibPath(options.libPath);
    std::cout << "SDK library: " << libPath << "\n";
    redirectVendorStdout();

    SdkApi sdk;
    if (!sdk.load(libPath)) return 3;

    sdk.setVDMocapDataCallBackFunc(onMocapData);
    sdk.setVDMocapDataWithVirtualCallBackFunc(onMocapDataWithVirtual);
    sdk.setDeviceBreakCallBackFunc(onDeviceBreak);

    std::unique_ptr<NetworkStreamer> networkStreamer;
    if (!options.sendIp.empty()) {
        networkStreamer.reset(new NetworkStreamer(
            options.sendIp,
            options.sendPort,
            g_initialBody));
        std::cout << "Body UDP target configured: " << options.sendIp << ':' << options.sendPort;
        if (options.durationSeconds > 0) {
            std::cout << ". Forwarding remains off in auto-record mode.\n";
        } else {
            std::cout << ". Press S after connecting to start forwarding.\n";
        }
    }

    std::unique_ptr<NetworkStreamer> handNetworkStreamer;
    if (!options.sendHandIp.empty()) {
        handNetworkStreamer.reset(new NetworkStreamer(
            options.sendHandIp,
            options.sendHandPort,
            g_initialRightHand,
            g_initialLeftHand));
        std::cout << "Hand UDP target configured: " << options.sendHandIp << ':' << options.sendHandPort;
        if (options.durationSeconds > 0) {
            std::cout << ". Forwarding remains off in auto-record mode.\n";
        } else {
            std::cout << ". Press H after connecting to start hand forwarding.\n";
        }
    }

    if (options.durationSeconds > 0) {
        runAutoRecord(sdk, options);
    } else {
        runInteractive(sdk, options, networkStreamer.get(), handNetworkStreamer.get());
    }

    disconnectDevice(sdk, true);
    g_networkStreamer.store(nullptr, std::memory_order_release);
    if (networkStreamer) {
        networkStreamer->stop();
        uint64_t dropped = networkStreamer->droppedFrameCount();
        if (dropped > 0) {
            std::cout << "Body network stream dropped " << dropped
                      << " old frames to preserve realtime delivery.\n";
        }
    }
    g_networkHandStreamer.store(nullptr, std::memory_order_release);
    if (handNetworkStreamer) {
        handNetworkStreamer->stop();
        uint64_t handDropped = handNetworkStreamer->droppedFrameCount();
        if (handDropped > 0) {
            std::cout << "Hand network stream dropped " << handDropped
                      << " old frames to preserve realtime delivery.\n";
        }
    }
    return shutdownRequested() ? shutdownExitCode() : 0;
}
