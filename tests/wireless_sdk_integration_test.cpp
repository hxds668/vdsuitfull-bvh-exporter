#include "../include/VDMocapSDK_VDSuitMini_DataType.h"
#include "sdk_virtual_serial.h"
#include "vdsuit_wireless_protocol.h"
#include "wireless_sdk_bridge.h"

#include <arpa/inet.h>
#include <dlfcn.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <list>
#include <string>
#include <thread>
#include <vector>

using namespace vdsuit_wireless;
using namespace VDSuitMiniDevice;

class SerialPortM {
public:
    static bool GetPortNo(std::list<std::string>& primary,
                          std::list<std::string>& secondary);
};

namespace {

std::atomic<uint64_t> gProcessedFrames(0);
std::atomic<uint64_t> gBreakCallbacks(0);

void onMocapDataWithVirtual(_MocapDataWithVirtual_ data)
{
    if (data.isUpdate) gProcessedFrames.fetch_add(1, std::memory_order_relaxed);
}

void onDeviceBreak()
{
    gBreakCallbacks.fetch_add(1, std::memory_order_relaxed);
}

std::vector<uint8_t> fromHex(const std::string& text)
{
    assert(text.size() % 2 == 0);
    std::vector<uint8_t> bytes;
    bytes.reserve(text.size() / 2);
    for (std::size_t i = 0; i < text.size(); i += 2) {
        const std::string byteText = text.substr(i, 2);
        bytes.push_back(static_cast<uint8_t>(std::strtoul(byteText.c_str(), nullptr, 16)));
    }
    return bytes;
}

void updateSequenceAndChecksum(std::vector<uint8_t>& frame, uint8_t sequence)
{
    assert(frame.size() >= 2);
    frame[1] = sequence;
    frame.back() = checksum8(frame.data(), frame.size() - 1);
}

std::vector<uint8_t> handshakePayload()
{
    std::vector<uint8_t> payload(53, 0);
    payload[0] = 1;
    payload[1] = 60;
    payload[2] = 1;
    payload[3] = 0x10;
    payload[4] = 0x79;
    for (std::size_t i = 5; i < 23; ++i) payload[i] = 0xff;
    const char version[] = "H_V3.1S_V3.1";
    std::memcpy(payload.data() + 23, version, 12);
    const uint8_t first[8] = {0x0e, 0xa1, 0xb6, 0x65, 0xef, 0x4a, 0xc9, 0x4d};
    const uint8_t mask[8] = {'@', '#', '$', '%', '^', '&', '*', '!'};
    for (std::size_t i = 0; i < 8; ++i) {
        payload[35 + i] = first[i];
        payload[43 + i] = static_cast<uint8_t>(first[i] ^ mask[i]);
    }
    payload[51] = 3;
    payload[52] = 0;
    return payload;
}

class FakeTransmitter {
public:
    FakeTransmitter(const std::string& address, uint16_t port)
        : address_(address), port_(port)
    {
    }

    bool start(std::string& error)
    {
        descriptor_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (descriptor_ < 0) {
            error = std::strerror(errno);
            return false;
        }
        sockaddr_in local {};
        local.sin_family = AF_INET;
        local.sin_port = htons(port_);
        if (inet_pton(AF_INET, address_.c_str(), &local.sin_addr) != 1 ||
            bind(descriptor_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
            error = std::strerror(errno);
            close(descriptor_);
            descriptor_ = -1;
            return false;
        }
        stopping_.store(false);
        thread_ = std::thread(&FakeTransmitter::run, this);
        return true;
    }

    void stop()
    {
        stopping_.store(true);
        if (descriptor_ >= 0) shutdown(descriptor_, SHUT_RDWR);
        if (thread_.joinable()) thread_.join();
        if (descriptor_ >= 0) close(descriptor_);
        descriptor_ = -1;
    }

    ~FakeTransmitter() { stop(); }

    uint64_t receivedCommands() const
    {
        return receivedCommands_.load(std::memory_order_relaxed);
    }

private:
    void sendFrame(const Frame& frame, const sockaddr_in& destination)
    {
        const std::vector<uint8_t> bytes = encodeFrame(frame);
        sendto(descriptor_, bytes.data(), bytes.size(), 0,
               reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
    }

    void handleCommand(const uint8_t* bytes,
                       std::size_t length,
                       const sockaddr_in& source)
    {
        Frame request;
        std::string error;
        if (!decodeFrame(bytes, length, request, error)) return;
        receivedCommands_.fetch_add(1, std::memory_order_relaxed);

        Frame response;
        response.target = kBroadcastTarget;
        response.command = request.command;
        switch (request.command) {
        case CommandLinkProbe:
            response.payload.push_back(1);
            sendFrame(response, source);
            break;
        case CommandHandshake:
            response.payload = handshakePayload();
            sendFrame(response, source);
            break;
        case CommandSetFrequency:
            if (request.payload.size() == 1) {
                response.payload.push_back(1);
                response.payload.push_back(request.payload[0]);
                sendFrame(response, source);
            }
            break;
        case CommandStream:
            streamDestination_ = source;
            streaming_ = true;
            sendMotionFrame();
            break;
        case CommandDisconnect:
            streaming_ = false;
            response.payload.push_back(1);
            sendFrame(response, source);
            break;
        default:
            break;
        }
    }

    void sendMotionFrame()
    {
        if (!streaming_) return;
        // One valid 83-byte Full-suit motion frame captured at the physical
        // receiver's UART. The sequence and checksum are updated per packet.
        static const std::string captured =
            "5b07004d03a812ff009380ff98e08000000000808d40981d65807588"
            "34602829de2402f75fc838431488d477116038becf7fcedf94681887"
            "deef4c88e1ffa8d8e4df1cff0000ff66b5b8ff88ffffb86d115d88";
        std::vector<uint8_t> frame = fromHex(captured);
        updateSequenceAndChecksum(frame, sequence_++);
        sendto(descriptor_, frame.data(), frame.size(), 0,
               reinterpret_cast<const sockaddr*>(&streamDestination_),
               sizeof(streamDestination_));
    }

    void run()
    {
        std::chrono::steady_clock::time_point nextMotion =
            std::chrono::steady_clock::now();
        while (!stopping_.load()) {
            const int timeout = streaming_ ? 10 : 100;
            pollfd pollDescriptor {descriptor_, POLLIN, 0};
            const int ready = poll(&pollDescriptor, 1, timeout);
            if (ready > 0 && (pollDescriptor.revents & POLLIN)) {
                uint8_t buffer[2048];
                sockaddr_in source {};
                socklen_t sourceLength = sizeof(source);
                const ssize_t count = recvfrom(
                    descriptor_, buffer, sizeof(buffer), 0,
                    reinterpret_cast<sockaddr*>(&source), &sourceLength);
                if (count > 0) {
                    handleCommand(buffer, static_cast<std::size_t>(count), source);
                }
            }
            const std::chrono::steady_clock::time_point now =
                std::chrono::steady_clock::now();
            if (streaming_ && now >= nextMotion) {
                sendMotionFrame();
                nextMotion = now + std::chrono::milliseconds(16);
            }
        }
    }

    std::string address_;
    uint16_t port_;
    int descriptor_ = -1;
    std::atomic<bool> stopping_ {false};
    std::atomic<uint64_t> receivedCommands_ {0};
    std::thread thread_;
    bool streaming_ = false;
    uint8_t sequence_ = 0;
    sockaddr_in streamDestination_ {};
};

template <typename T>
T loadSymbol(void* handle, const char* name)
{
    dlerror();
    T result = reinterpret_cast<T>(dlsym(handle, name));
    const char* error = dlerror();
    if (error || !result) {
        std::cerr << "missing SDK symbol " << name << '\n';
        std::abort();
    }
    return result;
}

} // namespace

int main(int argc, char** argv)
{
    const std::string sdkPath = argc > 1
        ? argv[1]
        : "./lib/arm64/libVDMocapSDK_miniArm64.so";

    WirelessSdkBridgeConfig config;
    config.interfaceName.clear();
    config.localAddress = "127.0.0.1";
    config.clientAddress = "127.0.0.2";
    config.destinationAddress = "127.0.0.2";
    config.port = 0;
    config.bindToDevice = false;

    WirelessSdkBridge bridge(config);
    std::string error;
    if (!bridge.open(error)) {
        std::cerr << "bridge open failed: " << error << '\n';
        return 1;
    }
    setSdkVirtualSerialPath(bridge.slavePath());

    FakeTransmitter transmitter(config.clientAddress, bridge.localPort());
    if (!transmitter.start(error)) {
        std::cerr << "fake transmitter failed: " << error << '\n';
        return 1;
    }

    void* sdk = dlopen(sdkPath.c_str(), RTLD_LAZY);
    if (!sdk) {
        std::cerr << "dlopen failed: " << dlerror() << '\n';
        return 1;
    }
    setSdkLibraryHandle(sdk);

    using InitialFunction = void (*)(_WorldSpace_, float (*)[3], float (*)[3],
                                     float (*)[3], int);
    using ConnectFunction = bool (*)();
    using DisconnectFunction = void (*)();
    using GetConnectStateFunction = bool (*)();
    using SetCallbackFunction = void (*)(void (*)(_MocapDataWithVirtual_));
    using SetBreakCallbackFunction = void (*)(void (*)());

    InitialFunction initial = loadSymbol<InitialFunction>(sdk, "Initial");
    ConnectFunction connectDevice = loadSymbol<ConnectFunction>(sdk, "Connect");
    DisconnectFunction disconnectDevice =
        loadSymbol<DisconnectFunction>(sdk, "DisConnect");
    GetConnectStateFunction getConnectState =
        loadSymbol<GetConnectStateFunction>(sdk, "GetConnectState");
    SetCallbackFunction setCallback = loadSymbol<SetCallbackFunction>(
        sdk, "SetVDMocapDataWithVirtualCallBackFunc");
    SetBreakCallbackFunction setBreakCallback = loadSymbol<SetBreakCallbackFunction>(
        sdk, "SetDeviceBreakCallBackFunc");

    setCallback(onMocapDataWithVirtual);
    setBreakCallback(onDeviceBreak);
    initial(WS_Geo, nullptr, nullptr, nullptr, 0);
    const bool connected = connectDevice() && getConnectState();

    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (connected && gProcessedFrames.load(std::memory_order_relaxed) == 0 &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const char* dropDelayText = std::getenv("VDSUIT_TEST_DROP_BEFORE_DISCONNECT_MS");
    if (dropDelayText) {
        transmitter.stop();
        const int dropDelay = std::atoi(dropDelayText);
        if (dropDelay > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(dropDelay));
        }
    }

    const bool stateBeforeDisconnect = getConnectState();
    disconnectDevice();
    setCallback(nullptr);
    setBreakCallback(nullptr);

    // Exercise the non-wireless fallback too. It may legitimately find no
    // physical port on the test host, but it must call the SDK implementation
    // rather than recurse into the interposer.
    clearSdkVirtualSerialPath();
    std::list<std::string> physicalPrimary;
    std::list<std::string> physicalSecondary;
    SerialPortM::GetPortNo(physicalPrimary, physicalSecondary);

    setSdkLibraryHandle(nullptr);
    dlclose(sdk);
    transmitter.stop();
    bridge.close();

    const WirelessSdkBridgeStats bridgeStats = bridge.stats();
    std::cerr << "integration diagnostics: connected=" << connected
              << " state-before-disconnect=" << stateBeforeDisconnect
              << " commands=" << transmitter.receivedCommands()
              << " serial-to-network=" << bridgeStats.serialFramesToNetwork
              << " network-to-serial=" << bridgeStats.networkFramesToSerial
              << " invalid-serial=" << bridgeStats.invalidSerialFrames
              << " invalid-network=" << bridgeStats.invalidNetworkFrames
              << " processed=" << gProcessedFrames.load(std::memory_order_relaxed)
              << " breaks=" << gBreakCallbacks.load(std::memory_order_relaxed)
              << '\n';

    if (!connected) {
        std::cerr << "SDK did not connect through the virtual serial bridge\n";
        return 1;
    }
    if (transmitter.receivedCommands() < 4) {
        std::cerr << "SDK protocol sequence was incomplete\n";
        return 1;
    }
    if (gProcessedFrames.load(std::memory_order_relaxed) == 0) {
        std::cerr << "SDK connected but produced no processed mocap callback\n";
        return 1;
    }

    std::cout << "wireless_sdk_integration_test: connected, commands="
              << transmitter.receivedCommands() << ", processed="
              << gProcessedFrames.load(std::memory_order_relaxed) << "\n";
    return 0;
}
