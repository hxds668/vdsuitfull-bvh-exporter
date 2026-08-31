#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace vdsuit_wireless {

struct WirelessSdkBridgeConfig {
    std::string interfaceName = "wlan0";
    std::string localAddress = "192.168.18.1";
    std::string clientAddress = "192.168.18.3";
    std::string destinationAddress = "255.255.255.255";
    uint16_t port = 8080;
    bool bindToDevice = true;
};

struct WirelessSdkBridgeStats {
    uint64_t serialFramesToNetwork = 0;
    uint64_t networkFramesToSerial = 0;
    uint64_t serialBytesToNetwork = 0;
    uint64_t networkBytesToSerial = 0;
    uint64_t invalidSerialFrames = 0;
    uint64_t invalidNetworkFrames = 0;
    uint64_t ignoredNetworkDatagrams = 0;
};

// Presents a pseudo terminal to the vendor SDK and transparently bridges the
// SDK's framed serial byte stream to the transmitter's UDP protocol.
class WirelessSdkBridge {
public:
    explicit WirelessSdkBridge(const WirelessSdkBridgeConfig& config);
    ~WirelessSdkBridge();

    WirelessSdkBridge(const WirelessSdkBridge&) = delete;
    WirelessSdkBridge& operator=(const WirelessSdkBridge&) = delete;

    bool open(std::string& error);
    void close();

    bool active() const;
    std::string slavePath() const;
    uint16_t localPort() const;
    // Returns -1 until the first valid transmitter datagram is received.
    int64_t millisecondsSinceLastNetworkFrame() const;
    WirelessSdkBridgeStats stats() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace vdsuit_wireless
