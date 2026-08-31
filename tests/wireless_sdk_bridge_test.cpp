#include "vdsuit_wireless_protocol.h"
#include "wireless_sdk_bridge.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cassert>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

using namespace vdsuit_wireless;

namespace {

std::vector<uint8_t> receiveDatagram(int descriptor)
{
    pollfd pollDescriptor {descriptor, POLLIN, 0};
    assert(poll(&pollDescriptor, 1, 2000) == 1);
    uint8_t buffer[2048];
    const ssize_t count = recv(descriptor, buffer, sizeof(buffer), 0);
    assert(count > 0);
    return std::vector<uint8_t>(buffer, buffer + count);
}

std::vector<uint8_t> readTerminalFrame(int descriptor, std::size_t expectedLength)
{
    std::vector<uint8_t> result;
    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (result.size() < expectedLength &&
           std::chrono::steady_clock::now() < deadline) {
        pollfd pollDescriptor {descriptor, POLLIN, 0};
        const int ready = poll(&pollDescriptor, 1, 100);
        if (ready <= 0) continue;
        uint8_t buffer[256];
        const ssize_t count = read(descriptor, buffer, sizeof(buffer));
        if (count > 0) result.insert(result.end(), buffer, buffer + count);
    }
    return result;
}

} // namespace

int main()
{
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
    assert(bridge.active());
    assert(!bridge.slavePath().empty());
    assert(bridge.localPort() != 0);

    const int terminal = open(bridge.slavePath().c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    assert(terminal >= 0);

    const int peer = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    assert(peer >= 0);
    sockaddr_in peerAddress {};
    peerAddress.sin_family = AF_INET;
    peerAddress.sin_port = htons(bridge.localPort());
    assert(inet_pton(AF_INET, config.clientAddress.c_str(), &peerAddress.sin_addr) == 1);
    assert(bind(peer, reinterpret_cast<sockaddr*>(&peerAddress), sizeof(peerAddress)) == 0);

    Frame link;
    link.target = kBroadcastTarget;
    link.command = CommandLinkProbe;
    const std::vector<uint8_t> linkBytes = encodeFrame(link);

    Frame frequency;
    frequency.target = kBroadcastTarget;
    frequency.command = CommandSetFrequency;
    frequency.payload.push_back(60);
    const std::vector<uint8_t> frequencyBytes = encodeFrame(frequency);

    assert(write(terminal, linkBytes.data(), 3) == 3);
    std::vector<uint8_t> remainder(linkBytes.begin() + 3, linkBytes.end());
    remainder.insert(remainder.end(), frequencyBytes.begin(), frequencyBytes.end());
    assert(write(terminal, remainder.data(), remainder.size()) ==
           static_cast<ssize_t>(remainder.size()));

    assert(receiveDatagram(peer) == linkBytes);
    assert(receiveDatagram(peer) == frequencyBytes);

    Frame response;
    response.target = kBroadcastTarget;
    response.command = CommandLinkProbe;
    response.payload.push_back(1);
    const std::vector<uint8_t> responseBytes = encodeFrame(response);

    sockaddr_in bridgeAddress {};
    bridgeAddress.sin_family = AF_INET;
    bridgeAddress.sin_port = htons(bridge.localPort());
    assert(inet_pton(AF_INET, config.localAddress.c_str(), &bridgeAddress.sin_addr) == 1);
    assert(sendto(peer, responseBytes.data(), responseBytes.size(), 0,
                  reinterpret_cast<sockaddr*>(&bridgeAddress), sizeof(bridgeAddress)) ==
           static_cast<ssize_t>(responseBytes.size()));
    assert(readTerminalFrame(terminal, responseBytes.size()) == responseBytes);
    const int64_t frameAge = bridge.millisecondsSinceLastNetworkFrame();
    assert(frameAge >= 0 && frameAge < 1000);

    const WirelessSdkBridgeStats stats = bridge.stats();
    assert(stats.serialFramesToNetwork == 2);
    assert(stats.networkFramesToSerial == 1);
    assert(stats.invalidSerialFrames == 0);
    assert(stats.invalidNetworkFrames == 0);

    close(peer);
    close(terminal);
    bridge.close();
    assert(!bridge.active());
    std::cout << "wireless_sdk_bridge_test: OK\n";
    return 0;
}
