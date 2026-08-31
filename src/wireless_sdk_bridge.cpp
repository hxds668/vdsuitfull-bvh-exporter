#include "wireless_sdk_bridge.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace vdsuit_wireless {
namespace {

constexpr uint8_t kFrameStart = 0x5b;
constexpr uint8_t kFrameEnd = 0x5d;
constexpr std::size_t kOuterOverhead = 6;
constexpr std::size_t kMinimumFrameLength = 11;
constexpr std::size_t kMaximumSdkFrameLength = 1024;

uint8_t checksum8(const uint8_t* data, std::size_t length)
{
    uint8_t result = 0;
    for (std::size_t i = 0; i < length; ++i) {
        result = static_cast<uint8_t>(result + data[i]);
    }
    return result;
}

std::size_t rawFrameLength(const uint8_t* data, std::size_t length)
{
    if (!data || length < 4 || data[0] != kFrameStart) return 0;
    const uint16_t bodyLength = static_cast<uint16_t>(
        (static_cast<uint16_t>(data[2]) << 8) | data[3]);
    return static_cast<std::size_t>(bodyLength) + kOuterOverhead;
}

bool validRawFrame(const uint8_t* data, std::size_t length)
{
    if (!data || length < kMinimumFrameLength ||
        length > kMaximumSdkFrameLength || data[0] != kFrameStart) {
        return false;
    }
    if (rawFrameLength(data, length) != length || data[length - 2] != kFrameEnd) {
        return false;
    }
    return checksum8(data, length - 1) == data[length - 1];
}

bool setCloseOnExec(int descriptor)
{
    const int flags = fcntl(descriptor, F_GETFD);
    return flags >= 0 && fcntl(descriptor, F_SETFD, flags | FD_CLOEXEC) == 0;
}

bool setNonBlocking(int descriptor)
{
    const int flags = fcntl(descriptor, F_GETFL);
    return flags >= 0 && fcntl(descriptor, F_SETFL, flags | O_NONBLOCK) == 0;
}

bool configureRawTerminal(int descriptor)
{
    termios attributes {};
    if (tcgetattr(descriptor, &attributes) != 0) return false;
    cfmakeraw(&attributes);
    attributes.c_cc[VMIN] = 0;
    attributes.c_cc[VTIME] = 0;
    return tcsetattr(descriptor, TCSANOW, &attributes) == 0;
}

} // namespace

class WirelessSdkBridge::Impl {
public:
    explicit Impl(const WirelessSdkBridgeConfig& config) : config_(config) {}

    ~Impl()
    {
        close();
    }

    bool open(std::string& error)
    {
        error.clear();
        if (active_.load()) return true;
        if (config_.port == 0 && config_.destinationAddress == "255.255.255.255") {
            error = "an ephemeral bridge port requires an explicit destination address";
            return false;
        }

        char terminalName[256] = {};
        if (openpty(&masterFd_, &keepaliveSlaveFd_, terminalName, nullptr, nullptr) != 0) {
            error = std::string("openpty failed: ") + std::strerror(errno);
            resetDescriptors();
            return false;
        }
        if (!setCloseOnExec(masterFd_) || !setCloseOnExec(keepaliveSlaveFd_) ||
            !setNonBlocking(masterFd_) || !configureRawTerminal(keepaliveSlaveFd_)) {
            error = std::string("could not configure pseudo terminal: ") +
                    std::strerror(errno);
            resetDescriptors();
            return false;
        }
        slavePath_ = terminalName;

        socketFd_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (socketFd_ < 0) {
            error = std::string("UDP socket failed: ") + std::strerror(errno);
            resetDescriptors();
            return false;
        }

        int enabled = 1;
        int receiveBuffer = 1024 * 1024;
        if (setsockopt(socketFd_, SOL_SOCKET, SO_REUSEADDR,
                       &enabled, sizeof(enabled)) != 0 ||
            setsockopt(socketFd_, SOL_SOCKET, SO_BROADCAST,
                       &enabled, sizeof(enabled)) != 0 ||
            setsockopt(socketFd_, SOL_SOCKET, SO_RCVBUF,
                       &receiveBuffer, sizeof(receiveBuffer)) != 0) {
            error = std::string("UDP socket configuration failed: ") +
                    std::strerror(errno);
            resetDescriptors();
            return false;
        }
        if (config_.bindToDevice &&
            setsockopt(socketFd_, SOL_SOCKET, SO_BINDTODEVICE,
                       config_.interfaceName.c_str(),
                       config_.interfaceName.size() + 1) != 0) {
            error = "cannot bind UDP bridge to " + config_.interfaceName + ": " +
                    std::strerror(errno);
            resetDescriptors();
            return false;
        }

        sockaddr_in local {};
        local.sin_family = AF_INET;
        local.sin_port = htons(config_.port);
        if (inet_pton(AF_INET, config_.localAddress.c_str(), &local.sin_addr) != 1) {
            error = "invalid bridge local IPv4 address";
            resetDescriptors();
            return false;
        }
        if (bind(socketFd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
            error = "cannot bind UDP bridge on " + config_.localAddress + ':' +
                    std::to_string(config_.port) + ": " + std::strerror(errno);
            resetDescriptors();
            return false;
        }

        socklen_t localLength = sizeof(local);
        if (getsockname(socketFd_, reinterpret_cast<sockaddr*>(&local), &localLength) != 0) {
            error = std::string("getsockname failed: ") + std::strerror(errno);
            resetDescriptors();
            return false;
        }
        localPort_ = ntohs(local.sin_port);

        destination_ = sockaddr_in {};
        destination_.sin_family = AF_INET;
        destination_.sin_port = htons(localPort_);
        if (inet_pton(AF_INET, config_.destinationAddress.c_str(),
                      &destination_.sin_addr) != 1 ||
            inet_pton(AF_INET, config_.clientAddress.c_str(),
                      &clientAddress_) != 1) {
            error = "invalid bridge client or destination IPv4 address";
            resetDescriptors();
            return false;
        }

        stopping_.store(false);
        active_.store(true);
        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            haveNetworkFrame_ = false;
            lastNetworkFrame_ = std::chrono::steady_clock::time_point {};
        }
        worker_ = std::thread(&Impl::run, this);
        return true;
    }

    void close()
    {
        stopping_.store(true);
        if (socketFd_ >= 0) shutdown(socketFd_, SHUT_RDWR);
        if (worker_.joinable()) worker_.join();
        active_.store(false);
        resetDescriptors();
        serialBuffer_.clear();
        slavePath_.clear();
        localPort_ = 0;
    }

    bool active() const { return active_.load(); }

    std::string slavePath() const { return slavePath_; }

    uint16_t localPort() const { return localPort_; }

    int64_t millisecondsSinceLastNetworkFrame() const
    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        if (!haveNetworkFrame_) return -1;
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - lastNetworkFrame_).count();
    }

    WirelessSdkBridgeStats stats() const
    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        return stats_;
    }

private:
    void resetDescriptors()
    {
        if (socketFd_ >= 0) {
            ::close(socketFd_);
            socketFd_ = -1;
        }
        if (masterFd_ >= 0) {
            ::close(masterFd_);
            masterFd_ = -1;
        }
        if (keepaliveSlaveFd_ >= 0) {
            ::close(keepaliveSlaveFd_);
            keepaliveSlaveFd_ = -1;
        }
    }

    void recordInvalidSerial()
    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        ++stats_.invalidSerialFrames;
    }

    void processSerialBytes(const uint8_t* data, std::size_t length)
    {
        serialBuffer_.insert(serialBuffer_.end(), data, data + length);
        while (!serialBuffer_.empty()) {
            std::vector<uint8_t>::iterator start =
                std::find(serialBuffer_.begin(), serialBuffer_.end(), kFrameStart);
            if (start != serialBuffer_.begin()) {
                if (start == serialBuffer_.end()) {
                    serialBuffer_.clear();
                    recordInvalidSerial();
                    return;
                }
                serialBuffer_.erase(serialBuffer_.begin(), start);
                recordInvalidSerial();
            }
            if (serialBuffer_.size() < 4) return;

            const std::size_t frameLength =
                rawFrameLength(serialBuffer_.data(), serialBuffer_.size());
            if (frameLength < kMinimumFrameLength ||
                frameLength > kMaximumSdkFrameLength) {
                serialBuffer_.erase(serialBuffer_.begin());
                recordInvalidSerial();
                continue;
            }
            if (serialBuffer_.size() < frameLength) return;
            if (!validRawFrame(serialBuffer_.data(), frameLength)) {
                serialBuffer_.erase(serialBuffer_.begin());
                recordInvalidSerial();
                continue;
            }

            const ssize_t sent = sendto(
                socketFd_, serialBuffer_.data(), frameLength, 0,
                reinterpret_cast<const sockaddr*>(&destination_), sizeof(destination_));
            if (sent == static_cast<ssize_t>(frameLength)) {
                std::lock_guard<std::mutex> lock(statsMutex_);
                ++stats_.serialFramesToNetwork;
                stats_.serialBytesToNetwork += frameLength;
            }
            serialBuffer_.erase(serialBuffer_.begin(),
                                serialBuffer_.begin() + frameLength);
        }
    }

    bool writeToSdk(const uint8_t* data, std::size_t length)
    {
        std::size_t offset = 0;
        while (offset < length && !stopping_.load()) {
            const ssize_t count = write(masterFd_, data + offset, length - offset);
            if (count > 0) {
                offset += static_cast<std::size_t>(count);
                continue;
            }
            if (count < 0 && errno == EINTR) continue;
            if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                pollfd descriptor {masterFd_, POLLOUT, 0};
                poll(&descriptor, 1, 100);
                continue;
            }
            return false;
        }
        return offset == length;
    }

    void receiveNetworkFrame()
    {
        uint8_t buffer[kMaximumSdkFrameLength + 1];
        sockaddr_in source {};
        socklen_t sourceLength = sizeof(source);
        const ssize_t count = recvfrom(
            socketFd_, buffer, sizeof(buffer), 0,
            reinterpret_cast<sockaddr*>(&source), &sourceLength);
        if (count <= 0) return;
        if (source.sin_family != AF_INET ||
            source.sin_port != htons(localPort_) ||
            source.sin_addr.s_addr != clientAddress_.s_addr) {
            std::lock_guard<std::mutex> lock(statsMutex_);
            ++stats_.ignoredNetworkDatagrams;
            return;
        }
        if (!validRawFrame(buffer, static_cast<std::size_t>(count))) {
            std::lock_guard<std::mutex> lock(statsMutex_);
            ++stats_.invalidNetworkFrames;
            return;
        }
        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            haveNetworkFrame_ = true;
            lastNetworkFrame_ = std::chrono::steady_clock::now();
        }
        if (writeToSdk(buffer, static_cast<std::size_t>(count))) {
            std::lock_guard<std::mutex> lock(statsMutex_);
            ++stats_.networkFramesToSerial;
            stats_.networkBytesToSerial += static_cast<uint64_t>(count);
        }
    }

    void receiveSerialData()
    {
        uint8_t buffer[4096];
        while (!stopping_.load()) {
            const ssize_t count = read(masterFd_, buffer, sizeof(buffer));
            if (count > 0) {
                processSerialBytes(buffer, static_cast<std::size_t>(count));
                continue;
            }
            if (count < 0 && errno == EINTR) continue;
            break;
        }
    }

    void run()
    {
        while (!stopping_.load()) {
            pollfd descriptors[2] = {
                {masterFd_, POLLIN, 0},
                {socketFd_, POLLIN, 0},
            };
            const int ready = poll(descriptors, 2, 200);
            if (ready < 0 && errno == EINTR) continue;
            if (ready <= 0) continue;
            if (descriptors[0].revents & POLLIN) receiveSerialData();
            if (descriptors[1].revents & POLLIN) receiveNetworkFrame();
        }
    }

    WirelessSdkBridgeConfig config_;
    int masterFd_ = -1;
    int keepaliveSlaveFd_ = -1;
    int socketFd_ = -1;
    uint16_t localPort_ = 0;
    std::string slavePath_;
    sockaddr_in destination_ {};
    in_addr clientAddress_ {};
    std::atomic<bool> stopping_ {false};
    std::atomic<bool> active_ {false};
    std::thread worker_;
    std::vector<uint8_t> serialBuffer_;
    mutable std::mutex statsMutex_;
    WirelessSdkBridgeStats stats_;
    bool haveNetworkFrame_ = false;
    std::chrono::steady_clock::time_point lastNetworkFrame_ {};
};

WirelessSdkBridge::WirelessSdkBridge(const WirelessSdkBridgeConfig& config)
    : impl_(new Impl(config))
{
}

WirelessSdkBridge::~WirelessSdkBridge() = default;

bool WirelessSdkBridge::open(std::string& error)
{
    return impl_->open(error);
}

void WirelessSdkBridge::close()
{
    impl_->close();
}

bool WirelessSdkBridge::active() const
{
    return impl_->active();
}

std::string WirelessSdkBridge::slavePath() const
{
    return impl_->slavePath();
}

uint16_t WirelessSdkBridge::localPort() const
{
    return impl_->localPort();
}

int64_t WirelessSdkBridge::millisecondsSinceLastNetworkFrame() const
{
    return impl_->millisecondsSinceLastNetworkFrame();
}

WirelessSdkBridgeStats WirelessSdkBridge::stats() const
{
    return impl_->stats();
}

} // namespace vdsuit_wireless
