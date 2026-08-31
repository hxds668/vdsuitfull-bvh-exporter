#include "vdsuit_wireless_protocol.h"
#include "wireless_hotspot.h"

#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace vdsuit_wireless;

volatile sig_atomic_t gShutdownRequested = 0;

void signalHandler(int)
{
    gShutdownRequested = 1;
}

bool shutdownRequested()
{
    return gShutdownRequested != 0;
}

void installSignalHandlers()
{
    struct sigaction action {};
    action.sa_handler = signalHandler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
}

struct AppOptions {
    std::string interfaceName = "wlan0";
    std::string ssid;
    std::string apAddress = "192.168.18.1";
};

void printUsage(const char* program)
{
    std::cout
        << "Usage: sudo " << program << " --ssid SERIAL [options]\n"
        << "Options:\n"
        << "  --ssid SERIAL       hotspot SSID; 1-32 decimal digits\n"
        << "  --ip ADDRESS        AP IPv4 address, default 192.168.18.1 (/24)\n"
        << "  --interface NAME    AP interface, default wlan0\n"
        << "  --help              show this help\n";
}

bool parseArguments(int argc, char** argv, AppOptions& options, bool& helpRequested)
{
    helpRequested = false;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--help") {
            printUsage(argv[0]);
            helpRequested = true;
            return false;
        }
        if (argument == "--ssid" && i + 1 < argc) {
            options.ssid = argv[++i];
        } else if (argument == "--ip" && i + 1 < argc) {
            options.apAddress = argv[++i];
        } else if (argument == "--interface" && i + 1 < argc) {
            options.interfaceName = argv[++i];
        } else {
            std::cerr << "Unknown or incomplete argument: " << argument << '\n';
            printUsage(argv[0]);
            return false;
        }
    }
    return true;
}

struct StreamStats {
    bool active = false;
    double fps = 0.0;
    double bytesPerSecond = 0.0;
    uint64_t totalPackets = 0;
    uint64_t totalBytes = 0;
    uint64_t missingPackets = 0;
    uint64_t duplicatePackets = 0;
    uint64_t outOfOrderPackets = 0;
    uint64_t invalidDatagrams = 0;
    std::size_t lastFrameLength = 0;
};

class UdpSession {
public:
    UdpSession(const std::string& interfaceName,
               const std::string& apAddress,
               const std::string& clientAddress,
               const std::string& destinationAddress = "255.255.255.255",
               bool bindToDevice = true)
        : interfaceName_(interfaceName),
          apAddress_(apAddress),
          clientAddress_(clientAddress),
          destinationAddress_(destinationAddress),
          bindToDevice_(bindToDevice)
    {
    }

    ~UdpSession()
    {
        close();
    }

    bool open(std::string& error)
    {
        error.clear();
        socketFd_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (socketFd_ < 0) {
            error = std::string("socket failed: ") + std::strerror(errno);
            return false;
        }

        int enabled = 1;
        int receiveBuffer = 1024 * 1024;
        if (setsockopt(socketFd_, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled)) != 0 ||
            setsockopt(socketFd_, SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled)) != 0 ||
            setsockopt(socketFd_, SOL_SOCKET, SO_RCVBUF,
                       &receiveBuffer, sizeof(receiveBuffer)) != 0) {
            error = std::string("socket configuration failed: ") + std::strerror(errno);
            closeSocket();
            return false;
        }
        if (bindToDevice_ &&
            setsockopt(socketFd_, SOL_SOCKET, SO_BINDTODEVICE,
                       interfaceName_.c_str(), interfaceName_.size() + 1) != 0) {
            error = std::string("cannot bind UDP socket to ") + interfaceName_ + ": " +
                    std::strerror(errno);
            closeSocket();
            return false;
        }

        sockaddr_in local {};
        local.sin_family = AF_INET;
        local.sin_port = htons(kProtocolPort);
        if (inet_pton(AF_INET, apAddress_.c_str(), &local.sin_addr) != 1) {
            error = "invalid local UDP address";
            closeSocket();
            return false;
        }
        if (bind(socketFd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
            error = std::string("cannot bind UDP 8080 on ") + apAddress_ + ": " +
                    std::strerror(errno);
            closeSocket();
            return false;
        }

        destination_.sin_family = AF_INET;
        destination_.sin_port = htons(kProtocolPort);
        if (inet_pton(AF_INET, destinationAddress_.c_str(), &destination_.sin_addr) != 1) {
            error = "invalid UDP destination address";
            closeSocket();
            return false;
        }
        if (inet_pton(AF_INET, clientAddress_.c_str(), &clientNetworkAddress_) != 1) {
            error = "invalid transmitter UDP address";
            closeSocket();
            return false;
        }

        stopping_.store(false);
        receiverThread_ = std::thread(&UdpSession::receiveLoop, this);
        return true;
    }

    void close()
    {
        stopping_.store(true);
        condition_.notify_all();
        if (socketFd_ >= 0) shutdown(socketFd_, SHUT_RDWR);
        if (receiverThread_.joinable()) receiverThread_.join();
        closeSocket();
    }

    bool connectAndStart(HandshakeInfo& handshake, std::string& error)
    {
        if (!linkProbe(error)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!performHandshake(handshake, error)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(700));
        if (!linkProbe(error)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        resetStreamStats();
        if (!startStream(error)) {
            setStreaming(false);
            return false;
        }
        return true;
    }

    bool disconnect(std::string& error)
    {
        Frame response;
        if (!sendAndWait(CommandDisconnect, std::vector<uint8_t>(), response, error)) {
            return false;
        }
        if (response.payload.size() != 1 || response.payload[0] != 1) {
            error = "disconnect response is not an ACK";
            return false;
        }
        setStreaming(false);
        return true;
    }

    bool setFrequency(int frequency, std::string& error)
    {
        if (!isSupportedFrequency(frequency)) {
            error = "supported rates are 60, 72, 80, and 96 Hz";
            return false;
        }
        if (!linkProbe(error)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(150));

        Frame response;
        const std::vector<uint8_t> payload(1, static_cast<uint8_t>(frequency));
        if (!sendAndWait(CommandSetFrequency, payload, response, error)) return false;
        if (response.payload.size() != 2 || response.payload[0] != 1 ||
            response.payload[1] != static_cast<uint8_t>(frequency)) {
            error = "set-frequency response did not ACK and echo the requested rate";
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        if (!linkProbe(error)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        resetStreamStats();
        if (!startStream(error)) {
            setStreaming(false);
            return false;
        }
        return true;
    }

    StreamStats sampleStats()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        const double seconds = std::chrono::duration<double>(now - lastStatsSample_).count();
        StreamStats stats;
        stats.active = streaming_;
        stats.totalPackets = totalPackets_;
        stats.totalBytes = totalBytes_;
        stats.missingPackets = missingPackets_;
        stats.duplicatePackets = duplicatePackets_;
        stats.outOfOrderPackets = outOfOrderPackets_;
        stats.invalidDatagrams = invalidDatagrams_;
        stats.lastFrameLength = lastFrameLength_;
        if (seconds > 0.0) {
            stats.fps = static_cast<double>(intervalPackets_) / seconds;
            stats.bytesPerSecond = static_cast<double>(intervalBytes_) / seconds;
        }
        intervalPackets_ = 0;
        intervalBytes_ = 0;
        lastStatsSample_ = now;
        return stats;
    }

    bool streaming() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return streaming_;
    }

private:
    struct QueuedFrame {
        uint64_t serial = 0;
        Frame frame;
    };

    void closeSocket()
    {
        if (socketFd_ >= 0) {
            ::close(socketFd_);
            socketFd_ = -1;
        }
    }

    bool sendCommand(uint8_t command,
                     const std::vector<uint8_t>& payload,
                     std::string& error)
    {
        Frame frame;
        frame.target = kBroadcastTarget;
        frame.command = command;
        frame.payload = payload;
        const std::vector<uint8_t> bytes = encodeFrame(frame);
        if (bytes.empty()) {
            error = "could not encode command frame";
            return false;
        }
        const ssize_t count = sendto(socketFd_, bytes.data(), bytes.size(), 0,
                                     reinterpret_cast<const sockaddr*>(&destination_),
                                     sizeof(destination_));
        if (count != static_cast<ssize_t>(bytes.size())) {
            error = std::string("UDP broadcast failed: ") + std::strerror(errno);
            return false;
        }
        return true;
    }

    bool sendAndWait(uint8_t command,
                     const std::vector<uint8_t>& payload,
                     Frame& response,
                     std::string& error)
    {
        for (int attempt = 1; attempt <= 3 && !shutdownRequested(); ++attempt) {
            uint64_t baseline = 0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                baseline = receiveSerial_;
            }
            if (!sendCommand(command, payload, error)) return false;

            const std::chrono::steady_clock::time_point deadline =
                std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
            std::unique_lock<std::mutex> lock(mutex_);
            while (!stopping_.load() && !shutdownRequested()) {
                for (std::deque<QueuedFrame>::iterator iterator = controlFrames_.begin();
                     iterator != controlFrames_.end(); ++iterator) {
                    if (iterator->serial > baseline && iterator->frame.command == command) {
                        response = iterator->frame;
                        controlFrames_.erase(iterator);
                        return true;
                    }
                }
                if (condition_.wait_until(lock, deadline) == std::cv_status::timeout) break;
            }
            error = "command 0x" + commandHex(command) + " timed out (attempt " +
                    std::to_string(attempt) + "/3)";
        }
        return false;
    }

    bool linkProbe(std::string& error)
    {
        Frame response;
        if (!sendAndWait(CommandLinkProbe, std::vector<uint8_t>(), response, error)) {
            return false;
        }
        if (response.payload.size() != 1 || response.payload[0] != 1) {
            error = "link probe response is not ready";
            return false;
        }
        return true;
    }

    bool performHandshake(HandshakeInfo& handshake, std::string& error)
    {
        Frame response;
        if (!sendAndWait(CommandHandshake, makeHandshakeRequest(0x03), response, error)) {
            return false;
        }
        return parseHandshakeResponse(response.payload, handshake, error);
    }

    bool startStream(std::string& error)
    {
        for (int attempt = 1; attempt <= 3 && !shutdownRequested(); ++attempt) {
            uint64_t baseline = 0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                baseline = streamArrivalSerial_;
            }
            if (!sendCommand(CommandStream, streamRequestPayload(), error)) return false;

            std::unique_lock<std::mutex> lock(mutex_);
            const bool received = condition_.wait_for(
                lock, std::chrono::milliseconds(1500),
                [this, baseline]() {
                    return stopping_.load() || shutdownRequested() ||
                           streamArrivalSerial_ > baseline;
                });
            if (received && streamArrivalSerial_ > baseline) return true;
            error = "stream did not start (attempt " + std::to_string(attempt) + "/3)";
        }
        return false;
    }

    static std::string commandHex(uint8_t command)
    {
        std::ostringstream output;
        output << std::hex << std::setfill('0') << std::setw(2)
               << static_cast<unsigned int>(command);
        return output.str();
    }

    void resetStreamStats()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        streaming_ = true;
        haveSequence_ = false;
        totalPackets_ = 0;
        totalBytes_ = 0;
        intervalPackets_ = 0;
        intervalBytes_ = 0;
        missingPackets_ = 0;
        duplicatePackets_ = 0;
        outOfOrderPackets_ = 0;
        lastFrameLength_ = 0;
        lastStatsSample_ = std::chrono::steady_clock::now();
    }

    void setStreaming(bool streaming)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        streaming_ = streaming;
    }

    void recordStreamFrame(const Frame& frame, std::size_t datagramLength)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++streamArrivalSerial_;
        if (streaming_) {
            ++totalPackets_;
            totalBytes_ += datagramLength;
            ++intervalPackets_;
            intervalBytes_ += datagramLength;
            lastFrameLength_ = datagramLength;

            if (!haveSequence_) {
                haveSequence_ = true;
                lastSequence_ = frame.sequence;
            } else {
                const uint8_t delta = static_cast<uint8_t>(frame.sequence - lastSequence_);
                if (delta == 0) {
                    ++duplicatePackets_;
                } else if (delta < 128) {
                    if (delta > 1) missingPackets_ += static_cast<uint64_t>(delta - 1);
                    lastSequence_ = frame.sequence;
                } else {
                    ++outOfOrderPackets_;
                }
            }
        }
        condition_.notify_all();
    }

    void receiveLoop()
    {
        while (!stopping_.load()) {
            pollfd descriptor {socketFd_, POLLIN, 0};
            const int ready = poll(&descriptor, 1, 200);
            if (ready < 0 && errno == EINTR) continue;
            if (ready <= 0 || !(descriptor.revents & POLLIN)) continue;

            uint8_t buffer[65536];
            sockaddr_in source {};
            socklen_t sourceLength = sizeof(source);
            const ssize_t count = recvfrom(socketFd_, buffer, sizeof(buffer), 0,
                                           reinterpret_cast<sockaddr*>(&source),
                                           &sourceLength);
            if (count <= 0) continue;
            if (source.sin_port != htons(kProtocolPort) ||
                source.sin_addr.s_addr != clientNetworkAddress_.s_addr) {
                continue;
            }

            Frame frame;
            std::string error;
            if (!decodeFrame(buffer, static_cast<std::size_t>(count), frame, error) ||
                frame.target != kBroadcastTarget) {
                std::lock_guard<std::mutex> lock(mutex_);
                ++invalidDatagrams_;
                continue;
            }
            if (frame.command == CommandStream) {
                recordStreamFrame(frame, static_cast<std::size_t>(count));
                continue;
            }

            std::lock_guard<std::mutex> lock(mutex_);
            QueuedFrame queued;
            queued.serial = ++receiveSerial_;
            queued.frame = frame;
            controlFrames_.push_back(queued);
            while (controlFrames_.size() > 32) controlFrames_.pop_front();
            condition_.notify_all();
        }
    }

    std::string interfaceName_;
    std::string apAddress_;
    std::string clientAddress_;
    std::string destinationAddress_;
    bool bindToDevice_ = true;
    int socketFd_ = -1;
    sockaddr_in destination_ {};
    in_addr clientNetworkAddress_ {};
    std::atomic<bool> stopping_ {false};
    std::thread receiverThread_;

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<QueuedFrame> controlFrames_;
    uint64_t receiveSerial_ = 0;
    uint64_t streamArrivalSerial_ = 0;
    bool streaming_ = false;
    bool haveSequence_ = false;
    uint8_t lastSequence_ = 0;
    uint64_t totalPackets_ = 0;
    uint64_t totalBytes_ = 0;
    uint64_t intervalPackets_ = 0;
    uint64_t intervalBytes_ = 0;
    uint64_t missingPackets_ = 0;
    uint64_t duplicatePackets_ = 0;
    uint64_t outOfOrderPackets_ = 0;
    uint64_t invalidDatagrams_ = 0;
    std::size_t lastFrameLength_ = 0;
    std::chrono::steady_clock::time_point lastStatsSample_ =
        std::chrono::steady_clock::now();
};

void showMenu(bool connected,
              int frequency,
              const HotspotConfig& hotspot,
              const ClientLease& lease,
              const StreamStats& stats);

std::string formatStats(const StreamStats& stats)
{
    std::ostringstream output;
    output << "Stream: ";
    if (!stats.active) {
        output << "inactive";
        return output.str();
    }
    output << "fps=" << std::fixed << std::setprecision(1) << stats.fps
           << ", rate=" << std::setprecision(1) << stats.bytesPerSecond / 1024.0
           << " KiB/s, packets=" << stats.totalPackets
           << ", missing=" << stats.missingPackets
           << ", duplicate=" << stats.duplicatePackets
           << ", out-of-order=" << stats.outOfOrderPackets
           << ", invalid=" << stats.invalidDatagrams
           << ", last=" << stats.lastFrameLength << " bytes";
    return output.str();
}

void showMenu(bool connected,
              int frequency,
              const HotspotConfig& hotspot,
              const ClientLease& lease,
              const StreamStats& stats)
{
    std::cout
        << "\n==== VDSuit Wireless Receiver Emulator ====\n"
        << "Hotspot: SSID=" << hotspot.ssid
        << ", " << hotspot.addressing.apAddress << "/24"
        << ", channel=" << hotspot.channel << "\n"
        << "Transmitter: " << lease.macAddress << " / " << lease.ipAddress << "\n"
        << "Protocol: " << (connected ? "connected" : "disconnected");
    if (connected) std::cout << ", " << frequency << " Hz";
    std::cout
        << '\n' << formatStats(stats)
        << "\n1. Link, handshake and start data\n"
        << "2. Disconnect\n"
        << "3. Set frame rate\n"
        << "0. Exit\n"
        << "Select: " << std::flush;
}

void refreshStatsLine(const StreamStats& stats)
{
    if (!isatty(STDOUT_FILENO)) return;

    // The Stream line is five rows above the Select prompt. Save the current
    // cursor (including any partially typed input), update only that line, and
    // restore the cursor without clearing the terminal.
    std::cout << "\033[s\033[5A\r\033[2K"
              << formatStats(stats)
              << "\033[u" << std::flush;
}

bool readFrequency(int& frequency)
{
    std::cout << "Frame rate (60/72/80/96, blank to cancel): " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) return false;
    if (line.empty()) return false;
    std::istringstream input(line);
    int value = 0;
    char trailing = 0;
    if (!(input >> value) || (input >> trailing) || !isSupportedFrequency(value)) {
        std::cout << "Invalid frame rate. Supported values: 60, 72, 80, 96.\n";
        return false;
    }
    frequency = value;
    return true;
}

} // namespace

#ifndef VDSUIT_WIRELESS_RECEIVER_NO_MAIN
int main(int argc, char** argv)
{
    installSignalHandlers();
    AppOptions options;
    bool helpRequested = false;
    if (!parseArguments(argc, argv, options, helpRequested)) return helpRequested ? 0 : 1;
    if (options.ssid.empty()) {
        std::cout << "Receiver serial / SSID: " << std::flush;
        if (!std::getline(std::cin, options.ssid)) return 1;
    }

    HotspotConfig hotspotConfig;
    hotspotConfig.interfaceName = options.interfaceName;
    hotspotConfig.ssid = options.ssid;
    std::string error;
    if (!validateInterfaceName(options.interfaceName)) {
        std::cerr << "Invalid interface name.\n";
        return 1;
    }
    if (!validateSerialSsid(options.ssid)) {
        std::cerr << "SSID/serial must contain 1-32 decimal digits.\n";
        return 1;
    }
    if (!deriveHotspotAddressing(options.apAddress, hotspotConfig.addressing, error)) {
        std::cerr << "Invalid --ip: " << error << '\n';
        return 1;
    }

    std::cout << "Starting isolated hotspot on " << hotspotConfig.interfaceName << "...\n";
    HotspotManager hotspot(hotspotConfig);
    if (!hotspot.start(error)) {
        std::cerr << "Hotspot startup failed: " << error << '\n';
        return 2;
    }
    std::cout << "Hotspot ready: SSID=" << hotspotConfig.ssid
              << ", password=" << hotspotConfig.passphrase
              << ", channel=" << hotspotConfig.channel
              << ", AP=" << hotspotConfig.addressing.apCidr
              << ", DHCP=" << hotspotConfig.addressing.clientAddress << "\n"
              << "Waiting for the transmitter to associate and obtain DHCP...\n";

    ClientLease lease;
    if (!hotspot.waitForClient(shutdownRequested, lease, error)) {
        if (!shutdownRequested()) std::cerr << "Transmitter wait failed: " << error << '\n';
        return shutdownRequested() ? 0 : 3;
    }
    std::cout << "Transmitter ready: MAC=" << lease.macAddress
              << ", IP=" << lease.ipAddress << "\n";

    UdpSession session(options.interfaceName,
                       hotspotConfig.addressing.apAddress,
                       hotspotConfig.addressing.clientAddress);
    if (!session.open(error)) {
        std::cerr << "UDP startup failed: " << error << '\n';
        return 4;
    }

    bool connected = false;
    int frequency = 0;
    bool showMenuNow = true;
    StreamStats latestStats = session.sampleStats();
    std::string line;
    while (!shutdownRequested()) {
        if (showMenuNow) {
            latestStats = session.sampleStats();
            showMenu(connected, frequency, hotspotConfig, lease, latestStats);
            showMenuNow = false;
        }

        pollfd inputDescriptor {STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&inputDescriptor, 1, 1000);
        if (ready < 0 && errno == EINTR) continue;
        if (ready == 0) {
            latestStats = session.sampleStats();
            refreshStatsLine(latestStats);
            continue;
        }
        if (ready < 0 || !(inputDescriptor.revents & (POLLIN | POLLHUP))) break;
        if (!std::getline(std::cin, line)) break;
        if (line.empty()) {
            showMenuNow = true;
            continue;
        }

        switch (line[0]) {
        case '1': {
            if (connected) {
                std::cout << "Already connected.\n";
                break;
            }
            HandshakeInfo handshake;
            if (!session.connectAndStart(handshake, error)) {
                std::cerr << "Connect/start failed: " << error << '\n';
                break;
            }
            connected = true;
            frequency = handshake.frequency;
            std::cout << "Connected: deviceType="
                      << static_cast<unsigned int>(handshake.deviceType)
                      << ", mode=" << static_cast<unsigned int>(handshake.communicationMode)
                      << ", version=" << handshake.version
                      << ", voltage=" << handshake.voltageMillivolts << " mV"
                      << ", frequency=" << frequency << " Hz\n";
            break;
        }
        case '2':
            if (!connected) {
                std::cout << "Already disconnected.\n";
            } else if (!session.disconnect(error)) {
                std::cerr << "Disconnect failed: " << error << '\n';
            } else {
                connected = false;
                frequency = 0;
                std::cout << "Disconnected.\n";
            }
            break;
        case '3': {
            if (!connected) {
                std::cout << "Connect the protocol first with option 1.\n";
                break;
            }
            int requestedFrequency = 0;
            if (!readFrequency(requestedFrequency)) break;
            if (!session.setFrequency(requestedFrequency, error)) {
                std::cerr << "Set frequency failed: " << error << '\n';
            } else {
                frequency = requestedFrequency;
                std::cout << "Frequency set to " << frequency << " Hz; stream restarted.\n";
            }
            break;
        }
        case '0':
            if (connected) {
                if (!session.disconnect(error)) {
                    std::cerr << "Final disconnect warning: " << error << '\n';
                }
                connected = false;
            }
            session.close();
            hotspot.stop();
            std::cout << "Hotspot stopped and " << options.interfaceName << " restored.\n";
            return 0;
        default:
            std::cout << "Unknown command.\n";
            break;
        }
        showMenuNow = true;
    }

    if (connected && !shutdownRequested()) {
        session.disconnect(error);
    }
    session.close();
    hotspot.stop();
    std::cout << "\nHotspot stopped and " << options.interfaceName << " restored.\n";
    return 0;
}
#endif
