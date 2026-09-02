#define VDSUIT_WIRELESS_RECEIVER_NO_MAIN
#include "../src/wireless_receiver.cpp"

#include <cassert>
#include <fstream>

namespace {

std::vector<uint8_t> bytesFromHex(const std::string& text)
{
    std::vector<uint8_t> output;
    for (std::size_t i = 0; i < text.size(); i += 2) {
        output.push_back(static_cast<uint8_t>(std::stoul(text.substr(i, 2), nullptr, 16)));
    }
    return output;
}

class MockTransmitter {
public:
    ~MockTransmitter()
    {
        stop();
    }

    bool start()
    {
        socketFd_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (socketFd_ < 0) return false;
        int enabled = 1;
        setsockopt(socketFd_, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
        sockaddr_in address {};
        address.sin_family = AF_INET;
        address.sin_port = htons(kProtocolPort);
        inet_pton(AF_INET, "127.0.0.2", &address.sin_addr);
        if (bind(socketFd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            ::close(socketFd_);
            socketFd_ = -1;
            return false;
        }
        stopping_.store(false);
        thread_ = std::thread(&MockTransmitter::run, this);
        return true;
    }

    void stop()
    {
        stopping_.store(true);
        if (socketFd_ >= 0) shutdown(socketFd_, SHUT_RDWR);
        if (thread_.joinable()) thread_.join();
        if (socketFd_ >= 0) ::close(socketFd_);
        socketFd_ = -1;
    }

    std::vector<uint8_t> commands() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return commands_;
    }

private:
    void reply(const sockaddr_in& destination,
               uint8_t command,
               const std::vector<uint8_t>& payload,
               uint8_t sequence = 0)
    {
        Frame response;
        response.sequence = sequence;
        response.target = kBroadcastTarget;
        response.command = command;
        response.payload = payload;
        const std::vector<uint8_t> bytes = encodeFrame(response);
        sendto(socketFd_, bytes.data(), bytes.size(), 0,
               reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
    }

    void run()
    {
        while (!stopping_.load()) {
            pollfd descriptor {socketFd_, POLLIN, 0};
            if (poll(&descriptor, 1, 200) <= 0 || !(descriptor.revents & POLLIN)) continue;
            uint8_t buffer[2048];
            sockaddr_in source {};
            socklen_t sourceLength = sizeof(source);
            const ssize_t count = recvfrom(socketFd_, buffer, sizeof(buffer), 0,
                                           reinterpret_cast<sockaddr*>(&source),
                                           &sourceLength);
            if (count <= 0) continue;

            Frame request;
            std::string error;
            if (!decodeFrame(buffer, static_cast<std::size_t>(count), request, error)) continue;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                commands_.push_back(request.command);
            }

            switch (request.command) {
            case CommandLinkProbe:
                reply(source, CommandLinkProbe, std::vector<uint8_t>(1, 1));
                break;
            case CommandHandshake:
                assert(validateHandshakeRequest(request.payload, 3, error));
                reply(source, CommandHandshake, bytesFromHex(
                    "013c01100dffffffffffffffffffffffffffffffffffff"
                    "485f56332e31535f56332e3105a39aa5b67121f84580be80e8570bd90300"));
                break;
            case CommandSetFrequency: {
                assert(request.payload.size() == 1);
                std::vector<uint8_t> payload;
                payload.push_back(1);
                payload.push_back(request.payload[0]);
                reply(source, CommandSetFrequency, payload);
                break;
            }
            case CommandStream:
                reply(source, CommandStream, std::vector<uint8_t>(12, 0), sequence_++);
                break;
            case CommandDisconnect:
                reply(source, CommandDisconnect, std::vector<uint8_t>(1, 1));
                break;
            default:
                break;
            }
        }
    }

    int socketFd_ = -1;
    std::atomic<bool> stopping_ {false};
    std::thread thread_;
    mutable std::mutex mutex_;
    std::vector<uint8_t> commands_;
    uint8_t sequence_ = 0;
};

void testSessionStateMachine()
{
    const std::string recordingPath =
        "/tmp/vdsuit_wireless_raw_test_" + std::to_string(getpid()) + ".jsonl";
    unlink(recordingPath.c_str());

    MockTransmitter transmitter;
    assert(transmitter.start());

    UdpSession session("lo", "127.0.0.1", "127.0.0.2", "127.0.0.2", false);
    std::string error;
    assert(session.open(error));
    assert(session.startRawRecording(recordingPath, error));

    HandshakeInfo handshake;
    assert(session.connectAndStart(handshake, error));
    assert(handshake.deviceType == 3);
    assert(handshake.frequency == 60);
    assert(session.streaming());
    StreamStats stats = session.sampleStats();
    assert(stats.totalPackets >= 1);

    assert(session.setFrequency(72, error));
    assert(session.streaming());

    RawRecordingStats recording = session.stopRawRecording();
    assert(!recording.active);
    assert(recording.error.empty());
    assert(recording.frameCount == 2);
    assert(recording.rawBytes > 0);

    std::ifstream input(recordingPath.c_str(), std::ios::binary);
    assert(input);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) lines.push_back(line);
    assert(lines.size() == recording.frameCount);

    for (std::size_t i = 0; i < lines.size(); ++i) {
        Frame expected;
        expected.sequence = static_cast<uint8_t>(i);
        expected.target = kBroadcastTarget;
        expected.command = CommandStream;
        expected.payload.assign(12, 0);
        const std::vector<uint8_t> expectedWire = encodeFrame(expected);

        assert(lines[i].find("\"schema\":\"vdsuit-wireless-raw-v1\"") !=
               std::string::npos);
        assert(lines[i].find("\"record_index\":" + std::to_string(i)) !=
               std::string::npos);
        assert(lines[i].find("\"sequence\":" + std::to_string(i)) !=
               std::string::npos);
        assert(lines[i].find("\"target\":65535") != std::string::npos);
        assert(lines[i].find("\"command\":201") != std::string::npos);
        assert(lines[i].find("\"payload_length\":12") != std::string::npos);
        assert(lines[i].find("\"wire_hex\":\"" +
                             hexString(expectedWire.data(), expectedWire.size()) + "\"") !=
               std::string::npos);
        assert(!lines[i].empty() && lines[i][0] == '{' && lines[i].back() == '}');
    }

    assert(session.disconnect(error));
    assert(!session.streaming());

    session.close();
    transmitter.stop();

    const std::vector<uint8_t> expected = {
        CommandLinkProbe, CommandHandshake, CommandLinkProbe, CommandStream,
        CommandLinkProbe, CommandSetFrequency, CommandLinkProbe, CommandStream,
        CommandDisconnect
    };
    assert(transmitter.commands() == expected);
    unlink(recordingPath.c_str());
}

} // namespace

int main()
{
    testSessionStateMachine();
    std::cout << "VDSuit wireless UDP session tests passed.\n";
    return 0;
}
