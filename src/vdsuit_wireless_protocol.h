#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace vdsuit_wireless {

constexpr uint16_t kBroadcastTarget = 0xffff;
constexpr uint16_t kProtocolPort = 8080;

enum Command : uint8_t {
    CommandDisconnect = 0x7f,
    CommandHandshake = 0x80,
    CommandLinkProbe = 0x81,
    CommandSetFrequency = 0x82,
    CommandStream = 0xc9,
};

struct Frame {
    uint8_t sequence = 0;
    uint16_t target = 0;
    uint8_t command = 0;
    std::vector<uint8_t> payload;
};

struct HandshakeInfo {
    uint8_t frequency = 0;
    uint8_t communicationMode = 0;
    uint16_t voltageMillivolts = 0;
    std::string version;
    uint8_t deviceType = 0;
};

uint8_t checksum8(const uint8_t* data, std::size_t length);

// The vendor SDK calls this operation encryption. It is a fixed bit/byte
// permutation and is intentionally exposed for deterministic protocol tests.
std::vector<uint8_t> encodeBodyPermutation(const std::vector<uint8_t>& plain);
std::vector<uint8_t> decodeBodyPermutation(const std::vector<uint8_t>& encoded);

std::vector<uint8_t> encodeFrame(const Frame& frame);
bool decodeFrame(const uint8_t* data,
                 std::size_t length,
                 Frame& frame,
                 std::string& error);

std::vector<uint8_t> makeHandshakeRequest(uint8_t requestedDeviceType = 0x03);
bool validateHandshakeRequest(const std::vector<uint8_t>& payload,
                              uint8_t expectedDeviceType,
                              std::string& error);
bool parseHandshakeResponse(const std::vector<uint8_t>& payload,
                            HandshakeInfo& info,
                            std::string& error);

std::vector<uint8_t> streamRequestPayload();
bool isSupportedFrequency(int frequency);
std::string hexString(const uint8_t* data, std::size_t length);

} // namespace vdsuit_wireless
