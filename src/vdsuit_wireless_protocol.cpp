#include "vdsuit_wireless_protocol.h"

#include <algorithm>
#include <array>
#include <iomanip>
#include <random>
#include <sstream>

namespace vdsuit_wireless {
namespace {

constexpr uint8_t kStartMarker = 0x5b;
constexpr uint8_t kEndMarker = 0x5d;
constexpr std::size_t kOuterOverhead = 6;
constexpr std::size_t kPlainBodyHeaderSize = 5;

const std::array<unsigned int, 8> kPermutation = {{7, 6, 4, 1, 3, 2, 5, 0}};
const std::array<uint8_t, 8> kHandshakeMask = {{
    '@', '#', '$', '%', '^', '&', '*', '!'
}};
const std::array<uint8_t, 8> kHandshakeRandomMask = {{
    'd', 't', '5', '6', 'x', 'y', '8', '8'
}};

uint16_t readBigEndian16(const uint8_t* data)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) |
                                 static_cast<uint16_t>(data[1]));
}

void appendBigEndian16(std::vector<uint8_t>& output, uint16_t value)
{
    output.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
    output.push_back(static_cast<uint8_t>(value & 0xff));
}

uint8_t decodeBits(uint8_t encoded)
{
    uint8_t plain = 0;
    for (unsigned int bit = 0; bit < 8; ++bit) {
        plain |= static_cast<uint8_t>(((encoded >> bit) & 1U) << kPermutation[bit]);
    }
    return plain;
}

uint8_t encodeBits(uint8_t plain)
{
    uint8_t encoded = 0;
    for (unsigned int bit = 0; bit < 8; ++bit) {
        encoded |= static_cast<uint8_t>(((plain >> kPermutation[bit]) & 1U) << bit);
    }
    return encoded;
}

bool bodyNeedsPermutation(uint16_t target, uint8_t command)
{
    if ((target >> 8) != 0xff) return true;
    return command != 0x80 && command != 0x81 && command != 0xd0 && command != 0xd1;
}

bool validateHandshakePair(const uint8_t* first, const uint8_t* second)
{
    for (std::size_t i = 0; i < kHandshakeMask.size(); ++i) {
        if (static_cast<uint8_t>(first[i] ^ kHandshakeMask[i]) != second[i]) {
            return false;
        }
    }
    return true;
}

} // namespace

uint8_t checksum8(const uint8_t* data, std::size_t length)
{
    uint8_t checksum = 0;
    for (std::size_t i = 0; i < length; ++i) {
        checksum = static_cast<uint8_t>(checksum + data[i]);
    }
    return checksum;
}

std::vector<uint8_t> encodeBodyPermutation(const std::vector<uint8_t>& plain)
{
    std::vector<uint8_t> encoded(plain.size());
    const std::size_t fullLength = (plain.size() / 8) * 8;

    for (std::size_t offset = 0; offset < fullLength; offset += 8) {
        for (std::size_t i = 0; i < 8; ++i) {
            encoded[offset + i] = encodeBits(plain[offset + kPermutation[i]]);
        }
    }
    for (std::size_t i = fullLength; i < plain.size(); ++i) {
        encoded[i] = encodeBits(plain[i]);
    }
    return encoded;
}

std::vector<uint8_t> decodeBodyPermutation(const std::vector<uint8_t>& encoded)
{
    std::vector<uint8_t> plain(encoded.size());
    const std::size_t fullLength = (encoded.size() / 8) * 8;

    for (std::size_t offset = 0; offset < fullLength; offset += 8) {
        for (std::size_t i = 0; i < 8; ++i) {
            plain[offset + kPermutation[i]] = decodeBits(encoded[offset + i]);
        }
    }
    for (std::size_t i = fullLength; i < encoded.size(); ++i) {
        plain[i] = decodeBits(encoded[i]);
    }
    return plain;
}

std::vector<uint8_t> encodeFrame(const Frame& frame)
{
    if (frame.payload.size() > 0xffffU - kPlainBodyHeaderSize) {
        return std::vector<uint8_t>();
    }

    std::vector<uint8_t> body;
    body.reserve(kPlainBodyHeaderSize + frame.payload.size());
    appendBigEndian16(body, frame.target);
    body.push_back(frame.command);
    appendBigEndian16(body, static_cast<uint16_t>(frame.payload.size()));
    body.insert(body.end(), frame.payload.begin(), frame.payload.end());

    if (bodyNeedsPermutation(frame.target, frame.command)) {
        body = encodeBodyPermutation(body);
    }

    std::vector<uint8_t> output;
    output.reserve(body.size() + kOuterOverhead);
    output.push_back(kStartMarker);
    output.push_back(frame.sequence);
    appendBigEndian16(output, static_cast<uint16_t>(body.size()));
    output.insert(output.end(), body.begin(), body.end());
    output.push_back(kEndMarker);
    output.push_back(checksum8(output.data(), output.size()));
    return output;
}

bool decodeFrame(const uint8_t* data,
                 std::size_t length,
                 Frame& frame,
                 std::string& error)
{
    error.clear();
    frame = Frame();
    if (!data || length < kOuterOverhead + kPlainBodyHeaderSize) {
        error = "frame is too short";
        return false;
    }
    if (data[0] != kStartMarker) {
        error = "missing 0x5b start marker";
        return false;
    }

    const uint16_t bodyLength = readBigEndian16(data + 2);
    if (length != static_cast<std::size_t>(bodyLength) + kOuterOverhead) {
        error = "outer length does not match datagram size";
        return false;
    }
    if (data[length - 2] != kEndMarker) {
        error = "missing 0x5d end marker";
        return false;
    }
    if (checksum8(data, length - 1) != data[length - 1]) {
        error = "checksum mismatch";
        return false;
    }

    std::vector<uint8_t> body(data + 4, data + 4 + bodyLength);
    if (body.size() < kPlainBodyHeaderSize) {
        error = "body is too short";
        return false;
    }

    // The target and command are readable only for the four commands excluded
    // from the SDK permutation. All other bodies must first be decoded.
    const uint16_t wireTarget = readBigEndian16(body.data());
    const uint8_t wireCommand = body[2];
    if (bodyNeedsPermutation(wireTarget, wireCommand)) {
        body = decodeBodyPermutation(body);
    }

    frame.sequence = data[1];
    frame.target = readBigEndian16(body.data());
    frame.command = body[2];
    const uint16_t payloadLength = readBigEndian16(body.data() + 3);
    if (body.size() != kPlainBodyHeaderSize + payloadLength) {
        error = "payload length does not match body size";
        return false;
    }
    frame.payload.assign(body.begin() + kPlainBodyHeaderSize, body.end());
    return true;
}

std::vector<uint8_t> makeHandshakeRequest(uint8_t requestedDeviceType)
{
    std::random_device randomDevice;
    std::mt19937 random(randomDevice());
    std::uniform_int_distribution<int> distribution(0, 254);

    std::vector<uint8_t> payload(18, 0);
    for (std::size_t i = 0; i < 8; ++i) {
        const uint8_t randomByte = static_cast<uint8_t>(distribution(random));
        payload[i] = static_cast<uint8_t>(randomByte ^ kHandshakeRandomMask[i]);
        payload[8 + i] = static_cast<uint8_t>(payload[i] ^ kHandshakeMask[i]);
    }
    payload[16] = requestedDeviceType;
    payload[17] = 0;
    return payload;
}

bool validateHandshakeRequest(const std::vector<uint8_t>& payload,
                              uint8_t expectedDeviceType,
                              std::string& error)
{
    error.clear();
    if (payload.size() != 18) {
        error = "handshake request must contain 18 bytes";
        return false;
    }
    if (!validateHandshakePair(payload.data(), payload.data() + 8)) {
        error = "handshake request XOR check failed";
        return false;
    }
    if (payload[16] != expectedDeviceType || payload[17] != 0) {
        error = "handshake request device type or reserved byte is invalid";
        return false;
    }
    return true;
}

bool parseHandshakeResponse(const std::vector<uint8_t>& payload,
                            HandshakeInfo& info,
                            std::string& error)
{
    error.clear();
    info = HandshakeInfo();
    if (payload.size() != 53) {
        error = "handshake response must contain 53 bytes";
        return false;
    }
    if (payload[0] != 1) {
        error = "handshake response reports failure";
        return false;
    }
    if (!validateHandshakePair(payload.data() + 35, payload.data() + 43)) {
        error = "handshake response XOR check failed";
        return false;
    }
    if (payload[52] != 0) {
        error = "handshake response reserved byte is invalid";
        return false;
    }

    info.frequency = payload[1];
    info.communicationMode = payload[2];
    info.voltageMillivolts = readBigEndian16(payload.data() + 3);
    info.version.assign(payload.begin() + 23, payload.begin() + 35);
    info.deviceType = payload[51];
    return true;
}

std::vector<uint8_t> streamRequestPayload()
{
    const uint8_t payload[] = {0x07, 0xff, 0xff, 0xff, 0x00, 0x00, 0x7f, 0x21};
    return std::vector<uint8_t>(payload, payload + sizeof(payload));
}

bool isSupportedFrequency(int frequency)
{
    return frequency == 60 || frequency == 72 || frequency == 80 || frequency == 96;
}

std::string hexString(const uint8_t* data, std::size_t length)
{
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < length; ++i) {
        output << std::setw(2) << static_cast<unsigned int>(data[i]);
    }
    return output.str();
}

} // namespace vdsuit_wireless
