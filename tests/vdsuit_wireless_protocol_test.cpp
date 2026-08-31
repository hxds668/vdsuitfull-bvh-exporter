#include "vdsuit_wireless_protocol.h"
#include "wireless_hotspot.h"

#include <cassert>
#include <cctype>
#include <iostream>
#include <string>
#include <vector>

using namespace vdsuit_wireless;

namespace {

unsigned int hexDigit(char value)
{
    if (value >= '0' && value <= '9') return static_cast<unsigned int>(value - '0');
    value = static_cast<char>(std::tolower(static_cast<unsigned char>(value)));
    return static_cast<unsigned int>(value - 'a' + 10);
}

std::vector<uint8_t> fromHex(const std::string& text)
{
    assert(text.size() % 2 == 0);
    std::vector<uint8_t> output;
    output.reserve(text.size() / 2);
    for (std::size_t i = 0; i < text.size(); i += 2) {
        output.push_back(static_cast<uint8_t>((hexDigit(text[i]) << 4) |
                                              hexDigit(text[i + 1])));
    }
    return output;
}

Frame decode(const std::string& text)
{
    const std::vector<uint8_t> bytes = fromHex(text);
    Frame frame;
    std::string error;
    assert(decodeFrame(bytes.data(), bytes.size(), frame, error));
    assert(error.empty());
    return frame;
}

void testCapturedCommandVectors()
{
    Frame link;
    link.target = kBroadcastTarget;
    link.command = CommandLinkProbe;
    const std::vector<uint8_t> linkBytes = encodeFrame(link);
    assert(hexString(linkBytes.data(), linkBytes.size()) ==
           "5b000005ffff8100005d3c");

    Frame frequency;
    frequency.target = kBroadcastTarget;
    frequency.command = CommandSetFrequency;
    frequency.payload.push_back(60);
    const std::vector<uint8_t> frequencyBytes = encodeFrame(frequency);
    assert(hexString(frequencyBytes.data(), frequencyBytes.size()) ==
           "5b000006ffff090080745db9");

    Frame frequencyResponse = decode("5b000007ffff09000880745dc2");
    assert(frequencyResponse.command == CommandSetFrequency);
    assert(frequencyResponse.payload == fromHex("013c"));

    Frame disconnect;
    disconnect.target = kBroadcastTarget;
    disconnect.command = CommandDisconnect;
    const std::vector<uint8_t> disconnectBytes = encodeFrame(disconnect);
    assert(hexString(disconnectBytes.data(), disconnectBytes.size()) ==
           "5b000005fffffe00005db9");

    Frame disconnectResponse = decode("5b000006fffffe0080805dba");
    assert(disconnectResponse.command == CommandDisconnect);
    assert(disconnectResponse.payload == fromHex("01"));

    Frame stream;
    stream.target = kBroadcastTarget;
    stream.command = CommandStream;
    stream.payload = streamRequestPayload();
    const std::vector<uint8_t> streamBytes = encodeFrame(stream);
    assert(hexString(streamBytes.data(), streamBytes.size()) ==
           "5b00000dffff10ff0093a8ffff0000fec05dc9");

    const Frame capturedStream = decode(
        "5b070011000030ff009380ffb004000000000000115dd6");
    assert(capturedStream.sequence == 7);
    assert(capturedStream.target == kBroadcastTarget);
    assert(capturedStream.command == CommandStream);
    assert(capturedStream.payload.size() == 12);
}

void testPermutationRoundTrips()
{
    for (std::size_t length = 0; length <= 40; ++length) {
        std::vector<uint8_t> plain(length);
        for (std::size_t i = 0; i < length; ++i) {
            plain[i] = static_cast<uint8_t>((i * 37 + length * 11) & 0xff);
        }
        assert(decodeBodyPermutation(encodeBodyPermutation(plain)) == plain);
    }
}

void testFrameValidation()
{
    Frame frame;
    frame.target = kBroadcastTarget;
    frame.command = CommandSetFrequency;
    frame.payload.push_back(72);
    std::vector<uint8_t> bytes = encodeFrame(frame);

    Frame decoded;
    std::string error;
    assert(decodeFrame(bytes.data(), bytes.size(), decoded, error));
    assert(decoded.command == CommandSetFrequency);
    assert(decoded.payload.size() == 1 && decoded.payload[0] == 72);

    bytes.back() ^= 1;
    assert(!decodeFrame(bytes.data(), bytes.size(), decoded, error));
    assert(error == "checksum mismatch");

    bytes = encodeFrame(frame);
    bytes[3] += 1;
    assert(!decodeFrame(bytes.data(), bytes.size(), decoded, error));
    assert(error == "outer length does not match datagram size");
}

void testHandshake()
{
    std::string error;
    const std::vector<uint8_t> request = makeHandshakeRequest(3);
    assert(validateHandshakeRequest(request, 3, error));

    std::vector<uint8_t> damaged = request;
    damaged[8] ^= 1;
    assert(!validateHandshakeRequest(damaged, 3, error));

    const Frame response = decode(
        "5b00003affff800035013c01100dffffffffffffffffffffffffffffffffffff"
        "485f56332e31535f56332e3105a39aa5b67121f84580be80e8570bd903005d67");
    HandshakeInfo info;
    assert(parseHandshakeResponse(response.payload, info, error));
    assert(info.frequency == 60);
    assert(info.communicationMode == 1);
    assert(info.voltageMillivolts == 4109);
    assert(info.version == "H_V3.1S_V3.1");
    assert(info.deviceType == 3);
}

void testFrequencyValidation()
{
    assert(isSupportedFrequency(60));
    assert(isSupportedFrequency(72));
    assert(isSupportedFrequency(80));
    assert(isSupportedFrequency(96));
    assert(!isSupportedFrequency(59));
    assert(!isSupportedFrequency(120));
}

void testHotspotConfiguration()
{
    HotspotAddressing addressing;
    std::string error;
    assert(deriveHotspotAddressing("192.168.18.1", addressing, error));
    assert(addressing.apCidr == "192.168.18.1/24");
    assert(addressing.clientAddress == "192.168.18.3");
    assert(addressing.networkCidr == "192.168.18.0/24");
    assert(addressing.broadcastAddress == "192.168.18.255");
    assert(deriveHotspotAddressing("10.20.30.9", addressing, error));
    assert(addressing.clientAddress == "10.20.30.3");
    assert(!deriveHotspotAddressing("8.8.8.1", addressing, error));
    assert(!deriveHotspotAddressing("192.168.18.3", addressing, error));
    assert(!deriveHotspotAddressing("not-an-ip", addressing, error));

    assert(validateInterfaceName("wlan0"));
    assert(validateInterfaceName("wlx00-11.2"));
    assert(!validateInterfaceName("wlan0;down"));
    assert(validateSerialSsid("12300283"));
    assert(!validateSerialSsid("serial-1"));

    HotspotConfig config;
    config.ssid = "12300283";
    assert(deriveHotspotAddressing("192.168.18.1", config.addressing, error));
    const std::string hostapd = makeHostapdConfig(config, "/run/test/control");
    assert(hostapd.find("interface=wlan0\n") != std::string::npos);
    assert(hostapd.find("ssid=12300283\n") != std::string::npos);
    assert(hostapd.find("channel=165\n") != std::string::npos);
    assert(hostapd.find("wpa_passphrase=12345678\n") != std::string::npos);
    assert(hostapd.find("rsn_pairwise=CCMP\n") != std::string::npos);

    const std::string dnsmasq = makeDnsmasqConfig(
        config, "/run/test/leases", "/run/test/pid");
    assert(dnsmasq.find(
        "dhcp-range=192.168.18.3,192.168.18.3,255.255.255.0,18h\n") !=
        std::string::npos);
    assert(dnsmasq.find("dhcp-option=3,192.168.18.1\n") != std::string::npos);
    assert(dnsmasq.find("dhcp-option=6,0.0.0.0\n") != std::string::npos);
    assert(dnsmasq.find("dhcp-option=28,255.255.255.255\n") != std::string::npos);
    assert(dnsmasq.find("dhcp-broadcast\n") == std::string::npos);
    assert(dnsmasq.find("port=0\n") != std::string::npos);
}

} // namespace

int main()
{
    testCapturedCommandVectors();
    testPermutationRoundTrips();
    testFrameValidation();
    testHandshake();
    testFrequencyValidation();
    testHotspotConfiguration();
    std::cout << "VDSuit wireless protocol tests passed.\n";
    return 0;
}
