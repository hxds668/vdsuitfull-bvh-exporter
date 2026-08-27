#include "bvh_exporter.h"
#include "mocap_skeleton.h"
#include "network_streamer.h"

#include <arpa/inet.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <netinet/in.h>
#include <poll.h>
#include <set>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

using namespace VDSuitMiniDevice;

namespace {

std::size_t countOccurrences(const std::string& text, const std::string& needle)
{
    std::size_t count = 0;
    std::size_t position = 0;
    while ((position = text.find(needle, position)) != std::string::npos) {
        ++count;
        position += needle.size();
    }
    return count;
}

std::string receiveDatagram(int socketFd, int timeoutMs)
{
    pollfd descriptor {socketFd, POLLIN, 0};
    if (poll(&descriptor, 1, timeoutMs) <= 0) return std::string();
    char buffer[65536];
    ssize_t count = recvfrom(socketFd, buffer, sizeof(buffer), 0, nullptr, nullptr);
    if (count <= 0) return std::string();
    return std::string(buffer, static_cast<std::size_t>(count));
}

void fillInitialPositions(float body[NODES_BODY][3],
                          float right[NODES_HAND][3],
                          float left[NODES_HAND][3])
{
    for (int i = 0; i < NODES_BODY; ++i) {
        body[i][0] = static_cast<float>(i);
        body[i][1] = static_cast<float>(i) + 0.25f;
        body[i][2] = static_cast<float>(i) + 0.5f;
    }
    for (int i = 0; i < NODES_HAND; ++i) {
        right[i][0] = 100.0f + static_cast<float>(i);
        right[i][1] = 110.0f + static_cast<float>(i);
        right[i][2] = 120.0f + static_cast<float>(i);
        left[i][0] = 200.0f + static_cast<float>(i);
        left[i][1] = 210.0f + static_cast<float>(i);
        left[i][2] = 220.0f + static_cast<float>(i);
    }
}

_MocapDataWithVirtual_ makeFrame(int frameIndex)
{
    _MocapDataWithVirtual_ md {};
    md.isUpdate = true;
    md.frameIndex = frameIndex;
    for (int i = 0; i < NODES_BODY; ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            md.position_body[i][axis] = 1000.0f + i * 10.0f + axis;
        }
        md.quaternion_body[i][0] = 1.0f;
        md.quaternion_body[i][1] = 0.01f * i;
    }
    for (int i = 0; i < NODES_HAND; ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            md.position_rHand[i][axis] = 2000.0f + i * 10.0f + axis;
            md.position_lHand[i][axis] = 3000.0f + i * 10.0f + axis;
        }
        md.quaternion_rHand[i][0] = 1.0f;
        md.quaternion_rHand[i][2] = 0.01f * i;
        md.quaternion_lHand[i][0] = 1.0f;
        md.quaternion_lHand[i][3] = 0.01f * i;
    }
    return md;
}

void setIdentityQuaternions(_MocapDataWithVirtual_& md)
{
    std::memset(md.quaternion_body, 0, sizeof(md.quaternion_body));
    std::memset(md.quaternion_rHand, 0, sizeof(md.quaternion_rHand));
    std::memset(md.quaternion_lHand, 0, sizeof(md.quaternion_lHand));
    for (int i = 0; i < NODES_BODY; ++i) md.quaternion_body[i][0] = 1.0f;
    for (int i = 0; i < NODES_HAND; ++i) {
        md.quaternion_rHand[i][0] = 1.0f;
        md.quaternion_lHand[i][0] = 1.0f;
    }
}

std::vector<double> readLastMotionLine(const std::string& path)
{
    std::ifstream input(path);
    assert(input.is_open());
    std::string line;
    std::string lastLine;
    while (std::getline(input, line)) {
        if (!line.empty()) lastLine = line;
    }
    std::istringstream valuesStream(lastLine);
    std::vector<double> values;
    double value = 0.0;
    while (valuesStream >> value) values.push_back(value);
    return values;
}

void testEndpointParsing()
{
    std::string ip;
    std::string error;
    uint16_t port = 0;
    assert(parseNetworkEndpoint("127.0.0.1:9000", ip, port, error));
    assert(ip == "127.0.0.1");
    assert(port == 9000);
    assert(!parseNetworkEndpoint("localhost:9000", ip, port, error));
    assert(!parseNetworkEndpoint("127.0.0.1:0", ip, port, error));
    assert(!parseNetworkEndpoint("127.0.0.1:65536", ip, port, error));
}

void testSkeletonAndFrameSerialization()
{
    const std::vector<MocapJointDefinition>& definitions = mocapJointDefinitions();
    assert(definitions.size() == 23);
    const int expectedSdkIndex[NODES_BODY] = {
        BN_Hips,
        BN_RightUpperLeg, BN_RightLowerLeg, BN_RightFoot, BN_RightToe,
        BN_LeftUpperLeg, BN_LeftLowerLeg, BN_LeftFoot, BN_LeftToe,
        BN_Spine, BN_Spine1, BN_Spine2, BN_Spine3, BN_Neck, BN_Head,
        BN_RightShoulder, BN_RightUpperArm, BN_RightLowerArm, BN_RightHand,
        BN_LeftShoulder, BN_LeftUpperArm, BN_LeftLowerArm, BN_LeftHand
    };
    std::set<std::string> names;
    for (std::size_t i = 0; i < definitions.size(); ++i) {
        assert(definitions[i].parentIndex < static_cast<int>(i));
        assert(names.insert(definitions[i].name).second);
        assert(definitions[i].source == MocapJointSource::Body);
        assert(definitions[i].sdkIndex == expectedSdkIndex[i]);
    }

    float body[NODES_BODY][3] {};
    float right[NODES_HAND][3] {};
    float left[NODES_HAND][3] {};
    fillInitialPositions(body, right, left);
    std::string skeleton = serializeSkeletonJsonLine(body);
    assert(skeleton.back() == '\n');
    assert(skeleton.find("\"type\":\"skeleton\"") != std::string::npos);
    assert(skeleton.find("\"joint_count\":23") != std::string::npos);
    assert(countOccurrences(skeleton, "\"initial_position\":") == 23);
    assert(countOccurrences(skeleton, "\"offset\":") == 23);

    _MocapDataWithVirtual_ md = makeFrame(77);
    assert(mocapJointPosition(md, definitions[BN_RightHand])[0] ==
           md.position_body[BN_RightHand][0]);
    assert(mocapJointPosition(md, definitions[BN_LeftHand])[0] ==
           md.position_body[BN_LeftHand][0]);
    assert(mocapJointPosition(md, definitions[BN_RightFoot])[0] ==
           md.position_body[BN_RightFoot][0]);
    assert(mocapJointQuaternion(md, definitions[BN_LeftUpperArm])[1] ==
           md.quaternion_body[BN_LeftUpperArm][1]);
    std::string frame = serializeFrameJsonLine(md);
    assert(frame.back() == '\n');
    assert(frame.find("\"type\":\"frame\"") != std::string::npos);
    assert(frame.find("\"frame_index\":77") != std::string::npos);
    assert(countOccurrences(frame, "\"position\":") == 23);
    assert(countOccurrences(frame, "\"quaternion\":") == 23);
    assert(frame.find("1220") != std::string::npos);
    assert(frame.find("2010") == std::string::npos);

    md.position_body[BN_Hips][0] = std::nanf("");
    frame = serializeFrameJsonLine(md);
    assert(frame.find("\"position\":[null,") != std::string::npos);
}

void testUdpDatagramsAndOrdering()
{
    int receiver = socket(AF_INET, SOCK_DGRAM, 0);
    assert(receiver >= 0);

    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    assert(bind(receiver, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
    socklen_t addressLength = sizeof(address);
    assert(getsockname(receiver, reinterpret_cast<sockaddr*>(&address), &addressLength) == 0);

    float body[NODES_BODY][3] {};
    float right[NODES_HAND][3] {};
    float left[NODES_HAND][3] {};
    fillInitialPositions(body, right, left);

    NetworkStreamer streamer("127.0.0.1", ntohs(address.sin_port), body);
    assert(!streamer.isRunning());
    assert(receiveDatagram(receiver, 100).empty());
    streamer.start();
    assert(streamer.isRunning());
    std::string first = receiveDatagram(receiver, 3000);
    assert(first.find("\"type\":\"skeleton\"") != std::string::npos);
    assert(first.back() == '\n');

    streamer.enqueueFrame(makeFrame(101));
    std::string second = receiveDatagram(receiver, 3000);
    assert(second.find("\"type\":\"frame\"") != std::string::npos);
    assert(second.find("\"frame_index\":101") != std::string::npos);
    assert(second.back() == '\n');

    streamer.enqueueFrame(makeFrame(102));
    std::string third = receiveDatagram(receiver, 3000);
    assert(third.find("\"frame_index\":102") != std::string::npos);
    assert(third.find("\"type\":\"skeleton\"") == std::string::npos);

    streamer.stop();
    assert(!streamer.isRunning());

    streamer.start();
    std::string restarted = receiveDatagram(receiver, 3000);
    assert(restarted.find("\"type\":\"skeleton\"") != std::string::npos);
    streamer.stop();
    close(receiver);
}

void testGlobalRotationsKeepOriginalSideAndComponentSigns()
{
    _MocapDataWithVirtual_ md = makeFrame(300);
    setIdentityQuaternions(md);
    const float source[4] = {-0.5f, 0.5f, 0.5f, 0.5f};
    for (int component = 0; component < 4; ++component) {
        md.quaternion_body[BN_RightUpperArm][component] = source[component];
    }
    float output[NODES_BODY][4] {};
    assert(copyGlobalQuaternions(md, output));
    assert(std::fabs(output[BN_RightUpperArm][0] + 0.5f) < 1e-6);
    assert(std::fabs(output[BN_RightUpperArm][1] - 0.5f) < 1e-6);
    assert(std::fabs(output[BN_RightUpperArm][2] - 0.5f) < 1e-6);
    assert(std::fabs(output[BN_RightUpperArm][3] - 0.5f) < 1e-6);
    assert(std::fabs(output[BN_LeftUpperArm][0] - 1.0f) < 1e-6);
    assert(std::fabs(output[BN_LeftUpperArm][1]) < 1e-6);
    assert(std::fabs(output[BN_LeftUpperArm][2]) < 1e-6);
    assert(std::fabs(output[BN_LeftUpperArm][3]) < 1e-6);

    _MocapDataWithVirtual_ invalid {};
    invalid.isUpdate = true;
    assert(!copyGlobalQuaternions(invalid, output));
}

void testBvhUsesOriginalBodyAndHandSources()
{
    _MocapDataWithVirtual_ md = makeFrame(200);
    setIdentityQuaternions(md);
    md.quaternion_body[BN_RightUpperLeg][0] = 0.7071068f;
    md.quaternion_body[BN_RightUpperLeg][1] = 0.7071068f;

    const std::string bodyPath = "/tmp/vdsuit_original_body_test.bvh";
    BvhExporter bodyExporter;
    bodyExporter.setMode(BvhExportMode::BodyOnly);
    bodyExporter.setPrependStudioRestFrame(false);
    bodyExporter.begin(60);
    bodyExporter.addFrame(md);
    assert(bodyExporter.saveToFile(bodyPath));
    std::vector<double> bodyValues = readLastMotionLine(bodyPath);
    assert(bodyValues.size() == 72);
    assert(std::fabs(bodyValues[6]) + std::fabs(bodyValues[7]) +
           std::fabs(bodyValues[8]) > 10.0);
    assert(std::fabs(bodyValues[18]) + std::fabs(bodyValues[19]) +
           std::fabs(bodyValues[20]) < 0.01);
    std::remove(bodyPath.c_str());

    md = makeFrame(201);
    setIdentityQuaternions(md);
    md.quaternion_rHand[HN_ThumbFinger][0] = 0.7071068f;
    md.quaternion_rHand[HN_ThumbFinger][1] = 0.7071068f;

    const std::string handsPath = "/tmp/vdsuit_original_hands_test.bvh";
    BvhExporter handsExporter;
    handsExporter.setMode(BvhExportMode::FullHands);
    handsExporter.setPrependStudioRestFrame(false);
    handsExporter.begin(60);
    handsExporter.addFrame(md);
    assert(handsExporter.saveToFile(handsPath));
    std::vector<double> handValues = readLastMotionLine(handsPath);
    assert(handValues.size() == 186);
    const std::size_t rightThumbColumn = 3 + 19 * 3;
    const std::size_t leftThumbColumn = 3 + 42 * 3;
    assert(std::fabs(handValues[rightThumbColumn]) +
           std::fabs(handValues[rightThumbColumn + 1]) +
           std::fabs(handValues[rightThumbColumn + 2]) > 10.0);
    assert(std::fabs(handValues[leftThumbColumn]) +
           std::fabs(handValues[leftThumbColumn + 1]) +
           std::fabs(handValues[leftThumbColumn + 2]) < 0.01);
    std::remove(handsPath.c_str());
}

void testHandJointDefinitions()
{
    const std::vector<MocapJointDefinition>& definitions = handJointDefinitions();
    assert(definitions.size() == 40);

    // Right hand (indices 0-19) and left hand (indices 20-39) each have own root
    assert(definitions[0].parentIndex == -1);
    assert(definitions[20].parentIndex == -1);
    assert(std::string(definitions[0].name) == "RightHand");
    assert(std::string(definitions[20].name) == "LeftHand");

    // Verify non-root joints have parent_index < their own index
    for (std::size_t i = 0; i < definitions.size(); ++i) {
        if (definitions[i].parentIndex >= 0) {
            assert(definitions[i].parentIndex < static_cast<int>(i));
        }
    }

    // Verify unique names
    std::set<std::string> names;
    for (std::size_t i = 0; i < definitions.size(); ++i) {
        assert(names.insert(definitions[i].name).second);
    }

    // Verify right hand source/sdkIndex
    for (int i = 0; i < 20; ++i) {
        assert(definitions[static_cast<std::size_t>(i)].source == MocapJointSource::RightHand);
        assert(definitions[static_cast<std::size_t>(i)].sdkIndex == i);
    }
    // Verify left hand source/sdkIndex
    for (int i = 0; i < 20; ++i) {
        assert(definitions[static_cast<std::size_t>(20 + i)].source == MocapJointSource::LeftHand);
        assert(definitions[static_cast<std::size_t>(20 + i)].sdkIndex == i);
    }

    // Verify finger chain parent indices (right hand)
    // Thumb: HN_Hand(0) -> ThumbFinger(1) -> ThumbFinger1(2) -> ThumbFinger2(3)
    assert(definitions[1].parentIndex == 0);   // RightThumbFinger -> RightHand
    assert(definitions[2].parentIndex == 1);   // RightThumbFinger1 -> RightThumbFinger
    assert(definitions[3].parentIndex == 2);   // RightThumbFinger2 -> RightThumbFinger1
    // Index: HN_Hand(0) -> IndexFinger(4) -> IndexFinger1(5) -> ... -> IndexFinger3(7)
    assert(definitions[4].parentIndex == 0);
    assert(definitions[5].parentIndex == 4);
    assert(definitions[7].parentIndex == 6);
    // Each finger root should parent to HN_Hand
    assert(definitions[1].parentIndex == 0);   // ThumbFinger
    assert(definitions[4].parentIndex == 0);   // IndexFinger
    assert(definitions[8].parentIndex == 0);   // MiddleFinger
    assert(definitions[12].parentIndex == 0);  // RingFinger
    assert(definitions[16].parentIndex == 0);  // PinkyFinger

    // Verify left hand finger chain parents (offset by 20)
    assert(definitions[21].parentIndex == 20);  // LeftThumbFinger -> LeftHand
    assert(definitions[24].parentIndex == 20);  // LeftIndexFinger -> LeftHand
}

void testHandSkeletonSerialization()
{
    float right[NODES_HAND][3] {};
    float left[NODES_HAND][3] {};
    float dummyBody[NODES_BODY][3] {};
    fillInitialPositions(dummyBody, right, left);

    std::string skeleton = serializeHandSkeletonJsonLine(right, left);
    assert(skeleton.back() == '\n');
    assert(skeleton.find("\"type\":\"skeleton\"") != std::string::npos);
    assert(skeleton.find("\"joint_count\":40") != std::string::npos);
    assert(countOccurrences(skeleton, "\"initial_position\":") == 40);
    assert(countOccurrences(skeleton, "\"offset\":") == 40);

    // Verify hand-specific joint names appear
    assert(skeleton.find("\"RightHand\"") != std::string::npos);
    assert(skeleton.find("\"LeftHand\"") != std::string::npos);
    assert(skeleton.find("\"RightThumbFinger\"") != std::string::npos);
    assert(skeleton.find("\"LeftPinkyFinger3\"") != std::string::npos);

    // Verify initial positions are written (right hand uses 100.0+i, left uses 200.0+i from fillInitialPositions)
    assert(skeleton.find("100") != std::string::npos);  // RightHand initial position
    assert(skeleton.find("200") != std::string::npos);  // LeftHand initial position
}

void testHandFrameSerialization()
{
    _MocapDataWithVirtual_ md = makeFrame(55);

    std::string frame = serializeHandFrameJsonLine(md);
    assert(frame.back() == '\n');
    assert(frame.find("\"type\":\"frame\"") != std::string::npos);
    assert(frame.find("\"frame_index\":55") != std::string::npos);
    assert(countOccurrences(frame, "\"position\":") == 40);
    assert(countOccurrences(frame, "\"quaternion\":") == 40);

    // Verify right hand data comes from rHand arrays (value 2000.0 + i*10)
    assert(frame.find("2000") != std::string::npos);
    // Verify left hand data comes from lHand arrays (value 3000.0 + i*10)
    assert(frame.find("3000") != std::string::npos);
    // Body data should not appear
    assert(frame.find("1000") == std::string::npos);

    // Test invalid quaternion rejection
    md.quaternion_rHand[HN_Hand][0] = std::nanf("");
    std::string invalidFrame = serializeHandFrameJsonLine(md);
    assert(invalidFrame.empty());

    // Reset and test with all-zeros quaternion (norm=0, should be rejected)
    md = makeFrame(55);
    std::memset(md.quaternion_rHand, 0, sizeof(md.quaternion_rHand));
    std::string zeroNormFrame = serializeHandFrameJsonLine(md);
    assert(zeroNormFrame.empty());
}

void testHandUdpDatagrams()
{
    int receiver = socket(AF_INET, SOCK_DGRAM, 0);
    assert(receiver >= 0);

    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    assert(bind(receiver, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
    socklen_t addressLength = sizeof(address);
    assert(getsockname(receiver, reinterpret_cast<sockaddr*>(&address), &addressLength) == 0);

    float right[NODES_HAND][3] {};
    float left[NODES_HAND][3] {};
    float dummyBody[NODES_BODY][3] {};
    fillInitialPositions(dummyBody, right, left);

    NetworkStreamer streamer("127.0.0.1", ntohs(address.sin_port), right, left);
    assert(!streamer.isRunning());
    assert(receiveDatagram(receiver, 100).empty());

    streamer.start();
    assert(streamer.isRunning());

    // First datagram must be the skeleton
    std::string first = receiveDatagram(receiver, 3000);
    assert(first.find("\"type\":\"skeleton\"") != std::string::npos);
    assert(first.find("\"joint_count\":40") != std::string::npos);
    assert(first.back() == '\n');

    // Then frames
    streamer.enqueueFrame(makeFrame(101));
    std::string second = receiveDatagram(receiver, 3000);
    assert(second.find("\"type\":\"frame\"") != std::string::npos);
    assert(second.find("\"frame_index\":101") != std::string::npos);
    assert(second.back() == '\n');

    // Restart sends skeleton again
    streamer.stop();
    assert(!streamer.isRunning());
    streamer.start();
    std::string restarted = receiveDatagram(receiver, 3000);
    assert(restarted.find("\"type\":\"skeleton\"") != std::string::npos);

    streamer.stop();
    close(receiver);
}

} // namespace

int main()
{
    testEndpointParsing();
    testSkeletonAndFrameSerialization();
    testUdpDatagramsAndOrdering();
    testGlobalRotationsKeepOriginalSideAndComponentSigns();
    testBvhUsesOriginalBodyAndHandSources();
    testHandJointDefinitions();
    testHandSkeletonSerialization();
    testHandFrameSerialization();
    testHandUdpDatagrams();
    std::cout << "network_streamer_test: OK\n";
    return 0;
}
