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

} // namespace

int main()
{
    testEndpointParsing();
    testSkeletonAndFrameSerialization();
    testUdpDatagramsAndOrdering();
    testGlobalRotationsKeepOriginalSideAndComponentSigns();
    testBvhUsesOriginalBodyAndHandSources();
    std::cout << "network_streamer_test: OK\n";
    return 0;
}
