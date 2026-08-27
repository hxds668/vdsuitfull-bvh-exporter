#include "network_streamer.h"

#include "mocap_skeleton.h"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

using namespace VDSuitMiniDevice;

namespace {

void writeFloat(std::ostream& os, float value)
{
    if (std::isfinite(value)) os << value;
    else os << "null";
}

void writeVec3(std::ostream& os, const float value[3])
{
    os << '[';
    writeFloat(os, value[0]);
    os << ',';
    writeFloat(os, value[1]);
    os << ',';
    writeFloat(os, value[2]);
    os << ']';
}

void writeQuat(std::ostream& os, const float value[4])
{
    os << '[';
    writeFloat(os, value[0]);
    os << ',';
    writeFloat(os, value[1]);
    os << ',';
    writeFloat(os, value[2]);
    os << ',';
    writeFloat(os, value[3]);
    os << ']';
}

void writeJsonString(std::ostream& os, const char* value)
{
    os << '"';
    for (const unsigned char* p = reinterpret_cast<const unsigned char*>(value); *p; ++p) {
        switch (*p) {
        case '"': os << "\\\""; break;
        case '\\': os << "\\\\"; break;
        case '\b': os << "\\b"; break;
        case '\f': os << "\\f"; break;
        case '\n': os << "\\n"; break;
        case '\r': os << "\\r"; break;
        case '\t': os << "\\t"; break;
        default:
            if (*p < 0x20) {
                os << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
                   << static_cast<int>(*p) << std::dec << std::setfill(' ');
            } else {
                os << static_cast<char>(*p);
            }
        }
    }
    os << '"';
}

const float* initialJointPosition(
    const MocapJointDefinition& joint,
    const float initialBody[NODES_BODY][3])
{
    return initialBody[joint.sdkIndex];
}

const float* initialHandJointPosition(
    const MocapJointDefinition& joint,
    const float initialRightHand[NODES_HAND][3],
    const float initialLeftHand[NODES_HAND][3])
{
    if (joint.source == MocapJointSource::RightHand)
        return initialRightHand[joint.sdkIndex];
    return initialLeftHand[joint.sdkIndex];
}

std::ostringstream jsonStream()
{
    std::ostringstream os;
    os.imbue(std::locale::classic());
    os << std::setprecision(std::numeric_limits<float>::max_digits10);
    return os;
}

bool copyValidQuaternion(const float input[4], float output[4])
{
    double normSquared = 0.0;
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(input[i])) return false;
        normSquared += static_cast<double>(input[i]) * input[i];
    }
    if (normSquared < 0.25) return false;
    for (int i = 0; i < 4; ++i) output[i] = input[i];
    return true;
}

} // namespace

bool copyGlobalQuaternions(
    const _MocapDataWithVirtual_& md,
    float outputGlobalQuaternions[NODES_BODY][4])
{
    const std::vector<MocapJointDefinition>& joints = mocapJointDefinitions();
    if (joints.size() != NODES_BODY) return false;

    for (int i = 0; i < NODES_BODY; ++i) {
        if (!copyValidQuaternion(
                mocapJointQuaternion(md, joints[static_cast<std::size_t>(i)]),
                outputGlobalQuaternions[i])) {
            return false;
        }
    }
    return true;
}

bool parseNetworkEndpoint(const std::string& value,
                          std::string& ip,
                          uint16_t& port,
                          std::string& error)
{
    std::size_t separator = value.rfind(':');
    if (separator == std::string::npos || separator == 0 ||
        separator + 1 >= value.size() || value.find(':') != separator) {
        error = "expected IPv4 address in IP:PORT form";
        return false;
    }

    std::string parsedIp = value.substr(0, separator);
    in_addr address {};
    if (inet_pton(AF_INET, parsedIp.c_str(), &address) != 1) {
        error = "invalid IPv4 address: " + parsedIp;
        return false;
    }

    std::string portText = value.substr(separator + 1);
    char* end = nullptr;
    errno = 0;
    long parsedPort = std::strtol(portText.c_str(), &end, 10);
    if (errno != 0 || !end || *end != '\0' || parsedPort < 1 || parsedPort > 65535) {
        error = "port must be an integer from 1 to 65535";
        return false;
    }

    ip = parsedIp;
    port = static_cast<uint16_t>(parsedPort);
    error.clear();
    return true;
}

std::string serializeSkeletonJsonLine(
    const float initialBody[NODES_BODY][3])
{
    const std::vector<MocapJointDefinition>& joints = mocapJointDefinitions();
    std::ostringstream os = jsonStream();
    os << "{\"type\":\"skeleton\",\"version\":1,\"joint_count\":" << joints.size()
       << ",\"coordinate_system\":\"WS_Geo\",\"position_unit\":\"m\""
       << ",\"quaternion_order\":\"wxyz\",\"joints\":[";

    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (i > 0) os << ',';
        const MocapJointDefinition& joint = joints[i];
        const float* initial = initialJointPosition(joint, initialBody);
        float offset[3] = {initial[0], initial[1], initial[2]};
        if (joint.parentIndex >= 0) {
            const float* parent = initialJointPosition(
                joints[static_cast<std::size_t>(joint.parentIndex)], initialBody);
            offset[0] -= parent[0];
            offset[1] -= parent[1];
            offset[2] -= parent[2];
        }

        os << "{\"index\":" << i << ",\"name\":";
        writeJsonString(os, joint.name);
        os << ",\"parent_index\":" << joint.parentIndex << ",\"initial_position\":";
        writeVec3(os, initial);
        os << ",\"offset\":";
        writeVec3(os, offset);
        os << '}';
    }
    os << "]}\n";
    return os.str();
}

std::string serializeFrameJsonLine(const _MocapDataWithVirtual_& md)
{
    float outputGlobalQuaternions[NODES_BODY][4];
    if (!copyGlobalQuaternions(md, outputGlobalQuaternions)) {
        return std::string();
    }
    const std::vector<MocapJointDefinition>& joints = mocapJointDefinitions();
    std::ostringstream os = jsonStream();
    os << "{\"type\":\"frame\",\"version\":1,\"frame_index\":" << md.frameIndex
       << ",\"joints\":[";
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (i > 0) os << ',';
        os << "{\"position\":";
        writeVec3(os, mocapJointPosition(md, joints[i]));
        os << ",\"quaternion\":";
        writeQuat(os, outputGlobalQuaternions[i]);
        os << '}';
    }
    os << "]}\n";
    return os.str();
}

const std::vector<MocapJointDefinition>& handJointDefinitions()
{
    static const std::vector<MocapJointDefinition> definitions = [] {
        struct HandJointEntry {
            const char* name;
            int parentIndex;
            MocapJointSource source;
            int sdkIndex;
        };

        static const HandJointEntry kRightHand[] = {
            {"RightHand",          -1, MocapJointSource::RightHand, HN_Hand},
            {"RightThumbFinger",    0, MocapJointSource::RightHand, HN_ThumbFinger},
            {"RightThumbFinger1",   1, MocapJointSource::RightHand, HN_ThumbFinger1},
            {"RightThumbFinger2",   2, MocapJointSource::RightHand, HN_ThumbFinger2},
            {"RightIndexFinger",    0, MocapJointSource::RightHand, HN_IndexFinger},
            {"RightIndexFinger1",   4, MocapJointSource::RightHand, HN_IndexFinger1},
            {"RightIndexFinger2",   5, MocapJointSource::RightHand, HN_IndexFinger2},
            {"RightIndexFinger3",   6, MocapJointSource::RightHand, HN_IndexFinger3},
            {"RightMiddleFinger",   0, MocapJointSource::RightHand, HN_MiddleFinger},
            {"RightMiddleFinger1",  8, MocapJointSource::RightHand, HN_MiddleFinger1},
            {"RightMiddleFinger2",  9, MocapJointSource::RightHand, HN_MiddleFinger2},
            {"RightMiddleFinger3", 10, MocapJointSource::RightHand, HN_MiddleFinger3},
            {"RightRingFinger",     0, MocapJointSource::RightHand, HN_RingFinger},
            {"RightRingFinger1",   12, MocapJointSource::RightHand, HN_RingFinger1},
            {"RightRingFinger2",   13, MocapJointSource::RightHand, HN_RingFinger2},
            {"RightRingFinger3",   14, MocapJointSource::RightHand, HN_RingFinger3},
            {"RightPinkyFinger",    0, MocapJointSource::RightHand, HN_PinkyFinger},
            {"RightPinkyFinger1",  16, MocapJointSource::RightHand, HN_PinkyFinger1},
            {"RightPinkyFinger2",  17, MocapJointSource::RightHand, HN_PinkyFinger2},
            {"RightPinkyFinger3",  18, MocapJointSource::RightHand, HN_PinkyFinger3},
        };

        static const HandJointEntry kLeftHand[] = {
            {"LeftHand",          -1, MocapJointSource::LeftHand, HN_Hand},
            {"LeftThumbFinger",    0, MocapJointSource::LeftHand, HN_ThumbFinger},
            {"LeftThumbFinger1",   1, MocapJointSource::LeftHand, HN_ThumbFinger1},
            {"LeftThumbFinger2",   2, MocapJointSource::LeftHand, HN_ThumbFinger2},
            {"LeftIndexFinger",    0, MocapJointSource::LeftHand, HN_IndexFinger},
            {"LeftIndexFinger1",   4, MocapJointSource::LeftHand, HN_IndexFinger1},
            {"LeftIndexFinger2",   5, MocapJointSource::LeftHand, HN_IndexFinger2},
            {"LeftIndexFinger3",   6, MocapJointSource::LeftHand, HN_IndexFinger3},
            {"LeftMiddleFinger",   0, MocapJointSource::LeftHand, HN_MiddleFinger},
            {"LeftMiddleFinger1",  8, MocapJointSource::LeftHand, HN_MiddleFinger1},
            {"LeftMiddleFinger2",  9, MocapJointSource::LeftHand, HN_MiddleFinger2},
            {"LeftMiddleFinger3", 10, MocapJointSource::LeftHand, HN_MiddleFinger3},
            {"LeftRingFinger",     0, MocapJointSource::LeftHand, HN_RingFinger},
            {"LeftRingFinger1",   12, MocapJointSource::LeftHand, HN_RingFinger1},
            {"LeftRingFinger2",   13, MocapJointSource::LeftHand, HN_RingFinger2},
            {"LeftRingFinger3",   14, MocapJointSource::LeftHand, HN_RingFinger3},
            {"LeftPinkyFinger",    0, MocapJointSource::LeftHand, HN_PinkyFinger},
            {"LeftPinkyFinger1",  16, MocapJointSource::LeftHand, HN_PinkyFinger1},
            {"LeftPinkyFinger2",  17, MocapJointSource::LeftHand, HN_PinkyFinger2},
            {"LeftPinkyFinger3",  18, MocapJointSource::LeftHand, HN_PinkyFinger3},
        };

        constexpr int kRightHandCount = static_cast<int>(sizeof(kRightHand) / sizeof(kRightHand[0]));
        constexpr int kLeftHandCount  = static_cast<int>(sizeof(kLeftHand) / sizeof(kLeftHand[0]));

        std::vector<MocapJointDefinition> result;
        result.reserve(static_cast<std::size_t>(kRightHandCount + kLeftHandCount));

        for (int i = 0; i < kRightHandCount; ++i) {
            result.push_back({kRightHand[i].name,
                              kRightHand[i].parentIndex,
                              kRightHand[i].source,
                              kRightHand[i].sdkIndex});
        }
        for (int i = 0; i < kLeftHandCount; ++i) {
            int leftParent = kLeftHand[i].parentIndex;
            if (leftParent >= 0) leftParent += kRightHandCount;
            result.push_back({kLeftHand[i].name,
                              leftParent,
                              kLeftHand[i].source,
                              kLeftHand[i].sdkIndex});
        }
        return result;
    }();
    return definitions;
}

std::string serializeHandSkeletonJsonLine(
    const float initialRightHand[NODES_HAND][3],
    const float initialLeftHand[NODES_HAND][3])
{
    const std::vector<MocapJointDefinition>& joints = handJointDefinitions();
    std::ostringstream os = jsonStream();
    os << "{\"type\":\"skeleton\",\"version\":1,\"joint_count\":" << joints.size()
       << ",\"coordinate_system\":\"WS_Geo\",\"position_unit\":\"m\""
       << ",\"quaternion_order\":\"wxyz\",\"joints\":[";

    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (i > 0) os << ',';
        const MocapJointDefinition& joint = joints[i];
        const float* initial = initialHandJointPosition(joint, initialRightHand, initialLeftHand);
        float offset[3] = {initial[0], initial[1], initial[2]};
        if (joint.parentIndex >= 0) {
            const float* parent = initialHandJointPosition(
                joints[static_cast<std::size_t>(joint.parentIndex)],
                initialRightHand, initialLeftHand);
            offset[0] -= parent[0];
            offset[1] -= parent[1];
            offset[2] -= parent[2];
        }

        os << "{\"index\":" << i << ",\"name\":";
        writeJsonString(os, joint.name);
        os << ",\"parent_index\":" << joint.parentIndex << ",\"initial_position\":";
        writeVec3(os, initial);
        os << ",\"offset\":";
        writeVec3(os, offset);
        os << '}';
    }
    os << "]}\n";
    return os.str();
}

std::string serializeHandFrameJsonLine(const _MocapDataWithVirtual_& md)
{
    const std::vector<MocapJointDefinition>& joints = handJointDefinitions();
    // Validate all hand quaternions before serializing
    for (std::size_t i = 0; i < joints.size(); ++i) {
        float tmp[4];
        if (!copyValidQuaternion(mocapJointQuaternion(md, joints[i]), tmp)) {
            return std::string();
        }
    }

    std::ostringstream os = jsonStream();
    os << "{\"type\":\"frame\",\"version\":1,\"frame_index\":" << md.frameIndex
       << ",\"joints\":[";
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (i > 0) os << ',';
        os << "{\"position\":";
        writeVec3(os, mocapJointPosition(md, joints[i]));
        os << ",\"quaternion\":";
        writeQuat(os, mocapJointQuaternion(md, joints[i]));
        os << '}';
    }
    os << "]}\n";
    return os.str();
}

NetworkStreamer::NetworkStreamer(
    const std::string& ip,
    uint16_t port,
    const float initialBody[NODES_BODY][3])
    : ip_(ip),
      port_(port),
      content_(NetworkStreamContent::Body),
      skeletonLine_(serializeSkeletonJsonLine(initialBody))
{
}

NetworkStreamer::NetworkStreamer(
    const std::string& ip,
    uint16_t port,
    const float initialRightHand[NODES_HAND][3],
    const float initialLeftHand[NODES_HAND][3])
    : ip_(ip),
      port_(port),
      content_(NetworkStreamContent::Hands),
      skeletonLine_(serializeHandSkeletonJsonLine(initialRightHand, initialLeftHand))
{
}

NetworkStreamer::~NetworkStreamer()
{
    stop();
}

void NetworkStreamer::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) return;
    stopping_ = false;
    running_ = true;
    worker_ = std::thread(&NetworkStreamer::workerLoop, this);
}

void NetworkStreamer::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) return;
        stopping_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();

    std::lock_guard<std::mutex> lock(mutex_);
    frames_.clear();
    running_ = false;
}

bool NetworkStreamer::isRunning() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return running_;
}

void NetworkStreamer::enqueueFrame(const _MocapDataWithVirtual_& md)
{
    if (!md.isUpdate) return;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || stopping_) return;
        if (frames_.size() >= kMaxQueuedFrames) {
            frames_.pop_front();
            ++droppedFrames_;
        }
        frames_.push_back(md);
    }
    condition_.notify_one();
}

uint64_t NetworkStreamer::droppedFrameCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return droppedFrames_;
}

bool NetworkStreamer::stopping() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return stopping_;
}

int NetworkStreamer::openSocket()
{
    int socketFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFd < 0) return -1;

    int flags = fcntl(socketFd, F_GETFL, 0);
    if (flags < 0 || fcntl(socketFd, F_SETFL, flags | O_NONBLOCK) < 0) {
        close(socketFd);
        return -1;
    }
    fcntl(socketFd, F_SETFD, FD_CLOEXEC);

    socketFd_.store(socketFd);
    return socketFd;
}

bool NetworkStreamer::sendDatagram(int socketFd, const std::string& data)
{
    if (stopping()) return false;
    sockaddr_in target {};
    target.sin_family = AF_INET;
    target.sin_port = htons(port_);
    inet_pton(AF_INET, ip_.c_str(), &target.sin_addr);
    ssize_t count = sendto(socketFd,
                           data.data(),
                           data.size(),
                           MSG_NOSIGNAL,
                           reinterpret_cast<sockaddr*>(&target),
                           sizeof(target));
    return count == static_cast<ssize_t>(data.size());
}

void NetworkStreamer::closeSocket()
{
    int socketFd = socketFd_.exchange(-1);
    if (socketFd >= 0) close(socketFd);
}

void NetworkStreamer::workerLoop()
{
    int socketFd = -1;
    while (!stopping() && socketFd < 0) {
        socketFd = openSocket();
        if (socketFd < 0) {
            std::cerr << "UDP stream could not create a socket; retrying.\n";
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait_for(lock, std::chrono::seconds(1), [this] { return stopping_; });
        }
    }
    if (socketFd < 0) return;

    if (!sendDatagram(socketFd, skeletonLine_)) {
        std::cerr << "UDP stream failed to send the skeleton datagram.\n";
    } else {
        std::cerr << "UDP stream ready: " << ip_ << ':' << port_
                  << " (skeleton sent once)\n";
    }

    while (!stopping()) {
        _MocapDataWithVirtual_ frame {};
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] { return stopping_ || !frames_.empty(); });
            if (stopping_) break;
            frame = frames_.front();
            frames_.pop_front();
        }
        std::string frameLine = (content_ == NetworkStreamContent::Body)
            ? serializeFrameJsonLine(frame)
            : serializeHandFrameJsonLine(frame);
        if (frameLine.empty()) {
            std::lock_guard<std::mutex> lock(mutex_);
            ++droppedFrames_;
            continue;
        }
        if (!sendDatagram(socketFd, frameLine)) {
            std::lock_guard<std::mutex> lock(mutex_);
            ++droppedFrames_;
        }
    }
    closeSocket();
}
