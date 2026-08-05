#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "../include/VDMocapSDK_VDSuitMini_DataType.h"

bool parseNetworkEndpoint(const std::string& value,
                          std::string& ip,
                          uint16_t& port,
                          std::string& error);

std::string serializeSkeletonJsonLine(
    const float initialBody[NODES_BODY][3]);

std::string serializeFrameJsonLine(
    const VDSuitMiniDevice::_MocapDataWithVirtual_& md);

bool buildMirroredGlobalQuaternions(
    const VDSuitMiniDevice::_MocapDataWithVirtual_& md,
    float outputGlobalQuaternions[NODES_BODY][4]);

class NetworkStreamer {
public:
    NetworkStreamer(const std::string& ip,
                    uint16_t port,
                    const float initialBody[NODES_BODY][3]);
    ~NetworkStreamer();

    void start();
    void stop();
    bool isRunning() const;
    void enqueueFrame(const VDSuitMiniDevice::_MocapDataWithVirtual_& md);

    uint64_t droppedFrameCount() const;

private:
    void workerLoop();
    int openSocket();
    bool sendDatagram(int socketFd, const std::string& data);
    bool stopping() const;
    void closeSocket();

    static const std::size_t kMaxQueuedFrames = 4;

    std::string ip_;
    uint16_t port_;
    std::string skeletonLine_;

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<VDSuitMiniDevice::_MocapDataWithVirtual_> frames_;
    bool running_ = false;
    bool stopping_ = false;
    uint64_t droppedFrames_ = 0;
    std::thread worker_;
    std::atomic<int> socketFd_ {-1};
};
