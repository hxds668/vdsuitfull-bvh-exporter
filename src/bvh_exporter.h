#pragma once

#include <ostream>
#include <pthread.h>
#include <string>
#include <vector>

#include "../include/VDMocapSDK_VDSuitMini_DataType.h"

enum class BvhExportMode {
    BodyOnly = 0,
    FullHands = 1
};

class BvhExporter {
public:
    BvhExporter();
    ~BvhExporter();

    void setMode(BvhExportMode mode);
    BvhExportMode mode() const;

    void setPrependStudioRestFrame(bool enabled);
    bool prependStudioRestFrame() const;

    void begin(int frequency);
    void cancel();
    bool isRecording() const;

    void addFrame(const VDSuitMiniDevice::_MocapDataWithVirtual_& md);
    bool saveToFile(const std::string& path);

    int capturedFrameCount() const;
    int outputFrameCount() const;

private:
    std::string buildMotionLine(const VDSuitMiniDevice::_MocapDataWithVirtual_& md) const;
    void writeHierarchy(std::ostream& os) const;
    void writeFullNode(std::ostream& os, int nodeIndex, int depth) const;
    void writeBodyNode(std::ostream& os, int nodeIndex, int depth) const;
    std::string studioRestFrame() const;

    pthread_mutex_t mutex_;
    BvhExportMode mode_ = BvhExportMode::FullHands;
    bool prependStudioRestFrame_ = true;
    bool recording_ = false;
    int frequency_ = 60;
    std::vector<std::string> frames_;
};
