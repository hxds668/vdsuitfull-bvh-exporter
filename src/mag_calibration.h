#pragma once

#include "VDMocapSDK_VDSuitMini_DataType.h"

#include <string>
#include <vector>

namespace vdsuit {

enum class MagCalibrationOutcome {
    InProgress,
    Success,
    PartialSuccess,
    Failed,
    InvalidResult
};

struct MagCalibrationSummary {
    MagCalibrationOutcome outcome = MagCalibrationOutcome::InProgress;
    std::vector<std::string> failedBodyNodes;
    std::vector<std::string> failedRightHandNodes;
    std::vector<std::string> failedLeftHandNodes;
    std::string error;
};

bool magCalibrationFunctionsAvailable(bool startAvailable,
                                      bool endAvailable,
                                      bool cancelAvailable,
                                      bool resultAvailable);

int magCalibrationProgressPercent(float progress);
const char* bodyNodeName(int node);
const char* handNodeName(int node);
MagCalibrationSummary summarizeMagCalibrationResult(
    const VDSuitMiniDevice::_MagCorrectResult_& result);

} // namespace vdsuit
