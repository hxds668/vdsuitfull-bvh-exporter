#include "mag_calibration.h"

#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

using namespace VDSuitMiniDevice;
using namespace vdsuit;

namespace {

void testCapabilities()
{
    assert(magCalibrationFunctionsAvailable(true, true, true, true));
    assert(!magCalibrationFunctionsAvailable(false, true, true, true));
    assert(!magCalibrationFunctionsAvailable(true, false, true, true));
    assert(!magCalibrationFunctionsAvailable(true, true, false, true));
    assert(!magCalibrationFunctionsAvailable(true, true, true, false));
}

void testProgress()
{
    assert(magCalibrationProgressPercent(-1.0f) == 0);
    assert(magCalibrationProgressPercent(0.0f) == 0);
    assert(magCalibrationProgressPercent(0.499f) == 50);
    assert(magCalibrationProgressPercent(1.0f) == 100);
    assert(magCalibrationProgressPercent(2.0f) == 100);
    assert(magCalibrationProgressPercent(std::nanf("")) == 0);
}

void testNodeNames()
{
    assert(std::string(bodyNodeName(BN_Hips)) == "Hips");
    assert(std::string(bodyNodeName(BN_LeftHand)) == "LeftHand");
    assert(bodyNodeName(-1) == nullptr);
    assert(bodyNodeName(NODES_BODY) == nullptr);
    assert(std::string(handNodeName(HN_Hand)) == "Hand");
    assert(std::string(handNodeName(HN_PinkyFinger3)) == "PinkyFinger3");
    assert(handNodeName(-1) == nullptr);
    assert(handNodeName(NODES_HAND) == nullptr);
}

void testInProgressAndSuccess()
{
    _MagCorrectResult_ result {};
    MagCalibrationSummary inProgress = summarizeMagCalibrationResult(result);
    assert(inProgress.outcome == MagCalibrationOutcome::InProgress);

    result.isFinished = true;
    result.isHaveSucceed = true;
    MagCalibrationSummary success = summarizeMagCalibrationResult(result);
    assert(success.outcome == MagCalibrationOutcome::Success);
    assert(success.failedBodyNodes.empty());
    assert(success.failedRightHandNodes.empty());
    assert(success.failedLeftHandNodes.empty());
}

void testPartialSuccess()
{
    _MagCorrectResult_ result {};
    result.isFinished = true;
    result.isHaveSucceed = true;
    result.bLength = 1;
    result.bFailedNodes[0] = BN_RightFoot;
    result.rLength = 1;
    result.rFailedNodes[0] = HN_IndexFinger2;
    result.lLength = 1;
    int invalidNode = 999;
    static_assert(sizeof(result.lFailedNodes[0]) == sizeof(invalidNode),
                  "Unexpected hand node enum ABI");
    std::memcpy(&result.lFailedNodes[0], &invalidNode, sizeof(invalidNode));

    MagCalibrationSummary summary = summarizeMagCalibrationResult(result);
    assert(summary.outcome == MagCalibrationOutcome::PartialSuccess);
    assert(summary.failedBodyNodes.size() == 1);
    assert(summary.failedBodyNodes[0] == "RightFoot");
    assert(summary.failedRightHandNodes.size() == 1);
    assert(summary.failedRightHandNodes[0] == "IndexFinger2");
    assert(summary.failedLeftHandNodes.size() == 1);
    assert(summary.failedLeftHandNodes[0] == "UnknownHandNode(999)");
}

void testFailureAndInvalidLengths()
{
    _MagCorrectResult_ result {};
    result.isFinished = true;
    MagCalibrationSummary failed = summarizeMagCalibrationResult(result);
    assert(failed.outcome == MagCalibrationOutcome::Failed);

    result.isHaveSucceed = true;
    result.bLength = NODES_BODY + 1;
    MagCalibrationSummary tooMany = summarizeMagCalibrationResult(result);
    assert(tooMany.outcome == MagCalibrationOutcome::InvalidResult);
    assert(!tooMany.error.empty());

    result.bLength = 0;
    result.rLength = -1;
    MagCalibrationSummary negative = summarizeMagCalibrationResult(result);
    assert(negative.outcome == MagCalibrationOutcome::InvalidResult);
}

} // namespace

int main()
{
    testCapabilities();
    testProgress();
    testNodeNames();
    testInProgressAndSuccess();
    testPartialSuccess();
    testFailureAndInvalidLengths();
    std::cout << "mag_calibration_test: OK\n";
    return 0;
}
