#include "mag_calibration.h"

#include <cmath>
#include <cstring>
#include <sstream>

using namespace VDSuitMiniDevice;

namespace vdsuit {
namespace {

std::string nodeLabel(const char* name, const char* unknownPrefix, int value)
{
    if (name) return name;
    std::ostringstream label;
    label << unknownPrefix << '(' << value << ')';
    return label.str();
}

bool validFailureLength(int length, int maximum)
{
    return length >= 0 && length <= maximum;
}

template <typename NodeEnum>
int nodeValue(const NodeEnum& node)
{
    static_assert(sizeof(NodeEnum) == sizeof(int),
                  "Unexpected vendor SDK node enum ABI");
    int value = 0;
    std::memcpy(&value, &node, sizeof(value));
    return value;
}

} // namespace

static_assert(sizeof(_MagCorrectResult_) == 272,
              "Unexpected vendor SDK MAGCORRECTRESULT ABI");

bool magCalibrationFunctionsAvailable(bool startAvailable,
                                      bool endAvailable,
                                      bool cancelAvailable,
                                      bool resultAvailable)
{
    return startAvailable && endAvailable && cancelAvailable && resultAvailable;
}

int magCalibrationProgressPercent(float progress)
{
    if (!std::isfinite(progress) || progress <= 0.0f) return 0;
    if (progress >= 1.0f) return 100;
    return static_cast<int>(progress * 100.0f + 0.5f);
}

const char* bodyNodeName(int node)
{
    static const char* const names[NODES_BODY] = {
        "Hips",
        "RightUpperLeg", "RightLowerLeg", "RightFoot", "RightToe",
        "LeftUpperLeg", "LeftLowerLeg", "LeftFoot", "LeftToe",
        "Spine", "Spine1", "Spine2", "Spine3", "Neck", "Head",
        "RightShoulder", "RightUpperArm", "RightLowerArm", "RightHand",
        "LeftShoulder", "LeftUpperArm", "LeftLowerArm", "LeftHand"
    };
    return node >= 0 && node < NODES_BODY ? names[node] : nullptr;
}

const char* handNodeName(int node)
{
    static const char* const names[NODES_HAND] = {
        "Hand",
        "ThumbFinger", "ThumbFinger1", "ThumbFinger2",
        "IndexFinger", "IndexFinger1", "IndexFinger2", "IndexFinger3",
        "MiddleFinger", "MiddleFinger1", "MiddleFinger2", "MiddleFinger3",
        "RingFinger", "RingFinger1", "RingFinger2", "RingFinger3",
        "PinkyFinger", "PinkyFinger1", "PinkyFinger2", "PinkyFinger3"
    };
    return node >= 0 && node < NODES_HAND ? names[node] : nullptr;
}

MagCalibrationSummary summarizeMagCalibrationResult(const _MagCorrectResult_& result)
{
    MagCalibrationSummary summary;
    if (!result.isFinished) return summary;

    if (!result.isHaveSucceed) {
        summary.outcome = MagCalibrationOutcome::Failed;
        return summary;
    }

    if (!validFailureLength(result.bLength, NODES_BODY) ||
        !validFailureLength(result.rLength, NODES_HAND) ||
        !validFailureLength(result.lLength, NODES_HAND)) {
        summary.outcome = MagCalibrationOutcome::InvalidResult;
        summary.error = "SDK returned an invalid failed-node count";
        return summary;
    }

    for (int i = 0; i < result.bLength; ++i) {
        int node = nodeValue(result.bFailedNodes[i]);
        summary.failedBodyNodes.push_back(nodeLabel(bodyNodeName(node), "UnknownBodyNode", node));
    }
    for (int i = 0; i < result.rLength; ++i) {
        int node = nodeValue(result.rFailedNodes[i]);
        summary.failedRightHandNodes.push_back(nodeLabel(handNodeName(node), "UnknownHandNode", node));
    }
    for (int i = 0; i < result.lLength; ++i) {
        int node = nodeValue(result.lFailedNodes[i]);
        summary.failedLeftHandNodes.push_back(nodeLabel(handNodeName(node), "UnknownHandNode", node));
    }

    bool hasFailures = !summary.failedBodyNodes.empty() ||
                       !summary.failedRightHandNodes.empty() ||
                       !summary.failedLeftHandNodes.empty();
    summary.outcome = hasFailures
        ? MagCalibrationOutcome::PartialSuccess
        : MagCalibrationOutcome::Success;
    return summary;
}

} // namespace vdsuit
