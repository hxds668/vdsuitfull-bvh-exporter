#pragma once

#include <vector>

#include "../include/VDMocapSDK_VDSuitMini_DataType.h"

enum class MocapJointSource {
    Body,
    RightHand,
    LeftHand
};

struct MocapJointDefinition {
    const char* name;
    int parentIndex;
    MocapJointSource source;
    int sdkIndex;
};

// Returns 23 output body joints in the SDK _BodyNodes_ order. Each output
// joint reads the SDK joint with the same index.
const std::vector<MocapJointDefinition>& mocapJointDefinitions();

const float* mocapJointPosition(
    const VDSuitMiniDevice::_MocapDataWithVirtual_& md,
    const MocapJointDefinition& joint);

const float* mocapJointQuaternion(
    const VDSuitMiniDevice::_MocapDataWithVirtual_& md,
    const MocapJointDefinition& joint);
