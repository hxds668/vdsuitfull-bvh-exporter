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

// Returns 23 output body joints in _BodyNodes_ order, with left/right limb
// data sources exchanged by swappedBodySdkIndex().
const std::vector<MocapJointDefinition>& mocapJointDefinitions();

// Maps an output body-joint index to its SDK source index. Left and right
// limb chains are intentionally exchanged while center joints are unchanged.
int swappedBodySdkIndex(int outputIndex);

const float* mocapJointPosition(
    const VDSuitMiniDevice::_MocapDataWithVirtual_& md,
    const MocapJointDefinition& joint);

const float* mocapJointQuaternion(
    const VDSuitMiniDevice::_MocapDataWithVirtual_& md,
    const MocapJointDefinition& joint);
