#include "bvh_exporter.h"
#include "mocap_skeleton.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace VDSuitMiniDevice;

namespace {

constexpr float kRadToDeg = 57.29578f;

static const int kBodyParent[NODES_BODY] = {
    -1, 0, 1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 12, 13, 12, 15, 16, 17, 12, 19, 20, 21
};

static const float kBodyOffset[NODES_BODY][3] = {
    { 0.0f, 111.0f,  0.0f},
    {-10.5f,  -8.8f,  0.0f},
    { 0.0f, -45.6f,  0.0f},
    { 0.0f, -46.7f,  0.0f},
    { 0.0f,  -8.0f, 12.1f},
    {10.5f,   -8.8f,  0.0f},
    { 0.0f, -45.6f,  0.0f},
    { 0.0f, -46.7f,  0.0f},
    { 0.0f,  -8.0f, 12.1f},
    { 0.0f,   9.7f,  0.0f},
    { 0.0f,  11.2f,  0.0f},
    { 0.0f,  12.1f,  0.0f},
    { 0.0f,  11.7f,  1.2f},
    { 0.0f,  12.0f, -1.2f},
    { 0.0f,  10.0f,  0.0f},
    {-4.9f,   4.0f, -1.2f},
    {-15.6f,  0.0f,  0.0f},
    {-25.9f,  0.0f,  0.0f},
    {-28.4f,  0.0f,  0.0f},
    { 4.9f,   4.0f, -1.2f},
    {15.6f,   0.0f,  0.0f},
    {25.9f,   0.0f,  0.0f},
    {28.4f,   0.0f,  0.0f}
};

static const char* kBodyName[NODES_BODY] = {
    "Hips", "RightUpperLeg", "RightLowerLeg", "RightFoot", "RightToe",
    "LeftUpperLeg", "LeftLowerLeg", "LeftFoot", "LeftToe",
    "Spine", "Spine1", "Spine2", "Spine3", "Neck", "Head",
    "RightShoulder", "RightUpperArm", "RightLowerArm", "RightHand",
    "LeftShoulder", "LeftUpperArm", "LeftLowerArm", "LeftHand"
};

using BvhSourceKind = MocapJointSource;

struct BvhNodeDef {
    const char* name;
    int parent;
    MocapJointSource source;
    int sdkIndex;
    float offset[3];
    float endOffset[3];
    bool hasEnd;
};

static const BvhNodeDef kFullNodes[] = {
    {"Hips", -1, MocapJointSource::Body, BN_Hips, {0.0f, 111.0f, 0.0f}, {0, 0, 0}, false},
    {"RightUpperLeg", 0, BvhSourceKind::Body, BN_RightUpperLeg, {-10.5f, -8.8f, 0.0f}, {0, 0, 0}, false},
    {"RightLowerLeg", 1, BvhSourceKind::Body, BN_RightLowerLeg, {0.0f, -45.6f, 0.0f}, {0, 0, 0}, false},
    {"RightFoot", 2, BvhSourceKind::Body, BN_RightFoot, {0.0f, -46.7f, 0.0f}, {0, 0, 0}, false},
    {"RightToe", 3, BvhSourceKind::Body, BN_RightToe, {0.0f, -8.0f, 12.1f}, {0.0f, 0.0f, 2.42f}, true},
    {"LeftUpperLeg", 0, BvhSourceKind::Body, BN_LeftUpperLeg, {10.5f, -8.8f, 0.0f}, {0, 0, 0}, false},
    {"LeftLowerLeg", 5, BvhSourceKind::Body, BN_LeftLowerLeg, {0.0f, -45.6f, 0.0f}, {0, 0, 0}, false},
    {"LeftFoot", 6, BvhSourceKind::Body, BN_LeftFoot, {0.0f, -46.7f, 0.0f}, {0, 0, 0}, false},
    {"LeftToe", 7, BvhSourceKind::Body, BN_LeftToe, {0.0f, -8.0f, 12.1f}, {0.0f, 0.0f, 2.42f}, true},
    {"Spine", 0, BvhSourceKind::Body, BN_Spine, {0.0f, 9.7f, 0.0f}, {0, 0, 0}, false},
    {"Spine1", 9, BvhSourceKind::Body, BN_Spine1, {0.0f, 11.2f, 0.0f}, {0, 0, 0}, false},
    {"Spine2", 10, BvhSourceKind::Body, BN_Spine2, {0.0f, 12.1f, 0.0f}, {0, 0, 0}, false},
    {"Spine3", 11, BvhSourceKind::Body, BN_Spine3, {0.0f, 11.7f, 1.2f}, {0, 0, 0}, false},
    {"Neck", 12, BvhSourceKind::Body, BN_Neck, {0.0f, 12.0f, -1.2f}, {0, 0, 0}, false},
    {"Head", 13, BvhSourceKind::Body, BN_Head, {0.0f, 10.0f, 0.0f}, {0.0f, 20.0f, 0.0f}, true},
    {"RightShoulder", 12, BvhSourceKind::Body, BN_RightShoulder, {-4.9f, 4.0f, -1.2f}, {0, 0, 0}, false},
    {"RightUpperArm", 15, BvhSourceKind::Body, BN_RightUpperArm, {-15.6f, 0.0f, 0.0f}, {0, 0, 0}, false},
    {"RightLowerArm", 16, BvhSourceKind::Body, BN_RightLowerArm, {-25.9f, 0.0f, 0.0f}, {0, 0, 0}, false},
    {"RightHand", 17, BvhSourceKind::Body, BN_RightHand, {-28.4f, 0.0f, 0.0f}, {0, 0, 0}, false},
    {"RightThumbFinger", 18, BvhSourceKind::RightHand, HN_ThumbFinger, {-3.4f, 0.3f, 4.2f}, {0, 0, 0}, false},
    {"RightThumbFinger1", 19, BvhSourceKind::RightHand, HN_ThumbFinger1, {-3.5f, 0.0f, 3.5f}, {0, 0, 0}, false},
    {"RightThumbFinger2", 20, BvhSourceKind::RightHand, HN_ThumbFinger2, {-2.5f, 0.0f, 2.4f}, {-2.145f, 0.0f, 2.0592f}, true},
    {"RightIndexFinger", 18, BvhSourceKind::RightHand, HN_IndexFinger, {-4.4f, 0.7f, 2.6f}, {0, 0, 0}, false},
    {"RightIndexFinger1", 22, BvhSourceKind::RightHand, HN_IndexFinger1, {-7.0f, -0.1f, 1.4f}, {0, 0, 0}, false},
    {"RightIndexFinger2", 23, BvhSourceKind::RightHand, HN_IndexFinger2, {-5.0f, -0.2f, 0.0f}, {0, 0, 0}, false},
    {"RightIndexFinger3", 24, BvhSourceKind::RightHand, HN_IndexFinger3, {-2.7f, -0.2f, 0.0f}, {-2.3814f, -0.1764f, 0.0f}, true},
    {"RightMiddleFinger", 18, BvhSourceKind::RightHand, HN_MiddleFinger, {-4.6f, 0.7f, 1.0f}, {0, 0, 0}, false},
    {"RightMiddleFinger1", 26, BvhSourceKind::RightHand, HN_MiddleFinger1, {-7.0f, -0.1f, 0.4f}, {0, 0, 0}, false},
    {"RightMiddleFinger2", 27, BvhSourceKind::RightHand, HN_MiddleFinger2, {-5.3f, -0.3f, 0.0f}, {0, 0, 0}, false},
    {"RightMiddleFinger3", 28, BvhSourceKind::RightHand, HN_MiddleFinger3, {-3.4f, -0.3f, 0.0f}, {-2.9988f, -0.2646f, 0.0f}, true},
    {"RightRingFinger", 18, BvhSourceKind::RightHand, HN_RingFinger, {-4.5f, 0.8f, -0.1f}, {0, 0, 0}, false},
    {"RightRingFinger1", 30, BvhSourceKind::RightHand, HN_RingFinger1, {-6.3f, -0.1f, -0.7f}, {0, 0, 0}, false},
    {"RightRingFinger2", 31, BvhSourceKind::RightHand, HN_RingFinger2, {-4.7f, -0.4f, 0.0f}, {0, 0, 0}, false},
    {"RightRingFinger3", 32, BvhSourceKind::RightHand, HN_RingFinger3, {-3.2f, -0.3f, 0.0f}, {-2.8224f, -0.2646f, 0.0f}, true},
    {"RightPinkyFinger", 18, BvhSourceKind::RightHand, HN_PinkyFinger, {-4.3f, 0.7f, -1.6f}, {0, 0, 0}, false},
    {"RightPinkyFinger1", 34, BvhSourceKind::RightHand, HN_PinkyFinger1, {-5.6f, -0.1f, -1.5f}, {0, 0, 0}, false},
    {"RightPinkyFinger2", 35, BvhSourceKind::RightHand, HN_PinkyFinger2, {-3.7f, -0.2f, 0.0f}, {0, 0, 0}, false},
    {"RightPinkyFinger3", 36, BvhSourceKind::RightHand, HN_PinkyFinger3, {-2.4f, -0.1f, 0.0f}, {-2.1168f, -0.0882f, 0.0f}, true},
    {"LeftShoulder", 12, BvhSourceKind::Body, BN_LeftShoulder, {4.9f, 4.0f, -1.2f}, {0, 0, 0}, false},
    {"LeftUpperArm", 38, BvhSourceKind::Body, BN_LeftUpperArm, {15.6f, 0.0f, 0.0f}, {0, 0, 0}, false},
    {"LeftLowerArm", 39, BvhSourceKind::Body, BN_LeftLowerArm, {25.9f, 0.0f, 0.0f}, {0, 0, 0}, false},
    {"LeftHand", 40, BvhSourceKind::Body, BN_LeftHand, {28.4f, 0.0f, 0.0f}, {0, 0, 0}, false},
    {"LeftThumbFinger", 41, BvhSourceKind::LeftHand, HN_ThumbFinger, {3.4f, 0.3f, 4.2f}, {0, 0, 0}, false},
    {"LeftThumbFinger1", 42, BvhSourceKind::LeftHand, HN_ThumbFinger1, {3.5f, 0.0f, 3.5f}, {0, 0, 0}, false},
    {"LeftThumbFinger2", 43, BvhSourceKind::LeftHand, HN_ThumbFinger2, {2.5f, 0.0f, 2.5f}, {2.145f, 0.0f, 2.145f}, true},
    {"LeftIndexFinger", 41, BvhSourceKind::LeftHand, HN_IndexFinger, {4.4f, 0.7f, 2.6f}, {0, 0, 0}, false},
    {"LeftIndexFinger1", 45, BvhSourceKind::LeftHand, HN_IndexFinger1, {7.0f, -0.1f, 1.4f}, {0, 0, 0}, false},
    {"LeftIndexFinger2", 46, BvhSourceKind::LeftHand, HN_IndexFinger2, {5.0f, -0.2f, 0.0f}, {0, 0, 0}, false},
    {"LeftIndexFinger3", 47, BvhSourceKind::LeftHand, HN_IndexFinger3, {2.7f, -0.2f, 0.0f}, {2.3814f, -0.1764f, 0.0f}, true},
    {"LeftMiddleFinger", 41, BvhSourceKind::LeftHand, HN_MiddleFinger, {4.6f, 0.7f, 1.0f}, {0, 0, 0}, false},
    {"LeftMiddleFinger1", 49, BvhSourceKind::LeftHand, HN_MiddleFinger1, {7.0f, -0.1f, 0.4f}, {0, 0, 0}, false},
    {"LeftMiddleFinger2", 50, BvhSourceKind::LeftHand, HN_MiddleFinger2, {5.3f, -0.3f, 0.0f}, {0, 0, 0}, false},
    {"LeftMiddleFinger3", 51, BvhSourceKind::LeftHand, HN_MiddleFinger3, {3.4f, -0.3f, 0.0f}, {2.9988f, -0.2646f, 0.0f}, true},
    {"LeftRingFinger", 41, BvhSourceKind::LeftHand, HN_RingFinger, {4.5f, 0.8f, -0.1f}, {0, 0, 0}, false},
    {"LeftRingFinger1", 53, BvhSourceKind::LeftHand, HN_RingFinger1, {6.3f, -0.1f, -0.7f}, {0, 0, 0}, false},
    {"LeftRingFinger2", 54, BvhSourceKind::LeftHand, HN_RingFinger2, {4.7f, -0.3f, 0.0f}, {0, 0, 0}, false},
    {"LeftRingFinger3", 55, BvhSourceKind::LeftHand, HN_RingFinger3, {3.2f, -0.2f, 0.0f}, {2.8224f, -0.1764f, 0.0f}, true},
    {"LeftPinkyFinger", 41, BvhSourceKind::LeftHand, HN_PinkyFinger, {4.3f, 0.7f, -1.6f}, {0, 0, 0}, false},
    {"LeftPinkyFinger1", 57, BvhSourceKind::LeftHand, HN_PinkyFinger1, {5.6f, -0.1f, -1.5f}, {0, 0, 0}, false},
    {"LeftPinkyFinger2", 58, BvhSourceKind::LeftHand, HN_PinkyFinger2, {3.7f, -0.2f, 0.1f}, {0, 0, 0}, false},
    {"LeftPinkyFinger3", 59, BvhSourceKind::LeftHand, HN_PinkyFinger3, {2.4f, -0.1f, -0.1f}, {2.1168f, -0.0882f, -0.0882f}, true}
};

constexpr int kFullNodeCount = static_cast<int>(sizeof(kFullNodes) / sizeof(kFullNodes[0]));

void quatMul(const float q1[4], const float q2[4], float out[4])
{
    out[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    out[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    out[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    out[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

void quatConj(const float q[4], float out[4])
{
    out[0] = q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = -q[3];
}

void quatToMatrix(const float q[4], float r[3][3])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    r[0][0] = 1.0f - 2.0f * y * y - 2.0f * z * z;
    r[0][1] = 2.0f * x * y - 2.0f * w * z;
    r[0][2] = 2.0f * x * z + 2.0f * w * y;
    r[1][0] = 2.0f * x * y + 2.0f * w * z;
    r[1][1] = 1.0f - 2.0f * x * x - 2.0f * z * z;
    r[1][2] = 2.0f * y * z - 2.0f * w * x;
    r[2][0] = 2.0f * x * z - 2.0f * w * y;
    r[2][1] = 2.0f * y * z + 2.0f * w * x;
    r[2][2] = 1.0f - 2.0f * x * x - 2.0f * y * y;
}

void matrixToEulerZXY(const float r[3][3], float e[3])
{
    float r12 = r[1][2];
    if (r12 > 1.0f) r12 = 1.0f;
    if (r12 < -1.0f) r12 = -1.0f;

    e[0] = std::atan2(r[1][0], r[1][1]) * kRadToDeg;
    e[1] = std::asin(-r12) * kRadToDeg;
    e[2] = std::atan2(r[0][2], r[2][2]) * kRadToDeg;
}

void worldQuatToBvh(const float childWorld[4], const float parentWorld[4], float e[3])
{
    float local[4];
    if (parentWorld) {
        float inv[4];
        quatConj(parentWorld, inv);
        quatMul(inv, childWorld, local);
    } else {
        local[0] = childWorld[0];
        local[1] = childWorld[1];
        local[2] = childWorld[2];
        local[3] = childWorld[3];
    }

    float r[3][3];
    float ezxy[3];
    quatToMatrix(local, r);
    matrixToEulerZXY(r, ezxy);
    e[0] = ezxy[2];
    e[1] = ezxy[1];
    e[2] = ezxy[0];
}

void sdkQuatToBvh(const float src[4], float dst[4])
{
    dst[0] = src[0];
    dst[1] = -src[1];
    dst[2] = src[3];
    dst[3] = src[2];
}

bool quatLooksValid(const float q[4])
{
    float n = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    return n > 0.25f;
}

const float* nodeQuat(const BvhNodeDef& node,
                      float bodyQ[NODES_BODY][4],
                      float rHandQ[NODES_HAND][4],
                      float lHandQ[NODES_HAND][4])
{
    if (node.source == BvhSourceKind::Body) return bodyQ[node.sdkIndex];
    if (node.source == BvhSourceKind::RightHand) return rHandQ[node.sdkIndex];
    return lHandQ[node.sdkIndex];
}

void computeBodyEuler(float bodyQ[NODES_BODY][4], float e[NODES_BODY][3])
{
    for (int i = 0; i < NODES_BODY; ++i) {
        int parent = kBodyParent[i];
        worldQuatToBvh(bodyQ[i], parent >= 0 ? bodyQ[parent] : nullptr, e[i]);
    }
}

void computeFullEuler(float bodyQ[NODES_BODY][4],
                      float rHandQ[NODES_HAND][4],
                      float lHandQ[NODES_HAND][4],
                      float e[][3])
{
    for (int i = 0; i < kFullNodeCount; ++i) {
        const float* child = nodeQuat(kFullNodes[i], bodyQ, rHandQ, lHandQ);
        const float* parentQuat = nullptr;
        int parent = kFullNodes[i].parent;
        if (parent >= 0) {
            if (kFullNodes[i].source == BvhSourceKind::RightHand &&
                kFullNodes[parent].source == BvhSourceKind::Body) {
                parentQuat = quatLooksValid(rHandQ[HN_Hand]) ? rHandQ[HN_Hand] : bodyQ[BN_RightHand];
            } else if (kFullNodes[i].source == BvhSourceKind::LeftHand &&
                       kFullNodes[parent].source == BvhSourceKind::Body) {
                parentQuat = quatLooksValid(lHandQ[HN_Hand]) ? lHandQ[HN_Hand] : bodyQ[BN_LeftHand];
            } else {
                parentQuat = nodeQuat(kFullNodes[parent], bodyQ, rHandQ, lHandQ);
            }
        }
        worldQuatToBvh(child, parentQuat, e[i]);
    }
}

bool hasFullChild(int node)
{
    for (int i = 0; i < kFullNodeCount; ++i) {
        if (kFullNodes[i].parent == node) return true;
    }
    return false;
}

bool hasBodyChild(int node)
{
    for (int i = 0; i < NODES_BODY; ++i) {
        if (kBodyParent[i] == node) return true;
    }
    return false;
}

} // namespace

const std::vector<MocapJointDefinition>& mocapJointDefinitions()
{
    static const std::vector<MocapJointDefinition> definitions = [] {
        std::vector<MocapJointDefinition> result;
        result.reserve(NODES_BODY);
        for (int i = 0; i < NODES_BODY; ++i) {
            MocapJointDefinition joint = {
                kBodyName[i],
                kBodyParent[i],
                MocapJointSource::Body,
                i
            };
            result.push_back(joint);
        }
        return result;
    }();
    return definitions;
}

const float* mocapJointPosition(const _MocapDataWithVirtual_& md,
                                const MocapJointDefinition& joint)
{
    if (joint.source == MocapJointSource::Body) {
        return md.position_body[joint.sdkIndex];
    }
    if (joint.source == MocapJointSource::RightHand) {
        return md.position_rHand[joint.sdkIndex];
    }
    return md.position_lHand[joint.sdkIndex];
}

const float* mocapJointQuaternion(const _MocapDataWithVirtual_& md,
                                  const MocapJointDefinition& joint)
{
    if (joint.source == MocapJointSource::Body) {
        return md.quaternion_body[joint.sdkIndex];
    }
    if (joint.source == MocapJointSource::RightHand) {
        return md.quaternion_rHand[joint.sdkIndex];
    }
    return md.quaternion_lHand[joint.sdkIndex];
}

BvhExporter::BvhExporter()
{
    pthread_mutex_init(&mutex_, nullptr);
}

BvhExporter::~BvhExporter()
{
    pthread_mutex_destroy(&mutex_);
}

void BvhExporter::setMode(BvhExportMode mode)
{
    pthread_mutex_lock(&mutex_);
    if (!recording_) mode_ = mode;
    pthread_mutex_unlock(&mutex_);
}

BvhExportMode BvhExporter::mode() const
{
    BvhExporter* self = const_cast<BvhExporter*>(this);
    pthread_mutex_lock(&self->mutex_);
    BvhExportMode mode = mode_;
    pthread_mutex_unlock(&self->mutex_);
    return mode;
}

void BvhExporter::setPrependStudioRestFrame(bool enabled)
{
    pthread_mutex_lock(&mutex_);
    if (!recording_) prependStudioRestFrame_ = enabled;
    pthread_mutex_unlock(&mutex_);
}

bool BvhExporter::prependStudioRestFrame() const
{
    BvhExporter* self = const_cast<BvhExporter*>(this);
    pthread_mutex_lock(&self->mutex_);
    bool enabled = prependStudioRestFrame_;
    pthread_mutex_unlock(&self->mutex_);
    return enabled;
}

void BvhExporter::begin(int frequency)
{
    pthread_mutex_lock(&mutex_);
    frames_.clear();
    frequency_ = frequency > 0 ? frequency : 60;
    recording_ = true;
    pthread_mutex_unlock(&mutex_);
}

void BvhExporter::cancel()
{
    pthread_mutex_lock(&mutex_);
    recording_ = false;
    frames_.clear();
    pthread_mutex_unlock(&mutex_);
}

bool BvhExporter::isRecording() const
{
    BvhExporter* self = const_cast<BvhExporter*>(this);
    pthread_mutex_lock(&self->mutex_);
    bool recording = recording_;
    pthread_mutex_unlock(&self->mutex_);
    return recording;
}

void BvhExporter::addFrame(const _MocapDataWithVirtual_& md)
{
    if (!md.isUpdate) return;
    pthread_mutex_lock(&mutex_);
    if (recording_) frames_.push_back(buildMotionLine(md));
    pthread_mutex_unlock(&mutex_);
}

bool BvhExporter::saveToFile(const std::string& path)
{
    std::vector<std::string> framesCopy;
    int frequency = 60;
    bool addRest = false;
    BvhExportMode mode;

    pthread_mutex_lock(&mutex_);
    recording_ = false;
    framesCopy = frames_;
    frames_.clear();
    frequency = frequency_;
    addRest = prependStudioRestFrame_;
    mode = mode_;
    pthread_mutex_unlock(&mutex_);

    std::ofstream out(path);
    if (!out.is_open()) return false;

    BvhExportMode oldMode = mode_;
    mode_ = mode;
    out << std::fixed << std::setprecision(4) << "HIERARCHY\n";
    writeHierarchy(out);

    int frames = static_cast<int>(framesCopy.size()) + (addRest ? 1 : 0);
    if (frequency <= 0) frequency = 60;
    out << "MOTION\nFrames: " << frames << "\nFrame Time: "
        << std::setprecision(7) << (1.0 / static_cast<double>(frequency)) << "\n";

    if (addRest) out << studioRestFrame();
    for (const std::string& line : framesCopy) out << line;
    mode_ = oldMode;
    return true;
}

int BvhExporter::capturedFrameCount() const
{
    BvhExporter* self = const_cast<BvhExporter*>(this);
    pthread_mutex_lock(&self->mutex_);
    int count = static_cast<int>(frames_.size());
    pthread_mutex_unlock(&self->mutex_);
    return count;
}

int BvhExporter::outputFrameCount() const
{
    BvhExporter* self = const_cast<BvhExporter*>(this);
    pthread_mutex_lock(&self->mutex_);
    int count = static_cast<int>(frames_.size()) + (prependStudioRestFrame_ ? 1 : 0);
    pthread_mutex_unlock(&self->mutex_);
    return count;
}

std::string BvhExporter::buildMotionLine(const _MocapDataWithVirtual_& md) const
{
    float rx = -md.position_body[BN_Hips][0] * 100.0f;
    float ry = (md.position_body[BN_Hips][2] - 1.11f) * 100.0f;
    float rz = md.position_body[BN_Hips][1] * 100.0f;

    float bodyQ[NODES_BODY][4];
    for (int i = 0; i < NODES_BODY; ++i) {
        sdkQuatToBvh(md.quaternion_body[i], bodyQ[i]);
    }

    float rHandQ[NODES_HAND][4];
    float lHandQ[NODES_HAND][4];
    for (int i = 0; i < NODES_HAND; ++i) {
        sdkQuatToBvh(md.quaternion_rHand[i], rHandQ[i]);
        sdkQuatToBvh(md.quaternion_lHand[i], lHandQ[i]);
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4) << rx << " " << ry << " " << rz;
    oss << std::setprecision(2);

    if (mode_ == BvhExportMode::FullHands) {
        float e[kFullNodeCount][3];
        computeFullEuler(bodyQ, rHandQ, lHandQ, e);
        for (int i = 0; i < kFullNodeCount; ++i) {
            oss << " " << e[i][0] << " " << e[i][1] << " " << e[i][2];
        }
    } else {
        float e[NODES_BODY][3];
        computeBodyEuler(bodyQ, e);
        oss << " " << e[BN_Hips][0] << " " << e[BN_Hips][1] << " " << e[BN_Hips][2];
        static const int dfs[] = {
            BN_RightUpperLeg, BN_RightLowerLeg, BN_RightFoot, BN_RightToe,
            BN_LeftUpperLeg, BN_LeftLowerLeg, BN_LeftFoot, BN_LeftToe,
            BN_Spine, BN_Spine1, BN_Spine2, BN_Spine3, BN_Neck, BN_Head,
            BN_RightShoulder, BN_RightUpperArm, BN_RightLowerArm, BN_RightHand,
            BN_LeftShoulder, BN_LeftUpperArm, BN_LeftLowerArm, BN_LeftHand
        };
        for (int node : dfs) oss << " " << e[node][0] << " " << e[node][1] << " " << e[node][2];
    }

    oss << "\n";
    return oss.str();
}

void BvhExporter::writeHierarchy(std::ostream& os) const
{
    if (mode_ == BvhExportMode::FullHands) {
        writeFullNode(os, 0, 0);
    } else {
        writeBodyNode(os, 0, 0);
    }
}

void BvhExporter::writeFullNode(std::ostream& os, int nodeIndex, int depth) const
{
    std::string tab(depth, '\t');
    const BvhNodeDef& node = kFullNodes[nodeIndex];
    if (depth == 0) {
        os << "ROOT " << node.name << "\n{\n";
        os << tab << "OFFSET " << node.offset[0] << " " << node.offset[1] << " " << node.offset[2] << "\n";
        os << tab << "CHANNELS 6 Xposition Yposition Zposition Yrotation Xrotation Zrotation\n";
    } else {
        os << tab << "JOINT " << node.name << "\n" << tab << "{\n";
        os << tab << "\tOFFSET " << node.offset[0] << " " << node.offset[1] << " " << node.offset[2] << "\n";
        os << tab << "\tCHANNELS 3 Yrotation Xrotation Zrotation\n";
    }

    for (int i = 0; i < kFullNodeCount; ++i) {
        if (kFullNodes[i].parent == nodeIndex) writeFullNode(os, i, depth + 1);
    }

    if (!hasFullChild(nodeIndex)) {
        float ex = node.hasEnd ? node.endOffset[0] : 0.0f;
        float ey = node.hasEnd ? node.endOffset[1] : 0.0f;
        float ez = node.hasEnd ? node.endOffset[2] : 0.0f;
        os << tab << "\tEnd Site\n" << tab << "\t{\n";
        os << tab << "\t\tOFFSET " << ex << " " << ey << " " << ez << "\n";
        os << tab << "\t}\n";
    }
    os << tab << "}\n";
}

void BvhExporter::writeBodyNode(std::ostream& os, int nodeIndex, int depth) const
{
    std::string tab(depth, '\t');
    if (depth == 0) {
        os << "ROOT " << kBodyName[nodeIndex] << "\n{\n";
        os << tab << "OFFSET " << kBodyOffset[nodeIndex][0] << " "
           << kBodyOffset[nodeIndex][1] << " " << kBodyOffset[nodeIndex][2] << "\n";
        os << tab << "CHANNELS 6 Xposition Yposition Zposition Yrotation Xrotation Zrotation\n";
    } else {
        os << tab << "JOINT " << kBodyName[nodeIndex] << "\n" << tab << "{\n";
        os << tab << "OFFSET " << kBodyOffset[nodeIndex][0] << " "
           << kBodyOffset[nodeIndex][1] << " " << kBodyOffset[nodeIndex][2] << "\n";
        os << tab << "CHANNELS 3 Yrotation Xrotation Zrotation\n";
    }

    for (int i = 0; i < NODES_BODY; ++i) {
        if (kBodyParent[i] == nodeIndex) writeBodyNode(os, i, depth + 1);
    }

    if (!hasBodyChild(nodeIndex)) {
        os << tab << "End Site\n" << tab << "{\n";
        os << tab << "\tOFFSET 0 0 0\n";
        os << tab << "}\n";
    }
    os << tab << "}\n";
}

std::string BvhExporter::studioRestFrame() const
{
    int nodeCount = mode_ == BvhExportMode::FullHands ? kFullNodeCount : NODES_BODY;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << "0.00 111.00 0.00";
    for (int i = 0; i < nodeCount; ++i) {
        oss << " 0.00 0.00 0.00";
    }
    oss << "\n";
    return oss.str();
}
