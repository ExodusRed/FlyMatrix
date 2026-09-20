#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "engine/Math.h"

namespace fly {

// Units are millimetres. An adult Drosophila is about 2.5 mm long, so a leg
// segment is a few tenths of a millimetre and the whole animal fits in a box
// roughly 3 mm on a side.

// The five joints of a fly leg, proximal to distal. These names match the
// `joint` column of data/bin/motor_map.tsv.
enum class Joint : std::uint8_t {
    ThC = 0,  // thorax-coxa: swings the whole leg forward and back
    CTr,      // coxa-trochanter: lifts and lowers the leg
    TrF,      // trochanter-femur: rotates the femur
    FTi,      // femur-tibia: the knee
    TiTa,     // tibia-tarsus: the ankle
    Count
};

constexpr int kJointCount = static_cast<int>(Joint::Count);
constexpr int kLegCount = 6;

const char* jointName(Joint j);

// Legs in the order used by motor_map.tsv: front/middle/hind x left/right.
enum class LegId : std::uint8_t {
    FrontL = 0, FrontR, MiddleL, MiddleR, HindL, HindR
};

const char* legName(LegId l);

struct JointSpec {
    V3 axis{0, 0, 1};     // rotation axis in the parent segment's frame
    float restAngle = 0.0f;
    float minAngle = -1.0f;
    float maxAngle = 1.0f;
    float segmentLength = 0.2f;  // length of the segment *distal* to this joint
    float segmentRadius = 0.03f;
};

struct Leg {
    LegId id{};
    V3 attach;        // where the coxa meets the thorax, in body frame
    float mirror = 1.0f;  // -1 for left legs, flipping the lateral axis
    std::array<JointSpec, kJointCount> joints{};
    std::array<float, kJointCount> angle{};  // current, radians
};

// A fly skeleton: one thorax with six three-segment-plus legs hanging off it.
//
// This is a kinematic description only -- no mass, no dynamics. Joint angles
// are set from outside (by the motor pools) and worldPose() resolves them into
// segment transforms for drawing.
class FlyBody {
public:
    FlyBody();

    Transform root;  // body position and orientation in the world

    std::array<Leg, kLegCount>& legs() { return legs_; }
    const std::array<Leg, kLegCount>& legs() const { return legs_; }

    float bodyLength() const { return bodyLength_; }
    float bodyRadius() const { return bodyRadius_; }

    // Set a joint angle, clamped to that joint's limits.
    void setAngle(LegId leg, Joint j, float radians);
    float angle(LegId leg, Joint j) const;

    // Return to the standing posture.
    void resetPose();

    // One drawable segment: a capsule from `a` to `b` in world space.
    struct SegmentPose {
        V3 a, b;
        float radius;
        LegId leg;
        Joint joint;
    };

    // Resolve the joint chain into world-space segments. Cleared and refilled,
    // so the caller can keep one vector across frames.
    void worldPose(std::vector<SegmentPose>& out) const;

    // World position of the foot (tarsus tip) of one leg.
    V3 footPosition(LegId leg) const;

private:
    std::array<Leg, kLegCount> legs_{};
    float bodyLength_ = 1.1f;
    float bodyRadius_ = 0.38f;
};

}  // namespace fly
