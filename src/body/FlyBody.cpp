#include "FlyBody.h"

#include "body/Anatomy.h"

#include <algorithm>

namespace fly {
namespace {

// Body axes: +X forward (head), +Y left, +Z up.
//
// Left and right legs are mirror images through the XZ plane. Reflecting a
// rotation negates the axis components that lie in the mirror plane: an axis
// along Y survives unchanged, while axes along X or Z flip. So the joints
// hinging about the lateral axis (ThC, CTr, FTi, TiTa) share one axis across
// both sides, and those about the fore-aft or vertical axis (the leg mount
// and TrF) get multiplied by the side. Getting this backwards leaves the two sides in
// visibly different poses.
constexpr V3 kForward{1, 0, 0};
constexpr V3 kLeft{0, 1, 0};
constexpr V3 kUp{0, 0, 1};

struct LegLayout {
    LegId id;
    float attachX;     // along the body, positive toward the head
    float side;        // +1 left, -1 right
    float lengthScale; // hind legs are the longest, front the shortest
    // Fixed outward tilt of the leg at its mounting, radians. Seeded from the
    // ThC rest angles this joint used to hold before ThC became the fore-aft
    // swing, so the standing splay is unchanged.
    float splay;
    // Rest angles for ThC, CTr, TrF, FTi, TiTa. Solved by
    // tools/solve_rest_pose.py so all six feet land on the ground at once --
    // five coupled angles per leg is not something to fit by eye.
    float rest[kJointCount];
};

// Leg lengths and attachment points follow the usual description of an adult
// Drosophila: three pairs on the thorax, front legs shortest and angled
// forward, hind legs longest and angled back.
constexpr LegLayout kLayout[kLegCount] = {
    {LegId::FrontL,   0.34f, +1.0f, 0.88f, 0.657f, {-0.803f, -1.401f, -0.073f,  1.746f, 0.938f}},
    {LegId::FrontR,   0.34f, -1.0f, 0.88f, 0.657f, {-0.803f, -1.401f, -0.073f,  1.746f, 0.938f}},
    {LegId::MiddleL,  0.02f, +1.0f, 1.00f, 0.909f, {-0.567f, -1.217f, -0.011f,  2.320f, 0.491f}},
    {LegId::MiddleR,  0.02f, -1.0f, 1.00f, 0.909f, {-0.567f, -1.217f, -0.011f,  2.320f, 0.491f}},
    // All six legs now fold the same way -- CTr negative, FTi positive -- and
    // reach fore and aft with ThC instead. The rest-pose solver picks that
    // configuration once for the whole animal and scores it 83x better than
    // the next best, so the legs agreeing is not a constraint imposed against
    // the geometry's wishes; it is what the geometry prefers once ThC is free
    // to swing. The ThC gradient across the leg pairs, -0.841 front to -0.061
    // hind, is what used to be faked by folding the front legs backwards.
    {LegId::HindL,   -0.30f, +1.0f, 1.12f, 0.893f, {-0.111f, -0.677f,  0.008f,  2.167f, 0.397f}},
    {LegId::HindR,   -0.30f, -1.0f, 1.12f, 0.893f, {-0.111f, -0.677f,  0.008f,  2.167f, 0.397f}},
};

// Segment lengths for a middle leg, in millimetres, scaled per leg above.
// The trochanter is a very short hinge between coxa and femur. Sourced in
// body/Anatomy.h along with the rest of the animal.
constexpr float kCoxaLen = anat::kCoxaLen;
constexpr float kTrochLen = anat::kTrochLen;
constexpr float kFemurLen = anat::kFemurLen;
constexpr float kTibiaLen = anat::kTibiaLen;
constexpr float kTarsusLen = anat::kTarsusLen;

}  // namespace

const char* jointName(Joint j) {
    switch (j) {
        case Joint::ThC: return "ThC";
        case Joint::CTr: return "CTr";
        case Joint::TrF: return "TrF";
        case Joint::FTi: return "FTi";
        case Joint::TiTa: return "TiTa";
        default: return "?";
    }
}

const char* legName(LegId l) {
    switch (l) {
        case LegId::FrontL: return "front_L";
        case LegId::FrontR: return "front_R";
        case LegId::MiddleL: return "middle_L";
        case LegId::MiddleR: return "middle_R";
        case LegId::HindL: return "hind_L";
        case LegId::HindR: return "hind_R";
        default: return "?";
    }
}

FlyBody::FlyBody() {
    for (int i = 0; i < kLegCount; ++i) {
        const LegLayout& L = kLayout[i];
        Leg& leg = legs_[i];
        leg.id = L.id;
        leg.mirror = L.side;
        leg.attach = {L.attachX, L.side * bodyRadius_ * 0.75f, -bodyRadius_ * 0.35f};

        const float s = L.lengthScale;

        // At rest every segment points straight down the frame's local -Z, so
        // each joint's axis is chosen for what it does to a downward-hanging
        // leg, and the chain rotates the frame as it goes.
        //
        // The leg is mounted with a fixed outward tilt about the fore-aft
        // axis, so it projects ventrolaterally the way a fly's coxa does.
        // This used to be ThC's rest angle, which meant the joint spent its
        // only degree of freedom holding a posture.
        leg.mount = Quat::axisAngle(kForward * L.side, L.splay);

        // ThC protracts and retracts: it swings the whole leg forward and
        // back, which is the step. The axis is lateral, so it rotates the leg
        // in the fore-aft plane while leaving the mount's outward tilt alone.
        //
        // This was an abduction axis, and that was wrong on the data's own
        // terms. The connectome names this joint's motor neurons "Tergopleural/
        // Pleural promotor MN", "Pleural remotor/abductor MN", "Sternal
        // anterior rotator MN" and "Sternal posterior rotator MN" -- promotor
        // and remotor, 62 motor neurons whose job is swinging the leg fore and
        // aft. Wiring them to lateral splay left the model with no joint that
        // could take a step, and forced the rest-pose solver to reach forward
        // with the front legs and backward with the hind legs using CTr and
        // FTi in opposite directions. That is why the six legs disagreed about
        // which way a joint lifts (flyphys test 3), and why one "extend"
        // command extended some legs and flexed others.
        // Range wide enough to swing both ways from every leg's rest angle.
        // At +/-0.9 the front legs, resting at -0.841, had 0.06 rad of
        // protraction left against 1.74 of retraction: they could only ever
        // drag backwards, which is what pitched the body during the gait
        // test.
        leg.joints[0] = {kLeft, L.rest[0], -1.6f, 1.6f, kCoxaLen * s, 0.045f};
        // CTr depresses: it swings the femur down within the leg's own plane,
        // about the local lateral axis. This is the joint the tergotrochanteral
        // jump muscle acts on, and it is the joint that levers the body off the
        // ground -- an abduction axis here cannot generate lift at all, in
        // either direction, which is what the earlier version did.
        leg.joints[1] = {kLeft, L.rest[1], -1.6f, 1.6f, kTrochLen * s, 0.040f};
        // TrF twists the femur about its own long axis, which by now is the
        // frame's local Z.
        leg.joints[2] = {kUp * L.side, L.rest[2], -0.9f, 0.9f, kFemurLen * s, 0.036f};
        // FTi is the knee, the joint with the largest range, hinging about the
        // same local lateral axis.
        leg.joints[3] = {kLeft, L.rest[3], -2.6f, 2.6f, kTibiaLen * s, 0.028f};
        // TiTa is the ankle, bending the same way but less.
        leg.joints[4] = {kLeft, L.rest[4], -1.2f, 1.5f, kTarsusLen * s, 0.020f};
    }
    resetPose();
    root.position = {0, 0, 0.62f};
}

void FlyBody::resetPose() {
    for (auto& leg : legs_) {
        for (int j = 0; j < kJointCount; ++j) leg.angle[j] = leg.joints[j].restAngle;
    }
}

void FlyBody::setAngle(LegId leg, Joint j, float radians) {
    Leg& L = legs_[static_cast<int>(leg)];
    const JointSpec& spec = L.joints[static_cast<int>(j)];
    L.angle[static_cast<int>(j)] = std::clamp(radians, spec.minAngle, spec.maxAngle);
}

float FlyBody::angle(LegId leg, Joint j) const {
    return legs_[static_cast<int>(leg)].angle[static_cast<int>(j)];
}

void FlyBody::worldPose(std::vector<SegmentPose>& out) const {
    out.clear();
    for (const auto& leg : legs_) {
        // Start at the coxa attachment, in world space, tilted outward by the
        // leg's fixed mount.
        Transform frame = root * Transform{leg.attach, leg.mount, 1.0f};

        for (int j = 0; j < kJointCount; ++j) {
            const JointSpec& spec = leg.joints[j];
            // Apply this joint's rotation, then walk down its segment. Each
            // segment runs along the frame's local -Z (downward at rest), so a
            // leg at rest hangs below the body.
            frame.rotation = (frame.rotation *
                              Quat::axisAngle(spec.axis, leg.angle[j])).normalised();

            const V3 a = frame.position;
            const V3 dir = frame.rotation.rotate({0, 0, -1});
            const V3 b = a + dir * spec.segmentLength;

            out.push_back({a, b, spec.segmentRadius, leg.id, static_cast<Joint>(j)});
            frame.position = b;
        }
    }
}

V3 FlyBody::footPosition(LegId leg) const {
    const Leg& L = legs_[static_cast<int>(leg)];
    Transform frame = root * Transform{L.attach, L.mount, 1.0f};
    for (int j = 0; j < kJointCount; ++j) {
        frame.rotation = (frame.rotation *
                          Quat::axisAngle(L.joints[j].axis, L.angle[j])).normalised();
        frame.position = frame.position +
                         frame.rotation.rotate({0, 0, -1}) * L.joints[j].segmentLength;
    }
    return frame.position;
}

}  // namespace fly
