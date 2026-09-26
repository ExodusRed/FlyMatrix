#pragma once

#include "body/FlyBody.h"

namespace fly {

// A walking gait described by where the feet go, not by what the joints do.
//
// Every version of this gait until now drove one joint per effect: ThC got a
// sinusoid because it "takes the step", CTr got a pulse because it "lifts".
// Those are names for muscles, and they are only descriptions of the joint's
// effect at a neutral pose the fly is nowhere near. Measured at the pose it
// actually stands in, one radian of ThC moves the foot:
//
//   leg        dx       dy       dz
//   front   -0.589   +0.380   -0.492
//   middle  -0.820   -0.568   +0.443
//   hind    -0.772   -1.122   +0.903
//
// The hind leg's stepping joint moves its foot further sideways than
// forwards, and dz is opposite in sign front to hind, so a single command
// sent to all six drives the front feet into the floor while the hind feet
// come off it. That is a pitch oscillator, and it is what the body was
// visibly doing -- 8 degrees peak to peak, sixteen times a second.
//
// So the gait is written in millimetres of foot travel, and inverse
// kinematics works out what each leg has to do to produce it. Sign and scale
// differences between the legs stop being something to discover and
// rediscover; they come out of the geometry.
struct GaitSpec {
    float periodMs = 60.0f;
    // Fore-aft foot travel either side of the rest point, millimetres. The
    // stride is twice this.
    float strideMm = 0.24f;
    // Peak height the foot reaches during swing, millimetres.
    float liftMm = 0.09f;
    // Share of the cycle a leg spends on the ground. A fly walking is near
    // 0.5; slower gaits go higher.
    float duty = 0.55f;
};

class GaitPlan {
public:
    static constexpr int kSamples = 256;

    // Solve the whole cycle once, for every leg. Costs a few thousand small
    // IK solves and is done before the trial starts, so the control loop is
    // a table lookup.
    void build(const GaitSpec& spec);

    // Joint angles relative to the rest pose, for one leg at a point in its
    // own cycle. `phase` is in cycles and is wrapped.
    void targets(int leg, float phase, float out[kJointCount]) const;

    // Largest distance by which the IK failed to reach a target on this leg,
    // millimetres. A leg that cannot follow the path it is given is a leg
    // running out of joint travel, and it should be reported rather than
    // silently approximated.
    float worstResidual(int leg) const { return worstResidual_[leg]; }
    // Where the foot actually goes, for checking the plan against the path
    // that was asked for.
    V3 plannedFoot(int leg, float phase) const;

    const GaitSpec& spec() const { return spec_; }

    // Joint change that moves this leg's foot one millimetre along a body
    // axis, holding the other two. Least-norm, taken at the rest pose.
    //
    // This is what a controller needs to say anything about a foot in
    // physical units: "carry less load, so drop 20 micrometres" rather than
    // "add 0.14 radians of CTr, except on the front legs where it is TiTa and
    // the sign is the other way".
    const float* basis(int leg, int axis) const { return basis_[leg][axis]; }

    // Joint angles, relative to rest, that put this leg's foot at an
    // arbitrary offset from where it rests.
    //
    // `targets()` above can only answer "where should this leg be at this
    // point in the cycle", which presumes there is a cycle. A leg whose
    // stance ends when it runs out of travel rather than when a clock says
    // so has no phase to look up, so it needs to ask for a foot position
    // directly. Solved by inverse kinematics over a grid at build time and
    // interpolated here, because the linear `basis()` above is only accurate
    // near the rest pose and a stride is not.
    void jointsForFoot(int leg, float dx, float dz,
                       float out[kJointCount]) const;

    // Extent of that grid, millimetres. Asking outside it is clamped.
    static constexpr int kFootNX = 41;
    static constexpr int kFootNZ = 11;
    static constexpr float kFootSpanX = 0.50f;
    static constexpr float kFootSpanZ = 0.25f;

private:
    GaitSpec spec_;
    float table_[kLegCount][kSamples][kJointCount] = {};
    V3 foot_[kLegCount][kSamples] = {};
    float worstResidual_[kLegCount] = {};
    float basis_[kLegCount][3][kJointCount] = {};
    float footGrid_[kLegCount][kFootNX][kFootNZ][kJointCount] = {};

    void buildBasis();
    void buildFootGrid();
};

// Where one foot should be at a point in its cycle, as an offset from its
// rest position in body coordinates. Exposed because the tests check the
// plan against it.
V3 footTarget(const GaitSpec& spec, float phase);

}  // namespace fly
