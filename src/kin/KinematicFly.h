#pragma once

#include <array>
#include <vector>

#include "body/FlyBody.h"
#include "engine/Math.h"

namespace fly::kin {

// Kinematic walker -- variant B2 of the locomotion work (docs/findings.md 13,
// anatomy v2 in 14).
//
// NO PHYSICS. Nothing here has mass, force or contact dynamics. It is built to
// make the fly *move* naturally and cheaply (real time in a browser), and it
// shares its idea of a step with the physical controller that will follow:
//
//   commands -> coordination -> per-leg step planning -> inverse kinematics
//
// Inspired by, not copied from, published work:
//  - Each leg runs its own swing/stance state machine, and gait is not
//    prescribed: legs lift when their foot nears the rear of its reach, and
//    local rules between neighbouring legs (a leg does not lift while a
//    neighbour swings; a neighbour's touchdown encourages lifting) make
//    tripod-like and wave-like coordination emerge with speed (Walknet).
//  - Swing duration is nearly constant and stance duration carries speed
//    (DeAngelis, Zavatone-Veth and Clark, eLife 2019; Chun et al. 2021).
//  - Stance feet stay planted in the world; the body moves over them.
//  - Legs use anatomy v2 (body/Anatomy.h): coxae on the ventral thorax near
//    the midline, a long forward-projecting front coxa, per-pair segment
//    lengths, a three-degree-of-freedom thorax-coxa ball joint, oblique hinge
//    axes, a one-way knee, and a five-tarsomere tarsus that lies on the
//    ground. IK distributes motion with per-leg-pair preferences: front and
//    hind legs mostly flex the knee, middle legs mostly rotate the coxa and
//    twist the femur (Anipose, Karashchuk et al. 2021).
// The physics skeleton (FlyBody, FlyPhysics) still uses the original anatomy.
class KinematicFly {
public:
    // Joint order: the 3-DOF coxa, then CTr, TrF, FTi. TiTa is not solved by
    // IK -- the tarsus follows the ground -- but its angle is reported.
    enum Dof { ThCPro = 0, ThCAbd, ThCRot, CTr, TrF, FTi, TiTa, DofCount };
    static constexpr int kIkDofs = 6;

    struct Command {
        float speed = 0.0f;  // 0..1 of maxSpeed, forward
        float turn = 0.0f;   // -1..1, positive turns left
    };

    struct Params {
        float maxSpeed = 18.0f;        // mm/s at speed = 1 (real flies: 10-25 mm/s)
        float maxYawRateDeg = 120.0f;  // deg/s at turn = +/-1 while walking
        float spinYawRateDeg = 60.0f;  // deg/s turning on the spot (speed 0)
        float swingTime = 0.032f;      // s; nearly constant across speeds
        float liftHeight = 0.20f;      // mm, peak clearance of the tarsus
        float reach[3] = {0.30f, 0.34f, 0.36f};  // front, middle, hind: mm from neutral
        float stepUrge = 0.55f;        // fraction of reach at which a leg wants to step
        float touchdownBoost = 0.25f;  // a neighbour's touchdown lowers the threshold
        float boostDecay = 0.06f;      // s for that encouragement to fade
        float bodyBob = 0.02f;         // mm of vertical bob per leg in swing
    };

    struct LegGeometry {
        V3 attach;            // coxa base, body frame
        float lengths[5];     // coxa, trochanter, femur, tibia, tarsus
        V3 neutralFoot;       // body frame
    };

    Params params;

    KinematicFly();

    void reset();
    void step(float dt, const Command& cmd);
    // Move the whole fly, feet included, by `d` (used to recentre on an
    // endless floor; the walk itself is unaffected).
    void translate(const V3& d);

    // Pose for drawing: coxa, trochanter, femur, tibia and five tarsomeres
    // per leg (tarsomeres are tagged Joint::TiTa).
    const Transform& root() const { return root_; }
    void segments(std::vector<FlyBody::SegmentPose>& out) const;
    bool swinging(int leg) const { return legs_[leg].swing; }

    // Diagnostics, for tests and the page.
    float groundSpeed() const { return measuredSpeed_; }
    float yawRateDeg() const { return measuredYawRate_; }
    float stepFrequency() const;
    float worstFootError() const { return worstIkError_; }
    float heading() const { return heading_; }
    const std::array<float, DofCount>& angles(int leg) const { return legs_[leg].q; }
    int swingCount() const;
    LegGeometry geometry(int leg) const;
    float standHeight() const { return baseHeight_; }
    // How well the rest pose reaches its neutral feet (mm). Large means the
    // anatomy cannot stand where it was asked to.
    float restError() const { return restIkError_; }

private:
    struct LegState {
        // Joint angles (radians), rest angles, IK preference weights, limits.
        std::array<float, DofCount> q{}, rest{}, weight{};
        std::array<float, DofCount> lo{}, hi{};
        // Geometry, body frame.
        V3 attach;
        Quat coxaFrame;          // rest orientation of the coxa (local -Z along it)
        float len[5]{};          // coxa, trochanter, femur, tibia, tarsus
        float side = 1.0f;       // +1 left, -1 right
        V3 neutralBody;          // neutral tarsal contact point
        V3 tarsusDirBody;        // horizontal direction the stance tarsus lies along
        // Walking state.
        V3 footWorld;            // tarsal contact point, world frame
        bool swing = false;
        float swingT = 0.0f;
        V3 liftoffWorld;
        float boost = 0.0f;
        int stepsTaken = 0;
    };

    // Forward kinematics to the ankle (tibia tip), body frame. Optionally
    // returns the joint positions: attach, coxa end, trochanter end, femur
    // end, ankle.
    V3 ankleFk(int leg, const std::array<float, DofCount>& q, std::vector<V3>* points = nullptr) const;
    // Where the ankle should be for a tarsal contact point at `foot` (body frame).
    V3 ankleTargetFor(int leg, const V3& footBody, float lift) const;
    void solveIk(int leg, const V3& ankleTargetBody, int iterations);
    V3 touchdownTargetBody(int leg) const;
    float urge(int leg) const;
    bool neighbourSwinging(int leg) const;

    std::array<LegState, kLegCount> legs_{};
    Transform root_;
    float heading_ = 0.0f;
    float baseHeight_ = 0.6f;
    float groundZ_ = 0.0f;
    V3 velocityBody_{};
    float yawRate_ = 0.0f;
    float measuredSpeed_ = 0.0f, measuredYawRate_ = 0.0f;
    float worstIkError_ = 0.0f;
    float restIkError_ = 0.0f;
    float clock_ = 0.0f;
    std::array<float, kLegCount> lastStepAt_{};
    std::array<float, kLegCount> stepInterval_{};
};

}  // namespace fly::kin
