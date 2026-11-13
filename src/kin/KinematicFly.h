#pragma once

#include <array>
#include <vector>

#include "body/FlyBody.h"
#include "engine/Math.h"

namespace fly::kin {

// Kinematic walker -- variant B2 of the locomotion work (see docs/findings.md 13).
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
//    tripod-like and wave-like coordination emerge with speed. That is the
//    central idea of Walknet (Cruse, Schilling and colleagues).
//  - Swing duration is nearly constant and stance duration carries speed,
//    which is how Drosophila coordination varies along one continuum
//    (DeAngelis, Zavatone-Veth and Clark, eLife 2019).
//  - Stance feet stay planted in the world; the body moves over them.
//  - The thorax-coxa joint gets three degrees of freedom, as the real ball
//    joint has, and inverse kinematics distributes each leg's motion over its
//    joints with per-leg-pair preferences: front and hind legs mostly flex the
//    knee (FTi), middle legs mostly rotate the coxa and twist the femur (the
//    Anipose findings, Karashchuk et al. 2021).
// The physics skeleton (FlyBody, FlyPhysics) is untouched; this model reads
// its geometry and rest pose and adds the two extra coxa degrees of freedom
// only here.
class KinematicFly {
public:
    // Joint order in this model: the 3-DOF coxa first, then the four distal
    // joints of FlyBody.
    enum Dof { ThCPro = 0, ThCAbd, ThCRot, CTr, TrF, FTi, TiTa, DofCount };

    struct Command {
        float speed = 0.0f;  // 0..1 of maxSpeed, forward
        float turn = 0.0f;   // -1..1, positive turns left
    };

    struct Params {
        float maxSpeed = 18.0f;        // mm/s at speed = 1 (real flies: 10-25 mm/s)
        float maxYawRateDeg = 120.0f;  // deg/s at turn = +/-1 while walking
        float spinYawRateDeg = 60.0f;  // deg/s turning on the spot (speed 0)
        float swingTime = 0.032f;      // s; nearly constant across speeds
        float liftHeight = 0.22f;      // mm, peak foot clearance
        float reach[3] = {0.30f, 0.34f, 0.36f};  // front, middle, hind: mm from neutral
        float stepUrge = 0.55f;        // fraction of reach at which a leg wants to step
        float touchdownBoost = 0.25f;  // Walknet-style: a neighbour's touchdown lowers the threshold
        float boostDecay = 0.06f;      // s for that encouragement to fade
        float tarsusAngleDeg = 32.0f;  // tarsus slope to the ground in stance
        float bodyBob = 0.025f;        // mm of vertical bob per leg in swing
    };

    Params params;

    KinematicFly();

    void reset();
    void step(float dt, const Command& cmd);
    // Move the whole fly, feet included, by `d` (used to recentre on an
    // endless floor; the walk itself is unaffected).
    void translate(const V3& d);

    // Pose for drawing.
    const Transform& root() const { return root_; }
    void segments(std::vector<FlyBody::SegmentPose>& out) const;
    bool swinging(int leg) const { return legs_[leg].swing; }

    // Diagnostics, for tests and the page.
    float groundSpeed() const { return measuredSpeed_; }
    float yawRateDeg() const { return measuredYawRate_; }
    float stepFrequency() const;     // steps per second per leg, recent average
    float worstFootError() const { return worstIkError_; }
    float heading() const { return heading_; }
    const std::array<float, DofCount>& angles(int leg) const { return legs_[leg].q; }
    int swingCount() const;

private:
    struct LegState {
        // Joint angles and the per-joint preference weights for IK.
        std::array<float, DofCount> q{}, rest{}, weight{};
        std::array<float, DofCount> lo{}, hi{};
        V3 neutralBody;      // resting foot position, body frame
        V3 footWorld;        // current foot (tarsus tip) position, world frame
        bool swing = false;
        float swingT = 0.0f; // seconds into the current swing
        V3 liftoffWorld;
        float boost = 0.0f;  // decaying encouragement from a neighbour's touchdown
        int stepsTaken = 0;
    };

    // Forward kinematics of one leg in the body frame. If `points` is given it
    // receives the joint positions (attach, after each segment).
    V3 legFk(int leg, const std::array<float, DofCount>& q, V3* tibiaEnd = nullptr,
             std::vector<V3>* points = nullptr, std::vector<Quat>* frames = nullptr) const;
    void solveIk(int leg, const V3& footTargetBody);
    V3 touchdownTargetBody(int leg) const;
    float urge(int leg) const;
    bool neighbourSwinging(int leg) const;

    FlyBody skeleton_;  // geometry and rest angles only
    std::array<LegState, kLegCount> legs_{};
    Transform root_;
    float heading_ = 0.0f;
    float baseHeight_ = 0.5f;
    float groundZ_ = 0.0f;
    V3 velocityBody_{};  // smoothed commanded velocity, body frame (x forward)
    float yawRate_ = 0.0f;
    float measuredSpeed_ = 0.0f, measuredYawRate_ = 0.0f;
    float worstIkError_ = 0.0f;
    float clock_ = 0.0f;
    std::array<float, kLegCount> lastStepAt_{};
    std::array<float, kLegCount> stepInterval_{};
};

}  // namespace fly::kin
