#pragma once

#include <array>
#include <string>
#include <vector>

#include "body/FlyBody.h"
#include "body/FlyModel.h"
#include "engine/Math.h"

namespace fly::kin {

// Kinematic walker -- variant B2 of the locomotion work (docs/findings.md 13-15).
//
// NO PHYSICS. Nothing here has mass, force or contact dynamics. It is built to
// make the fly *move* naturally and cheaply (real time in a browser):
//
//   commands -> coordination -> per-leg step planning -> inverse kinematics
//
// The body is the flybody fruit fly model (FlyModel: Vaxenburg, Siwanowicz,
// Merel et al., Nature 2025; Apache-2.0) -- its measured skeleton, joint axes
// and ranges, and its zero pose, which is a natural standing pose. Each leg
// has seven actuated joints (coxa abduct/twist/swing, femur twist/swing,
// tibia, tarsus) and four passive tarsal joints.
//
// The controller is our own, inspired by published work:
//  - Each leg runs its own swing/stance state machine, and gait is not
//    prescribed: legs lift when their foot nears the rear of its reach, and
//    local rules between neighbouring legs (no lift-off while a neighbour
//    swings; a neighbour's touchdown encourages lifting) make tripod-like and
//    wave-like coordination emerge with speed (Walknet).
//  - Swing duration is nearly constant and stance duration carries speed
//    (DeAngelis et al., eLife 2019; Chun et al., eLife 2021).
//  - Stance feet stay planted in the world; the body moves over them.
//  - IK distributes motion with per-leg-pair preferences: front and hind legs
//    mostly flex the femur-tibia joint, middle legs mostly rotate the coxa and
//    twist the femur (Anipose, Karashchuk et al. 2021).
class KinematicFly {
public:
    // Per-leg actuated joints, in this order.
    enum Dof { CoxaAbduct = 0, CoxaTwist, Coxa, FemurTwist, Femur, Tibia, Tarsus, DofCount };

    struct Command {
        float speed = 0.0f;  // 0..1 of maxSpeed, forward
        float turn = 0.0f;   // -1..1, positive turns left
    };

    struct Params {
        float maxSpeed = 18.0f;        // mm/s at speed = 1 (real flies: 10-25 mm/s)
        float maxYawRateDeg = 120.0f;  // deg/s at turn = +/-1 while walking
        float spinYawRateDeg = 60.0f;  // deg/s turning on the spot
        float swingTime = 0.032f;      // s; nearly constant across speeds
        float liftHeight = 0.28f;      // mm, peak claw clearance
        float reach[3] = {0.40f, 0.48f, 0.48f};  // front, middle, hind: mm from neutral
        float stepUrge = 0.55f;
        float touchdownBoost = 0.25f;
        float boostDecay = 0.06f;
        float bodyBob = 0.03f;         // mm per leg in swing
        int ikIterations = 4;          // Gauss-Newton steps per leg per frame (warm-started)
    };

    Params params;

    explicit KinematicFly(const FlyModel& model);

    void reset();
    void step(float dt, const Command& cmd);
    void translate(const V3& d);

    // Joint angles for the whole model (FlyModel::joints() order) and the
    // root transform, for FlyModel::pose().
    const std::vector<float>& jointAngles() const { return q_; }
    const Transform& root() const { return root_; }
    bool swinging(int leg) const { return legs_[leg].swing; }
    // Which leg a model body belongs to (-1 for none), for highlighting.
    int legOfBody(int body) const;

    // Diagnostics.
    float groundSpeed() const { return measuredSpeed_; }
    float stepFrequency() const;
    float worstFootError() const { return worstIkError_; }
    // Same, for planted feet only: this is what would show as a sliding foot.
    float worstStanceError() const { return worstStanceError_; }
    float restError() const { return restIkError_; }
    float heading() const { return heading_; }
    float standHeight() const { return baseHeight_; }
    int swingCount() const;
    std::array<float, DofCount> legAngles(int leg) const;  // offsets from rest
    V3 neutralFoot(int leg) const { return legs_[leg].neutralBody; }

private:
    struct LegState {
        std::array<int, DofCount> joint{};    // model joint index per DOF
        std::array<int, 4> passive{};         // tarsus2..tarsus5
        std::vector<int> chain;               // model bodies from coxa to claw
        int ankleBody = -1, clawBody = -1;
        std::array<float, DofCount> q{}, weight{}, lo{}, hi{};
        float ankleRise = 0.0f;               // rest ankle height above the claw
        V3 neutralBody;                       // rest claw position, body frame
        V3 footWorld;
        bool swing = false;
        float swingT = 0.0f;
        V3 liftoffWorld;
        float boost = 0.0f;
    };

    // Positions of the ankle and claw in the body (thorax) frame for leg
    // angles `q`, with the passive tarsus at its current curl.
    void legFk(int leg, const std::array<float, DofCount>& q, V3& ankle, V3& claw) const;
    void solveIk(int leg, const V3& clawTarget, int iterations);
    V3 touchdownTargetBody(int leg) const;
    float urge(int leg) const;
    bool neighbourSwinging(int leg) const;
    void writeJointAngles();

    const FlyModel& model_;
    std::array<LegState, kLegCount> legs_{};
    std::vector<float> q_;
    std::vector<int> bodyLeg_;
    Transform root_;
    float heading_ = 0.0f;
    float baseHeight_ = 1.2f;
    float groundZ_ = 0.0f;
    V3 velocityBody_{};
    float yawRate_ = 0.0f;
    float measuredSpeed_ = 0.0f;
    float worstIkError_ = 0.0f, restIkError_ = 0.0f, worstStanceError_ = 0.0f;
    float clock_ = 0.0f;
    std::array<float, kLegCount> lastStepAt_{};
    std::array<float, kLegCount> stepInterval_{};
};

}  // namespace fly::kin
