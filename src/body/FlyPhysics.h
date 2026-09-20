#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "body/FlyBody.h"
#include "body/MotorPools.h"
#include "engine/Physics.h"

namespace fly {

// Builds a physical fly from the kinematic skeleton and drives it with muscle
// torques instead of setting joint angles directly.
//
// Units are millimetres, milligrams and seconds throughout, so gravity is
// 9810 mm/s^2 and a force is mg*mm/s^2.
class FlyPhysics {
public:
    struct Params {
        float thoraxMass = 400.0f;   // micrograms; a fly is about 1 mg
        float abdomenMass = 350.0f;
        float headMass = 90.0f;

        // Anatomical leg segments would weigh a fraction of a microgram, which
        // against a 400 ug thorax is a mass ratio of thousands to one -- the
        // condition an iterative solver handles worst, showing up as rubbery
        // legs and jitter. Segments are floored well above their true mass.
        // This trades physical accuracy for a solver that stays together, and
        // it is the single most important number here.
        float minSegmentMass = 12.0f;
        float segmentMassScale = 1.0f;

        // Muscle strength: peak torque one fully activated pool can produce.
        //
        // Scale matters here. The fly weighs about 1 mg, so its weight is
        // ~1e7 ug*mm/s^2, and a leg joint with a ~0.5 mm moment arm needs
        // torques of order 1e6 just to hold the animal up. Values that look
        // reasonable as bare numbers are three orders of magnitude too small
        // and the legs simply fold.
        float maxMuscleTorque = 1.5e6f;    // ug*mm^2/s^2
        // The jump muscle is in a class of its own -- the tergotrochanteral
        // muscle drives one of the most powerful movements a fly makes.
        //
        // This multiplier is tuned, not derived: below about 20 the fly never
        // leaves the ground. Part of the reason it has to be so large is that
        // TTMn is only 2 neurons inside a coxa-trochanter pool of ~18, so
        // averaging the pool dilutes the one muscle that matters.
        float jumpTorqueScale = 60.0f;
        // Passive joint stiffness holds the rest posture when no muscle is
        // driving, standing in for cuticle elasticity and resting muscle tone.
        float jointStiffness = 3.0e6f;
        float jointDamping = 400.0f;

        // 8 kHz keeps the stiff joint springs inside their stability limit;
        // at 4 kHz the solver diverges within a couple of milliseconds.
        float substepHz = 8000.0f;  // physics steps per simulated second
    };

    Params params;

    void build(const FlyBody& skeleton);

    // Advance by dt seconds, driving joints from the current muscle
    // activations. Runs several substeps internally.
    void step(float dtSeconds, const MotorPools& pools);

    // Copy the solved body poses back out for drawing.
    void readPose(std::vector<FlyBody::SegmentPose>& out) const;

    const RigidBody& thorax() const { return world.bodies[thorax_]; }
    float bodyHeight() const { return world.bodies[thorax_].position.z; }
    // Highest point the thorax has reached since the last reset, which is how
    // a jump is measured.
    float peakHeight() const { return peakHeight_; }
    bool airborne() const { return airborne_; }

    void reset(const FlyBody& skeleton);

    PhysicsWorld world;

private:
    struct SegmentRef {
        std::uint32_t body;
        float length;
        float radius;
        LegId leg;
        Joint joint;
    };

    std::uint32_t thorax_ = 0, abdomen_ = 0, head_ = 0;
    std::vector<SegmentRef> segments_;
    // Joint index in world.joints for each [leg][joint].
    std::array<std::array<std::uint32_t, kJointCount>, kLegCount> jointIndex_{};
    std::array<std::array<float, kJointCount>, kLegCount> restAngle_{};

    float peakHeight_ = 0.0f;
    bool airborne_ = false;
    float restHeight_ = 0.0f;
};

}  // namespace fly
