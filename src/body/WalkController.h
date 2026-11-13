#pragma once

#include <array>

#include "body/FlyBody.h"

namespace fly {

// A central pattern generator for walking.
//
// THIS IS A MODELLING CHOICE, NOT THE CONNECTOME. docs/findings.md (9) shows
// the rhythm does not fall out of the wired premotor layer, and that anything
// added to produce it has to be labelled as an addition. This is that
// addition: a standard, biologically motivated controller of the kind
// NeuroMechFly v2 also uses, which turns high-level commands -- speed and
// turning, the kind of signal descending neurons carry -- into joint angle
// targets for the posture servos (FlyPhysics::Params::manualTarget).
//
// Rhythm: one phase oscillator per leg, coupled Kuramoto-style towards the
// alternating tripod (front_L, middle_R, hind_L against the other three).
// Coupling makes the gait self-correcting: knock one leg's phase and it is
// pulled back into the pattern rather than drifting forever.
//
// Step shape: each cycle is a stance phase, where ThC retracts at constant
// angular speed so the foot pushes the body forward at a steady rate, and a
// swing phase, where the leg protracts quickly while CTr lifts the foot clear.
// The earlier imposed gait used sin() for both, which spends half the stance
// nearly stationary and lifts the foot too little to clear the ground.
class WalkController {
public:
    struct Command {
        // -1..1. Positive walks forward, negative backward, 0 stands.
        float speed = 0.0f;
        // -1..1. Positive turns left (counter-clockwise seen from above):
        // the right legs take longer strides than the left ones.
        float turn = 0.0f;
    };

    // Defaults from parameter sweeps with flyphys --walk (see findings 12):
    // the combination that stayed upright walking straight AND turning both
    // ways, about 10 mm/s at 20 Hz -- the slow end of the 10-25 mm/s a real
    // fly walks at. Faster settings (lift 1.0, duty 0.55: 12 mm/s) tipped
    // over on turns or turned the wrong way.
    struct Params {
        float maxFrequencyHz = 20.0f;  // step frequency at |speed| = 1
        float minFrequencyHz = 6.0f;   // slowest rhythm while still walking
        float strideRad = 0.45f;       // ThC half-amplitude at |speed| = 1
        float liftRad = 0.8f;          // peak CTr lift during swing
        float dutyFactor = 0.65f;      // fraction of the cycle in stance (at full speed)
        float coupling = 30.0f;        // phase coupling strength, 1/s
        float rampPerSecond = 4.0f;    // how fast speed/turn follow the command

        // Heading hold (see holdHeading). Without it the fly wanders up to
        // 100 degrees in 1.5 s of "straight" walking: every step pushes
        // slightly sideways and nothing corrects it. A real fly corrects
        // course with vision (the optomotor response) and its halteres.
        float maxYawRateDeg = 90.0f;   // turn = +/-1 asks for this many deg/s
        // Gentle on purpose: at 2.0 / 0.7 the corrections themselves rocked
        // the body to 20 degrees of roll.
        float headingGain = 1.0f;      // turn command per radian of error
        float maxCorrection = 0.35f;   // cap on the corrective turn command
    };

    Params params;

    WalkController();

    // Advance the rhythm by dt seconds and write ThC/CTr targets (radians,
    // offsets from rest) for all six legs.
    void step(float dt, const Command& cmd,
              float targets[kLegCount][kJointCount]);

    // Course keeping. Treats cmd.turn as a requested turning *rate*,
    // integrates it into a desired heading, and returns a command whose turn
    // steers the measured heading (radians, from the thorax) towards it.
    // Call before step(). Also a modelling choice: it stands in for the
    // fly's visual and haltere feedback, which the body model does not have.
    Command holdHeading(float dt, const Command& cmd, float headingRad);

    void reset();

    // For display and tests: cycle phase in [0, 1) and whether a leg is
    // currently in swing.
    float phase(int leg) const { return phase_[leg]; }
    bool swinging(int leg) const;
    float speed() const { return speed_; }
    float turn() const { return turn_; }

private:
    std::array<float, kLegCount> phase_{};
    // Commands are smoothed: a real fly does not switch gait in one step, and
    // an instant jump in target angles is a shove the solver has to absorb.
    float speed_ = 0.0f;
    float turn_ = 0.0f;
    float beta_ = 0.65f;  // duty factor in use (it rises as the fly slows)
    bool haveHeading_ = false;
    float desiredHeading_ = 0.0f;
};

}  // namespace fly
