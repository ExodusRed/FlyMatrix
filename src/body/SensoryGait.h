#pragma once

#include "body/FlyPhysics.h"
#include "body/GaitPlan.h"

namespace fly {

// Walking without a clock.
//
// Every gait this project has had was a clock: phase = 2*pi*t/period, six legs
// assigned to two tripods, joint targets read from a table. An insect has
// nothing of the sort. Each leg is very largely its own controller, and a
// stance ends when the leg runs out of backward travel or its load falls
// away -- a *condition*, not a time. The tripod is then something that
// emerges from a handful of rules between neighbouring legs, not something
// imposed on them.
//
// Two things follow that the clock cannot produce.
//
// First, gait becomes a function of speed. Swing duration is roughly constant
// in insects while stance duration shrinks as the animal speeds up, so duty
// factor falls out of speed rather than being a number to tune, and the gait
// should walk itself through wave -> tetrapod -> tripod. That is a prediction
// this file can be tested against.
//
// Second, a clock-driven leg lifts while still carrying load and plants while
// still unloaded, because the clock cannot know. Those are impulsive events,
// and impulsive events are what the solver has been choking on. A
// sensory-triggered gait may therefore be kinder to the numerics as well as
// more faithful, which is worth measuring rather than assuming.
//
// The coordination rules are the Cruse/Walknet family. They are
// phenomenological -- a good description of how insects walk, NOT a claim
// about Drosophila circuitry -- and must never be reported as the connectome
// walking.
struct SensoryGaitParams {
    // Nominal walking speed, mm/s. Used only to size the stance timeout;
    // the actual speed is emergent. See `stanceLean`.
    float speed = 4.0f;

    // How far behind the foot's *measured* position the stance target is
    // commanded, millimetres. This is the propulsion, and it is the whole
    // drive signal.
    //
    // The first version ramped the stance target open-loop at a commanded
    // speed, assuming the body would follow. When it did not -- and it never
    // quite does, because five other legs are also negotiating -- the command
    // walked away from where the foot actually was and the leg was dragged.
    // That is precisely the impulsive event this controller exists to avoid,
    // reintroduced in the controller itself.
    //
    // Commanding a fixed small offset *behind the measured foot* cannot
    // diverge. The leg pushes back against the ground, the ground pushes the
    // body forward, and speed comes out rather than going in -- which is also
    // the honest arrangement, since a descending neuron sets drive, not
    // velocity.
    // How far the commanded stance target may lag behind or lead the foot's
    // measured position, millimetres. This is anti-windup, and it is the
    // whole propulsion story.
    //
    // Two wrong versions came first. An open-loop ramp at a commanded speed
    // walks away from where the foot actually is the moment the body fails to
    // keep up, and the leg is dragged -- the exact impulsive event this
    // controller exists to remove. Locking each target to its own foot
    // instead cannot diverge, but then the six stance legs no longer agree on
    // a speed, so they push against each other and the animal goes nowhere:
    // 0.35 mm/s with 5.9 feet down at all times.
    //
    // Stance legs have to share one backward rate in order to cooperate, and
    // that rate has to be prevented from running away. So: integrate the
    // common rate, clamp the result to a bounded offset from the measurement.
    float maxLag = 0.05f;

    // Swing duration, milliseconds. Held roughly constant with speed, as in
    // the animal.
    float swingMs = 25.0f;

    // Anterior and posterior extreme positions: how far fore and aft of its
    // rest point a foot may be. Stance ends at the PEP.
    float aep = 0.22f;
    float pep = -0.22f;

    // Peak foot lift during swing, millimetres.
    float liftMm = 0.09f;

    // Coordination rules, as gains so each can be switched off and measured.
    //
    // 1: a swinging leg suppresses the swing of its anterior neighbour, so
    //    adjacent legs do not lift together. This is the rule that keeps the
    //    animal standing up.
    // 2: a leg that has just touched down releases the swing of its anterior
    //    neighbour, which is what makes the wave travel front-ward.
    // 3: a leg approaching its own PEP releases the swing of its posterior
    //    neighbour, which sets the spacing between them.
    //
    // Direction matters and is a design choice: rules 1 and 2 run caudal to
    // rostral, rule 3 rostral to caudal.
    bool rule1 = true;
    bool rule2 = true;
    bool rule3 = true;
    // How close to its PEP a leg must be, as a fraction of stance travel,
    // before rule 3 counts it as "approaching".
    float rule3Frac = 0.65f;
    // Contralateral legs of the same segment do not swing together either.
    bool contra = true;

    // A stance leg carrying less than this fraction of the mean stance load
    // is treated as having lost the ground, and may swing early.
    float unloadFrac = 0.08f;
    // ...but not until it has used this much of its stance travel.
    //
    // Without the guard the rule eats the gait. The front legs carry very
    // little load in this body, so they read as slipping the instant they
    // touch down, swing again immediately, and take 43 steps in the time the
    // middle legs take 17 -- while also holding rule 3 permanently against
    // their posterior neighbours. A leg that has just been placed is not
    // slipping, it is waiting for weight.
    float minStanceFrac = 0.5f;

    // Load sharing, as in GaitPlan's planned gait: trim each stance foot up
    // or down until it carries its share. Six rigid legs under a rigid body
    // is statically indeterminate without it.
    float shareGain = 0.4f;
    float shareClamp = 0.04f;

    // Safety valve. A stance that has run this much past its expected
    // duration is ended regardless of the rules, so a deadlock cannot leave
    // every leg waiting for another. Multiples of the nominal stance time.
    float stanceTimeout = 3.0f;
};

class SensoryGait {
public:
    enum class Phase : unsigned char { Stance, Swing };

    void reset(const SensoryGaitParams& params);

    // One control step. Writes joint targets, relative to rest, for every leg.
    void update(float dt, const FlyPhysics& phys, const GaitPlan& plan,
                float out[kLegCount][kJointCount]);

    Phase phase(int leg) const { return phase_[leg]; }
    float footX(int leg) const { return footX_[leg]; }
    // Fraction of the run this leg has spent in stance, for duty factor.
    float dutyOf(int leg) const {
        const float t = stanceTime_[leg] + swingTime_[leg];
        return t > 0.0f ? stanceTime_[leg] / t : 0.0f;
    }
    int stepsTaken(int leg) const { return steps_[leg]; }
    // True if any leg had to be released by the timeout rather than by a
    // rule. A gait that relies on the safety valve is not coordinating.
    int timeouts() const { return timeouts_; }

private:
    bool swingAllowed(int leg) const;

    SensoryGaitParams p_;
    Phase phase_[kLegCount] = {};
    float footX_[kLegCount] = {};     // fore-aft foot offset from rest, mm
    float swingFrom_[kLegCount] = {}; // where this swing started
    float swingU_[kLegCount] = {};    // progress through swing, 0..1
    float zTrim_[kLegCount] = {};
    float refX_[kLegCount] = {};   // maps measured foot x onto footX_
    bool anchored_ = false;
    float sinceTouchdown_[kLegCount] = {};
    float inStanceFor_[kLegCount] = {};
    float stanceTime_[kLegCount] = {};
    float swingTime_[kLegCount] = {};
    int steps_[kLegCount] = {};
    int timeouts_ = 0;
};

// Which legs are neighbours, for the coordination rules.
//
// Index order is front_L, front_R, middle_L, middle_R, hind_L, hind_R, so the
// ipsilateral chains run 4 -> 2 -> 0 on the left and 5 -> 3 -> 1 on the right,
// caudal to rostral.
int anteriorNeighbour(int leg);   // -1 for the front legs
int posteriorNeighbour(int leg);  // -1 for the hind legs
int contralateral(int leg);

}  // namespace fly
