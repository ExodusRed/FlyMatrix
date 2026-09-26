#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "body/Anatomy.h"
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
        // Trunk mass and geometry come from body/Anatomy.h, which is the
        // single sourced description of the animal. They used to be literals
        // here and again in the renderer, which is how the two drifted apart.
        float thoraxMass = anat::kThoraxMass;   // micrograms; a fly is ~1 mg
        float abdomenMass = anat::kAbdomenMass;
        // How far behind the thorax centre the abdomen sits. It overhangs
        // the hind legs, so this is a lever arm on the body's pitch.
        float abdomenOffsetX = anat::kAbdomenX;
        float headMass = anat::kHeadMass;

        // Anatomical leg segments would weigh a fraction of a microgram, which
        // against a 400 ug thorax is a mass ratio of thousands to one -- the
        // condition an iterative solver handles worst, showing up as rubbery
        // legs and jitter. Segments are floored well above their true mass.
        // This trades physical accuracy for a solver that stays together, and
        // it is the single most important number here.
        // The tarsomeres below the TiTa joint have no motor neurons of
        // their own, so they are passive: a weak servo back toward straight
        // and a short range, which is what lets the foot drape over the
        // ground instead of meeting it as a single rigid spike.
        //
        // Stiffness is a fraction of postureTorque. Measured: the fly stands
        // from 0.3 upward, but walking needs 1.0 -- at 0.35 the foot is too
        // floppy and the gait tips the body to 77 degrees of pitch.
        //
        // Be clear about what this buys at the shipped value. Results are
        // identical from 1.0 to 20.0, which means the servo is holding the
        // tarsomeres straight and the chain is behaving near-rigidly while
        // walking. The articulation is anatomically right and the compliance
        // is real below 1.0, but the fly cannot yet walk in that regime, so
        // what ships is a jointed tarsus that mostly acts like a stiff one.
        float tarsusStiffness = 1.0f;
        float tarsusRangeRad = 0.45f;

        // A contact probe on every tarsomere rather than only the foot tip.
        //
        // On, now that the rest pose lays the tarsus flat. A real fly's
        // tarsus lies *along* the ground, which is what its five tarsomeres
        // and its adhesive pads are for, so a single point probe at the tip
        // was both wrong and the reason legs appeared to pass through the
        // floor -- nothing else on the leg was ever tested against it.
        //
        // This could not be switched on before: the rest pose put a point
        // foot on the floor, which left the rest of a jointed tarsus below
        // it, and enabling the probes pushed the body from 0.55 mm to 0.94
        // and left one foot down.
        bool tarsusProbes = true;
        // Contact points on the coxa, trochanter, femur and tibia too.
        // Without them the upper leg has no collision at all and runs
        // straight through the floor.
        bool upperLegProbes = true;

        // Lowered from 12 once the rest pose was re-solved for the new leg
        // lengths: at 12 the fly could not lift a swing leg without falling,
        // because 754 ug of leg against a 740 ug trunk is a lot of mass to
        // throw around. The foot then dragged through the whole cycle -- of
        // a 1.19 mm sweep relative to the body, 1.19 mm was loaded -- and the
        // return stroke pushed the body backwards nearly as hard as the power
        // stroke pushed it forwards. Stride efficiency 35%.
        //
        // At 8 the legs are light enough to lift, and stride efficiency goes
        // to 70% and walking speed from 11.5 to 14.9 mm/s. Below 8 the legs
        // go rubbery again and the fly stops standing.
        float minSegmentMass = 8.0f;
        // Tarsomeres get their own floor, and it is the same as the rest.
        //
        // The reasoning for lowering it was that a huge mass ratio between
        // neighbours is what an iterative solver handles worst, and that this
        // applies to a coxa hanging off a 330 ug thorax but not between one
        // tiny tarsomere and the next. The measurement disagreed: at 4 ug the
        // legs go rubbery and the fly sags to 0.447 mm, at 8 to 0.513, and it
        // only stands from 10 upward -- which saves 48 ug and no margin.
        //
        // This is the model's largest physical inaccuracy and it is worth
        // stating plainly. 54 leg segments floored at 12 ug give 754 ug of
        // leg against a 740 ug trunk, so the fly masses 1494 ug where a real
        // one is about 1000 and its legs are perhaps 8% of it. The floor is a
        // solver-stability number, not an anatomical one, and it dominates
        // the animal.
        float minTarsomereMass = 12.0f;
        float segmentMassScale = 1.0f;

        // Torque produced by a postural muscle at full activation. Individual
        // muscles scale this by their own strength (see MotorPools), so the
        // jump muscle needs no special case here.
        //
        // Scale matters here. The fly weighs about 1 mg, so its weight is
        // ~1e7 ug*mm/s^2, and a leg joint with a ~0.5 mm moment arm needs
        // torques of order 1e6 just to hold the animal up. Values that look
        // reasonable as bare numbers are three orders of magnitude too small
        // and the legs simply fold.
        // The one remaining free parameter in the muscle model: it converts
        // a motor neuron's segmentation volume into torque, and nothing
        // measures that conversion. Relative strengths between muscles are
        // no longer tuned -- they come from measured neuron size -- but the
        // overall scale still has to be set.
        //
        // At this value the giant fibre produces a 4.96 mm jump from a
        // 0.58 mm stance, against roughly 4.6 mm ballistic for a real
        // Drosophila taking off at 0.3 m/s. The usable window is narrow and
        // not monotonic: 4e5 and 6e5 both diverge sooner than this does.
        // THIS IS NOT A CALIBRATION, and it would be a mistake to treat the
        // jump height it produces as one.
        //
        // The giant fibre jump is chaotic with respect to this parameter.
        // Sampled across the plausible band, in millimetres of peak height
        // against a real escape takeoff of about 4.6 mm:
        //
        //   1.4e6  12.91     2.2e6   2.38     2.7e6   4.43
        //   1.6e6   2.71     2.4e6   2.77     2.8e6   3.21
        //   1.8e6   4.71     2.5e6   6.32     2.9e6   9.77
        //   2.0e6   2.98     2.6e6   3.38     3.0e6  17.51
        //
        // There is no plateau. Sampling more finely does not find one -- what
        // looked like a smooth stretch at 2.2/2.4/2.6 was aliasing, and the
        // points between it are 6.32 and 4.43. A jump is a brief, violent,
        // near-threshold event and small timing differences flip the outcome,
        // which is why merely caching the world inertia -- a change that
        // alters no physics -- moved this from 4.32 mm to 2.98.
        //
        // 2.7e6 is one sample that happens to land near the animal. Its
        // neighbours give 3.21 and 9.77. Choosing it because 4.43 looks right
        // would be fitting to noise, so it is chosen and labelled instead.
        //
        // Those numbers were measured against the previous rest pose. After
        // the tarsus was laid flat the same 2.7e6 gives 9.96 mm instead of
        // 4.43, which makes the point better than the table does.
        //
        // The model cannot currently produce a reproducible escape jump of a
        // given height. That is a real limitation, not a tuning problem.
        float maxMuscleTorque = 2.7e6f;    // ug*mm^2/s^2
        // Torque budget a joint can spend holding its posture. This is a
        // bound on an impulse, not a spring gain, so it can be raised freely
        // without threatening the integrator.
        float postureTorque = 6.0e6f;
        // Fraction of joint angle error corrected per substep.
        float servoRate = 0.9f;
        // Servo rate for the tarsomere joints, separately from the leg.
        //
        // They had the leg's 0.9, which with the servo's want =
        // servoRate * error * invDt means a demanded spin of 7200 rad/s per
        // radian of error at 8 kHz. A leg joint never achieves that: its
        // bounded torque cannot move that much inertia in a substep. A
        // tarsomere has almost none, so it achieves it exactly. Standing, the
        // errors are small and the peak spin anywhere in the animal is 183
        // rad/s. Walking, the distal tarsomere reaches 13,000 rad/s even in
        // runs that look perfectly healthy, and the body speed ceiling then
        // fires thousands of times a second, each firing rescaling one body
        // without its neighbours and prising the joints apart.
        float tarsusServoRate = 0.9f;
        float jointDamping = 0.6f;
        // Extra torque a fully activated muscle adds on top of posture, and
        // how far it shifts the joint's target angle.
        float muscleExcursionRad = 0.9f;

        // Hill muscle mechanics, both of which our muscles lacked.
        //
        // Force-length: a muscle produces its peak force near one length
        // and almost none fully shortened or fully stretched. Without it a
        // muscle keeps pulling at the end of its range, which is a large
        // part of why sustained drive made the fly climb: it was pushing
        // against its own joint stops indefinitely.
        //
        // Force-velocity: force falls as a muscle shortens quickly, and
        // rises somewhat when it is being stretched. That is real physical
        // damping, and a limb without it has nothing opposing fast motion.
        bool hillMuscle = true;
        // Joint rate, rad/s, at which shortening force halves.
        float hillShorteningRate = 12.0f;
        // Force multiplier when the muscle is being stretched instead.
        float hillLengtheningGain = 1.4f;
        // Fraction of the distance to a joint limit over which force
        // tapers to nothing.
        float hillTaperFrac = 0.35f;

        // 8 kHz keeps the stiff joint springs inside their stability limit;
        // at 4 kHz the solver diverges within a couple of milliseconds.
        // Diagnostic: when forceJoint is a valid joint index, every leg's
        // joint of that kind is driven with this constant value instead of by
        // the nervous system. Isolates the mechanics, which is the only way to
        // answer "can this joint lift the body at all" without guessing at
        // sign conventions.
        int forceJoint = -1;
        float forceDrive = 0.0f;
        // Restrict the diagnostic drive to one leg. The six legs do not
        // all sit in the same joint configuration, so a whole-body
        // average can read as "no effect" when legs are in fact pushing
        // hard in opposite directions.
        int forceLeg = -1;  // -1 drives every leg

        // Drive supplied from outside, per leg and per joint, used when
        // useManualDrive is set. This is how the gait test feeds a hand-built
        // pattern to the body without going through the nervous system.
        //
        // A pattern injected here is NOT connectome-driven walking and must
        // never be reported as such. Its purpose is to answer a question the
        // nervous system cannot be blamed for: given a correct gait signal,
        // can this body walk at all? If it cannot, no amount of work on the
        // network will produce walking, and that is worth knowing first.
        bool useManualDrive = false;
        float manualDrive[kLegCount][kJointCount] = {};

        // Target joint angles, radians, offset from the rest pose, used when
        // useManualTarget is set. The posture servo keeps its full torque
        // budget and drives the joint to the commanded angle.
        //
        // This is position control, which applyDrive deliberately does not
        // give the neural path: there, drive is feed-forward torque and it
        // *reduces* the postural hold, so a large command means a hard shove
        // with a weak servo. That is right for a jump and useless for placing
        // a foot, which is what a step is. Commanding angles here separates
        // "can the body walk" from "can a torque-only controller walk it".
        bool useManualTarget = false;
        float manualTarget[kLegCount][kJointCount] = {};

        float substepHz = 8000.0f;  // physics steps per simulated second
    };

    Params params;

    void build(const FlyBody& skeleton);

    // Advance by dt seconds, driving joints from the current muscle
    // activations. Runs several substeps internally.
    void step(float dtSeconds, const MotorPools& pools);
    // Mechanics-only step: drives nothing except the diagnostic
    // forceJoint, so the body can be probed without a nervous system.
    void step(float dtSeconds);

    // Copy the solved body poses back out for drawing.
    void readPose(std::vector<FlyBody::SegmentPose>& out) const;

    // Solved angle of one joint, in radians from the built pose.
    float jointAngle(int leg, int joint) const {
        return world.jointAngle(world.joints[jointIndex_[leg][joint]]);
    }

    const RigidBody& thorax() const { return world.bodies[thorax_]; }
    float bodyHeight() const { return world.bodies[thorax_].position.z; }
    // Highest point the thorax has reached since the last reset, which is how
    // a jump is measured.
    float peakHeight() const { return peakHeight_; }
    // Largest joint speed the muscle model has been asked about, rad/s.
    // Calibrating a force-velocity curve needs to know the range.
    float peakJointRate() const { return peakJointRate_; }
    // Largest |drive| any joint has been commanded with. drive is in
    // units of median-motor-neuron force, so peak torque in physical
    // units is this times maxMuscleTorque.
    float peakDrive() const { return peakDrive_; }
    // Which leg and link a body index belongs to, so a diagnostic can name
    // the part rather than print a number. link 0..4 are the five leg
    // segments; 5 and up are tarsomeres. Returns false for the trunk.
    bool bodyPart(std::uint32_t body, int& leg, int& link) const;

    // Total mass of every body, micrograms.
    float totalMass() const;
    // Mass-weighted centre of the whole animal, world millimetres. The thorax
    // is not it: the legs and abdomen carry enough mass to move it, and the
    // fore-aft distance between this and the centre of pressure is what
    // decides whether gravity is quietly pitching the body over.
    V3 centreOfMass() const;
    bool airborne() const { return airborne_; }

    // Which probe index is which, so a contact can be named rather than
    // just counted. Feet come first, one per leg, then the trunk.
    std::size_t footProbe(int leg) const { return static_cast<std::size_t>(leg); }
    std::size_t thoraxProbe() const { return kLegCount; }
    std::size_t abdomenProbe() const { return kLegCount + 1; }
    std::size_t headProbe() const { return kLegCount + 2; }
    // True when any part of the trunk is touching the ground.
    bool trunkGrounded() const {
        for (const auto& c : world.contacts) {
            if (c.probe >= kLegCount) return true;
        }
        return false;
    }

    // What a leg's proprioceptors actually measure.
    //
    // Chordotonal organs report the leg's configuration and campaniform
    // sensilla the load on it. Neither is resolved per joint here, because
    // the connectome cannot tell us which joint a given sensory neuron
    // watches (see tools/sensory_map.py), so both are reported per leg.

    // Straight-line distance from the leg's attachment to its foot.
    float legSpan(int leg) const;
    // World position of a leg's foot: the tip of its last tarsomere.
    V3 footPosition(int leg) const;
    // Which leg a contact probe belongs to, -1 for the trunk.
    int probeLeg(std::uint32_t probe) const {
        return probe < probeLeg_.size() ? probeLeg_[probe] : -1;
    }
    // The same distance in the pose the body was built in.
    float legSpanRest(int leg) const { return legSpanRest_[leg]; }
    // Normal impulse currently carried by this leg's foot, 0 if airborne.
    float footLoad(int leg) const;

    void reset(const FlyBody& skeleton);

    PhysicsWorld world;

private:
    void advance(float dtSeconds);
    void applyDrive(HingeJoint& hj, float restAngle, float drive) const;
    float hillFactor(const HingeJoint& hj, float drive) const;

    struct SegmentRef {
        std::uint32_t body;
        float length;
        float radius;
        LegId leg;
        Joint joint;
    };

    std::uint32_t thorax_ = 0, abdomen_ = 0, head_ = 0;
    std::vector<SegmentRef> segments_;
    // Index into segments_ of each leg's most distal tarsomere -- the part
    // that actually touches the ground. The first kLegCount*kJointCount
    // entries of segments_ keep their old layout so every l*kJointCount + j
    // index still means what it did; the extra tarsomeres live past them.
    std::array<std::uint32_t, kLegCount> footSegment_{};
    // Which leg each contact probe belongs to, -1 for the trunk. The first
    // kLegCount probes are still the foot tips in leg order, so probe index
    // equals leg index there and the feet-down count is unchanged; this maps
    // the extra tarsomere probes back to their leg.
    std::vector<int> probeLeg_;
    // Joint index in world.joints for each [leg][joint].
    std::array<std::array<std::uint32_t, kJointCount>, kLegCount> jointIndex_{};
    std::array<std::array<float, kJointCount>, kLegCount> restAngle_{};

    float peakHeight_ = 0.0f;
    mutable float peakJointRate_ = 0.0f;
    float peakDrive_ = 0.0f;
    bool airborne_ = false;
    float restHeight_ = 0.0f;
    std::array<float, kLegCount> legSpanRest_{};
};

}  // namespace fly
