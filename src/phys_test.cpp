// flyphys -- mechanics-only test harness for the fly body.
//
// Deliberately does not touch the connectome, the nervous system or SDL, so it
// starts instantly and every result is about the body alone. When the fly fails
// to move, the question "is this the neurons, the muscle model, the solver or
// the geometry?" is otherwise unanswerable, and guessing at it is expensive.
//
//   flyphys                 run every check
//   flyphys --trace FTi -15 print a time series for one forced joint

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "body/Anatomy.h"
#include "body/FlyBody.h"
#include "body/FlyPhysics.h"
#include "body/GaitPlan.h"

using namespace fly;

namespace {

constexpr float kDt = 0.001f;  // seconds

struct Result {
    float height;
    float peak;
    // Fore-aft travel of the thorax. ThC protracts and retracts, so
    // "does it lift" is the wrong question for it -- the right one is
    // whether it drives the body forward.
    float travelX;
    std::size_t contacts;
    float anchorError;
    std::size_t worstAnchor;
    float footLateral;
    float angle[kJointCount];
    // Worst joint deviation across all six legs, not just the one we
    // happen to print. Reporting a single leg hid the fact that the
    // others were not holding at all.
    float worstAngle;
    int worstLeg, worstJoint;
    float trunkDrop;   // true weld slip, orientation-aware
    float pitchDeg;    // nose-up pitch of the thorax
    // Straight-line thorax-to-foot distance, against what the rest pose
    // says it should be. If the joints hold and the foot is planted but
    // this has shrunk, the linkage itself is compressing.
    float reach, reachRest;
    // Per-leg: how much shorter the attachment-to-foot span has become
    // than the rest pose says it should be. If the joints hold but this
    // is non-zero, the linkage itself is being compressed, and if the
    // hind legs compress more than the front ones the body pitches.
    float legShort[kLegCount];
    int feetDown;
    bool thoraxDown, abdomenDown, headDown;
    // Did the fly leave the ground at any point during the probe? If it did,
    // the height change is a flight path and says nothing about which way the
    // joint lifts.
    bool tookOff;
    bool diverged;
};

// Settle the body for `ms`, optionally forcing one joint, and report where it
// ended up and what every joint of the front-left leg did.
float g_stiffness = -1.0f;
float g_servo = -1.0f;
float g_abdMass = -1.0f;
float g_abdX = 0.0f;
int g_iters = -1;
float g_baum = -1.0f;
float g_corr = -1.0f;
float g_substep = -1.0f;
float g_tarsus = -1.0f;
float g_minseg = -1.0f;
float g_friction = -1.0f;
float g_mintar = -1.0f;
// Drive used by tests 2, 3 and 4 to probe a joint.
//
// It was +/-15 throughout, chosen when maxMuscleTorque was 5e5. At the
// current 2.7e6 that is some ten body weights on a single joint, and the
// tests stopped measuring which way a joint lifts and started measuring
// how far it throws the animal -- height changes of 3.4, 10.3 and 19.8 mm
// on a fly that stands 0.55 mm off the ground.
//
// Worse, it inverted the answers. At +/-15 the table said CTr -15 sinks and
// the legs disagree about it; at +/-3, which is about two body weights on the
// joint, CTr -3 lifts on every leg and CTr +3 sinks on every leg. The signs
// the rest of the project reads off this test were being taken from a fly
// being thrown into the air.
// 1.5 is the drive at which this test is actually usable: no row takes off,
// every joint moves, and nine of ten per-leg verdicts agree. At 3.0 half the
// rows of test 2 are flight paths and four of ten legs disagree; at 1.0 most
// joints do not move at all. The window is narrow because joint drive is not
// graded -- below a threshold the posture servo holds, above it the joint
// slams to its stop.
float g_probeDrive = 1.5f;
// Dense trace window, seconds. The bucket table said a run was steady and
// then was not; this prints every frame across the transition, so the event
// can be read rather than inferred from ten samples in twelve seconds.
float g_from = -1.0f, g_to = -1.0f;
float g_hillVmax = -1.0f;
int g_hill = -1;
int g_inter = -1;
int g_upper = -1;
float g_ms = 300.0f;
bool g_standOnly = false;

Result run(float ms, int forceJoint, float forceDrive, int forceLeg = -1) {
    FlyBody skeleton;
    FlyPhysics phys;
    if (g_stiffness > 0.0f) phys.params.postureTorque = g_stiffness;
    if (g_servo > 0.0f) phys.params.servoRate = g_servo;
    if (g_substep > 0.0f) phys.params.substepHz = g_substep;
    if (g_tarsus > 0.0f) phys.params.tarsusStiffness = g_tarsus;
    if (g_minseg > 0.0f) phys.params.minSegmentMass = g_minseg;
    if (g_mintar > 0.0f) phys.params.minTarsomereMass = g_mintar;
    if (g_upper >= 0) phys.params.upperLegProbes = (g_upper != 0);
    if (g_abdMass >= 0.0f) phys.params.abdomenMass = g_abdMass;
    if (g_abdX != 0.0f) phys.params.abdomenOffsetX = g_abdX;
    phys.params.forceJoint = forceJoint;
    phys.params.forceDrive = forceDrive;
    phys.params.forceLeg = forceLeg;
    phys.build(skeleton);
    // Solver knobs live on the world, which build() recreates, so they
    // have to be applied afterwards.
    if (g_iters > 0) phys.world.params.iterations = g_iters;
    if (g_baum > 0.0f) phys.world.params.baumgarte = g_baum;
    if (g_corr > 0.0f) phys.world.params.maxCorrectionVelocity = g_corr;
    if (g_friction > 0.0f) phys.world.params.friction = g_friction;
    if (g_inter >= 0) phys.world.params.interleave = (g_inter != 0);

    const int n = static_cast<int>(ms / 1000.0f / kDt);
    Result r{};
    for (int i = 0; i < n; ++i) {
        phys.step(kDt);
        // Settling from the initial drop takes a moment; only count airtime
        // after that.
        if (i > 50 && phys.world.contacts.empty()) r.tookOff = true;
        if (!std::isfinite(phys.bodyHeight()) ||
            std::fabs(phys.bodyHeight()) > 1e4f) {
            r.diverged = true;
            break;
        }
    }
    r.height = phys.bodyHeight();
    r.travelX = phys.thorax().position.x;
    r.peak = phys.peakHeight();
    r.contacts = phys.world.contacts.size();
    r.anchorError = phys.world.maxAnchorError();
    r.worstAnchor = phys.world.worstAnchorJoint();
    r.feetDown = 0;
    r.thoraxDown = r.abdomenDown = r.headDown = false;
    {
        // A leg is down if *any* of its probes touches, not only the one at
        // the very tip. With the tarsus lying flat along the ground a leg
        // commonly rests on its middle tarsomeres, and counting tip probes
        // alone reported a standing fly as having four feet down.
        bool legDown[kLegCount] = {};
        for (const auto& c : phys.world.contacts) {
            const int l = phys.probeLeg(c.probe);
            if (l >= 0) legDown[l] = true;
            else if (c.probe == phys.thoraxProbe()) r.thoraxDown = true;
            else if (c.probe == phys.abdomenProbe()) r.abdomenDown = true;
            else if (c.probe == phys.headProbe()) r.headDown = true;
        }
        for (const bool d : legDown) if (d) ++r.feetDown;
    }
    {
        // How far the front-left foot has slid from where it started.
        //
        // Taken from footPosition rather than pose[kJointCount - 1], which
        // was the whole tarsus until it was split into five tarsomeres and
        // is now ta1 -- 40% of the way along the foot.
        r.footLateral = length(phys.footPosition(0) -
                               skeleton.footPosition(LegId::FrontL));
    }
    for (int j = 0; j < kJointCount; ++j) r.angle[j] = phys.jointAngle(0, j);
    r.worstAngle = 0.0f;
    r.worstLeg = r.worstJoint = 0;
    for (int l = 0; l < kLegCount; ++l) {
        for (int j = 0; j < kJointCount; ++j) {
            const float a = std::fabs(phys.jointAngle(l, j));
            if (a > r.worstAngle) {
                r.worstAngle = a;
                r.worstLeg = l;
                r.worstJoint = j;
            }
        }
    }
    // How far the abdomen has dropped relative to the thorax. If the
    // weld held, this is zero.
    {
        const auto& th = phys.world.bodies[0];
        const auto& ab = phys.world.bodies[1];
        // Compare the abdomen against where the thorax's own orientation puts
        // it, not against a fixed height difference. The abdomen sits 0.78 mm
        // behind the thorax, so a few degrees of body pitch shifts it
        // vertically even when the weld is perfectly rigid -- measuring raw
        // z-difference reports that as weld failure.
        // Read the offset the body was actually built with. This used to
        // be a literal -0.78, and when the anatomy moved the abdomen to
        // -0.86 the test reported the 0.08 mm difference as weld slip. The
        // weld was fine. Comparing against a number the model no longer
        // uses is how this file previously invented 79 um of slip that
        // turned out to be body pitch.
        const float abdX = (g_abdX != 0.0f) ? g_abdX : phys.params.abdomenOffsetX;
        const V3 expected = th.position + th.orientation.rotate({abdX, 0, -0.04f});
        r.trunkDrop = length(ab.position - expected);
        // Pitch: how far the thorax's forward axis has tilted out of level.
        const V3 fwd = th.orientation.rotate({1, 0, 0});
        r.pitchDeg = std::asin(std::clamp(fwd.z, -1.0f, 1.0f)) * 57.2958f;
    }
    {
        r.reach = length(phys.footPosition(0) - phys.world.bodies[0].position);
        r.reachRest = length(skeleton.footPosition(LegId::FrontL) -
                             skeleton.root.position);

        // Per-leg compression: coxa to foot now, against coxa to foot at
        // rest. Both sides have to mean the same thing, and for a while they
        // did not -- the physics side stopped at ta1 while the skeleton side
        // ran to the end of its single unsplit tarsus, so this differenced
        // two different lengths and reported the gap as compression.
        std::vector<FlyBody::SegmentPose> pose;
        phys.readPose(pose);
        std::vector<FlyBody::SegmentPose> restPose;
        skeleton.worldPose(restPose);
        for (int l = 0; l < kLegCount; ++l) {
            const std::size_t base = static_cast<std::size_t>(l) * kJointCount;
            const float now = length(phys.footPosition(l) - pose[base].a);
            const float was = length(skeleton.footPosition(static_cast<LegId>(l)) -
                                     restPose[base].a);
            r.legShort[l] = now - was;
        }
    }
    return r;
}

int traceOne(const std::string& jointName_, float drive) {
    int joint = -1;
    for (int j = 0; j < kJointCount; ++j) {
        if (jointName_ == jointName(static_cast<Joint>(j))) joint = j;
    }
    if (joint < 0) {
        std::fprintf(stderr, "unknown joint: %s\n", jointName_.c_str());
        return 1;
    }

    FlyBody skeleton;
    FlyPhysics phys;
    if (g_stiffness > 0.0f) phys.params.postureTorque = g_stiffness;
    if (g_servo > 0.0f) phys.params.servoRate = g_servo;
    if (g_substep > 0.0f) phys.params.substepHz = g_substep;
    if (g_tarsus > 0.0f) phys.params.tarsusStiffness = g_tarsus;
    if (g_minseg > 0.0f) phys.params.minSegmentMass = g_minseg;
    if (g_mintar > 0.0f) phys.params.minTarsomereMass = g_mintar;
    if (g_upper >= 0) phys.params.upperLegProbes = (g_upper != 0);
    if (g_abdMass >= 0.0f) phys.params.abdomenMass = g_abdMass;
    if (g_abdX != 0.0f) phys.params.abdomenOffsetX = g_abdX;
    phys.params.forceJoint = joint;
    phys.params.forceDrive = drive;
    phys.build(skeleton);

    std::printf("forcing %s at drive %+.1f\n\n", jointName_.c_str(), drive);
    std::printf("%8s %9s %9s", "t (ms)", "height", "contacts");
    for (int j = 0; j < kJointCount; ++j) {
        std::printf(" %8s", jointName(static_cast<Joint>(j)));
    }
    std::printf("\n");

    for (int i = 0; i <= 200; ++i) {
        if (i % 20 == 0) {
            std::printf("%8.0f %9.4f %9zu", i * kDt * 1000.0f, phys.bodyHeight(),
                        phys.world.contacts.size());
            for (int j = 0; j < kJointCount; ++j) {
                std::printf(" %+8.3f", phys.jointAngle(0, j));
            }
            std::printf("\n");
        }
        phys.step(kDt);
    }
    return 0;
}

}  // namespace

// Can this body walk if something hands it a correct gait?
//
// This imposes a tripod pattern by hand. It is NOT connectome-driven walking
// and must never be reported as such -- no neuron is involved and the rhythm
// comes from a sine wave. Its purpose is to separate two failures that look
// identical from outside: a body that cannot walk however it is driven, and a
// nervous system that is not producing a gait. Until the first is ruled out,
// work on the second is unfalsifiable.
//
// The pattern is the textbook alternating tripod: front-left, middle-right and
// hind-left swing together while the other three are in stance. Within a leg,
// ThC retracts through stance to push the body forward, and during swing CTr
// lifts the foot clear while ThC protracts to reset it.
// One gait trial. `jitter` displaces the starting pose very slightly, which
// is how the same gait is asked the same question several times.
struct GaitResult {
    float speed = 0.0f, stride = 0.0f, duty = 0.0f, pitch = 0.0f;
    float height = 0.0f;
    float peakRate = 0.0f;
    bool upright = false;
};

struct GaitParams {
    // 60 ms and a 0.65 rad stride, chosen after the geometry was fixed.
    //
    // The old defaults were 160 ms and 0.2 rad, forced there because faster
    // stepping launched the fly. That was never really an energy-pumping
    // limit: the legs were passing through the floor and the tarsal chain was
    // coming apart, and the discontinuous contacts that produced amplified
    // every perturbation. With every segment given collision and the rest
    // pose solved for ground clearance, 60 ms works and is four times
    // quicker, at 9 of 9 trials upright with the pitch varying by 0.6 degrees
    // across the whole perturbation range.
    float periodMs = 60.0f;
    float swing = 0.4f;    // ThC fore-aft amplitude, radians
    float lift = 0.3f;     // CTr lift during swing
    float toe = 0.2f;      // TiTa curl during swing, so a flat tarsus clears

    // --- stabilisation, all hand-built and none of it neural ---
    //
    // postureGain/postureRate change how far a leg *extends*, from body
    // attitude and its rate. That is a stiffness response.
    float postureGain = 0.0f;
    float postureRate = 0.0f;

    // stepGain changes where a foot is *placed*, from how fast the body is
    // travelling.
    //
    // This is the lever that actually stabilises legged locomotion, and it
    // was missing. A running machine or animal does not stay upright by
    // stiffening its legs; it stays upright by putting the next foot down
    // ahead of where its momentum is carrying it, further ahead the faster
    // it goes. Raibert's rule, and the reason the first hopping robots
    // worked. In this model ThC sets fore-aft foot position directly, so the
    // whole of it is one term: shift the swing target by k times forward
    // velocity.
    float stepGain = 0.0f;

    // Cancel the foot lift that the stepping joint itself produces.
    //
    // ThC swings the leg, and at the rest pose the foot's height depends
    // on ThC to first order -- dz/dThC is -0.49 mm/rad on the front legs
    // and +0.90 on the hind, opposite in sign. A 0.4 rad step therefore
    // drives the front feet 0.2 mm down while the hind feet go 0.36 mm up,
    // sixteen times a second. That is not a gait with a bit of bounce on
    // it; it is a pitch oscillator that happens to also move forwards, and
    // it is what the body was visibly doing.
    //
    // 1.0 applies the full first-order cancellation, 0 leaves the old
    // behaviour for comparison.
    // 0.9 rather than 1.0. Cancelling the lift exactly was measured worse
    // than cancelling nine tenths of it -- 8 of 9 trials upright against 9 of
    // 9 -- and the exact table is within 6% of the linear estimate, so this
    // is not the linearisation running out. A foot held at precisely constant
    // height has no vertical give at all, and the last tenth of the
    // correction is what lets a stance leg absorb a bad step instead of
    // levering against it.
    float levelGain = 0.9f;

    // Plan the gait as a foot path rather than as a set of joint
    // sinusoids. The joint version needs a per-leg correction for the
    // lift ThC introduces, and even corrected it leaves the front legs
    // stepping the opposite way to the other four. Planning in foot
    // space makes both of those the inverse kinematics' problem.
    bool planned = true;
    // Fore-aft foot travel either side of rest, mm. 0.24 reproduces the
    // 0.475 mm stride the joint-space gait reached.
    float strideMm = 0.24f;
    float liftMm = 0.09f;
    float duty = 0.55f;

    // Load sharing between the stance legs.
    //
    // Six rigid legs holding up a rigid body is statically
    // indeterminate: nothing in the geometry decides how the weight
    // divides, so whichever feet are commanded lowest take all of it.
    // With the feet planned to a fixed height that is exactly what
    // happened -- the hind pair carried 95% of the animal and the front
    // legs touched the floor while bearing one per cent each. The
    // joint-space gait hid this by bouncing, which spread the load by
    // accident.
    //
    // A fly does not leave it to chance. Campaniform sensilla report leg
    // load continuously and the nervous system uses them to distribute
    // weight. This is that, as a controller: a stance leg carrying more
    // than its share lifts its foot slightly, one carrying less drops
    // it. Millimetres per second of trim per unit of relative load
    // error.
    float shareGain = 0.4f;
    // How far the trim may travel, millimetres. Enough to even out the
    // difference between feet, far less than a step.
    float shareClamp = 0.04f;

    // Drive the joints as forces through the muscle model, rather than as
    // position targets held by the posture servo.
    //
    // This is the difference between an actuator and a servo, and finding 19
    // says it is the whole ceiling. A position servo can only add energy: its
    // target jumps, it shoves, and at gait frequencies that pumps the body
    // into the air. A muscle produces force, and the Hill force-velocity
    // relation makes that force fall as the muscle shortens quickly -- which
    // dissipates. The Hill model has been in FlyPhysics for three sessions
    // and the walking path has been bypassing it.
    bool forceMode = false;

    // Baseline muscle activation, added to every joint's drive in force mode.
    //
    // This is what findings 3 said was missing and it has been missing ever
    // since: "a controller needs a baseline output to modulate, and ours is
    // zero". A real fly holds its posture with the tonic firing of its slow
    // motor neurons. This model holds it with postureTorque, an engineering
    // servo standing in for those neurons, and as long as that servo is
    // stiffer than the muscles it is the servo doing the walking.
    //
    // With a tonic term the muscles can carry the body themselves and
    // postureTorque can come down, which is the only way the force path
    // becomes more than a weaker servo.
    float tonic = 0.0f;

    // Seconds of walking per trial. Two was enough to compare settings and
    // is not enough to claim the fly walks: a gait can look fine for two
    // seconds and fall over in the third.
    float seconds = 2.0f;

    int trials = 5;
    // Starting-height offset for a single verbose trial, so a particular
    // failing trajectory can be reproduced and watched.
    float jitter = 0.0f;
};

// How much the foot rises when the stepping joint swings.
//
// ThC is the joint that takes the step, and the gait drives it with a
// sinusoid. If the foot moved purely fore and aft that would be all it
// did. It does not: at the rest pose every leg sits well away from ThC
// zero -- the front legs at -1.237 rad -- and away from zero the foot's
// height depends on ThC to *first order*, so half the step comes out as
// lift. The measured hop scales linearly with swing amplitude, which is
// the signature of exactly this and not of the leg vaulting over its own
// foot, which would be quadratic.
//
// The correction is one number per leg: the CTr offset that cancels the
// height the ThC offset just introduced, read off the forward kinematics
// rather than guessed. It is a linearisation, good while the swing is
// small, and it is what a Cartesian foot trajectory would do properly.
// Samples of the exact correction, spanning +/- kLevelSpan radians of ThC.
// The first-order coefficient is right at the rest pose and drifts away from
// it; over a 0.4 rad swing that drift is a third of the correction. Solving
// the height exactly at each of these and interpolating costs nothing at run
// time, because it is all done once before the trial starts.
constexpr int kLevelSamples = 65;
constexpr float kLevelSpan = 1.0f;

struct HeightComp {
    float dzdThC[kLegCount] = {};
    float dzdCTr[kLegCount] = {};
    float ctrPerThC[kLegCount] = {};
    float table[kLegCount][kLevelSamples] = {};
};

// The CTr offset that puts the foot back at its rest height, given a ThC
// offset. Bisection on CTr: foot height is monotone in CTr over the range a
// leg actually uses, and a bracket that fails to straddle simply returns the
// linear estimate.
float solveCtrForHeight(FlyBody& b, LegId id, float restZ, float thcRest,
                        float ctrRest, float thcOff, float linear) {
    b.setAngle(id, Joint::ThC, thcRest + thcOff);
    auto heightAt = [&](float ctrOff) {
        b.setAngle(id, Joint::CTr, ctrRest + ctrOff);
        return b.footPosition(id).z - restZ;
    };
    float lo = linear - 1.0f, hi = linear + 1.0f;
    float flo = heightAt(lo), fhi = heightAt(hi);
    float result = linear;
    if (flo * fhi < 0.0f) {
        for (int it = 0; it < 40; ++it) {
            const float mid = 0.5f * (lo + hi);
            const float fm = heightAt(mid);
            if (flo * fm <= 0.0f) { hi = mid; fhi = fm; }
            else { lo = mid; flo = fm; }
        }
        result = 0.5f * (lo + hi);
    }
    b.setAngle(id, Joint::CTr, ctrRest);
    b.setAngle(id, Joint::ThC, thcRest);
    return result;
}

HeightComp solveHeightComp() {
    FlyBody b;
    HeightComp hc;
    const float eps = 0.01f;
    for (int l = 0; l < kLegCount; ++l) {
        const LegId id = static_cast<LegId>(l);
        const float thc0 = b.angle(id, Joint::ThC);
        const float ctr0 = b.angle(id, Joint::CTr);
        b.setAngle(id, Joint::ThC, thc0 + eps);
        const float zp = b.footPosition(id).z;
        b.setAngle(id, Joint::ThC, thc0 - eps);
        const float zm = b.footPosition(id).z;
        b.setAngle(id, Joint::ThC, thc0);
        b.setAngle(id, Joint::CTr, ctr0 + eps);
        const float zp2 = b.footPosition(id).z;
        b.setAngle(id, Joint::CTr, ctr0 - eps);
        const float zm2 = b.footPosition(id).z;
        b.setAngle(id, Joint::CTr, ctr0);
        hc.dzdThC[l] = (zp - zm) / (2.0f * eps);
        hc.dzdCTr[l] = (zp2 - zm2) / (2.0f * eps);
        hc.ctrPerThC[l] = (std::fabs(hc.dzdCTr[l]) > 1e-6f)
                              ? -hc.dzdThC[l] / hc.dzdCTr[l]
                              : 0.0f;

        const float restZ = b.footPosition(id).z;
        for (int k = 0; k < kLevelSamples; ++k) {
            const float thcOff =
                kLevelSpan * (2.0f * k / (kLevelSamples - 1) - 1.0f);
            hc.table[l][k] = solveCtrForHeight(b, id, restZ, thc0, ctr0,
                                               thcOff,
                                               hc.ctrPerThC[l] * thcOff);
        }
    }
    return hc;
}

// Interpolate the exact correction, falling back to the linear coefficient
// outside the tabulated range rather than clamping, so a large foot-placement
// shift degrades smoothly instead of hitting a wall.
float levelOffset(const HeightComp& hc, int leg, float thcOff) {
    if (thcOff <= -kLevelSpan || thcOff >= kLevelSpan) {
        return hc.ctrPerThC[leg] * thcOff;
    }
    const float u = (thcOff + kLevelSpan) / (2.0f * kLevelSpan) *
                    (kLevelSamples - 1);
    const int i0 = static_cast<int>(u);
    const int i1 = std::min(i0 + 1, kLevelSamples - 1);
    const float f = u - i0;
    return hc.table[leg][i0] * (1.0f - f) + hc.table[leg][i1] * f;
}

int jacobianReport() {
    const HeightComp hc = solveHeightComp();

    // What each joint actually does to the foot, in millimetres per radian,
    // at the pose the animal stands in. Written down because every joint in
    // this model has been described by the name of the muscle that drives it
    // -- ThC "takes the step", CTr "lifts" -- and those descriptions turn out
    // to be claims about a neutral pose the fly is nowhere near.
    {
        FlyBody fb;
        std::printf("=== foot motion per joint, mm/rad ===\n");
        std::printf("%-10s %-6s %8s %8s %8s %8s\n", "leg", "joint",
                    "dx", "dy", "dz", "|d|");
        const float eps = 0.01f;
        for (int l = 0; l < kLegCount; l += 2) {
            const LegId id = static_cast<LegId>(l);
            for (int j = 0; j < kJointCount; ++j) {
                const Joint jt = static_cast<Joint>(j);
                const float a0 = fb.angle(id, jt);
                fb.setAngle(id, jt, a0 + eps);
                const V3 pp = fb.footPosition(id);
                fb.setAngle(id, jt, a0 - eps);
                const V3 pm = fb.footPosition(id);
                fb.setAngle(id, jt, a0);
                const V3 d = (pp - pm) * (1.0f / (2.0f * eps));
                std::printf("%-10s %-6s %8.3f %8.3f %8.3f %8.3f\n",
                            legName(id), jointName(jt), d.x, d.y, d.z,
                            std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z));
            }
        }
        std::printf("\n");
    }

    std::printf("=== foot height sensitivity at the rest pose ===\n");
    std::printf("%-10s %12s %12s %12s %10s %10s\n", "leg", "dz/dThC", "dz/dCTr", "CTr per ThC", "exact-0.4", "exact+0.4");
    for (int l = 0; l < kLegCount; ++l) {
        std::printf("%-10s %12.4f %12.4f %12.4f %10.4f %10.4f\n",
                    legName(static_cast<LegId>(l)), hc.dzdThC[l],
                    hc.dzdCTr[l], hc.ctrPerThC[l],
                    levelOffset(hc, l, -0.4f), levelOffset(hc, l, 0.4f));
    }
    return 0;
}
GaitResult gaitTrial(const GaitParams& gp, float jitter, bool verbose);

// Run the same gait several times from slightly different starting states and
// report the spread.
//
// A single two-second run is not a property of the gait. Three separate
// changes that altered no physics at all -- caching the world inertia,
// splitting solveJoints into passes, and re-solving the rest pose -- each
// moved the headline walking figure by a factor of two or more, because the
// system is chaotically sensitive and one trajectory is one sample. Reporting
// a median across perturbed starts gives a number that survives a recompile.
int gaitTest(const GaitParams& gp) {
    const int trials = gp.trials;
    if (trials <= 1) {
        const GaitResult r = gaitTrial(gp, gp.jitter, true);
        return r.upright && std::fabs(r.speed) > 0.5f ? 0 : 2;
    }

    std::printf("=== gait: imposed tripod, NOT driven by the connectome ===\n");
    std::printf("period %.0f ms (%.1f Hz), swing %.2f, lift %.2f, toe %.2f\n",
                gp.periodMs, 1000.0f / gp.periodMs, gp.swing, gp.lift, gp.toe);
    std::printf("pgain %.2f, prate %.4f, step %.4f\n",
                gp.postureGain, gp.postureRate, gp.stepGain);
    std::printf("%d trials from perturbed starts\n\n", trials);
    std::printf("%8s %10s %10s %8s %8s\n",
                "jitter", "speed", "stride", "duty", "pitch");

    std::vector<float> speeds, pitches;
    int upright = 0;
    float peakRate = 0.0f;
    for (int k = 0; k < trials; ++k) {
        // Spread the perturbation either side of the nominal start.
        const float jit = 0.004f * (static_cast<float>(k) -
                                    0.5f * static_cast<float>(trials - 1));
        const GaitResult r = gaitTrial(gp, jit, false);
        speeds.push_back(r.speed);
        pitches.push_back(r.pitch);
        if (r.upright) ++upright;
        peakRate = std::max(peakRate, r.peakRate);
        std::printf("%+8.3f %10.2f %10.3f %8.2f %8.1f\n",
                    jit, r.speed, r.stride, r.duty, r.pitch);
    }
    std::sort(speeds.begin(), speeds.end());
    std::sort(pitches.begin(), pitches.end());
    const float medS = speeds[speeds.size() / 2];
    const float medP = pitches[pitches.size() / 2];
    std::printf("\nmedian speed %.2f mm/s (range %.2f to %.2f), "
                "median pitch %.1f deg (range %.1f to %.1f)\n",
                medS, speeds.front(), speeds.back(),
                medP, pitches.front(), pitches.back());
    std::printf("upright in %d of %d trials\n", upright, trials);
    // Report the fraction rather than a verdict that rounds a bare majority
    // up to "it walks". Five of nine is not the same claim as nine of nine.
    const bool ok = upright * 2 >= trials && std::fabs(medS) > 0.5f;
    std::printf("  -> %s (%d of %d upright)\n",
                ok ? "walks in a majority of trials" : "not a repeatable gait",
                upright, trials);
    return (upright * 2 >= trials && std::fabs(medS) > 0.5f) ? 0 : 2;
}

GaitResult gaitTrial(const GaitParams& gp, float jitter, bool verbose) {
    const float periodMs = gp.periodMs;
    const float amplitude = gp.swing;
    const float lift = gp.lift;
    const float toeLift = gp.toe;
    const float postureGain = gp.postureGain;
    const float postureRate = gp.postureRate;
    const float stepGain = gp.stepGain;
    FlyBody skeleton;
    FlyPhysics phys;
    if (g_stiffness > 0.0f) phys.params.postureTorque = g_stiffness;
    if (g_substep > 0.0f) phys.params.substepHz = g_substep;
    if (g_tarsus > 0.0f) phys.params.tarsusStiffness = g_tarsus;
    if (g_minseg > 0.0f) phys.params.minSegmentMass = g_minseg;
    if (g_mintar > 0.0f) phys.params.minTarsomereMass = g_mintar;
    if (g_upper >= 0) phys.params.upperLegProbes = (g_upper != 0);
    if (g_hill >= 0) phys.params.hillMuscle = (g_hill != 0);
    if (g_hillVmax > 0.0f) phys.params.hillShorteningRate = g_hillVmax;
    phys.params.useManualTarget = !gp.forceMode;
    phys.params.useManualDrive = gp.forceMode;
    // Displace the starting height very slightly. Enough that the trajectory
    // differs, far too little to change what the gait is being asked to do.
    skeleton.root.position.z += jitter;
    phys.build(skeleton);
    if (g_iters > 0) phys.world.params.iterations = g_iters;
    if (g_friction > 0.0f) phys.world.params.friction = g_friction;
    if (g_inter >= 0) phys.world.params.interleave = (g_inter != 0);

    const HeightComp hc = solveHeightComp();

    GaitPlan plan;
    {
        GaitSpec spec;
        spec.periodMs = gp.periodMs;
        spec.strideMm = gp.strideMm;
        spec.liftMm = gp.liftMm;
        spec.duty = gp.duty;
        plan.build(spec);
    }

    // Tripod A = front_L, middle_R, hind_L; tripod B is the other three.
    const bool tripodA[kLegCount] = {true, false, false, true, true, false};

    const float startX = phys.thorax().position.x;
    const float startZ = phys.bodyHeight();
    if (verbose) {
        std::printf("=== gait: imposed tripod, NOT driven by the connectome ===\n");
        std::printf("period %.0f ms (%.1f Hz), ThC swing %.2f rad, "
                    "CTr lift %.2f, toe %.2f\n\n",
                    periodMs, 1000.0f / periodMs, amplitude, lift, toeLift);
        std::printf("%8s %10s %10s %9s %9s\n",
                    "t (ms)", "x (mm)", "height", "contacts", "pitch");
    }

    const int n = static_cast<int>(gp.seconds / kDt);
    // Per-leg vertical trim, millimetres, carried between frames.
    float zTrim[kLegCount] = {};
    float worstPitch = 0.0f;
    float contactSum = 0.0f;
    // Fore-aft excursion of one foot relative to the body. This is the
    // geometry's answer to "how long can a stride be", independent of
    // whether the foot grips: a leg that only sweeps 0.3 mm relative to the
    // thorax cannot move the body further than that per step however well it
    // holds the ground.
    float footRelMin = 1e9f, footRelMax = -1e9f;
    // How much of that sweep happens while the foot is actually loaded.
    float stanceRelMin = 1e9f, stanceRelMax = -1e9f;

    // Where the load sits fore and aft, against where the weight sits.
    //
    // A body pushed along by feet below its centre of mass takes a nose-up
    // moment, and the only thing that cancels it is the support moving
    // rearward so gravity pulls the nose back down. If the impulse-weighted
    // centre of pressure sits *behind* the centre of mass instead, gravity
    // adds to the nose-up moment rather than opposing it and the attitude has
    // no equilibrium to return to. That is a geometry fault, not a gain
    // fault, and no amount of postural feedback fixes it -- which is what the
    // pgain sweep found.
    constexpr int kBuckets = 12;
    double copNum[kBuckets] = {}, copDen[kBuckets] = {};
    double pitchSum[kBuckets] = {};
    // Hop amplitude and vertical speed. A position servo driven at gait
    // frequency adds energy every time its target jumps, and the signature
    // is a vertical oscillation that grows bucket on bucket until the body
    // leaves the floor. A trip, by contrast, shows no growth and then one
    // bad bucket.
    double hMin[kBuckets], hMax[kBuckets], vzSum[kBuckets] = {};

    double loadPerLeg[kBuckets][kLegCount] = {};
    int bucketN[kBuckets] = {};
    for (int k = 0; k < kBuckets; ++k) { hMin[k] = 1e9; hMax[k] = -1e9; }
    for (int i = 0; i < n; ++i) {
        const float t = i * kDt * 1000.0f;
        const float phase = 6.2831853f * t / periodMs;
        if (gp.planned && gp.shareGain > 0.0f) {
            float load[kLegCount] = {};
            float total = 0.0f;
            int stanceCount = 0;
            bool inStance[kLegCount] = {};
            for (int l = 0; l < kLegCount; ++l) {
                const float pl = tripodA[l] ? phase : phase + 3.14159265f;
                float u = pl / 6.2831853f;
                u -= std::floor(u);
                inStance[l] = u < gp.duty;
                load[l] = phys.footLoad(l);
                if (inStance[l]) {
                    total += load[l];
                    ++stanceCount;
                }
            }
            if (stanceCount > 0 && total > 1e-9f) {
                const float want = total / static_cast<float>(stanceCount);
                for (int l = 0; l < kLegCount; ++l) {
                    if (!inStance[l]) continue;
                    zTrim[l] += gp.shareGain * ((load[l] - want) / want) * kDt;
                    zTrim[l] = std::clamp(zTrim[l], -gp.shareClamp,
                                          gp.shareClamp);
                }
            }
        }
        for (int l = 0; l < kLegCount; ++l) {
            const float p = tripodA[l] ? phase : phase + 3.14159265f;
            const float s = std::sin(p);
            const float c = std::cos(p);
            // Retract through stance, protract through swing. `amplitude` is
            // now the ThC swing in radians -- a real fly's coxa swings on the
            // order of 0.3 rad -- rather than an abstract drive number.
            // Foot placement. Shift the whole swing target by how fast the
            // body is travelling, so a fly carrying forward momentum puts its
            // feet down further forward and catches itself, instead of
            // stepping under a body that has already moved on.
            //
            // ThC positive is retraction, so forward velocity has to move the
            // target negative to place the foot ahead of the body -- hence the
            // minus. Backwards, this would accelerate the fall.
            // Only the swing leg. Foot placement is a rule about where the
            // next foot lands, and a planted foot cannot be placed anywhere
            // -- shifting its target just drags the body along the ground.
            // Applied to every leg at every phase it made things steadily
            // worse: 2 of 5 trials upright at a gain of 0, 1 of 5 at 0.005,
            // none at all above that.
            const float vx = phys.thorax().velocity.x;
            const float place = (c > 0.0f) ? stepGain * vx : 0.0f;
            float* out = gp.forceMode ? phys.params.manualDrive[l]
                                      : phys.params.manualTarget[l];
            // Tonic baseline on the joints that carry weight, in the
            // direction that extends the leg against the ground. Signs from
            // flyphys test 2 at the usable probe drive: CTr -1.5 lifts, and
            // FTi -1.5 sinks, so FTi's extending direction is positive.
            //
            // Every target is written fresh each frame. manualDrive persists
            // between frames -- it lives in params, not in a per-step buffer
            // -- so accumulating into it instead of assigning made the drive
            // grow without bound and flung the fly at 130 mm/s.
            const float tonicCTr = gp.forceMode ? -gp.tonic : 0.0f;
            const float tonicFTi = gp.forceMode ? gp.tonic : 0.0f;
            if (gp.planned) {
                // The whole leg comes from the plan.
                float q[kJointCount];
                plan.targets(l, p / 6.2831853f, q);
                for (int j = 0; j < kJointCount; ++j) out[j] = q[j];
                const float* up = plan.basis(l, 2);
                for (int j = 0; j < kJointCount; ++j) out[j] += up[j] * zTrim[l];
                out[static_cast<int>(Joint::FTi)] += tonicFTi;
                out[static_cast<int>(Joint::CTr)] += tonicCTr;
            } else {
            out[static_cast<int>(Joint::FTi)] = tonicFTi;
            const float thcOff = -amplitude * s - place;
            out[static_cast<int>(Joint::ThC)] = thcOff;
            // Hold the foot at the height the rest pose put it at, whatever
            // ThC is doing. Applied in swing as well as stance: it is a
            // correction to where the leg *would* be, so the deliberate swing
            // arc adds on top of a level baseline rather than on top of a
            // sawtooth.
            const float level = gp.levelGain * levelOffset(hc, l, thcOff);
            // Lift only during swing, which is the half where the leg is
            // protracting. A leg that lifts during stance just drops the body.
            //
            // Positive CTr folds the leg and lifts the foot. Negative extends
            // it downward, which is the direction that raises the *body* --
            // flyphys test 2 reports CTr -15 as LIFTS, and taking that to mean
            // "lift the foot" drove every swing leg into the floor, so all six
            // feet stayed planted and the fly shuffled backwards.
            out[static_cast<int>(Joint::CTr)] =
                tonicCTr + level + ((c > 0.0f) ? lift * c : 0.0f);
            // Curl the tarsus during swing.
            //
            // A point foot can be planted and lifted straight up. A tarsus
            // lying flat along the ground cannot: lifting it at the ankle
            // alone leaves the far end dragging, which is the toe catching on
            // every step. A real fly rolls the foot, and the tarsus has to
            // come up with the leg.
            out[static_cast<int>(Joint::TiTa)] = (c > 0.0f) ? toeLift * c : 0.0f;
            }

            // Postural feedback, and it is a hand-built controller: no
            // neuron is involved and it must not be reported as the nervous
            // system stabilising anything.
            //
            // It is here to answer the question the open-loop sweep left
            // hanging. Every gait setting that produced a biological stride
            // fell over, and every setting that stayed upright produced a
            // short one, which says the fly is not short of stride but short
            // of correction. A real fly is correcting continuously -- through
            // campaniform sensilla reporting leg load and halteres reporting
            // body rotation -- and the faster it walks the more it needs to.
            //
            // The rule is the simplest thing that could work: if the body is
            // pitching nose-up, extend the front legs less and the hind legs
            // more, and the reverse nose-down. Roll does the same across left
            // and right.
            if (postureGain > 0.0f || postureRate > 0.0f) {
                const V3 fwd = phys.thorax().orientation.rotate({1, 0, 0});
                const V3 lat = phys.thorax().orientation.rotate({0, 1, 0});
                const float pitchErr = fwd.z;   // + is nose-up
                const float rollErr = lat.z;    // + is left side up
                // Front legs are +1, hind legs -1; left legs +1, right -1.
                const float fore = (l < 2) ? +1.0f : (l < 4 ? 0.0f : -1.0f);
                const float side = (l % 2 == 0) ? +1.0f : -1.0f;
                // Rate term, and it is the biologically correct signal.
                //
                // A haltere is a gyroscope: it reports the body's angular
                // *velocity*, not its angle. Proportional correction on angle
                // alone stabilised pitch from 87 to 19 degrees and cost most
                // of the speed, which is what a P controller with no damping
                // does -- it fights the error after the error exists.
                const V3 w = phys.thorax().angularVelocity;
                const float pitchRate = -w.y;   // + is pitching nose-up
                const float rollRate = w.x;     // + is left side rising
                const float corr =
                    postureGain * (pitchErr * fore + rollErr * side) +
                    postureRate * (pitchRate * fore + rollRate * side);
                // Note the sign. CTr negative extends the leg and raises the
                // body at that corner (flyphys test 2: CTr -15 LIFTS), so
                // correcting a nose-up pitch means making the front legs'
                // CTr *more positive*. Getting this backwards drove the fly
                // backwards at -48 mm/s with a duty factor of 0.13.
                out[static_cast<int>(Joint::CTr)] += corr;
            }
        }
        phys.step(kDt);
        const V3 fwd = phys.thorax().orientation.rotate({1, 0, 0});
        const float pitch = std::asin(std::clamp(fwd.z, -1.0f, 1.0f)) * 57.2958f;
        worstPitch = std::max(worstPitch, std::fabs(pitch));
        {
            bool legDown[kLegCount] = {};
            for (const auto& c : phys.world.contacts) {
                const int l = phys.probeLeg(c.probe);
                if (l >= 0) legDown[l] = true;
            }
            for (const bool d : legDown) if (d) contactSum += 1.0f;
        }
        {
            const int bkt = std::min(kBuckets - 1, i * kBuckets / n);
            const V3 com = phys.centreOfMass();
            for (const auto& c : phys.world.contacts) {
                const int l = phys.probeLeg(c.probe);
                if (l < 0 || c.normalImpulse <= 0.0f) continue;
                const RigidBody& b = phys.world.bodies[c.body];
                const V3 w = b.position + b.orientation.rotate(c.localPoint);
                copNum[bkt] += static_cast<double>(w.x - com.x) * c.normalImpulse;
                copDen[bkt] += c.normalImpulse;
                loadPerLeg[bkt][l] += c.normalImpulse;
            }
            pitchSum[bkt] += pitch;
            const float h = phys.bodyHeight();
            hMin[bkt] = std::min(hMin[bkt], static_cast<double>(h));
            hMax[bkt] = std::max(hMax[bkt], static_cast<double>(h));
            vzSum[bkt] += std::fabs(phys.thorax().velocity.z);
            ++bucketN[bkt];
        }
        {
            const int probe = static_cast<int>(LegId::MiddleL);
            const V3 f = phys.footPosition(probe);
            const float rel = f.x - phys.thorax().position.x;
            footRelMin = std::min(footRelMin, rel);
            footRelMax = std::max(footRelMax, rel);
            if (phys.footLoad(probe) > 0.0f) {
                stanceRelMin = std::min(stanceRelMin, rel);
                stanceRelMax = std::max(stanceRelMax, rel);
            }
        }
        if (!std::isfinite(phys.bodyHeight())) {
            if (verbose) std::printf("DIVERGED at %.0f ms\n", t);
            return {};
        }
        const bool inWindow = g_from >= 0.0f && t >= g_from * 1000.0f &&
                              t <= g_to * 1000.0f;
        if (verbose && inWindow) {
            bool legDown[kLegCount] = {};
            for (const auto& c : phys.world.contacts) {
                const int l = phys.probeLeg(c.probe);
                if (l >= 0) legDown[l] = true;
            }
            int down = 0;
            for (const bool d : legDown) if (d) ++down;
            int thx = 0, abd = 0, hd = 0;
            for (const auto& c : phys.world.contacts) {
                if (c.probe == phys.thoraxProbe()) ++thx;
                else if (c.probe == phys.abdomenProbe()) ++abd;
                else if (c.probe == phys.headProbe()) ++hd;
            }
            std::printf("W %8.1f h %7.4f pitch %7.2f vz %8.2f vx %8.2f legs %d anchor %7.4f  thx %d abd %d head %d\n",
                        t, phys.bodyHeight(), pitch,
                        phys.thorax().velocity.z, phys.thorax().velocity.x,
                        down, phys.world.maxAnchorError(),
                        thx, abd, hd);
        }
        if (verbose && !inWindow && (i % (n / 10) == 0 || i == n - 1)) {
            std::printf("%8.0f %10.4f %10.4f %9zu %9.2f\n",
                        t, phys.thorax().position.x - startX, phys.bodyHeight(),
                        phys.world.contacts.size(), pitch);
        }
    }

    const float travel = phys.thorax().position.x - startX;
    const float held = phys.bodyHeight();

    // Speed is stride length times step frequency, and the two say very
    // different things about what is limiting it. A fly that steps quickly
    // but travels little per step is scuffing, not walking.
    const float stepHz = 1000.0f / periodMs;
    const float strideMm = (travel / gp.seconds) / stepHz;
    // Duty factor: the share of the cycle a leg spends on the ground. A real
    // fly walking fast is near 0.5; a fly with every foot down all the time
    // is dragging.
    const float meanFeet = contactSum / static_cast<float>(n);
    if (verbose) {
        std::printf("\n%6s %8s %9s %8s %8s   %s\n", "t (s)", "pitch",
                    "CoP-CoM", "hop mm", "|vz|", "share of load: fL fR mL mR hL hR");
        for (int k = 0; k < kBuckets; ++k) {
            if (bucketN[k] == 0) continue;
            const double tMid = gp.seconds * (k + 0.5) / kBuckets;
            const double cop = (copDen[k] > 0.0) ? copNum[k] / copDen[k] : 0.0;
            double tot = 0.0;
            for (int l = 0; l < kLegCount; ++l) tot += loadPerLeg[k][l];
            std::printf("%6.2f %8.2f %9.4f %8.4f %8.2f  ", tMid,
                        pitchSum[k] / bucketN[k], cop, hMax[k] - hMin[k],
                        vzSum[k] / bucketN[k]);
            for (int l = 0; l < kLegCount; ++l) {
                std::printf(" %4.2f", (tot > 0.0) ? loadPerLeg[k][l] / tot : 0.0);
            }
            std::printf("\n");
        }
        std::printf("\n");
    }
    if (verbose) std::printf("foot sweep %.3f mm relative to body, of which "
                "%.3f mm loaded\n", footRelMax - footRelMin,
                (stanceRelMax > stanceRelMin) ? stanceRelMax - stanceRelMin : 0.0f);
    if (verbose) std::printf("stride %.3f mm at %.1f Hz, mean feet down %.2f of 6 "
                "(duty %.2f)\n", strideMm, stepHz, meanFeet, meanFeet / 6.0f);
    if (verbose) std::printf("\ntravelled %+.3f mm in 2.0 s (%.2f mm/s), "
                "height %.3f -> %.3f, worst pitch %.1f deg\n",
                travel, travel / 2.0f, startZ, held, worstPitch);
    // A walking fly does a few body lengths a second; 1 mm/s is slow but it is
    // locomotion rather than twitching in place.
    const bool moved = std::fabs(travel) > 1.0f;
    const bool upright = held > 0.40f && worstPitch < 25.0f;
    if (verbose) {
        // Deliberately not "the body can walk". One trial is one sample of a
        // chaotically sensitive system, and reading a single run as a
        // capability is exactly the mistake this project made twice.
        std::printf("  -> %s (one trial only -- run several)\n",
            (moved && upright) ? "travelled and stayed upright"
            : (!moved ? "no net travel" : "travelled but did not stay upright"));
    }

    GaitResult out;
    out.speed = travel / gp.seconds;
    out.stride = strideMm;
    out.duty = meanFeet / 6.0f;
    out.pitch = worstPitch;
    out.height = held;
    out.upright = moved && upright;
    out.peakRate = phys.peakJointRate();
    return out;
}

// Per-segment report for one leg: where each piece sits, how big a gap there
// is to the next one, and whether it is touching the floor.
//
// Built because the fly was visibly coming apart on screen while every number
// in this harness said it was fine. The worst anchor error, 0.0616 mm, renders
// at roughly 370 px/mm as a 23-pixel break between tibia and tarsus -- plainly
// visible, and dismissed for two sessions as a small number.
int footReport(int leg) {
    FlyBody skeleton;
    FlyPhysics phys;
    if (g_tarsus > 0.0f) phys.params.tarsusStiffness = g_tarsus;
    phys.build(skeleton);
    for (int i = 0; i < 300; ++i) phys.step(kDt);

    std::vector<FlyBody::SegmentPose> pose;
    phys.readPose(pose);

    std::printf("=== leg %s after settling ===\n",
                legName(static_cast<LegId>(leg)));
    std::printf("%-10s %9s %9s %9s %9s  %s\n",
                "segment", "prox z", "dist z", "radius", "gap um", "contact");

    // The leg's own five segments, then its tarsomeres, which live past the
    // first kLegCount*kJointCount entries of segments_.
    std::vector<std::size_t> idx;
    for (int j = 0; j < kJointCount; ++j) {
        idx.push_back(static_cast<std::size_t>(leg) * kJointCount + j);
    }
    const std::size_t tarsBase =
        static_cast<std::size_t>(kLegCount) * kJointCount +
        static_cast<std::size_t>(leg) * (anat::kTarsomereCount - 1);
    for (int k = 0; k < anat::kTarsomereCount - 1; ++k) {
        idx.push_back(tarsBase + static_cast<std::size_t>(k));
    }

    static const char* names[] = {"coxa", "troch", "femur", "tibia", "ta1",
                                  "ta2", "ta3", "ta4", "ta5"};
    for (std::size_t n = 0; n < idx.size(); ++n) {
        if (idx[n] >= pose.size()) continue;
        const auto& s = pose[idx[n]];
        float gap = 0.0f;
        if (n + 1 < idx.size() && idx[n + 1] < pose.size()) {
            gap = length(pose[idx[n + 1]].a - s.b) * 1000.0f;
        }
        const bool touching = s.b.z <= s.radius + 0.002f;
        std::printf("%-10s %9.4f %9.4f %9.4f %9.1f  %s\n",
                    names[n], s.a.z, s.b.z, s.radius, gap,
                    touching ? "yes" : "");
    }
    std::printf("\nworst anchor error over all joints: %.4f mm at joint %zu\n",
                phys.world.maxAnchorError(), phys.world.worstAnchorJoint());
    return 0;
}

int main(int argc, char** argv) {
    for (int i = 1; i < argc - 1; ++i) {
        if (std::strcmp(argv[i], "--posture") == 0) {
            g_stiffness = std::stof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--servo") == 0) {
            g_servo = std::stof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--abdomen-mass") == 0) {
            g_abdMass = std::stof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--abdomen-x") == 0) {
            g_abdX = std::stof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--iters") == 0) {
            g_iters = std::atoi(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--baumgarte") == 0) {
            g_baum = std::stof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--ms") == 0) {
            g_ms = std::stof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--corr") == 0) {
            g_corr = std::stof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--substep") == 0) {
            g_substep = std::stof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--tarsus") == 0) {
            g_tarsus = std::stof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--minseg") == 0) {
            g_minseg = std::stof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--friction") == 0) {
            g_friction = std::stof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--no-upper") == 0) {
            g_upper = 0;
        }
        if (std::strcmp(argv[i], "--mintar") == 0) {
            g_mintar = std::stof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--interleave") == 0) {
            g_inter = std::atoi(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--probe-drive") == 0) {
            g_probeDrive = std::stof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--from") == 0) {
            g_from = std::stof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "--to") == 0) {
            g_to = std::stof(argv[i + 1]);
        }
    }
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--stand-only") == 0) g_standOnly = true;
    }
    if (argc >= 3 && std::strcmp(argv[1], "--trace") == 0) {
        const float drive = (argc >= 4) ? std::stof(argv[3]) : 15.0f;
        return traceOne(argv[2], drive);
    }
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--jacobian") == 0) {
            return jacobianReport();
        }
        if (std::strcmp(argv[i], "--foot") == 0) {
            return footReport((i + 1 < argc) ? std::atoi(argv[i + 1]) : 2);
        }
        if (std::strcmp(argv[i], "--gait") == 0) {
            // Named rather than positional. Seven positional arguments was
            // already one too many to remember, and the controller search
            // adds more.
            GaitParams gp;
            for (int k = 1; k < argc; ++k) {
                const char* a = argv[k];
                // Valueless flags must still be reachable in last position,
                // so this guards the value rather than the loop bound.
                const char* v = (k + 1 < argc) ? argv[k + 1] : "0";
                if (!std::strcmp(a, "--period")) gp.periodMs = std::stof(v);
                else if (!std::strcmp(a, "--swing")) gp.swing = std::stof(v);
                else if (!std::strcmp(a, "--lift")) gp.lift = std::stof(v);
                else if (!std::strcmp(a, "--toe")) gp.toe = std::stof(v);
                else if (!std::strcmp(a, "--pgain")) gp.postureGain = std::stof(v);
                else if (!std::strcmp(a, "--prate")) gp.postureRate = std::stof(v);
                else if (!std::strcmp(a, "--step")) gp.stepGain = std::stof(v);
                else if (!std::strcmp(a, "--level")) gp.levelGain = std::stof(v);
                else if (!std::strcmp(a, "--stride")) gp.strideMm = std::stof(v);
                else if (!std::strcmp(a, "--liftmm")) gp.liftMm = std::stof(v);
                else if (!std::strcmp(a, "--duty")) gp.duty = std::stof(v);
                else if (!std::strcmp(a, "--share")) gp.shareGain = std::stof(v);
                else if (!std::strcmp(a, "--share-clamp")) gp.shareClamp = std::stof(v);
                else if (!std::strcmp(a, "--joint-gait")) gp.planned = false;
                else if (!std::strcmp(a, "--trials")) gp.trials = std::atoi(v);
                else if (!std::strcmp(a, "--jitter")) gp.jitter = std::stof(v);
                else if (!std::strcmp(a, "--force")) gp.forceMode = true;
                else if (!std::strcmp(a, "--seconds")) gp.seconds = std::stof(v);
                else if (!std::strcmp(a, "--no-hill")) g_hill = 0;
                else if (!std::strcmp(a, "--vmax")) g_hillVmax = std::stof(v);
                else if (!std::strcmp(a, "--tonic")) gp.tonic = std::stof(v);
            }
            return gaitTest(gp);
        }
    }

    std::printf("=== 1. does the body stand on its own? ===\n");
    const Result base = run(g_ms, -1, 0.0f);
    std::printf("no drive: height %.4f mm, %zu contacts%s\n",
                base.height, base.contacts, base.diverged ? "  DIVERGED" : "");
    // What "working" means, as a test rather than an impression: the fly
    // carries its own weight on six feet, none of the trunk touches the floor,
    // it sits near the height its rest pose was solved for, and it is roughly
    // level.
    const bool onSixFeet = base.feetDown == kLegCount;
    const bool trunkClear = !base.thoraxDown && !base.abdomenDown && !base.headDown;
    const bool rideHeight = base.height > 0.52f;
    const bool level = std::fabs(base.pitchDeg) < 2.0f;
    const bool stands = !base.diverged && onSixFeet && trunkClear &&
                        rideHeight && level;

    std::printf("  contacts: %d feet%s%s%s\n", base.feetDown,
                base.thoraxDown ? " + THORAX" : "",
                base.abdomenDown ? " + ABDOMEN" : "",
                base.headDown ? " + HEAD" : "");
    std::printf("  max joint anchor error %.4f mm at joint %zu"
                "   front foot moved %.4f mm\n",
                base.anchorError, base.worstAnchor, base.footLateral);
    if (base.anchorError > 0.02f) {
        std::printf("  WARNING: the linkage is stretching, not just bending\n");
    }
    // Contact count alone no longer says anything about the trunk. With the
    // tarsus lying flat every leg contributes several contacts -- a healthy
    // standing fly has 22 -- so ask the trunk probes directly instead of
    // inferring from a total that used to be six.
    if (base.thoraxDown || base.abdomenDown || base.headDown) {
        std::printf("  WARNING: %zu contacts -- the trunk is on the ground\n",
                    base.contacts);
    }
    std::printf("  worst joint across all six legs: %s %s = %+.3f rad\n",
                legName(static_cast<LegId>(base.worstLeg)),
                jointName(static_cast<Joint>(base.worstJoint)), base.worstAngle);
    std::printf("  weld slip %.4f mm   body pitch %+.2f deg (nose up positive)\n",
                base.trunkDrop, base.pitchDeg);
    std::printf("  thorax-to-foot reach %.4f mm, rest %.4f mm, shortfall %+.4f mm\n",
                base.reach, base.reachRest, base.reach - base.reachRest);
    std::printf("  per-leg span change (mm): ");
    for (int l = 0; l < kLegCount; ++l) {
        std::printf("%s=%+.4f  ", legName(static_cast<LegId>(l)), base.legShort[l]);
    }
    std::printf("\n");
    std::printf("  front-left joint angles after settling:");
    for (int j = 0; j < kJointCount; ++j) {
        std::printf("  %s=%+.3f", jointName(static_cast<Joint>(j)), base.angle[j]);
    }
    std::printf("\n  -> %s\n", stands ? "PASS" : "FAIL");
    if (!stands) {
        if (!onSixFeet) std::printf("     only %d feet down\n", base.feetDown);
        if (!trunkClear) std::printf("     trunk is on the ground\n");
        if (!rideHeight) std::printf("     riding at %.3f mm, want > 0.520\n", base.height);
        if (!level) std::printf("     pitched %.2f deg, want within 2\n", base.pitchDeg);
    }
    std::printf("\n");

    // --stand-only was parsed and then never read, so it ran the whole suite
    // -- now 71 separate 300 ms simulations -- every time it was used as a
    // quick regression check.
    if (g_standOnly) return stands ? 0 : 2;

    std::printf("=== 2. does each joint respond to torque, and does it lift? ===\n");
    std::printf("baseline height %.4f, joint angles all start at 0\n\n", base.height);
    std::printf("%-6s %7s %10s %10s  %s\n",
                "joint", "drive", "angle", "height", "verdict");

    bool anyLifts = false;
    for (int j = 0; j < kJointCount; ++j) {
        for (const float d : {-g_probeDrive, g_probeDrive}) {
            const Result r = run(g_ms, j, d);
            const float moved = r.angle[j] - base.angle[j];
            const float lifted = r.height - base.height;
            const char* verdict;
            if (r.diverged) verdict = "DIVERGED";
            // A fly that left the ground is not telling us about lift. This
            // row is a flight path, and reading a sign off it is how
            // kJointDriveSign came to be set from a launch.
            else if (r.tookOff) verdict = "TOOK OFF -- unusable";
            else if (std::fabs(moved) < 0.01f) verdict = "joint did not move";
            else if (lifted > 0.05f) { verdict = "LIFTS"; anyLifts = true; }
            else if (lifted < -0.05f) verdict = "sinks";
            else verdict = "moves, no lift";
            std::printf("%-6s %+7.1f %+10.3f %+10.4f  %s\n",
                        jointName(static_cast<Joint>(j)), d, moved, lifted, verdict);
        }
    }

    // --- 3. per-leg breakdown -------------------------------------------
    //
    // Test 2 drives one joint on all six legs at once and reports a single
    // height. That hides disagreement: if three legs lift and three sink, the
    // average reads "moves, no lift" and the joint looks useless when it is in
    // fact strong and miswired. The rest pose is solved per leg by IK, and
    // nothing in that solve required the six legs to come out in the same
    // joint configuration, so this is a live possibility rather than a
    // theoretical one.
    std::printf("\n=== 3. does each leg agree about which way a joint lifts? ===\n");
    std::printf("height change per leg, driven one leg at a time\n\n");
    std::printf("%-6s %7s", "joint", "drive");
    for (int l = 0; l < kLegCount; ++l) {
        std::printf(" %9s", legName(static_cast<LegId>(l)));
    }
    std::printf("   verdict\n");

    for (int j = 0; j < kJointCount; ++j) {
        for (const float d : {-g_probeDrive, g_probeDrive}) {
            std::printf("%-6s %+7.1f", jointName(static_cast<Joint>(j)), d);
            int up = 0, down = 0;
            for (int l = 0; l < kLegCount; ++l) {
                const Result r = run(g_ms, j, d, l);
                const float lifted = r.height - base.height;
                if (lifted > 0.02f) ++up;
                else if (lifted < -0.02f) ++down;
                std::printf(" %+9.4f", lifted);
            }
            std::printf("   %s\n", (up && down) ? "LEGS DISAGREE"
                                 : up           ? "all lift"
                                 : down         ? "all sink"
                                                : "no effect");
        }
    }

    // --- 4. propulsion ---------------------------------------------------
    //
    // Lift is the wrong question for ThC. Its motor neurons are named
    // promotor and remotor in the connectome -- they swing the leg fore and
    // aft, which is the step. A joint that propels the body along X while
    // keeping it at height is doing its job even though tests 2 and 3 would
    // score it "moves, no lift".
    //
    // Walking needs exactly this: a joint whose two directions drive the body
    // forward and backward, consistently across all six legs.
    std::printf("\n=== 4. can any joint propel the body fore-aft? ===\n");
    std::printf("%-6s %7s %11s %11s  %s\n",
                "joint", "drive", "travel x", "height", "verdict");
    for (int j = 0; j < kJointCount; ++j) {
        for (const float d : {-g_probeDrive, g_probeDrive}) {
            const Result r = run(g_ms, j, d);
            const float dx = r.travelX - base.travelX;
            const float dz = r.height - base.height;
            const char* verdict;
            if (r.diverged) verdict = "DIVERGED";
            else if (std::fabs(dx) < 0.02f) verdict = "no travel";
            else if (dz < -0.06f) verdict = "travels, but collapses";
            else verdict = (dx > 0.0f) ? "FORWARD" : "BACKWARD";
            std::printf("%-6s %+7.1f %+11.4f %+11.4f  %s\n",
                        jointName(static_cast<Joint>(j)), d, dx, dz, verdict);
        }
    }

    std::printf("\n%s\n", anyLifts
        ? "At least one joint can raise the body, so a jump is mechanically possible."
        : "NO joint can raise the body. Muscle strength and neural drive are "
          "irrelevant until this is fixed.");
    return stands && anyLifts ? 0 : 2;
}
