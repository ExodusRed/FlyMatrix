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

#include "body/FlyBody.h"
#include "body/FlyPhysics.h"

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

    const int n = static_cast<int>(ms / 1000.0f / kDt);
    Result r{};
    for (int i = 0; i < n; ++i) {
        phys.step(kDt);
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
    r.feetDown = 0;
    r.thoraxDown = r.abdomenDown = r.headDown = false;
    for (const auto& c : phys.world.contacts) {
        if (c.probe < static_cast<std::uint32_t>(kLegCount)) ++r.feetDown;
        else if (c.probe == phys.thoraxProbe()) r.thoraxDown = true;
        else if (c.probe == phys.abdomenProbe()) r.abdomenDown = true;
        else if (c.probe == phys.headProbe()) r.headDown = true;
    }
    {
        // How far the front-left foot has slid from where it started.
        std::vector<FlyBody::SegmentPose> pose;
        phys.readPose(pose);
        const V3 foot = pose[kJointCount - 1].b;
        const V3 want = skeleton.footPosition(LegId::FrontL);
        r.footLateral = length(foot - want);
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
        std::vector<FlyBody::SegmentPose> pose;
        phys.readPose(pose);
        const V3 foot = pose[kJointCount - 1].b;
        r.reach = length(foot - phys.world.bodies[0].position);
        r.reachRest = length(skeleton.footPosition(LegId::FrontL) -
                             skeleton.root.position);

        std::vector<FlyBody::SegmentPose> restPose;
        skeleton.worldPose(restPose);
        for (int l = 0; l < kLegCount; ++l) {
            const std::size_t base = static_cast<std::size_t>(l) * kJointCount;
            const float now = length(pose[base + kJointCount - 1].b - pose[base].a);
            const float was =
                length(restPose[base + kJointCount - 1].b - restPose[base].a);
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
int gaitTest(float periodMs, float amplitude) {
    FlyBody skeleton;
    FlyPhysics phys;
    if (g_stiffness > 0.0f) phys.params.postureTorque = g_stiffness;
    if (g_substep > 0.0f) phys.params.substepHz = g_substep;
    if (g_tarsus > 0.0f) phys.params.tarsusStiffness = g_tarsus;
    if (g_minseg > 0.0f) phys.params.minSegmentMass = g_minseg;
    phys.params.useManualTarget = true;
    phys.build(skeleton);
    if (g_iters > 0) phys.world.params.iterations = g_iters;

    // Tripod A = front_L, middle_R, hind_L; tripod B is the other three.
    const bool tripodA[kLegCount] = {true, false, false, true, true, false};

    const float startX = phys.thorax().position.x;
    const float startZ = phys.bodyHeight();
    std::printf("=== gait: imposed tripod, NOT driven by the connectome ===\n");
    std::printf("period %.0f ms (%.1f Hz), amplitude %.1f\n\n",
                periodMs, 1000.0f / periodMs, amplitude);
    std::printf("%8s %10s %10s %9s %9s\n",
                "t (ms)", "x (mm)", "height", "contacts", "pitch");

    const int n = static_cast<int>(2000.0f / 1000.0f / kDt);
    float worstPitch = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float t = i * kDt * 1000.0f;
        const float phase = 6.2831853f * t / periodMs;
        for (int l = 0; l < kLegCount; ++l) {
            const float p = tripodA[l] ? phase : phase + 3.14159265f;
            const float s = std::sin(p);
            const float c = std::cos(p);
            // Retract through stance, protract through swing. `amplitude` is
            // now the ThC swing in radians -- a real fly's coxa swings on the
            // order of 0.3 rad -- rather than an abstract drive number.
            phys.params.manualTarget[l][static_cast<int>(Joint::ThC)] = -amplitude * s;
            // Lift only during swing, which is the half where the leg is
            // protracting. A leg that lifts during stance just drops the body.
            //
            // Positive CTr folds the leg and lifts the foot. Negative extends
            // it downward, which is the direction that raises the *body* --
            // flyphys test 2 reports CTr -15 as LIFTS, and taking that to mean
            // "lift the foot" drove every swing leg into the floor, so all six
            // feet stayed planted and the fly shuffled backwards.
            phys.params.manualTarget[l][static_cast<int>(Joint::CTr)] =
                (c > 0.0f) ? amplitude * 0.6f * c : 0.0f;
        }
        phys.step(kDt);
        const V3 fwd = phys.thorax().orientation.rotate({1, 0, 0});
        const float pitch = std::asin(std::clamp(fwd.z, -1.0f, 1.0f)) * 57.2958f;
        worstPitch = std::max(worstPitch, std::fabs(pitch));
        if (!std::isfinite(phys.bodyHeight())) {
            std::printf("DIVERGED at %.0f ms\n", t);
            return 2;
        }
        if (i % (n / 10) == 0 || i == n - 1) {
            std::printf("%8.0f %10.4f %10.4f %9zu %9.2f\n",
                        t, phys.thorax().position.x - startX, phys.bodyHeight(),
                        phys.world.contacts.size(), pitch);
        }
    }

    const float travel = phys.thorax().position.x - startX;
    const float held = phys.bodyHeight();
    std::printf("\ntravelled %+.3f mm in 2.0 s (%.2f mm/s), height %.3f -> %.3f, "
                "worst pitch %.1f deg\n",
                travel, travel / 2.0f, startZ, held, worstPitch);
    // A walking fly does a few body lengths a second; 1 mm/s is slow but it is
    // locomotion rather than twitching in place.
    const bool moved = std::fabs(travel) > 1.0f;
    const bool upright = held > 0.40f && worstPitch < 25.0f;
    std::printf("  -> %s\n", (moved && upright)
        ? "the body can walk when driven correctly"
        : (!moved ? "no net travel" : "travelled but did not stay upright"));
    return (moved && upright) ? 0 : 2;
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
    }
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--stand-only") == 0) g_standOnly = true;
    }
    if (argc >= 3 && std::strcmp(argv[1], "--trace") == 0) {
        const float drive = (argc >= 4) ? std::stof(argv[3]) : 15.0f;
        return traceOne(argv[2], drive);
    }
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--gait") == 0) {
            const float periodMs = (i + 1 < argc) ? std::stof(argv[i + 1]) : 40.0f;
            // Radians of ThC swing, not an abstract drive number.
            const float amp = (i + 2 < argc) ? std::stof(argv[i + 2]) : 0.3f;
            return gaitTest(periodMs, amp);
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
    std::printf("  max joint anchor error %.4f mm   front foot moved %.4f mm\n",
                base.anchorError, base.footLateral);
    if (base.anchorError > 0.02f) {
        std::printf("  WARNING: the linkage is stretching, not just bending\n");
    }
    // Six feet, plus any trunk probe. More than six means the fly has sagged
    // onto its belly, and no leg can lift a body already resting on the floor.
    if (base.contacts > 6) {
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
        for (const float d : {-15.0f, 15.0f}) {
            const Result r = run(g_ms, j, d);
            const float moved = r.angle[j] - base.angle[j];
            const float lifted = r.height - base.height;
            const char* verdict;
            if (r.diverged) verdict = "DIVERGED";
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
        for (const float d : {-15.0f, 15.0f}) {
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
        for (const float d : {-15.0f, 15.0f}) {
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
