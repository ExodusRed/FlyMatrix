// flykin -- tests for the kinematic walker (variant B2, no physics).
//
//   flykin                       run the standard scenarios
//   flykin <seconds> <speed> <turn>   one scenario, with a gait timeline
//
// What a natural-looking walk has to show, measured rather than eyeballed:
// commanded speed and turning actually achieved, a believable step frequency
// and duty factor, coordination (how many legs swing together), feet that hit
// their targets, and joints used the way the leg pairs of a real fly use them.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "kin/KinematicFly.h"
#include "body/FlyModel.h"
#include <vector>

using namespace fly;
using namespace fly::kin;

namespace {

// The flybody model, loaded once. Run flykin from the FlyMatrix directory, or
// set FLYMODEL to the file.
const FlyModel& model() {
    static const FlyModel m = [] {
        const char* env = std::getenv("FLYMODEL");
        return FlyModel::load(env ? env : "web/data/flymodel.bin");
    }();
    return m;
}

struct Stats {
    float speed = 0, yawRate = 0, stepHz = 0, worstIk = 0, worstStance = 0;
    std::array<float, 3> duty{};                 // per leg pair
    std::array<float, 4> swingHist{};            // fraction of time with 0,1,2,3 legs swinging
    std::array<std::array<float, KinematicFly::DofCount>, 3> range{};  // joint range used, per pair
    float tripodShare = 0;                       // of 3-swing moments, fraction that are true tripods
};

Stats run(float seconds, float speed, float turn, bool timeline) {
    KinematicFly fly(model());
    // Experiment overrides, e.g. KIN_ITERS=12 KIN_LIFT=0.2 flykin
    if (const char* v = std::getenv("KIN_ITERS")) fly.params.ikIterations = std::atoi(v);
    if (const char* v = std::getenv("KIN_LIFT")) fly.params.liftHeight = static_cast<float>(std::atof(v));
    const float dt = 0.001f;
    const KinematicFly::Command cmd{speed, turn};
    const int n = static_cast<int>(seconds / dt);
    const int warm = 300;  // let the gait settle before measuring
    Stats st;
    std::array<int, kLegCount> stanceTicks{};
    std::array<std::array<float, KinematicFly::DofCount>, kLegCount> lo{}, hi{};
    for (auto& a : lo) a.fill(1e9f);
    for (auto& a : hi) a.fill(-1e9f);
    int counted = 0, threeSwing = 0, tripods = 0;
    const float startHeading = fly.heading();
    float headingAtWarm = 0;
    std::array<std::string, kLegCount> lines;

    for (int i = 0; i < n; ++i) {
        fly.step(dt, cmd);
        if (i == warm) headingAtWarm = fly.heading();
        if (i < warm) continue;
        ++counted;
        int swinging = 0;
        for (int l = 0; l < kLegCount; ++l) {
            if (fly.swinging(l)) ++swinging; else ++stanceTicks[l];
            const auto q = fly.legAngles(l);
            for (int d = 0; d < KinematicFly::DofCount; ++d) {
                lo[l][d] = std::min(lo[l][d], q[d]);
                hi[l][d] = std::max(hi[l][d], q[d]);
            }
        }
        st.swingHist[std::min(swinging, 3)] += 1;
        if (swinging == 3) {
            ++threeSwing;
            // Tripod A = front_L, middle_R, hind_L; tripod B the other three.
            const bool a = fly.swinging(0) && fly.swinging(3) && fly.swinging(4);
            const bool b = fly.swinging(1) && fly.swinging(2) && fly.swinging(5);
            if (a || b) ++tripods;
        }
        st.worstIk = std::max(st.worstIk, fly.worstFootError());
        st.worstStance = std::max(st.worstStance, fly.worstStanceError());
        if (timeline && i >= warm && i < warm + 400 && i % 4 == 0) {
            for (int l = 0; l < kLegCount; ++l) lines[l] += fly.swinging(l) ? '^' : '_';
        }
    }
    st.speed = fly.groundSpeed();
    st.yawRate = (fly.heading() - headingAtWarm) * 57.2958f / ((n - warm) * dt);
    (void)startHeading;
    st.stepHz = fly.stepFrequency();
    for (int p = 0; p < 3; ++p) {
        st.duty[p] = 0.5f * (stanceTicks[2 * p] + stanceTicks[2 * p + 1]) / counted;
        for (int d = 0; d < KinematicFly::DofCount; ++d) {
            st.range[p][d] = 0.5f * ((hi[2 * p][d] - lo[2 * p][d]) + (hi[2 * p + 1][d] - lo[2 * p + 1][d]));
        }
    }
    for (float& h : st.swingHist) h /= counted;
    st.tripodShare = threeSwing ? static_cast<float>(tripods) / threeSwing : 0.0f;

    if (timeline) {
        static const char* names[kLegCount] = {"front_L ", "front_R ", "middle_L", "middle_R", "hind_L  ", "hind_R  "};
        std::printf("gait, 400 ms after warm-up (^ swing, _ stance, 4 ms per character):\n");
        for (int l = 0; l < kLegCount; ++l) std::printf("  %s %s\n", names[l], lines[l].c_str());
        std::printf("\n");
    }
    return st;
}

void report(const char* name, float seconds, float speed, float turn, bool timeline) {
    const Stats s = run(seconds, speed, turn, timeline);
    std::printf("%-14s speed %5.1f mm/s  yaw %+6.1f deg/s  steps %4.1f Hz  duty F/M/H %.2f %.2f %.2f  "
                "swinging 0/1/2/3: %2.0f/%2.0f/%2.0f/%2.0f%%  tripod %3.0f%%  foot error stance %.3f / any %.3f mm\n",
                name, s.speed, s.yawRate, s.stepHz, s.duty[0], s.duty[1], s.duty[2],
                100 * s.swingHist[0], 100 * s.swingHist[1], 100 * s.swingHist[2], 100 * s.swingHist[3],
                100 * s.tripodShare, s.worstStance, s.worstIk);
    if (timeline) {
        static const char* dof[KinematicFly::DofCount] = {"CxAbd", "CxTwist", "Coxa", "FeTwist", "Femur", "Tibia", "Tarsus"};
        static const char* pair[3] = {"front ", "middle", "hind  "};
        std::printf("\njoint range used (radians), per leg pair:\n        ");
        for (const char* d : dof) std::printf("%8s", d);
        std::printf("\n");
        for (int p = 0; p < 3; ++p) {
            std::printf("  %s", pair[p]);
            for (int d = 0; d < KinematicFly::DofCount; ++d) std::printf("%8.3f", s.range[p][d]);
            std::printf("\n");
        }
    }
}

}  // namespace

// Geometry of the standing pose, for comparing the body against anatomy.
void printPose() {
    FlyBody body;
    std::printf("thorax centre height %.3f mm\n\n", body.root.position.z);
    std::printf("%-9s %22s %22s %8s %8s\n", "leg", "coxa attach (x,y,z)", "foot (x,y,z)", "length", "reach");
    for (int l = 0; l < kLegCount; ++l) {
        const Leg& L = body.legs()[l];
        float len = 0;
        for (const auto& j : L.joints) len += j.segmentLength;
        const V3 a = body.root.apply(L.attach);
        const V3 f = body.footPosition(static_cast<LegId>(l));
        const float reach = std::sqrt((f.x - a.x) * (f.x - a.x) + (f.y - a.y) * (f.y - a.y));
        std::printf("%-9s (%6.3f,%6.3f,%6.3f) (%6.3f,%6.3f,%6.3f) %8.3f %8.3f\n", legName(static_cast<LegId>(l)),
                    a.x, a.y, a.z, f.x, f.y, f.z, len, reach);
    }
    std::printf("\nsegment lengths per leg (coxa, trochanter, femur, tibia, tarsus):\n");
    for (int l = 0; l < kLegCount; l += 2) {
        const Leg& L = body.legs()[l];
        std::printf("  %-8s", legName(static_cast<LegId>(l)));
        for (const auto& j : L.joints) std::printf(" %.3f", j.segmentLength);
        std::printf("\n");
    }
    const V3 ff = body.footPosition(LegId::FrontL), hf = body.footPosition(LegId::HindL);
    std::printf("\nfront-to-hind foot spread 2L = %.3f mm, thorax height r = %.3f mm, L/r = %.2f\n",
                ff.x - hf.x, body.root.position.z, 0.5f * (ff.x - hf.x) / body.root.position.z);

    // The kinematic walker on the flybody model, for comparison.
    KinematicFly kin(model());
    std::printf("\n=== flybody model (kinematic walker) ===\n");
    std::printf("thorax centre height %.3f mm, rest pose fit error %.4f mm\n\n", kin.standHeight(), kin.restError());
    for (int l = 0; l < kLegCount; ++l) {
        const V3 f = kin.neutralFoot(l);
        std::printf("%-9s neutral claw (%6.3f,%6.3f,%6.3f)\n", legName(static_cast<LegId>(l)), f.x, f.y, f.z);
    }
    const float L = 0.5f * (kin.neutralFoot(0).x - kin.neutralFoot(4).x);
    std::printf("\nfront-to-hind foot spread 2L = %.3f mm, L/r = %.2f (real flies 1-2)\n", 2 * L,
                L / kin.standHeight());
}

// The flybody model's zero pose, to see how it stands before any IK.
void printModel(const char* path) {
    const FlyModel m = FlyModel::load(path);
    std::vector<float> q(m.joints().size(), 0.0f);
    std::vector<Transform> w;
    m.pose(Transform{}, q, w);
    std::printf("%zu bodies, %zu joints, %zu meshes\n\n", m.bodies().size(), m.joints().size(), m.meshes().size());
    for (std::size_t i = 0; i < m.bodies().size(); ++i) {
        const auto& b = m.bodies()[i];
        const std::string& n = b.name;
        const bool interesting = n == "thorax" || n == "head" || n == "abdomen" || n.find("coxa") == 0 ||
                                 n.find("tarsus_T") == 0 || n.find("claw") == 0 || n.find("wing") == 0 ||
                                 n.find("femur") == 0 || n.find("tibia") == 0;
        if (!interesting) continue;
        std::printf("%-20s world (%7.3f %7.3f %7.3f)", n.c_str(), w[i].position.x, w[i].position.y, w[i].position.z);
        for (int j : b.joints) {
            const auto& jt = m.joints()[static_cast<std::size_t>(j)];
            std::printf("  [%s %.2f..%.2f]", jt.name.c_str(), jt.lo, jt.hi);
        }
        std::printf("\n");
    }
}

int main(int argc, char** argv) {
    if (argc >= 3 && std::string(argv[1]) == "--model") {
        try {
            printModel(argv[2]);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "error: %s\n", e.what());
            return 1;
        }
        return 0;
    }
    if (argc >= 2 && std::string(argv[1]) == "--pose") {
        printPose();
        return 0;
    }
    if (argc >= 4) {
        report("custom", std::stof(argv[1]), std::stof(argv[2]), std::stof(argv[3]), true);
        return 0;
    }
    std::printf("=== kinematic walker (no physics) ===\n\n");
    report("fast", 3.0f, 1.0f, 0.0f, false);
    report("medium", 3.0f, 0.5f, 0.0f, false);
    report("slow", 3.0f, 0.2f, 0.0f, false);
    report("turn left", 3.0f, 0.6f, 0.6f, false);
    report("turn right", 3.0f, 0.6f, -0.6f, false);
    report("spin on spot", 3.0f, 0.0f, 1.0f, false);
    std::printf("\n");
    report("fast detail", 3.0f, 1.0f, 0.0f, true);
    return 0;
}
