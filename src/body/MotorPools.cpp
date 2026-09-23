#include "MotorPools.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace fly {
namespace {

int jointIndex(const std::string& name) {
    for (int j = 0; j < kJointCount; ++j) {
        if (name == jointName(static_cast<Joint>(j))) return j;
    }
    return -1;
}

int legIndex(const std::string& name) {
    for (int l = 0; l < kLegCount; ++l) {
        if (name == legName(static_cast<LegId>(l))) return l;
    }
    return -1;
}

// Direction 0 folds the leg or pulls it back; 1 straightens or pushes forward.
int directionIndex(const std::string& d) {
    if (d == "flex" || d == "retract") return 0;
    if (d == "extend" || d == "protract") return 1;
    return -1;
}

struct MuscleProfile {
    const char* pattern;  // empty matches everything, so it must come last
    float tauMs;
    float strength;
};

// Per-muscle mechanics. The dataset says which muscle a neuron drives; this
// table says how that muscle behaves.
//
// The tergotrochanteral muscle is the jump muscle, and it is not a postural
// muscle with the volume turned up. It is fast-twitch -- a real escape takeoff
// is over in about five milliseconds, where a postural muscle takes thirty to
// develop force at all -- and it is very strong for its size. Modelling it
// like everything else was why the jump previously needed a tuned global
// multiplier and still arrived 150 ms late.
constexpr MuscleProfile kProfiles[] = {
    {"TTMn",        8.0f, 22.0f},
    {"Tergotr.",    8.0f, 22.0f},
    {"STTMm",      10.0f, 10.0f},
    // The tibia extensor contributes to a takeoff too, though far less.
    {"Ti extensor", 12.0f, 3.0f},
    {"",            30.0f, 1.0f},
};

// Sign relating a muscle's anatomical direction to the skeleton's joint angle,
// so that "extend" pushes the body up rather than down.
//
// These are measured, not reasoned. src/phys_test.cpp drives each joint in
// isolation and reports which direction raises the body; an earlier version of
// this table was derived by arguing about the geometry instead, and had the
// coxa-trochanter sign backwards, which meant the jump muscle splayed the leg
// sideways and the fly sank. Re-run `flyphys` after any change to the leg
// axes or the rest pose, because these signs follow from both.
constexpr float kJointDriveSign[kJointCount] = {
    -1.0f,  // ThC   measured: -15 lifts
    +1.0f,  // CTr   measured: +15 lifts
    +1.0f,  // TrF   neither direction lifts much
    -1.0f,  // FTi   measured: -15 lifts, and lifts most
    +1.0f,  // TiTa  measured: +15 lifts
};

const MuscleProfile& profileFor(const std::string& muscleName) {
    for (const auto& p : kProfiles) {
        if (*p.pattern == '\0' || muscleName.find(p.pattern) != std::string::npos) {
            return p;
        }
    }
    return kProfiles[sizeof(kProfiles) / sizeof(kProfiles[0]) - 1];
}

}  // namespace

MotorPools MotorPools::load(const std::string& tsvPath, Params params) {
    std::ifstream f(tsvPath);
    if (!f) {
        throw std::runtime_error(
            "cannot open " + tsvPath +
            "\nBuild it from the project root with: python tools/motor_map.py");
    }

    MotorPools mp;
    mp.p_ = params;

    // Key is leg/joint/direction/muscle, so the same muscle on a different leg
    // stays a separate entry with its own activation.
    std::unordered_map<std::string, std::uint16_t> lookup;
    std::vector<double> allSizes;

    std::string line;
    std::getline(f, line);  // header
    std::size_t rows = 0;
    std::uint32_t maxIdx = 0;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;

        std::istringstream ss(line);
        std::string leg, joint, dir, idxStr, type, sizeStr;
        if (!std::getline(ss, leg, '\t') || !std::getline(ss, joint, '\t') ||
            !std::getline(ss, dir, '\t') || !std::getline(ss, idxStr, '\t') ||
            !std::getline(ss, type, '\t')) {
            continue;
        }
        std::getline(ss, sizeStr);
        const double neuronSize = sizeStr.empty() ? 0.0 : std::stod(sizeStr);

        const int l = legIndex(leg), j = jointIndex(joint), d = directionIndex(dir);
        if (l < 0 || j < 0 || d < 0) continue;

        const std::string key = leg + "|" + joint + "|" + dir + "|" + type;
        auto it = lookup.find(key);
        if (it == lookup.end()) {
            const auto& prof = profileFor(type);
            Muscle m;
            m.name = type;
            m.leg = l;
            m.joint = j;
            m.dir = d;
            m.tauMs = prof.tauMs;
            m.strength = prof.strength;
            const auto id = static_cast<std::uint16_t>(mp.muscles_.size());
            mp.muscles_.push_back(std::move(m));
            mp.index_[l][j][d].push_back(id);
            it = lookup.emplace(key, id).first;
        }

        const auto idx = static_cast<std::uint32_t>(std::stoul(idxStr));
        mp.muscles_[it->second].neurons.push_back(idx);
        mp.muscles_[it->second].totalSize += neuronSize;
        allSizes.push_back(neuronSize);
        maxIdx = std::max(maxIdx, idx);
        ++rows;
    }

    if (rows == 0) throw std::runtime_error(tsvPath + " contained no usable rows");

    // Muscle strength from measured motor neuron size rather than a hand table.
    //
    // Drosophila leg motor neurons follow a size principle: force per spike
    // spans roughly a hundredfold from small slow units to large fast ones
    // (Azevedo et al. 2020). A muscle's force capacity is the sum over its
    // motor units, and activation here is already a mean rate across them, so
    // summed size is the right quantity. Expressed in units of the median
    // single leg motor neuron, which puts a typical postural muscle near 1.
    if (params.strengthFromSize && !allSizes.empty()) {
        std::vector<double> sorted = allSizes;
        std::sort(sorted.begin(), sorted.end());
        const double median = sorted[sorted.size() / 2];
        if (median > 0.0) {
            for (auto& m : mp.muscles_) {
                if (m.totalSize > 0.0) {
                    m.strength = static_cast<float>(m.totalSize / median);
                }
            }
        }
    }

    mp.muscleOf_.assign(static_cast<std::size_t>(maxIdx) + 1, 0u);
    for (std::size_t m = 0; m < mp.muscles_.size(); ++m) {
        for (const auto n : mp.muscles_[m].neurons) {
            mp.muscleOf_[n] = static_cast<std::uint16_t>(m + 1);
        }
    }
    mp.mapped_ = rows;
    return mp;
}

bool MotorPools::isMotorNeuron(std::uint32_t idx) const {
    return idx < muscleOf_.size() && muscleOf_[idx] != 0u;
}

void MotorPools::reset() {
    for (auto& m : muscles_) {
        m.activation = 0.0f;
        m.spikes = 0;
    }
}

void MotorPools::accumulate(const LIFNetwork& net) {
    // One indexed lookup per spike. Only 334 of 176,422 neurons drive a
    // muscle, so almost every spike falls straight through.
    for (const auto s : net.lastSpikes()) {
        if (s >= muscleOf_.size()) continue;
        const std::uint16_t packed = muscleOf_[s];
        if (packed != 0u) ++muscles_[packed - 1].spikes;
    }
}

void MotorPools::update(float dtMs) {
    if (dtMs <= 0.0f) return;
    for (auto& m : muscles_) {
        if (m.neurons.empty()) {
            m.spikes = 0;
            continue;
        }
        // Activation is a leaky integrator driven by spikes, not a low-pass
        // filter applied to a windowed rate estimate.
        //
        // The difference matters for a fast muscle. Measuring "spikes in the
        // last millisecond" is mostly measuring zero: the jump muscle fires
        // around 140 Hz, one spike every 7 ms, so nearly every window is
        // empty and a 4 ms filter decays to nothing in between. Integrating
        // spikes directly summates them the way real muscle force does, and
        // the steady-state activation works out to rate / rateForFullActivation
        // regardless of how long the step is.
        const float tauSec = m.tauMs * 0.001f;
        const float perSpike = 1.0f / (static_cast<float>(m.neurons.size()) *
                                       p_.rateForFullActivation * tauSec);
        m.activation *= std::exp(-dtMs / m.tauMs);
        m.activation += static_cast<float>(m.spikes) * perSpike;
        m.activation = std::min(m.activation, 1.0f);
        m.spikes = 0;
    }
}

float MotorPools::drive(int leg, int joint) const {
    float sum = 0.0f;
    for (int d = 0; d < 2; ++d) {
        float side = 0.0f;
        for (const auto id : index_[leg][joint][d]) {
            side += muscles_[id].activation * muscles_[id].strength;
        }
        sum += (d == 1) ? side : -side;
    }
    return sum * kJointDriveSign[joint];
}

float MotorPools::peakActivation(int leg, int joint) const {
    float best = 0.0f;
    for (int d = 0; d < 2; ++d) {
        for (const auto id : index_[leg][joint][d]) {
            best = std::max(best, muscles_[id].activation);
        }
    }
    return best;
}

void MotorPools::applyToSkeleton(FlyBody& body) const {
    for (int l = 0; l < kLegCount; ++l) {
        for (int j = 0; j < kJointCount; ++j) {
            // Drive is unbounded because muscle strengths differ; for the
            // kinematic path it has to be squashed back into a sane angle.
            const float d = std::clamp(drive(l, j), -1.0f, 1.0f);
            const JointSpec& spec = body.legs()[l].joints[j];
            body.setAngle(static_cast<LegId>(l), static_cast<Joint>(j),
                          spec.restAngle + d * p_.excursionRad);
        }
    }
}

}  // namespace fly
