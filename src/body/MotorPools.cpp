#include "MotorPools.h"

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

// Column 0 of the pair is the direction that folds the leg or pulls it back;
// column 1 straightens or pushes it forward.
int directionIndex(const std::string& d) {
    if (d == "flex" || d == "retract") return 0;
    if (d == "extend" || d == "protract") return 1;
    return -1;
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

    std::string line;
    std::getline(f, line);  // header
    std::size_t rows = 0, skipped = 0;
    std::uint32_t maxIdx = 0;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;

        std::istringstream ss(line);
        std::string leg, joint, dir, idxStr, type;
        if (!std::getline(ss, leg, '\t') || !std::getline(ss, joint, '\t') ||
            !std::getline(ss, dir, '\t') || !std::getline(ss, idxStr, '\t')) {
            ++skipped;
            continue;
        }
        const int l = legIndex(leg), j = jointIndex(joint), d = directionIndex(dir);
        if (l < 0 || j < 0 || d < 0) {
            ++skipped;
            continue;
        }
        const auto idx = static_cast<std::uint32_t>(std::stoul(idxStr));
        mp.pools_[l][j][d].neurons.push_back(idx);
        maxIdx = std::max(maxIdx, idx);
        ++rows;
    }

    if (rows == 0) {
        throw std::runtime_error(tsvPath + " contained no usable rows");
    }

    mp.poolOf_.assign(static_cast<std::size_t>(maxIdx) + 1, 0u);
    for (int l = 0; l < kLegCount; ++l) {
        for (int j = 0; j < kJointCount; ++j) {
            for (int d = 0; d < 2; ++d) {
                for (const auto n : mp.pools_[l][j][d].neurons) {
                    mp.poolOf_[n] = packPool(l, j, d);
                }
            }
        }
    }
    mp.mapped_ = rows;
    return mp;
}

bool MotorPools::isMotorNeuron(std::uint32_t idx) const {
    return idx < poolOf_.size() && poolOf_[idx] != 0u;
}

void MotorPools::reset() {
    for (auto& leg : pools_) {
        for (auto& joint : leg) {
            for (auto& pool : joint) {
                pool.activation = 0.0f;
                pool.spikes = 0;
            }
        }
    }
}

void MotorPools::accumulate(const LIFNetwork& net) {
    // One indexed lookup per spike. Only 334 of 176,422 neurons are in a pool,
    // so almost every spike falls straight through.
    for (const auto s : net.lastSpikes()) {
        if (s >= poolOf_.size()) continue;
        const std::uint16_t packed = poolOf_[s];
        if (packed == 0u) continue;
        const int id = packed - 1;
        ++pools_[id / (kJointCount * 2)][(id / 2) % kJointCount][id % 2].spikes;
    }
}

void MotorPools::apply(float dtMs, FlyBody& body) {
    if (dtMs <= 0.0f) return;
    const float alpha = 1.0f - std::exp(-dtMs / p_.tauActivationMs);

    for (int l = 0; l < kLegCount; ++l) {
        for (int j = 0; j < kJointCount; ++j) {
            float act[2] = {0.0f, 0.0f};
            for (int d = 0; d < 2; ++d) {
                Pool& pool = pools_[l][j][d];
                float target = 0.0f;
                if (!pool.neurons.empty()) {
                    // Mean firing rate of the pool over this interval, in Hz.
                    const float rate = 1000.0f * static_cast<float>(pool.spikes) /
                                       (dtMs * static_cast<float>(pool.neurons.size()));
                    target = std::min(1.0f, rate / p_.rateForFullActivation);
                }
                pool.activation += (target - pool.activation) * alpha;
                pool.spikes = 0;
                act[d] = pool.activation;
            }

            // Antagonists oppose each other: co-contraction holds the joint
            // still rather than driving it to an extreme, which is what a real
            // pair of opposing muscles does.
            const float drive = act[1] - act[0];
            const auto leg = static_cast<LegId>(l);
            const auto joint = static_cast<Joint>(j);
            const JointSpec& spec = body.legs()[l].joints[j];
            // setAngle clamps to the joint limits, so a joint already near one
            // end simply runs out of travel rather than being scaled down.
            body.setAngle(leg, joint,
                          spec.restAngle + drive * p_.excursionRad);
        }
    }
}

}  // namespace fly
