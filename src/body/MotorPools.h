#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "body/FlyBody.h"
#include "core/LIFNetwork.h"

namespace fly {

// Couples the simulated nervous system to the simulated body.
//
// Each joint of each leg has two antagonist pools of real motor neurons, read
// from data/bin/motor_map.tsv, which the connectome names after the muscles
// they drive. Spikes from a pool are low-pass filtered into an activation
// level, and the difference between the two antagonists sets the joint angle.
//
// The filter is not a smoothing convenience. Muscle force develops over tens of
// milliseconds, far slower than a spike, so a real motor neuron's firing *rate*
// is what a muscle responds to. It also resolves the timescale mismatch between
// a 0.1 ms neural step and a display frame.
class MotorPools {
public:
    struct Params {
        // Time constant of the rise and fall of muscle activation. Drosophila
        // leg muscles are on the order of 20-40 ms.
        float tauActivationMs = 30.0f;
        // Firing rate, in Hz, that counts as a fully activated muscle.
        float rateForFullActivation = 120.0f;
        // How far a fully one-sided activation drives a joint from rest, in
        // radians. A fixed angle rather than a fraction of the distance to the
        // joint limit: the hind knee rests bent the opposite way to the others,
        // so proportional excursion made it swing eight times further than the
        // front knee for identical drive.
        float excursionRad = 0.7f;
    };

    // Throws std::runtime_error if the map is missing or names no known joint.
    static MotorPools load(const std::string& tsvPath, Params params = {});

    // Accumulate spikes from one simulation step.
    void accumulate(const LIFNetwork& net);
    // Advance activation by `dtMs` of accumulated spikes and write the
    // resulting angles into the body.
    void apply(float dtMs, FlyBody& body);

    void reset();

    // One side of one joint.
    struct Pool {
        std::vector<std::uint32_t> neurons;
        float activation = 0.0f;   // 0..1, low-pass filtered rate
        std::uint32_t spikes = 0;  // since the last apply()
    };

    // [leg][joint][0 = flexor/retractor, 1 = extensor/protractor]
    const std::array<std::array<std::array<Pool, 2>, kJointCount>, kLegCount>&
    pools() const { return pools_; }

    std::size_t mappedNeurons() const { return mapped_; }

    // True if this neuron index belongs to any motor pool -- used by the
    // renderer to highlight them.
    bool isMotorNeuron(std::uint32_t idx) const;

private:
    Params p_;
    std::array<std::array<std::array<Pool, 2>, kJointCount>, kLegCount> pools_{};
    // Indexed by neuron: 0 for anything that is not a motor neuron, otherwise
    // 1 + the packed pool id, so a spike maps to its pool in one lookup.
    std::vector<std::uint16_t> poolOf_;

    static std::uint16_t packPool(int leg, int joint, int dir) {
        return static_cast<std::uint16_t>(((leg * kJointCount + joint) * 2 + dir) + 1);
    }
    std::size_t mapped_ = 0;
};

}  // namespace fly
