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
// The connectome names every leg motor neuron after the muscle it drives, so
// muscles are modelled individually rather than averaged into one pool per
// joint. That distinction turned out to matter: the tergotrochanteral jump
// muscle is two neurons inside a coxa-trochanter extensor group of about
// eighteen, and pooling them together diluted the one muscle that produces an
// escape jump down to a fraction of its output, while also giving it a
// postural muscle's slow response.
//
// Each muscle low-pass filters its own motor neurons' spikes into an
// activation with its own time constant. The filter is not smoothing for its
// own sake: muscle force develops over milliseconds, so firing *rate* is what
// a muscle responds to, and it also bridges the 0.1 ms neural step to a
// physics step.
class MotorPools {
public:
    struct Params {
        // Firing rate, in Hz, that counts as a fully activated muscle.
        float rateForFullActivation = 120.0f;
        // Used only by the kinematic path: how far a drive of 1 swings a joint
        // from rest, in radians. A fixed angle rather than a fraction of the
        // distance to the joint limit -- the hind knee rests bent the opposite
        // way to the others, so proportional excursion made it swing eight
        // times further than the front knee for identical drive.
        float excursionRad = 0.7f;
    };

    struct Muscle {
        std::string name;
        std::vector<std::uint32_t> neurons;
        int leg = 0, joint = 0, dir = 0;  // dir 0 = flex/retract
        // Time constant of activation rise and fall. Postural muscles are
        // slow; the jump muscle is fast-twitch.
        float tauMs = 30.0f;
        // Torque at full activation, relative to a postural muscle.
        float strength = 1.0f;
        float activation = 0.0f;
        std::uint32_t spikes = 0;
    };

    // Throws std::runtime_error if the map is missing or names no known joint.
    static MotorPools load(const std::string& tsvPath, Params params = {});

    // Accumulate spikes from one simulation step.
    void accumulate(const LIFNetwork& net);
    // Advance every muscle's activation by dtMs of accumulated spikes.
    void update(float dtMs);
    // Kinematic path: write joint angles straight into the skeleton, for
    // running without physics.
    void applyToSkeleton(FlyBody& body) const;

    void reset();

    // Net drive on one joint: extensors minus flexors, each weighted by its
    // own strength. Positive extends. Deliberately not bounded to [-1, 1] --
    // a strong muscle at full activation returns a large number, which is the
    // whole point of giving muscles individual strengths.
    float drive(int leg, int joint) const;
    // Strongest single activation on a joint, for display.
    float peakActivation(int leg, int joint) const;

    const std::vector<Muscle>& muscles() const { return muscles_; }
    std::size_t mappedNeurons() const { return mapped_; }
    bool isMotorNeuron(std::uint32_t idx) const;

private:
    Params p_;
    std::vector<Muscle> muscles_;
    // Indices into muscles_, per [leg][joint][direction].
    std::array<std::array<std::array<std::vector<std::uint16_t>, 2>,
                          kJointCount>, kLegCount> index_{};
    // Indexed by neuron: 0 for anything that is not a motor neuron, otherwise
    // 1 + the muscle index, so a spike maps to its muscle in one lookup.
    std::vector<std::uint16_t> muscleOf_;
    std::size_t mapped_ = 0;
};

}  // namespace fly
