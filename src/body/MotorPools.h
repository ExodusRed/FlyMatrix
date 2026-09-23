#pragma once

#include <algorithm>
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
        // Derive each motor unit's force from its neuron's size instead of
        // the hand-written profile table.
        bool strengthFromSize = true;
    };

    // One motor unit: a single motor neuron and the muscle fibres it drives.
    //
    // Each carries its own activation rather than sharing the muscle's. Real
    // muscles recruit units in order of size -- small slow ones first, large
    // fast ones only under strong drive (Azevedo et al. 2020) -- so force is
    // graded by *which* units are firing, not just how fast. Averaging over a
    // muscle erases that: it made every unit fire together, and with strengths
    // derived from size that meant any broad activation produced maximal
    // force everywhere at once.
    struct MotorUnit {
        std::uint32_t neuron = 0;
        // This unit's force contribution, in units of the median leg motor
        // neuron, taken from its segmentation volume.
        float sizeRel = 1.0f;
        float activation = 0.0f;
        std::uint32_t spikes = 0;
    };

    struct Muscle {
        std::string name;
        std::vector<MotorUnit> units;
        int leg = 0, joint = 0, dir = 0;  // dir 0 = flex/retract
        // Time constant of activation rise and fall. Postural muscles are
        // slow; the jump muscle is fast-twitch.
        float tauMs = 30.0f;

        // Summed force this muscle can produce, all units fully active.
        float maxForce() const {
            float f = 0.0f;
            for (const auto& u : units) f += u.sizeRel;
            return f;
        }
        // Force it is producing now.
        float force() const {
            float f = 0.0f;
            for (const auto& u : units) f += u.activation * u.sizeRel;
            return f;
        }
        // Strongest single unit activation, for display.
        float peakActivation() const {
            float a = 0.0f;
            for (const auto& u : units) a = std::max(a, u.activation);
            return a;
        }
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
    // Which unit within that muscle, so a spike reaches its own
    // motor unit rather than the muscle as a whole.
    std::vector<std::uint16_t> unitOf_;
    std::size_t mapped_ = 0;
};

}  // namespace fly
