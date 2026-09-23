#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "body/FlyBody.h"
#include "body/FlyPhysics.h"
#include "core/LIFNetwork.h"

namespace fly {

// The return half of the sensorimotor loop.
//
// Until this existed the nervous system could command the body but never hear
// back from it, which makes the whole thing a puppet: no error to correct, no
// reflexes, no way to know where a leg is. These are the real proprioceptors
// from the connectome -- 403 chordotonal organ neurons that report a leg's
// configuration and 175 campaniform sensilla that report the load on it --
// driven by the actual state of the simulated body.
//
// Each is assigned to a leg by tools/sensory_map.py, which follows the neuron
// forward through the connectome and asks whose motor neurons it reaches. That
// recovers the leg, which the dataset does not annotate for sensory neurons
// because their cell bodies sit out in the leg itself.
//
// It does not recover the joint. The same walk tallied per joint just
// reproduces the size of each motor pool, so no claim is made about which
// joint a neuron watches, and the signals below are deliberately per-leg
// quantities rather than per-joint ones.
class SensoryOrgans {
public:
    struct Params {
        // Only use neurons whose leg assignment is this clean. Intersegmental
        // proprioceptors are real, but an ambiguous one fed the wrong leg's
        // state is worse than one left silent.
        float minPurity = 0.8f;

        // Drive, in mV/ms, at full stimulus. The same units the giant fibre is
        // driven with, where 200 is a hard experimental stimulation.
        float maxDrive = 90.0f;

        // Leg compression, as a fraction of the leg's rest span, that counts
        // as a full chordotonal signal. A fly's legs flex by a good fraction
        // of their length in normal use, so this is deliberately not tiny.
        float fullCompression = 0.25f;

        // Foot load counting as a full campaniform signal. One leg carrying
        // roughly a sixth of the body's weight is the resting case, so full
        // scale is several times that.
        float fullLoad = 4.0f;

        // Chordotonal organs are phasic as well as tonic: they respond to
        // movement, not only position. This weights the rate of change of leg
        // span against its absolute deviation.
        //
        // maxDrive below zero does NOT implement reflex reversal, though it
        // looks as though it might. The drive is clamp(signal, 0, 1) *
        // maxDrive, so a negative gain hyperpolarises the sensory neurons in
        // proportion to compression -- and a neuron that is already silent
        // cannot be silenced further, so nothing propagates. Measured: gains
        // of -2, -10, -50 and -200 give bit-identical results, and identical
        // to a gain of zero. A real reflex reversal happens downstream, in
        // whether the sensory path is routed through an inhibitory
        // interneuron, which is a property of the network and not of this
        // gain. See docs/findings.md section 2.
        float velocityWeight = 0.35f;
    };

    static SensoryOrgans load(const std::string& tsvPath, Params params = {});

    // Read the body's state and drive the sensory neurons from it. Only
    // touches neurons in the map, so a stimulus applied elsewhere -- the giant
    // fibre, say -- is left alone.
    void sense(const FlyPhysics& phys, LIFNetwork& net, float dtMs);

    void reset();

    std::size_t chordotonalCount() const { return chordotonal_.size(); }
    std::size_t campaniformCount() const { return campaniform_.size(); }

    // Last drive applied to each leg's organs, for display and testing.
    float chordotonalDrive(int leg) const { return chordotonalDrive_[leg]; }
    float campaniformDrive(int leg) const { return campaniformDrive_[leg]; }

private:
    struct Organ {
        std::uint32_t neuron;
        int leg;
    };

    Params p_;
    std::vector<Organ> chordotonal_;
    std::vector<Organ> campaniform_;
    std::array<float, kLegCount> chordotonalDrive_{};
    std::array<float, kLegCount> campaniformDrive_{};
    std::array<float, kLegCount> lastSpan_{};
    bool haveLastSpan_ = false;
};

}  // namespace fly
