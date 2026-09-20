#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "Connectome.h"

namespace fly {

// Leaky integrate-and-fire parameters.
//
// The core follows the shape of the model in Shiu et al. 2024 (a LIF network
// over the FlyWire connectome): a fixed voltage step per synapse, uniform
// delay, hard threshold and reset. Three additions go beyond that baseline --
// spike-frequency adaptation, size-scaled excitability, and graded
// transmission -- each documented where it appears below.
struct LifParams {
    float vRest = -52.0f;        // mV
    float vThreshold = -45.0f;   // mV
    float vReset = -52.0f;       // mV
    float tauM = 20.0f;          // membrane time constant, ms
    float refractoryMs = 2.2f;
    float delayMs = 1.8f;        // uniform axonal + synaptic delay

    // Centre of the usable band measured by tools/sweep.py with the defaults
    // below (adaptation on, size scaling off). Below ~0.065 an evoked cascade
    // dies out; above ~0.116 the network saturates.
    float epspPerSynapse = 0.085f;  // mV of depolarisation per synapse
    float dtMs = 0.1f;

    // Modulatory transmitters (dopamine, serotonin, octopamine) have sign 0.
    // They act on slow timescales this model does not represent, so by default
    // their connections carry no current at all. Set this to treat them as
    // weak excitation instead.
    float modulatoryGain = 0.0f;

    // --- Spike-frequency adaptation ---
    //
    // Real neurons fatigue under sustained drive. Without this, nothing stops a
    // neuron pinning at the refractory ceiling (~450 Hz at the default
    // refractory period), which is far outside anything a fly does, and it
    // makes the network's behaviour hinge sharply on epspPerSynapse.
    //
    // Each spike raises that neuron's own threshold by adaptIncrement; the
    // raise decays with tauAdapt. At a steady rate r the threshold settles
    // around adaptIncrement * r * tauAdapt above baseline, so 100 Hz costs
    // ~6 mV here while 400 Hz costs ~24 mV and is strongly suppressed.
    // Set adaptIncrement to 0 to disable.
    float adaptIncrement = 0.4f;  // mV of added threshold per spike
    float tauAdapt = 150.0f;      // ms

    // --- Size-scaled excitability (off by default -- see below) ---
    //
    // A larger cell has more membrane to charge, so the same synaptic current
    // moves its voltage less. Incoming steps are scaled by
    // sizeRel^-sizeExponent, clamped to keep the extremes of a 1.7-million-fold
    // size range from producing absurd values. 1.0 would be the naive
    // capacitance-proportional reading; 0.5 is gentler, since segmentation
    // volume is a rough proxy for membrane area and real neurons regulate
    // their own excitability.
    //
    // This defaults to OFF because it does not survive its own test. At 0.5 it
    // scales 96.5% of neurons smoothly (only 3.5% hit a clamp), so it behaves
    // as intended -- but tools/sweep.py measures it *narrowing* the usable band
    // of synaptic gain, from 1.60x to 1.41x, rather than widening it. It is
    // plausible physics with no measured benefit, so it stays available and
    // unused. Set sizeExponent to 0.5 to turn it back on.
    float sizeExponent = 0.0f;
    float sizeScaleMin = 0.25f;
    float sizeScaleMax = 4.0f;

    // --- Graded (non-spiking) neurons ---
    //
    // Neurons flagged graded in the connectome release transmitter
    // continuously in proportion to depolarisation, and never spike. Output is
    // expressed as the firing rate a spiking neuron would need to deliver the
    // same current, so it shares the epspPerSynapse scale.
    //
    // gradedMaxDrive caps output at that multiple of threshold depolarisation.
    // gradedDeltaHz is a laziness threshold: a graded neuron only re-propagates
    // when its output moves by more than this, which keeps transmission sparse
    // instead of touching every out-edge on every timestep.
    float gradedRateAtThreshold = 50.0f;  // Hz-equivalent at v == vThreshold
    float gradedMaxDrive = 3.0f;
    float gradedDeltaHz = 2.0f;
};

struct StepStats {
    std::uint32_t spikeCount = 0;
    float meanVoltage = 0.0f;
    // Graded neurons that re-propagated this step. Usually small; if it sits
    // near the graded population size every step, gradedDeltaHz is too tight.
    std::uint32_t gradedUpdates = 0;
};

// Clock-driven LIF simulation over a Connectome.
//
// Spiking neurons deliver their output through a ring buffer of pending
// voltage steps, one slot per delay step, so propagation costs only the
// out-degree of neurons that actually fired. Graded neurons instead maintain a
// persistent per-target current, updated only when their output changes
// materially.
class LIFNetwork {
public:
    LIFNetwork(const Connectome& conn, const LifParams& params);

    // Advance one dt. Returns spike/voltage summary for this step.
    StepStats step();

    void reset();

    // Constant current injected every step, in mV per ms, indexed by neuron.
    // This is how you drive sensory neurons.
    void setStimulus(std::uint32_t neuron, float mvPerMs);
    void clearStimulus();

    // Neurons that fired on the most recent step. Graded neurons never appear
    // here -- read gradedOutput() for those.
    std::span<const std::uint32_t> lastSpikes() const { return spikes_; }
    std::span<const float> voltages() const { return v_; }
    // Cumulative spike count per neuron since the last reset().
    std::span<const std::uint32_t> spikeTotals() const { return spikeTotal_; }
    // Current output of each neuron in Hz-equivalent; zero for spiking ones.
    std::span<const float> gradedOutput() const { return gradedOut_; }
    std::span<const std::uint32_t> gradedNeurons() const { return gradedList_; }
    // Per-neuron multiplier applied to incoming current, from cell size.
    std::span<const float> inputScale() const { return inputScale_; }

    std::uint64_t stepIndex() const { return t_; }
    float timeMs() const { return static_cast<float>(t_) * p_.dtMs; }
    const LifParams& params() const { return p_; }

private:
    void propagateGraded(StepStats& stats);

    const Connectome& c_;
    LifParams p_;

    std::uint32_t n_ = 0;
    std::uint32_t delaySteps_ = 1;
    std::uint16_t refractorySteps_ = 1;
    float leak_ = 0.0f;        // dt / tauM, precomputed
    float adaptDecay_ = 1.0f;  // exp(-dt / tauAdapt), precomputed

    std::vector<float> v_;
    std::vector<float> adapt_;       // added to threshold, decays toward 0
    std::vector<float> inputScale_;  // from cell size, constant after setup
    std::vector<std::uint16_t> refractory_;
    std::vector<std::uint8_t> graded_;
    std::vector<float> stimulus_;
    std::vector<std::uint32_t> spikeTotal_;
    std::vector<std::uint32_t> spikes_;

    // Persistent current from graded neurons, in mV/ms, before input scaling.
    std::vector<float> tonic_;
    std::vector<float> gradedOut_;       // current output, Hz-equivalent
    std::vector<float> gradedSent_;      // output at last propagation
    std::vector<std::uint32_t> gradedList_;

    // delaySteps_ slots of pending per-neuron voltage steps, flattened.
    std::vector<float> ring_;
    std::uint64_t t_ = 0;

    float* ringSlot(std::uint64_t step) {
        return ring_.data() + (step % delaySteps_) * n_;
    }
};

}  // namespace fly
