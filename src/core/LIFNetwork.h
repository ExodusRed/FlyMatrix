#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "Connectome.h"

namespace fly {

// Leaky integrate-and-fire parameters.
//
// These follow the shape of the model in Shiu et al. 2024 (a LIF network over
// the FlyWire connectome): a fixed voltage step per synapse, uniform delay,
// hard threshold and reset. The numbers below are a reasonable starting point
// rather than a claim of exact replication -- that paper fit a different
// dataset, and epspPerSynapse in particular is the knob that decides whether
// this network sits quiet, balanced, or saturated. Sweep it first.
struct LifParams {
    float vRest = -52.0f;        // mV
    float vThreshold = -45.0f;   // mV
    float vReset = -52.0f;       // mV
    float tauM = 20.0f;          // membrane time constant, ms
    float refractoryMs = 2.2f;
    float delayMs = 1.8f;        // uniform axonal + synaptic delay
    // Swept against this connectome: below ~0.06 an evoked cascade dies out,
    // above ~0.1 the network saturates at the refractory ceiling. 0.08 puts
    // an evoked response near 1.8 Hz network-wide with ~4% of neurons active.
    float epspPerSynapse = 0.08f;   // mV of depolarisation per synapse
    float dtMs = 0.1f;

    // Modulatory transmitters (dopamine, serotonin, octopamine) have sign 0.
    // They act on slow timescales this model does not represent, so by default
    // their connections carry no current at all. Set this to treat them as
    // weak excitation instead.
    float modulatoryGain = 0.0f;
};

struct StepStats {
    std::uint32_t spikeCount = 0;
    float meanVoltage = 0.0f;
};

// Clock-driven LIF simulation over a Connectome.
//
// Spikes are delivered through a ring buffer of pending voltage steps, one
// slot per delay step, so propagation costs only the out-degree of neurons
// that actually fired.
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

    // Neurons that fired on the most recent step.
    std::span<const std::uint32_t> lastSpikes() const { return spikes_; }
    std::span<const float> voltages() const { return v_; }
    // Cumulative spike count per neuron since the last reset().
    std::span<const std::uint32_t> spikeTotals() const { return spikeTotal_; }

    std::uint64_t stepIndex() const { return t_; }
    float timeMs() const { return static_cast<float>(t_) * p_.dtMs; }
    const LifParams& params() const { return p_; }

private:
    const Connectome& c_;
    LifParams p_;

    std::uint32_t n_ = 0;
    std::uint32_t delaySteps_ = 1;
    std::uint16_t refractorySteps_ = 1;
    float leak_ = 0.0f;  // dt / tauM, precomputed

    std::vector<float> v_;
    std::vector<std::uint16_t> refractory_;
    std::vector<float> stimulus_;
    std::vector<std::uint32_t> spikeTotal_;
    std::vector<std::uint32_t> spikes_;

    // delaySteps_ slots of pending per-neuron voltage steps, flattened.
    std::vector<float> ring_;
    std::uint64_t t_ = 0;

    float* ringSlot(std::uint64_t step) {
        return ring_.data() + (step % delaySteps_) * n_;
    }
};

}  // namespace fly
