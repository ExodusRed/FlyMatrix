#include "LIFNetwork.h"

#include <algorithm>
#include <cmath>

namespace fly {

LIFNetwork::LIFNetwork(const Connectome& conn, const LifParams& params)
    : c_(conn), p_(params), n_(conn.neuronCount()) {
    delaySteps_ = std::max(1u, static_cast<std::uint32_t>(
                                   std::lround(p_.delayMs / p_.dtMs)));
    refractorySteps_ = static_cast<std::uint16_t>(
        std::max(0L, std::lround(p_.refractoryMs / p_.dtMs)));
    leak_ = p_.dtMs / p_.tauM;

    v_.resize(n_);
    refractory_.resize(n_);
    stimulus_.assign(n_, 0.0f);
    spikeTotal_.resize(n_);
    ring_.assign(static_cast<std::size_t>(delaySteps_) * n_, 0.0f);
    spikes_.reserve(n_ / 16);
    reset();
}

void LIFNetwork::reset() {
    std::fill(v_.begin(), v_.end(), p_.vRest);
    std::fill(refractory_.begin(), refractory_.end(), std::uint16_t{0});
    std::fill(spikeTotal_.begin(), spikeTotal_.end(), 0u);
    std::fill(ring_.begin(), ring_.end(), 0.0f);
    spikes_.clear();
    t_ = 0;
}

void LIFNetwork::setStimulus(std::uint32_t neuron, float mvPerMs) {
    if (neuron < n_) stimulus_[neuron] = mvPerMs;
}

void LIFNetwork::clearStimulus() {
    std::fill(stimulus_.begin(), stimulus_.end(), 0.0f);
}

StepStats LIFNetwork::step() {
    spikes_.clear();

    // Inputs scheduled to arrive now. Once consumed, this same slot becomes
    // the one that spikes emitted this step will land in, delaySteps_ ahead.
    float* arriving = ringSlot(t_);

    const float drive = p_.dtMs;  // stimulus is mV/ms
    double vSum = 0.0;

    for (std::uint32_t i = 0; i < n_; ++i) {
        const float in = arriving[i];
        arriving[i] = 0.0f;

        if (refractory_[i] > 0) {
            --refractory_[i];
            v_[i] = p_.vReset;   // clamped, and arriving input is discarded
            vSum += p_.vReset;
            continue;
        }

        float v = v_[i];
        v += (p_.vRest - v) * leak_ + in + stimulus_[i] * drive;

        if (v >= p_.vThreshold) {
            spikes_.push_back(i);
            ++spikeTotal_[i];
            v = p_.vReset;
            refractory_[i] = refractorySteps_;
        }
        v_[i] = v;
        vSum += v;
    }

    // Propagate. Writing into the slot we just drained schedules delivery
    // delaySteps_ from now.
    float* pending = arriving;
    for (const std::uint32_t j : spikes_) {
        const std::int8_t sign = c_.signs()[j];
        const float gain = (sign == 0) ? p_.modulatoryGain
                                       : static_cast<float>(sign);
        if (gain == 0.0f) continue;

        const float amp = gain * p_.epspPerSynapse;
        const auto targets = c_.targetsOf(j);
        const auto weights = c_.weightsOf(j);
        for (std::size_t k = 0; k < targets.size(); ++k) {
            pending[targets[k]] += amp * static_cast<float>(weights[k]);
        }
    }

    ++t_;
    return {static_cast<std::uint32_t>(spikes_.size()),
            static_cast<float>(vSum / (n_ ? n_ : 1))};
}

}  // namespace fly
