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
    adaptDecay_ = (p_.tauAdapt > 0.0f) ? std::exp(-p_.dtMs / p_.tauAdapt) : 0.0f;

    v_.resize(n_);
    adapt_.resize(n_);
    refractory_.resize(n_);
    stimulus_.assign(n_, 0.0f);
    spikeTotal_.resize(n_);
    tonic_.resize(n_);
    gradedOut_.resize(n_);
    gradedSent_.resize(n_);
    ring_.assign(static_cast<std::size_t>(delaySteps_) * n_, 0.0f);
    spikes_.reserve(n_ / 16);

    graded_.resize(n_);
    for (std::uint32_t i = 0; i < n_; ++i) {
        graded_[i] = c_.isGraded(i) ? 1u : 0u;
        if (graded_[i]) gradedList_.push_back(i);
    }

    // Input scaling from cell size is fixed for the run, so resolve the pow()
    // once here rather than per step.
    inputScale_.resize(n_);
    const auto sizes = c_.sizeRel();
    for (std::uint32_t i = 0; i < n_; ++i) {
        float s = 1.0f;
        if (p_.sizeExponent != 0.0f && sizes[i] > 0.0f) {
            s = std::pow(sizes[i], -p_.sizeExponent);
            s = std::clamp(s, p_.sizeScaleMin, p_.sizeScaleMax);
        }
        inputScale_[i] = s;
    }

    reset();
}

void LIFNetwork::reset() {
    std::fill(v_.begin(), v_.end(), p_.vRest);
    std::fill(adapt_.begin(), adapt_.end(), 0.0f);
    std::fill(refractory_.begin(), refractory_.end(), std::uint16_t{0});
    std::fill(spikeTotal_.begin(), spikeTotal_.end(), 0u);
    std::fill(ring_.begin(), ring_.end(), 0.0f);
    // tonic_ and gradedSent_ must clear together: tonic_ is maintained by
    // accumulating differences against gradedSent_, so one without the other
    // would leave stale current in the network.
    std::fill(tonic_.begin(), tonic_.end(), 0.0f);
    std::fill(gradedOut_.begin(), gradedOut_.end(), 0.0f);
    std::fill(gradedSent_.begin(), gradedSent_.end(), 0.0f);
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
    StepStats stats;

    // Inputs scheduled to arrive now. Once consumed, this same slot becomes
    // the one that spikes emitted this step will land in, delaySteps_ ahead.
    float* arriving = ringSlot(t_);

    const float dt = p_.dtMs;
    double vSum = 0.0;

    for (std::uint32_t i = 0; i < n_; ++i) {
        // Synaptic input is scaled by the postsynaptic cell's size here, once,
        // rather than at every edge during propagation.
        const float in = (arriving[i] + tonic_[i] * dt) * inputScale_[i];
        arriving[i] = 0.0f;
        const float drive = in + stimulus_[i] * dt;

        if (graded_[i]) {
            // No threshold, no reset, no refractory period -- a graded neuron
            // just tracks its input, and its output is read off its voltage.
            float v = v_[i];
            v += (p_.vRest - v) * leak_ + drive;
            v_[i] = v;
            vSum += v;
            continue;
        }

        adapt_[i] *= adaptDecay_;

        if (refractory_[i] > 0) {
            --refractory_[i];
            v_[i] = p_.vReset;  // clamped, and arriving input is discarded
            vSum += p_.vReset;
            continue;
        }

        float v = v_[i] + (p_.vRest - v_[i]) * leak_ + drive;

        if (v >= p_.vThreshold + adapt_[i]) {
            spikes_.push_back(i);
            ++spikeTotal_[i];
            adapt_[i] += p_.adaptIncrement;
            v = p_.vReset;
            refractory_[i] = refractorySteps_;
        }
        v_[i] = v;
        vSum += v;
    }

    // Propagate spikes. Writing into the slot just drained schedules delivery
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

    propagateGraded(stats);

    ++t_;
    stats.spikeCount = static_cast<std::uint32_t>(spikes_.size());
    stats.meanVoltage = static_cast<float>(vSum / (n_ ? n_ : 1));
    return stats;
}

void LIFNetwork::propagateGraded(StepStats& stats) {
    if (gradedList_.empty()) return;

    const float span = p_.vThreshold - p_.vRest;
    if (span <= 0.0f) return;

    for (const std::uint32_t j : gradedList_) {
        // Output expressed as the firing rate a spiking neuron would need to
        // deliver the same current, so it shares the epspPerSynapse scale.
        const float drive = std::clamp((v_[j] - p_.vRest) / span,
                                       0.0f, p_.gradedMaxDrive);
        const float outHz = drive * p_.gradedRateAtThreshold;
        gradedOut_[j] = outHz;

        const float delta = outHz - gradedSent_[j];
        if (std::fabs(delta) < p_.gradedDeltaHz) continue;

        const std::int8_t sign = c_.signs()[j];
        const float gain = (sign == 0) ? p_.modulatoryGain
                                       : static_cast<float>(sign);
        if (gain != 0.0f) {
            // tonic_ holds mV/ms, so convert the Hz-equivalent delta into a
            // per-millisecond rate. Only the change is applied, which is why
            // gradedSent_ has to track what was last sent.
            const float amp = gain * (delta / 1000.0f) * p_.epspPerSynapse;
            const auto targets = c_.targetsOf(j);
            const auto weights = c_.weightsOf(j);
            for (std::size_t k = 0; k < targets.size(); ++k) {
                tonic_[targets[k]] += amp * static_cast<float>(weights[k]);
            }
        }
        gradedSent_[j] = outHz;
        ++stats.gradedUpdates;
    }
}

}  // namespace fly
