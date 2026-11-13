#include "body/WalkController.h"

#include <algorithm>
#include <cmath>

namespace fly {

namespace {

constexpr float kTwoPi = 6.2831853f;

// Tripod A = front_L, middle_R, hind_L, at phase 0; tripod B half a cycle on.
constexpr std::array<float, kLegCount> kTripodOffset = {0.0f, 0.5f, 0.5f, 0.0f, 0.0f, 0.5f};

bool isLeft(int leg) { return leg % 2 == 0; }  // LegId order: L, R, L, R, L, R

float smoothstep(float x) {
    x = std::clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

float approach(float value, float target, float maxDelta) {
    if (target > value) return std::min(target, value + maxDelta);
    return std::max(target, value - maxDelta);
}

}  // namespace

WalkController::WalkController() { reset(); }

void WalkController::reset() {
    for (int l = 0; l < kLegCount; ++l) phase_[l] = kTripodOffset[l];
    speed_ = 0.0f;
    turn_ = 0.0f;
    haveHeading_ = false;
}

WalkController::Command WalkController::holdHeading(float dt, const Command& cmd,
                                                    float headingRad) {
    if (!haveHeading_) {
        desiredHeading_ = headingRad;
        haveHeading_ = true;
    }
    // Only steer the course while actually walking; standing still, the
    // desired heading follows the body so it doesn't wind up.
    if (std::fabs(cmd.speed) < 1e-3f) {
        desiredHeading_ = headingRad;
        return cmd;
    }
    desiredHeading_ += std::clamp(cmd.turn, -1.0f, 1.0f) *
                       params.maxYawRateDeg * 0.01745329f * dt;

    float err = desiredHeading_ - headingRad;
    err = std::atan2(std::sin(err), std::cos(err));  // wrap to [-pi, pi]
    // If the body falls far behind the requested heading (a sharp turn the
    // gait cannot follow), don't let the error grow without bound.
    if (std::fabs(err) > 0.6f) {
        desiredHeading_ = headingRad + std::copysign(0.6f, err);
        err = std::copysign(0.6f, err);
    }

    Command out = cmd;
    // Walking backwards, the same stride asymmetry turns the other way.
    const float sign = cmd.speed < 0.0f ? -1.0f : 1.0f;
    out.turn = sign * std::clamp(params.headingGain * err,
                                 -params.maxCorrection, params.maxCorrection);
    return out;
}

bool WalkController::swinging(int leg) const {
    return std::fabs(speed_) > 1e-3f && phase_[leg] >= beta_;
}

void WalkController::step(float dt, const Command& cmd,
                          float targets[kLegCount][kJointCount]) {
    const float ramp = params.rampPerSecond * dt;
    speed_ = approach(speed_, std::clamp(cmd.speed, -1.0f, 1.0f), ramp);
    turn_ = approach(turn_, std::clamp(cmd.turn, -1.0f, 1.0f), ramp);

    const float mag = std::fabs(speed_);
    // Flies speed up mainly by stepping faster, partly by striding longer.
    const float freq = mag < 1e-3f ? 0.0f
        : params.minFrequencyHz + (params.maxFrequencyHz - params.minFrequencyHz) * mag;
    const float strideScale = 0.6f + 0.4f * mag;

    // --- rhythm: coupled phase oscillators ---
    std::array<float, kLegCount> next = phase_;
    for (int i = 0; i < kLegCount; ++i) {
        float pull = 0.0f;
        for (int j = 0; j < kLegCount; ++j) {
            if (j == i) continue;
            // Desired: phase_[j] - phase_[i] == offset[j] - offset[i].
            const float want = kTripodOffset[j] - kTripodOffset[i];
            pull += std::sin(kTwoPi * (phase_[j] - phase_[i] - want));
        }
        const float dphase = freq + (params.coupling / kTwoPi) * pull / (kLegCount - 1);
        next[i] = phase_[i] + dphase * dt;
        next[i] -= std::floor(next[i]);
    }
    phase_ = next;

    // --- step shape ---
    // Slower walking keeps more feet down, as in real flies, which move from
    // a tripod towards tetrapod-like coordination as they slow: the duty
    // factor rises and the foot is lifted less. A slow swing held high left
    // three legs in the air for a long time and tipped the fly over.
    const float beta = std::min(0.8f, params.dutyFactor + 0.2f * (1.0f - mag));
    beta_ = beta;
    const float lift = params.liftRad * (0.5f + 0.5f * mag);
    for (int l = 0; l < kLegCount; ++l) {
        // Turning: the outer side strides longer, the inner side shorter.
        // Seen from above, turning left (turn > 0) puts the right legs on the
        // outside. Kept to +/-50%: a 2:0 asymmetry tipped the fly over.
        const float side = isLeft(l) ? (1.0f - 0.5f * turn_) : (1.0f + 0.5f * turn_);
        const float amp = params.strideRad * strideScale * side *
                          (mag < 1e-3f ? 0.0f : 1.0f);
        // Walking backwards runs the same cycle with the stroke reversed.
        const float dir = speed_ < 0.0f ? -1.0f : 1.0f;

        const float u = phase_[l];
        float thc = 0.0f, liftShape = 0.0f;
        if (u < beta) {
            // Stance: protracted -> retracted at constant angular speed.
            const float s = u / beta;
            thc = amp * (1.0f - 2.0f * s);
        } else {
            // Swing: retracted -> protracted, eased at both ends, foot lifted.
            const float s = (u - beta) / (1.0f - beta);
            thc = amp * (-1.0f + 2.0f * smoothstep(s));
            liftShape = std::sin(3.14159265f * s);
        }

        // `thc` above is positive when protracted. The joint itself is the
        // other way round: *negative* ThC swings the leg forward. The imposed
        // gait in flyphys lifts the foot while ThC runs from +A to -A, and the
        // front legs rest at -0.84 rad "reaching forward" (findings 7).
        // Positive CTr folds the leg and lifts the foot.
        targets[l][static_cast<int>(Joint::ThC)] = -dir * thc;
        targets[l][static_cast<int>(Joint::CTr)] =
            lift * liftShape * std::min(1.0f, mag * 3.0f);
    }
}

}  // namespace fly
