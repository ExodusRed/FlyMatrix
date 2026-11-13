#include "kin/KinematicFly.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fly::kin {
namespace {

constexpr float kPi = 3.14159265f;

int pairOf(int leg) { return leg / 2; }  // 0 front, 1 middle, 2 hind

// LegId order (front/middle/hind x left/right) -> flybody's naming suffix.
std::string suffixOf(int leg) {
    return "_T" + std::to_string(pairOf(leg) + 1) + (leg % 2 == 0 ? "_left" : "_right");
}

V3 toBody(const Transform& t, const V3& p) {
    const Quat conj{t.rotation.w, -t.rotation.x, -t.rotation.y, -t.rotation.z};
    return conj.rotate(p - t.position);
}

float smoothstep(float x) {
    x = std::clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

// IK preference weights per leg pair, for (coxa abduct, coxa twist, coxa
// swing, femur twist, femur, tibia, tarsus). A LOW weight makes a joint cheap
// to move: front and hind legs step mostly at the femur-tibia joint, middle
// legs mostly by rotating the coxa and twisting the femur (Anipose). Coxa
// abduction is kept expensive.
constexpr float kWeights[3][KinematicFly::DofCount] = {
    {3.0f, 1.0f, 0.4f, 1.5f, 0.5f, 0.15f, 0.6f},  // front
    {3.0f, 0.4f, 0.3f, 0.4f, 0.6f, 0.80f, 0.6f},  // middle
    {3.0f, 1.5f, 1.0f, 1.5f, 0.5f, 0.15f, 0.6f},  // hind
};

template <int N>
bool solveLinear(float a[N][N], float b[N], float x[N]) {
    for (int c = 0; c < N; ++c) {
        int piv = c;
        for (int r = c + 1; r < N; ++r) {
            if (std::fabs(a[r][c]) > std::fabs(a[piv][c])) piv = r;
        }
        if (std::fabs(a[piv][c]) < 1e-12f) return false;
        if (piv != c) {
            for (int k = 0; k < N; ++k) std::swap(a[c][k], a[piv][k]);
            std::swap(b[c], b[piv]);
        }
        for (int r = c + 1; r < N; ++r) {
            const float f = a[r][c] / a[c][c];
            for (int k = c; k < N; ++k) a[r][k] -= f * a[c][k];
            b[r] -= f * b[c];
        }
    }
    for (int r = N - 1; r >= 0; --r) {
        float s = b[r];
        for (int k = r + 1; k < N; ++k) s -= a[r][k] * x[k];
        x[r] = s / a[r][r];
    }
    return true;
}

}  // namespace

KinematicFly::KinematicFly(const FlyModel& model) : model_(model) {
    static const char* kDofNames[DofCount] = {"coxa_abduct", "coxa_twist", "coxa", "femur_twist",
                                              "femur", "tibia", "tarsus"};
    static const char* kChain[] = {"coxa", "femur", "tibia", "tarsus", "tarsus2", "tarsus3", "tarsus4", "claw"};
    bodyLeg_.assign(model_.bodies().size(), -1);
    for (int l = 0; l < kLegCount; ++l) {
        LegState& s = legs_[l];
        const std::string sfx = suffixOf(l);
        for (int d = 0; d < DofCount; ++d) {
            s.joint[d] = model_.jointIndex(std::string(kDofNames[d]) + sfx);
            if (s.joint[d] < 0) throw std::runtime_error(std::string("fly model lacks joint ") + kDofNames[d] + sfx);
            const auto& j = model_.joints()[static_cast<std::size_t>(s.joint[d])];
            s.lo[d] = j.lo;
            s.hi[d] = j.hi;
            s.weight[d] = kWeights[pairOf(l)][d];
        }
        for (int k = 0; k < 4; ++k) s.passive[k] = model_.jointIndex("tarsus" + std::to_string(k + 2) + sfx);
        s.chain.clear();
        for (const char* b : kChain) {
            const int idx = model_.bodyIndex(std::string(b) + sfx);
            if (idx < 0) throw std::runtime_error(std::string("fly model lacks body ") + b + sfx);
            s.chain.push_back(idx);
            bodyLeg_[static_cast<std::size_t>(idx)] = l;
        }
        s.ankleBody = s.chain[3];
        s.clawBody = s.chain.back();
    }
    reset();
}

void KinematicFly::reset() {
    // Everything that is not a leg starts at its spring rest angle: that folds
    // the wings back over the abdomen, as on a walking fly. The legs instead
    // use the zero pose, which is a natural standing pose (their spring rest
    // angles describe a tucked, flying posture).
    q_.assign(model_.joints().size(), 0.0f);
    for (std::size_t j = 0; j < q_.size(); ++j) q_[j] = model_.joints()[j].ref;
    for (const auto& s : legs_) {
        for (const int b : s.chain) {
            for (const int j : model_.bodies()[static_cast<std::size_t>(b)].joints) q_[static_cast<std::size_t>(j)] = 0.0f;
        }
    }
    float clawZ = 0.0f;
    restIkError_ = 0.0f;
    for (int l = 0; l < kLegCount; ++l) {
        LegState& s = legs_[l];
        // Rest = the model's zero pose, which is a natural standing pose.
        s.q.fill(0.0f);
        V3 ankle, claw;
        legFk(l, s.q, ankle, claw);
        s.neutralBody = claw;
        s.ankleRise = ankle.z - claw.z;
        clawZ += claw.z;
        s.swing = false;
        s.swingT = 0.0f;
        s.boost = 0.0f;
    }
    baseHeight_ = -clawZ / kLegCount;
    groundZ_ = 0.0f;
    heading_ = 0.0f;
    root_ = Transform{{0.0f, 0.0f, baseHeight_}, Quat{}, 1.0f};
    for (int l = 0; l < kLegCount; ++l) {
        // Feet start exactly on the floor, so the IK fixes up the few microns
        // by which the zero pose's claws are not level.
        V3 f = root_.apply(legs_[l].neutralBody);
        f.z = groundZ_;
        legs_[l].footWorld = f;
        legs_[l].neutralBody.z = -baseHeight_;
        lastStepAt_[l] = 0.0f;
        stepInterval_[l] = 0.0f;
        worstIkError_ = 0.0f;
        solveIk(l, legs_[l].neutralBody, 50);
        restIkError_ = std::max(restIkError_, worstIkError_);
    }
    velocityBody_ = {};
    yawRate_ = 0.0f;
    measuredSpeed_ = 0.0f;
    worstIkError_ = 0.0f;
    clock_ = 0.0f;
    writeJointAngles();
}

void KinematicFly::legFk(int leg, const std::array<float, DofCount>& q, V3& ankle, V3& claw) const {
    const LegState& s = legs_[leg];
    // The coxa's parent is the thorax, whose frame is the body frame.
    Transform t{};
    for (std::size_t c = 0; c < s.chain.size(); ++c) {
        const auto& b = model_.bodies()[static_cast<std::size_t>(s.chain[c])];
        t = t * Transform{b.pos, b.quat, 1.0f};
        for (const int j : b.joints) {
            float angle = 0.0f;
            for (int d = 0; d < DofCount; ++d) {
                if (s.joint[d] == j) angle = q[d];
            }
            if (angle != 0.0f) {
                t.rotation = (t.rotation * Quat::axisAngle(model_.joints()[static_cast<std::size_t>(j)].axis, angle)).normalised();
            }
        }
        if (s.chain[c] == s.ankleBody) ankle = t.position;
    }
    claw = t.position;
}

void KinematicFly::solveIk(int leg, const V3& target, int iterations) {
    LegState& s = legs_[leg];
    constexpr int kRes = 4 + DofCount;
    constexpr float kLambda = 0.004f;
    // In stance, keep the tarsus at its natural slope: the ankle stays as far
    // above the claw as it is when the fly stands at rest. In swing, let it
    // go: a lifted leg's tarsus hangs and trails, and holding the stance slope
    // forced the whole leg up past its joint limits (0.24 mm foot error at a
    // 0.28 mm lift, independent of IK iterations).
    const float ankleWeight = s.swing ? 0.0f : 0.5f;

    auto residuals = [&](const std::array<float, DofCount>& q, float r[kRes]) {
        V3 ankle, claw;
        legFk(leg, q, ankle, claw);
        r[0] = claw.x - target.x;
        r[1] = claw.y - target.y;
        r[2] = claw.z - target.z;
        r[3] = ankleWeight * (ankle.z - (target.z + s.ankleRise));
        for (int d = 0; d < DofCount; ++d) r[4 + d] = std::sqrt(kLambda * s.weight[d]) * q[d];
    };

    for (int it = 0; it < iterations; ++it) {
        float r0[kRes];
        residuals(s.q, r0);
        float jac[kRes][DofCount];
        for (int d = 0; d < DofCount; ++d) {
            auto qd = s.q;
            const float h = 1e-3f;
            qd[d] += h;
            float r1[kRes];
            residuals(qd, r1);
            for (int k = 0; k < kRes; ++k) jac[k][d] = (r1[k] - r0[k]) / h;
        }
        float a[DofCount][DofCount] = {};
        float g[DofCount] = {};
        for (int i = 0; i < DofCount; ++i) {
            for (int j = 0; j < DofCount; ++j) {
                float sum = 0.0f;
                for (int k = 0; k < kRes; ++k) sum += jac[k][i] * jac[k][j];
                a[i][j] = sum;
            }
            a[i][i] += 1e-4f;
            float sum = 0.0f;
            for (int k = 0; k < kRes; ++k) sum += jac[k][i] * r0[k];
            g[i] = -sum;
        }
        float dq[DofCount];
        if (!solveLinear<DofCount>(a, g, dq)) break;
        for (int d = 0; d < DofCount; ++d) {
            s.q[d] = std::clamp(s.q[d] + std::clamp(dq[d], -0.3f, 0.3f), s.lo[d], s.hi[d]);
        }
    }
    V3 ankle, claw;
    legFk(leg, s.q, ankle, claw);
    worstIkError_ = std::max(worstIkError_, length(claw - target));
    if (!s.swing) worstStanceError_ = std::max(worstStanceError_, length(claw - target));
}

void KinematicFly::writeJointAngles() {
    for (const auto& s : legs_) {
        for (int d = 0; d < DofCount; ++d) q_[static_cast<std::size_t>(s.joint[d])] = s.q[d];
    }
}

V3 KinematicFly::touchdownTargetBody(int leg) const {
    const LegState& s = legs_[leg];
    const V3 r = s.neutralBody;
    const V3 v{velocityBody_.x - yawRate_ * r.y, yawRate_ * r.x, 0.0f};
    const float speed = std::sqrt(v.x * v.x + v.y * v.y);
    const float reach = params.reach[pairOf(leg)];
    V3 t = r;
    if (speed > 1e-4f) {
        const float place = reach * 0.8f * std::min(1.0f, speed / 4.0f);
        t.x += v.x / speed * place;
        t.y += v.y / speed * place;
    }
    return t;
}

float KinematicFly::urge(int leg) const {
    const LegState& s = legs_[leg];
    const V3 foot = toBody(root_, s.footWorld);
    const float dx = foot.x - s.neutralBody.x, dy = foot.y - s.neutralBody.y;
    const float reach = params.reach[pairOf(leg)];
    const V3 r = s.neutralBody;
    const V3 v{velocityBody_.x - yawRate_ * r.y, yawRate_ * r.x, 0.0f};
    const float speed = std::sqrt(v.x * v.x + v.y * v.y);
    if (speed < 1e-3f) return std::sqrt(dx * dx + dy * dy) / reach;
    const float along = (dx * v.x + dy * v.y) / speed;
    const float across = std::fabs(-dx * v.y + dy * v.x) / speed;
    return (std::max(0.0f, -along) + 0.5f * across) / reach;
}

bool KinematicFly::neighbourSwinging(int leg) const {
    if (legs_[leg ^ 1].swing) return true;
    const int ahead = leg - 2, behind = leg + 2;
    if (ahead >= 0 && legs_[ahead].swing) return true;
    if (behind < kLegCount && legs_[behind].swing) return true;
    return false;
}

int KinematicFly::swingCount() const {
    int n = 0;
    for (const auto& s : legs_) n += s.swing ? 1 : 0;
    return n;
}

float KinematicFly::stepFrequency() const {
    float sum = 0.0f;
    int n = 0;
    for (int l = 0; l < kLegCount; ++l) {
        if (stepInterval_[l] > 0.0f && clock_ - lastStepAt_[l] < 0.5f) {
            sum += 1.0f / stepInterval_[l];
            ++n;
        }
    }
    return n ? sum / n : 0.0f;
}

std::array<float, KinematicFly::DofCount> KinematicFly::legAngles(int leg) const { return legs_[leg].q; }

int KinematicFly::legOfBody(int body) const {
    return body >= 0 && body < static_cast<int>(bodyLeg_.size()) ? bodyLeg_[static_cast<std::size_t>(body)] : -1;
}

void KinematicFly::step(float dt, const Command& cmd) {
    clock_ += dt;

    const float speedCmd = std::clamp(cmd.speed, 0.0f, 1.0f);
    const float turnCmd = std::clamp(cmd.turn, -1.0f, 1.0f);
    const float yawMax = (speedCmd > 0.05f ? params.maxYawRateDeg : params.spinYawRateDeg) * kPi / 180.0f;
    const float k = 1.0f - std::exp(-dt / 0.15f);
    velocityBody_.x += (speedCmd * params.maxSpeed - velocityBody_.x) * k;
    yawRate_ += (turnCmd * yawMax - yawRate_) * k;

    // --- body: glides over the planted feet ---
    const V3 before = root_.position;
    heading_ += yawRate_ * dt;
    root_.position.x += std::cos(heading_) * velocityBody_.x * dt;
    root_.position.y += std::sin(heading_) * velocityBody_.x * dt;
    const float wantZ = baseHeight_ - params.bodyBob * static_cast<float>(swingCount());
    root_.position.z += (wantZ - root_.position.z) * (1.0f - std::exp(-dt / 0.02f));
    root_.rotation = Quat::axisAngle({0, 0, 1}, heading_);
    const V3 moved = root_.position - before;
    measuredSpeed_ += (std::sqrt(moved.x * moved.x + moved.y * moved.y) / dt - measuredSpeed_) *
                      std::min(1.0f, dt / 0.2f);

    // --- swing legs ---
    for (int l = 0; l < kLegCount; ++l) {
        LegState& s = legs_[l];
        s.boost = std::max(0.0f, s.boost - dt / params.boostDecay * params.touchdownBoost);
        if (!s.swing) continue;
        s.swingT += dt;
        const float u = s.swingT / params.swingTime;
        V3 target = root_.apply(touchdownTargetBody(l));
        target.z = groundZ_;
        if (u >= 1.0f) {
            s.footWorld = target;
            s.swing = false;
            const int nb[3] = {l ^ 1, l - 2, l + 2};
            for (int n : nb) {
                if (n >= 0 && n < kLegCount) legs_[n].boost = params.touchdownBoost;
            }
            continue;
        }
        V3 p = s.liftoffWorld + (target - s.liftoffWorld) * smoothstep(u);
        p.z = groundZ_ + params.liftHeight * std::sin(kPi * u);
        s.footWorld = p;
    }

    // --- stance legs: decide who lifts next, most urgent first ---
    std::array<int, kLegCount> order{0, 1, 2, 3, 4, 5};
    std::array<float, kLegCount> u{};
    for (int l = 0; l < kLegCount; ++l) u[l] = legs_[l].swing ? -1.0f : urge(l);
    std::sort(order.begin(), order.end(), [&](int a, int b) { return u[a] > u[b]; });
    const bool moving = std::fabs(velocityBody_.x) > 0.05f || std::fabs(yawRate_) > 0.02f;
    for (int l : order) {
        LegState& s = legs_[l];
        if (s.swing) continue;
        const bool forced = u[l] >= 1.0f;
        const float threshold = moving ? params.stepUrge - s.boost : 0.15f;
        const bool wants = u[l] >= threshold && !neighbourSwinging(l) && swingCount() < 3;
        if (!(forced || wants)) continue;
        s.swing = true;
        s.swingT = 0.0f;
        s.liftoffWorld = s.footWorld;
        if (lastStepAt_[l] > 0.0f) stepInterval_[l] = clock_ - lastStepAt_[l];
        lastStepAt_[l] = clock_;
    }

    // --- inverse kinematics ---
    worstStanceError_ = 0.0f;
    worstIkError_ = 0.0f;
    for (int l = 0; l < kLegCount; ++l) solveIk(l, toBody(root_, legs_[l].footWorld), params.ikIterations);
    writeJointAngles();
}

void KinematicFly::translate(const V3& d) {
    root_.position = root_.position + d;
    for (auto& s : legs_) {
        s.footWorld = s.footWorld + d;
        s.liftoffWorld = s.liftoffWorld + d;
    }
}

}  // namespace fly::kin
