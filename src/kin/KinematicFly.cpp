#include "kin/KinematicFly.h"

#include <algorithm>
#include <cmath>

namespace fly::kin {
namespace {

constexpr float kPi = 3.14159265f;

int pairOf(int leg) { return leg / 2; }  // 0 front, 1 middle, 2 hind
int sideOf(int leg) { return leg % 2 == 0 ? 1 : -1; }  // +1 left

// World point -> body frame (Transform has no inverse; scale is always 1 here).
V3 toBody(const Transform& t, const V3& p) {
    const Quat conj{t.rotation.w, -t.rotation.x, -t.rotation.y, -t.rotation.z};
    return conj.rotate(p - t.position);
}

float smoothstep(float x) {
    x = std::clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

// IK preference weights, per leg pair, in DOF order
// (ThC pro, ThC abd, ThC rot, CTr, TrF, FTi, TiTa). A LOW weight makes a joint
// cheap to move, so the solver uses it first. Front and hind legs step mostly
// by flexing the knee; middle legs mostly by rotating the coxa and twisting the
// femur. Abduction at the coxa is kept expensive: real legs swing fore-aft far
// more than they splay.
constexpr float kWeights[3][KinematicFly::DofCount] = {
    {0.40f, 3.0f, 2.0f, 1.0f, 2.0f, 0.15f, 0.6f},  // front
    {0.30f, 3.0f, 0.4f, 1.0f, 0.4f, 0.80f, 0.6f},  // middle
    {0.50f, 3.0f, 2.0f, 1.0f, 2.0f, 0.15f, 0.6f},  // hind
};

// Solve A x = b for a small dense system (Gaussian elimination, partial pivot).
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

KinematicFly::KinematicFly() { reset(); }

void KinematicFly::reset() {
    skeleton_ = FlyBody{};
    const auto& legs = skeleton_.legs();
    float footZ = 0.0f;
    for (int l = 0; l < kLegCount; ++l) {
        LegState& s = legs_[l];
        const Leg& L = legs[l];
        s.rest = {L.joints[0].restAngle, 0.0f, 0.0f, L.joints[1].restAngle,
                  L.joints[2].restAngle, L.joints[3].restAngle, L.joints[4].restAngle};
        s.lo = {L.joints[0].minAngle, -0.5f, -0.7f, L.joints[1].minAngle,
                L.joints[2].minAngle, L.joints[3].minAngle, L.joints[4].minAngle};
        s.hi = {L.joints[0].maxAngle, 0.5f, 0.7f, L.joints[1].maxAngle,
                L.joints[2].maxAngle, L.joints[3].maxAngle, L.joints[4].maxAngle};
        for (int d = 0; d < DofCount; ++d) s.weight[d] = kWeights[pairOf(l)][d];
        s.q = s.rest;
        s.neutralBody = legFk(l, s.rest);
        footZ += s.neutralBody.z;
        s.swing = false;
        s.swingT = 0.0f;
        s.boost = 0.0f;
        s.stepsTaken = 0;
    }
    // Stand the body at the height the rest pose puts its feet on the ground.
    baseHeight_ = -footZ / kLegCount;
    groundZ_ = 0.0f;
    heading_ = 0.0f;
    root_ = Transform{{0.0f, 0.0f, baseHeight_}, Quat{}, 1.0f};
    for (int l = 0; l < kLegCount; ++l) {
        V3 f = root_.apply(legs_[l].neutralBody);
        f.z = groundZ_;
        legs_[l].footWorld = f;
        lastStepAt_[l] = 0.0f;
        stepInterval_[l] = 0.0f;
    }
    velocityBody_ = {};
    yawRate_ = 0.0f;
    measuredSpeed_ = measuredYawRate_ = 0.0f;
    worstIkError_ = 0.0f;
    clock_ = 0.0f;
}

V3 KinematicFly::legFk(int leg, const std::array<float, DofCount>& q, V3* tibiaEnd,
                       std::vector<V3>* points, std::vector<Quat>* frames) const {
    const Leg& L = skeleton_.legs()[leg];
    const float side = static_cast<float>(sideOf(leg));
    Transform frame{L.attach, L.mount, 1.0f};
    if (points) points->push_back(frame.position);

    auto advance = [&](const V3& axis, float angle, float length) {
        frame.rotation = (frame.rotation * Quat::axisAngle(axis, angle)).normalised();
        if (frames) frames->push_back(frame.rotation);
        frame.position = frame.position + frame.rotation.rotate({0, 0, -1}) * length;
        if (points) points->push_back(frame.position);
    };

    // The 3-DOF coxa: protraction about the lateral axis (FlyBody's ThC), then
    // abduction about the fore-aft axis and rotation about the coxa's own long
    // axis, both mirrored per side.
    frame.rotation = (frame.rotation * Quat::axisAngle(L.joints[0].axis, q[ThCPro])).normalised();
    frame.rotation = (frame.rotation * Quat::axisAngle(V3{side, 0, 0}, q[ThCAbd])).normalised();
    advance(V3{0, 0, side}, q[ThCRot], L.joints[0].segmentLength);
    advance(L.joints[1].axis, q[CTr], L.joints[1].segmentLength);
    advance(L.joints[2].axis, q[TrF], L.joints[2].segmentLength);
    advance(L.joints[3].axis, q[FTi], L.joints[3].segmentLength);
    if (tibiaEnd) *tibiaEnd = frame.position;
    advance(L.joints[4].axis, q[TiTa], L.joints[4].segmentLength);
    return frame.position;
}

void KinematicFly::solveIk(int leg, const V3& target) {
    LegState& s = legs_[leg];
    const float tarsusLen = skeleton_.legs()[leg].joints[4].segmentLength;
    const float tarsusRise = tarsusLen * std::sin(params.tarsusAngleDeg * kPi / 180.0f);
    constexpr int kRes = 4 + DofCount;  // foot xyz, tarsus slope, preferences
    constexpr float kLambda = 0.004f;   // overall strength of the preferences
    constexpr float kSlopeWeight = 0.5f;

    auto residuals = [&](const std::array<float, DofCount>& q, float r[kRes]) {
        V3 tib;
        const V3 tip = legFk(leg, q, &tib);
        r[0] = tip.x - target.x;
        r[1] = tip.y - target.y;
        r[2] = tip.z - target.z;
        // Keep the tarsus lying at a natural slope rather than standing on its tip.
        r[3] = kSlopeWeight * (tib.z - (target.z + tarsusRise));
        for (int d = 0; d < DofCount; ++d) {
            r[4 + d] = std::sqrt(kLambda * s.weight[d]) * (q[d] - s.rest[d]);
        }
    };

    // Damped Gauss-Newton, warm-started from last frame's angles. A few
    // iterations a frame are plenty because the target moves only a little.
    for (int it = 0; it < 4; ++it) {
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
            a[i][i] += 1e-4f;  // Levenberg damping
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
    const V3 tip = legFk(leg, s.q);
    worstIkError_ = std::max(worstIkError_, length(tip - target));
}

V3 KinematicFly::touchdownTargetBody(int leg) const {
    const LegState& s = legs_[leg];
    const V3 r = s.neutralBody;
    // Velocity of the body point above this foot, in the body frame. A planted
    // foot drifts the opposite way, so the foot is placed *ahead* along this
    // direction -- further for legs on the outside of a turn, which is what
    // makes them take longer strides without any explicit rule.
    const V3 v{velocityBody_.x - yawRate_ * r.y, yawRate_ * r.x, 0.0f};
    const float speed = std::sqrt(v.x * v.x + v.y * v.y);
    const float reach = params.reach[pairOf(leg)];
    V3 t = r;
    if (speed > 1e-4f) {
        // Ramp the placement in with speed so slow walking takes small steps.
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
    // How far the foot has fallen behind its neutral point, along the
    // direction it is being dragged, plus any sideways error.
    const float along = (dx * v.x + dy * v.y) / speed;
    const float across = std::fabs(-dx * v.y + dy * v.x) / speed;
    return (std::max(0.0f, -along) + 0.5f * across) / reach;
}

bool KinematicFly::neighbourSwinging(int leg) const {
    const int contra = leg ^ 1;
    if (legs_[contra].swing) return true;
    const int ahead = leg - 2, behind = leg + 2;  // same side, adjacent segment
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

void KinematicFly::step(float dt, const Command& cmd) {
    clock_ += dt;

    // --- commands: ease towards the requested velocity and turning rate ---
    const float speedCmd = std::clamp(cmd.speed, 0.0f, 1.0f);
    const float turnCmd = std::clamp(cmd.turn, -1.0f, 1.0f);
    const float wantV = speedCmd * params.maxSpeed;
    const float yawMax = (speedCmd > 0.05f ? params.maxYawRateDeg : params.spinYawRateDeg) * kPi / 180.0f;
    const float wantYaw = turnCmd * yawMax;
    const float k = 1.0f - std::exp(-dt / 0.15f);
    velocityBody_.x += (wantV - velocityBody_.x) * k;
    yawRate_ += (wantYaw - yawRate_) * k;

    // --- body: glides over the planted feet ---
    const V3 before = root_.position;
    const float beforeHeading = heading_;
    heading_ += yawRate_ * dt;
    const float c = std::cos(heading_), sn = std::sin(heading_);
    root_.position.x += c * velocityBody_.x * dt;
    root_.position.y += sn * velocityBody_.x * dt;
    // The body dips a little while legs are in the air.
    const float wantZ = baseHeight_ - params.bodyBob * static_cast<float>(swingCount());
    root_.position.z += (wantZ - root_.position.z) * (1.0f - std::exp(-dt / 0.02f));
    root_.rotation = Quat::axisAngle({0, 0, 1}, heading_);

    const V3 moved = root_.position - before;
    const float inst = std::sqrt(moved.x * moved.x + moved.y * moved.y) / dt;
    measuredSpeed_ += (inst - measuredSpeed_) * std::min(1.0f, dt / 0.2f);
    measuredYawRate_ += ((heading_ - beforeHeading) / dt * 180.0f / kPi - measuredYawRate_) *
                        std::min(1.0f, dt / 0.2f);

    // --- swing legs: arc from lift-off towards a moving touchdown target ---
    for (int l = 0; l < kLegCount; ++l) {
        LegState& s = legs_[l];
        s.boost = std::max(0.0f, s.boost - dt / params.boostDecay * params.touchdownBoost);
        if (!s.swing) continue;
        s.swingT += dt;
        const float u = s.swingT / params.swingTime;
        V3 target = root_.apply(touchdownTargetBody(l));
        target.z = groundZ_;
        if (u >= 1.0f) {
            // Touchdown. Neighbours are now encouraged to lift (Walknet's
            // "a leg that has just stepped makes its neighbours more likely to").
            s.footWorld = target;
            s.swing = false;
            ++s.stepsTaken;
            const int nb[3] = {l ^ 1, l - 2, l + 2};
            for (int n : nb) {
                if (n >= 0 && n < kLegCount) legs_[n].boost = params.touchdownBoost;
            }
            continue;
        }
        const float e = smoothstep(u);
        V3 p = s.liftoffWorld + (target - s.liftoffWorld) * e;
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

    // --- inverse kinematics for every leg ---
    worstIkError_ = 0.0f;
    for (int l = 0; l < kLegCount; ++l) solveIk(l, toBody(root_, legs_[l].footWorld));
}

void KinematicFly::translate(const V3& d) {
    root_.position = root_.position + d;
    for (auto& s : legs_) {
        s.footWorld = s.footWorld + d;
        s.liftoffWorld = s.liftoffWorld + d;
    }
}

void KinematicFly::segments(std::vector<FlyBody::SegmentPose>& out) const {
    out.clear();
    std::vector<V3> pts;
    for (int l = 0; l < kLegCount; ++l) {
        pts.clear();
        legFk(l, legs_[l].q, nullptr, &pts);
        // pts: attach, coxa end, trochanter end, femur end, tibia end, tip.
        const auto& joints = skeleton_.legs()[l].joints;
        for (int j = 0; j < kJointCount; ++j) {
            out.push_back({root_.apply(pts[j]), root_.apply(pts[j + 1]), joints[j].segmentRadius,
                           static_cast<LegId>(l), static_cast<Joint>(j)});
        }
    }
}

}  // namespace fly::kin
