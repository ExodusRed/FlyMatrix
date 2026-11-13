#include "kin/KinematicFly.h"

#include <algorithm>
#include <cmath>

#include "body/Anatomy.h"

namespace fly::kin {
namespace {

constexpr float kPi = 3.14159265f;

int pairOf(int leg) { return leg / 2; }  // 0 front, 1 middle, 2 hind
float sideOf(int leg) { return leg % 2 == 0 ? 1.0f : -1.0f; }  // +1 left

const anat::v2::LegSpec& specOf(int leg) {
    switch (pairOf(leg)) {
        case 0: return anat::v2::kFront;
        case 1: return anat::v2::kMiddle;
        default: return anat::v2::kHind;
    }
}

V3 mirrorY(const V3& v, float side) { return {v.x, v.y * side, v.z}; }

// World point -> body frame (Transform has no inverse; scale is always 1 here).
V3 toBody(const Transform& t, const V3& p) {
    const Quat conj{t.rotation.w, -t.rotation.x, -t.rotation.y, -t.rotation.z};
    return conj.rotate(p - t.position);
}

// Quaternion of the rotation whose columns are the orthonormal axes x, y, z.
Quat fromBasis(const V3& x, const V3& y, const V3& z) {
    const float trace = x.x + y.y + z.z;
    Quat q;
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        q = {0.25f * s, (y.z - z.y) / s, (z.x - x.z) / s, (x.y - y.x) / s};
    } else if (x.x > y.y && x.x > z.z) {
        const float s = std::sqrt(1.0f + x.x - y.y - z.z) * 2.0f;
        q = {(y.z - z.y) / s, 0.25f * s, (y.x + x.y) / s, (z.x + x.z) / s};
    } else if (y.y > z.z) {
        const float s = std::sqrt(1.0f + y.y - x.x - z.z) * 2.0f;
        q = {(z.x - x.z) / s, (y.x + x.y) / s, 0.25f * s, (z.y + y.z) / s};
    } else {
        const float s = std::sqrt(1.0f + z.z - x.x - y.y) * 2.0f;
        q = {(x.y - y.x) / s, (z.x + x.z) / s, (z.y + y.z) / s, 0.25f * s};
    }
    return q.normalised();
}

float smoothstep(float x) {
    x = std::clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

// IK preference weights per leg pair, for (ThC pro, ThC abd, ThC rot, CTr,
// TrF, FTi). A LOW weight makes a joint cheap to move, so the solver uses it
// first: front and hind legs step mostly with the knee, middle legs mostly by
// rotating the coxa and twisting the femur. Coxa abduction stays expensive.
constexpr float kWeights[3][KinematicFly::kIkDofs] = {
    {0.40f, 3.0f, 2.0f, 1.0f, 2.0f, 0.15f},  // front
    {0.30f, 3.0f, 0.4f, 1.0f, 0.4f, 0.80f},  // middle
    {1.00f, 3.0f, 2.0f, 1.0f, 2.0f, 0.15f},  // hind: coxa swing dearer, v1 leaned on it
};

// Tarsomere radii for drawing, proximal to distal.
constexpr float kTarsomereRadius[anat::kTarsomereCount] = {0.020f, 0.018f, 0.016f, 0.015f, 0.014f};
constexpr float kSegmentRadius[4] = {0.050f, 0.040f, 0.036f, 0.028f};  // coxa, troch, femur, tibia

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
    worstIkError_ = 0.0f;
    for (int l = 0; l < kLegCount; ++l) {
        LegState& s = legs_[l];
        const anat::v2::LegSpec& spec = specOf(l);
        s.side = sideOf(l);
        s.attach = mirrorY(spec.attach, s.side);
        s.len[0] = spec.coxa;
        s.len[1] = spec.trochanter;
        s.len[2] = spec.femur;
        s.len[3] = spec.tibia;
        s.len[4] = spec.tarsus;
        s.neutralBody = mirrorY(spec.foot, s.side);
        const V3 td = mirrorY(spec.tarsusDir, s.side);
        s.tarsusDirBody = normalise(V3{td.x, td.y, 0.0f});

        // Coxa rest frame: local -Z along the coxa, local X as close to the
        // body's forward axis as the coxa allows, local Y completing it. For
        // a mirrored pair this gives mirrored frames, so a hinge about local
        // Y bends both sides the same way (see FlyBody.cpp on mirroring).
        const V3 down = normalise(mirrorY(spec.coxaDir, s.side));
        const V3 zAxis = down * -1.0f;
        const V3 fwd{1, 0, 0};
        const V3 xAxis = normalise(fwd - zAxis * dot(fwd, zAxis));
        const V3 yAxis = cross(zAxis, xAxis);
        s.coxaFrame = fromBasis(xAxis, yAxis, zAxis);

        s.lo = {-1.2f, -0.6f, -1.2f, -1.8f, -0.8f, anat::v2::kKneeMin, -1.5f};
        s.hi = {1.2f, 0.6f, 1.2f, 1.8f, 0.8f, anat::v2::kKneeMax, 1.5f};
        for (int d = 0; d < kIkDofs; ++d) s.weight[d] = kWeights[pairOf(l)][d];

        // Rest pose: solve for the neutral foot from a folded starting guess
        // (femur raised, knee bent), regularised towards that guess so the
        // solution stays in the same insect-like branch.
        s.rest = {0.0f, 0.0f, 0.0f, -1.2f, 0.0f, 2.0f, 0.0f};
        s.q = s.rest;
        solveIk(l, ankleTargetFor(l, s.neutralBody, 0.0f), 400);
        s.rest = s.q;
        s.swing = false;
        s.swingT = 0.0f;
        s.boost = 0.0f;
        s.stepsTaken = 0;
    }
    restIkError_ = worstIkError_;
    baseHeight_ = anat::v2::kStandHeight;
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

V3 KinematicFly::ankleFk(int leg, const std::array<float, DofCount>& q, std::vector<V3>* points) const {
    const LegState& s = legs_[leg];
    const float side = s.side;
    const float obl = anat::v2::kHingeObliquityDeg * kPi / 180.0f;
    const V3 X{1, 0, 0}, Y{0, 1, 0}, Z{0, 0, 1};
    Quat r = s.coxaFrame;
    V3 p = s.attach;
    if (points) points->push_back(p);
    auto segment = [&](float length) {
        p = p + r.rotate({0, 0, -1}) * length;
        if (points) points->push_back(p);
    };
    // Thorax-coxa ball joint: swing (about local Y), abduction (local X) and
    // rotation about the coxa's own axis (local Z); X and Z mirror per side.
    r = (r * Quat::axisAngle(Y, q[ThCPro])).normalised();
    r = (r * Quat::axisAngle(X * side, q[ThCAbd])).normalised();
    r = (r * Quat::axisAngle(Z * side, q[ThCRot])).normalised();
    segment(s.len[0]);
    // CTr and FTi hinge about axes tilted off the perpendicular, the way real
    // hinges run between two condyles. TrF twists the femur about its axis.
    const V3 ctrAxis = Quat::axisAngle(Z * side, obl).rotate(Y);
    r = (r * Quat::axisAngle(ctrAxis, q[CTr])).normalised();
    segment(s.len[1]);
    r = (r * Quat::axisAngle(Z * side, q[TrF])).normalised();
    segment(s.len[2]);
    const V3 ftiAxis = Quat::axisAngle(Z * side, -obl).rotate(Y);
    r = (r * Quat::axisAngle(ftiAxis, q[FTi])).normalised();
    segment(s.len[3]);
    return p;
}

V3 KinematicFly::ankleTargetFor(int leg, const V3& foot, float /*lift*/) const {
    const LegState& s = legs_[leg];
    // The first tarsomere slopes from the ankle down to the contact point;
    // the rest of the tarsus lies along the ground beyond it.
    const float l1 = s.len[4] * anat::kTarsomereFrac[0];
    const float slope = anat::v2::kTarsusSlopeDeg * kPi / 180.0f;
    const V3 d = s.tarsusDirBody;
    return {foot.x - d.x * std::cos(slope) * l1, foot.y - d.y * std::cos(slope) * l1,
            foot.z + std::sin(slope) * l1};
}

void KinematicFly::solveIk(int leg, const V3& target, int iterations) {
    LegState& s = legs_[leg];
    constexpr int kRes = 3 + kIkDofs;
    constexpr float kLambda = 0.004f;

    auto residuals = [&](const std::array<float, DofCount>& q, float r[kRes]) {
        const V3 a = ankleFk(leg, q);
        r[0] = a.x - target.x;
        r[1] = a.y - target.y;
        r[2] = a.z - target.z;
        for (int d = 0; d < kIkDofs; ++d) {
            r[3 + d] = std::sqrt(kLambda * s.weight[d]) * (q[d] - s.rest[d]);
        }
    };

    for (int it = 0; it < iterations; ++it) {
        float r0[kRes];
        residuals(s.q, r0);
        float jac[kRes][kIkDofs];
        for (int d = 0; d < kIkDofs; ++d) {
            auto qd = s.q;
            const float h = 1e-3f;
            qd[d] += h;
            float r1[kRes];
            residuals(qd, r1);
            for (int k = 0; k < kRes; ++k) jac[k][d] = (r1[k] - r0[k]) / h;
        }
        float a[kIkDofs][kIkDofs] = {};
        float g[kIkDofs] = {};
        for (int i = 0; i < kIkDofs; ++i) {
            for (int j = 0; j < kIkDofs; ++j) {
                float sum = 0.0f;
                for (int k = 0; k < kRes; ++k) sum += jac[k][i] * jac[k][j];
                a[i][j] = sum;
            }
            a[i][i] += 1e-4f;  // Levenberg damping
            float sum = 0.0f;
            for (int k = 0; k < kRes; ++k) sum += jac[k][i] * r0[k];
            g[i] = -sum;
        }
        float dq[kIkDofs];
        if (!solveLinear<kIkDofs>(a, g, dq)) break;
        for (int d = 0; d < kIkDofs; ++d) {
            s.q[d] = std::clamp(s.q[d] + std::clamp(dq[d], -0.3f, 0.3f), s.lo[d], s.hi[d]);
        }
    }
    const V3 a = ankleFk(leg, s.q);
    worstIkError_ = std::max(worstIkError_, length(a - target));
}

V3 KinematicFly::touchdownTargetBody(int leg) const {
    const LegState& s = legs_[leg];
    const V3 r = s.neutralBody;
    // Velocity of the body point above this foot, in the body frame. A planted
    // foot drifts the opposite way, so the foot is placed *ahead* along this
    // direction -- further for legs on the outside of a turn.
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

KinematicFly::LegGeometry KinematicFly::geometry(int leg) const {
    const LegState& s = legs_[leg];
    return {s.attach, {s.len[0], s.len[1], s.len[2], s.len[3], s.len[4]}, s.neutralBody};
}

void KinematicFly::step(float dt, const Command& cmd) {
    clock_ += dt;

    // --- commands: ease towards the requested velocity and turning rate ---
    const float speedCmd = std::clamp(cmd.speed, 0.0f, 1.0f);
    const float turnCmd = std::clamp(cmd.turn, -1.0f, 1.0f);
    const float wantV = speedCmd * params.maxSpeed;
    const float yawMax = (speedCmd > 0.05f ? params.maxYawRateDeg : params.spinYawRateDeg) * kPi / 180.0f;
    const float k = 1.0f - std::exp(-dt / 0.15f);
    velocityBody_.x += (wantV - velocityBody_.x) * k;
    yawRate_ += (turnCmd * yawMax - yawRate_) * k;

    // --- body: glides over the planted feet ---
    const V3 before = root_.position;
    const float beforeHeading = heading_;
    heading_ += yawRate_ * dt;
    root_.position.x += std::cos(heading_) * velocityBody_.x * dt;
    root_.position.y += std::sin(heading_) * velocityBody_.x * dt;
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
            s.footWorld = target;
            s.swing = false;
            ++s.stepsTaken;
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

    // --- inverse kinematics: ankle to where the tarsus needs it ---
    worstIkError_ = 0.0f;
    for (int l = 0; l < kLegCount; ++l) {
        solveIk(l, ankleTargetFor(l, toBody(root_, legs_[l].footWorld), 0.0f), 4);
    }
    // Report the ankle angle: between the tibia and the first tarsomere.
    for (int l = 0; l < kLegCount; ++l) {
        LegState& s = legs_[l];
        std::vector<V3> pts;
        ankleFk(l, s.q, &pts);
        const V3 tibia = normalise(pts[4] - pts[3]);
        const V3 ta1 = normalise(toBody(root_, s.footWorld) - pts[4]);
        s.q[TiTa] = std::acos(std::clamp(dot(tibia, ta1), -1.0f, 1.0f));
    }
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
        const LegState& s = legs_[l];
        pts.clear();
        ankleFk(l, s.q, &pts);
        // pts: attach, coxa end, trochanter end, femur end, ankle.
        for (int j = 0; j < 4; ++j) {
            out.push_back({root_.apply(pts[j]), root_.apply(pts[j + 1]), kSegmentRadius[j],
                           static_cast<LegId>(l), static_cast<Joint>(j)});
        }
        // Tarsus: ta1 from the ankle down to the contact point, then ta2-ta5
        // lying along the ground (or trailing, slightly curled, in swing).
        const V3 ankle = root_.apply(pts[4]);
        const V3 contact = s.footWorld;
        out.push_back({ankle, contact, kTarsomereRadius[0], static_cast<LegId>(l), Joint::TiTa});
        const V3 dir = root_.rotation.rotate(s.tarsusDirBody);
        V3 p = contact;
        for (int t = 1; t < anat::kTarsomereCount; ++t) {
            const float segLen = s.len[4] * anat::kTarsomereFrac[t];
            // On the ground the tarsus lies flat; in the air it droops a little
            // more with each segment.
            const float droop = s.swing ? 0.25f * static_cast<float>(t) : 0.0f;
            V3 next = p + dir * (segLen * std::cos(droop));
            next.z = s.swing ? p.z - segLen * std::sin(droop) : groundZ_ + 0.004f;
            out.push_back({p, next, kTarsomereRadius[t], static_cast<LegId>(l), Joint::TiTa});
            p = next;
        }
    }
}

}  // namespace fly::kin
