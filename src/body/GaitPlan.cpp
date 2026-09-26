#include "GaitPlan.h"

#include <algorithm>
#include <cmath>

namespace fly {
namespace {

constexpr float kPi = 3.14159265358979f;

float wrap01(float v) {
    v -= std::floor(v);
    return (v < 0.0f || v >= 1.0f) ? 0.0f : v;
}

// Solve a 3x3 system by Gaussian elimination with partial pivoting. Three
// unknowns does not justify pulling in a linear algebra library, and writing
// it out keeps the damping explicit.
bool solve3(float A[3][3], const float b[3], float x[3]) {
    float m[3][4];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) m[i][j] = A[i][j];
        m[i][3] = b[i];
    }
    for (int c = 0; c < 3; ++c) {
        int piv = c;
        for (int r = c + 1; r < 3; ++r) {
            if (std::fabs(m[r][c]) > std::fabs(m[piv][c])) piv = r;
        }
        if (std::fabs(m[piv][c]) < 1e-20f) return false;
        if (piv != c) for (int j = 0; j < 4; ++j) std::swap(m[c][j], m[piv][j]);
        const float inv = 1.0f / m[c][c];
        for (int r = 0; r < 3; ++r) {
            if (r == c) continue;
            const float f = m[r][c] * inv;
            for (int j = c; j < 4; ++j) m[r][j] -= f * m[c][j];
        }
    }
    for (int i = 0; i < 3; ++i) x[i] = m[i][3] / m[i][i];
    return true;
}

// Damped least squares onto a foot target, with a pull toward the rest pose
// to settle the two redundant degrees of freedom.
//
// Five joints reaching a three-dimensional point is redundant, and without
// the null-space term the solution wanders: consecutive samples along the
// cycle pick unrelated postures and the tabulated path becomes discontinuous,
// which the physics then has to absorb as an impulse. Warm starting from the
// previous sample does most of the work; this keeps it honest over a full
// cycle.
float solveFootIK(FlyBody& body, LegId id, const V3& target,
                  const float rest[kJointCount]) {
    const int leg = static_cast<int>(id);
    (void)leg;
    float err = 0.0f;
    for (int it = 0; it < 80; ++it) {
        const V3 e = target - body.footPosition(id);
        err = length(e);
        if (err < 1e-6f) break;

        // Numeric Jacobian, 3 x kJointCount.
        float J[3][kJointCount];
        const float eps = 1e-4f;
        for (int j = 0; j < kJointCount; ++j) {
            const Joint jt = static_cast<Joint>(j);
            const float a0 = body.angle(id, jt);
            body.setAngle(id, jt, a0 + eps);
            const V3 pp = body.footPosition(id);
            body.setAngle(id, jt, a0 - eps);
            const V3 pm = body.footPosition(id);
            body.setAngle(id, jt, a0);
            const V3 d = (pp - pm) * (1.0f / (2.0f * eps));
            J[0][j] = d.x;
            J[1][j] = d.y;
            J[2][j] = d.z;
        }

        // (J J^T + lambda I) y = e, then dq = J^T y.
        float A[3][3];
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                float s = 0.0f;
                for (int j = 0; j < kJointCount; ++j) s += J[r][j] * J[c][j];
                A[r][c] = s + (r == c ? 1e-4f : 0.0f);
            }
        }
        const float bvec[3] = {e.x, e.y, e.z};
        float y[3];
        if (!solve3(A, bvec, y)) break;

        float dq[kJointCount];
        for (int j = 0; j < kJointCount; ++j) {
            dq[j] = J[0][j] * y[0] + J[1][j] * y[1] + J[2][j] * y[2];
        }

        // Null-space pull: the part of (rest - q) that the foot cannot see.
        float pull[kJointCount];
        for (int j = 0; j < kJointCount; ++j) {
            pull[j] = rest[j] - body.angle(id, static_cast<Joint>(j));
        }
        float Jp[3] = {0, 0, 0};
        for (int r = 0; r < 3; ++r) {
            for (int j = 0; j < kJointCount; ++j) Jp[r] += J[r][j] * pull[j];
        }
        float yp[3];
        if (solve3(A, Jp, yp)) {
            for (int j = 0; j < kJointCount; ++j) {
                const float proj =
                    J[0][j] * yp[0] + J[1][j] * yp[1] + J[2][j] * yp[2];
                dq[j] += 0.05f * (pull[j] - proj);
            }
        }

        for (int j = 0; j < kJointCount; ++j) {
            const Joint jt = static_cast<Joint>(j);
            body.setAngle(id, jt, body.angle(id, jt) + dq[j]);
        }
    }
    return err;
}

}  // namespace

V3 footTarget(const GaitSpec& spec, float phase) {
    const float u = wrap01(phase);
    const float duty = std::clamp(spec.duty, 0.1f, 0.95f);
    if (u < duty) {
        // Stance. The foot slides straight back along the ground, from
        // forward to aft, at the rest height. No lift and no lateral swing:
        // that is the whole point of planning in foot space.
        const float w = u / duty;
        return {spec.strideMm * (1.0f - 2.0f * w), 0.0f, 0.0f};
    }
    // Swing. Back to the front, over an arc that clears the ground.
    const float w = (u - duty) / (1.0f - duty);
    return {spec.strideMm * (-1.0f + 2.0f * w), 0.0f,
            spec.liftMm * std::sin(kPi * w)};
}

void GaitPlan::build(const GaitSpec& spec) {
    spec_ = spec;
    buildBasis();
    buildFootGrid();
    FlyBody rester;
    float rest[kLegCount][kJointCount];
    V3 restFoot[kLegCount];
    for (int l = 0; l < kLegCount; ++l) {
        const LegId id = static_cast<LegId>(l);
        for (int j = 0; j < kJointCount; ++j) {
            rest[l][j] = rester.angle(id, static_cast<Joint>(j));
        }
        restFoot[l] = rester.footPosition(id);
    }

    FlyBody work;
    for (int l = 0; l < kLegCount; ++l) {
        const LegId id = static_cast<LegId>(l);
        worstResidual_[l] = 0.0f;
        // Start each leg from its rest pose; every later sample warm starts
        // from the one before, which is what keeps the path continuous.
        for (int j = 0; j < kJointCount; ++j) {
            work.setAngle(id, static_cast<Joint>(j), rest[l][j]);
        }
        for (int k = 0; k < kSamples; ++k) {
            const float u = static_cast<float>(k) / kSamples;
            const V3 want = restFoot[l] + footTarget(spec, u);
            const float r = solveFootIK(work, id, want, rest[l]);
            worstResidual_[l] = std::max(worstResidual_[l], r);
            for (int j = 0; j < kJointCount; ++j) {
                table_[l][k][j] =
                    work.angle(id, static_cast<Joint>(j)) - rest[l][j];
            }
            foot_[l][k] = work.footPosition(id) - restFoot[l];
        }
    }
}

void GaitPlan::buildFootGrid() {
    FlyBody rester;
    float rest[kJointCount];
    FlyBody work;
    for (int l = 0; l < kLegCount; ++l) {
        const LegId id = static_cast<LegId>(l);
        for (int j = 0; j < kJointCount; ++j) {
            rest[j] = rester.angle(id, static_cast<Joint>(j));
        }
        const V3 restFoot = rester.footPosition(id);

        // Warm start along each column from the row below it, and each new
        // column from the previous column's base, so neighbouring grid cells
        // land on the same branch of a redundant solution. Without that the
        // interpolation crosses between postures and the leg snaps.
        float column[kJointCount];
        for (int j = 0; j < kJointCount; ++j) column[j] = rest[j];

        for (int ix = 0; ix < kFootNX; ++ix) {
            const float dx = kFootSpanX *
                             (2.0f * ix / (kFootNX - 1) - 1.0f);
            for (int j = 0; j < kJointCount; ++j) {
                work.setAngle(id, static_cast<Joint>(j), column[j]);
            }
            for (int iz = 0; iz < kFootNZ; ++iz) {
                const float dz = kFootSpanZ * iz / (kFootNZ - 1);
                const V3 want = restFoot + V3{dx, 0.0f, dz};
                const float r = solveFootIK(work, id, want, rest);
                worstResidual_[l] = std::max(worstResidual_[l], r);
                for (int j = 0; j < kJointCount; ++j) {
                    footGrid_[l][ix][iz][j] =
                        work.angle(id, static_cast<Joint>(j)) - rest[j];
                }
                if (iz == 0) {
                    for (int j = 0; j < kJointCount; ++j) {
                        column[j] = work.angle(id, static_cast<Joint>(j));
                    }
                }
            }
        }
    }
}

void GaitPlan::jointsForFoot(int leg, float dx, float dz,
                             float out[kJointCount]) const {
    const float u = std::clamp((dx + kFootSpanX) / (2.0f * kFootSpanX),
                               0.0f, 1.0f) * (kFootNX - 1);
    const float v = std::clamp(dz / kFootSpanZ, 0.0f, 1.0f) * (kFootNZ - 1);
    const int i0 = std::min(static_cast<int>(u), kFootNX - 2);
    const int j0 = std::min(static_cast<int>(v), kFootNZ - 2);
    const float fu = u - static_cast<float>(i0);
    const float fv = v - static_cast<float>(j0);
    for (int j = 0; j < kJointCount; ++j) {
        const float a = footGrid_[leg][i0][j0][j] * (1.0f - fv) +
                        footGrid_[leg][i0][j0 + 1][j] * fv;
        const float b = footGrid_[leg][i0 + 1][j0][j] * (1.0f - fv) +
                        footGrid_[leg][i0 + 1][j0 + 1][j] * fv;
        out[j] = a * (1.0f - fu) + b * fu;
    }
}

void GaitPlan::buildBasis() {
    FlyBody body;
    for (int l = 0; l < kLegCount; ++l) {
        const LegId id = static_cast<LegId>(l);
        float J[3][kJointCount];
        const float eps = 1e-4f;
        for (int j = 0; j < kJointCount; ++j) {
            const Joint jt = static_cast<Joint>(j);
            const float a0 = body.angle(id, jt);
            body.setAngle(id, jt, a0 + eps);
            const V3 pp = body.footPosition(id);
            body.setAngle(id, jt, a0 - eps);
            const V3 pm = body.footPosition(id);
            body.setAngle(id, jt, a0);
            const V3 d = (pp - pm) * (1.0f / (2.0f * eps));
            J[0][j] = d.x;
            J[1][j] = d.y;
            J[2][j] = d.z;
        }
        float A[3][3];
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                float sum = 0.0f;
                for (int j = 0; j < kJointCount; ++j) sum += J[r][j] * J[c][j];
                A[r][c] = sum + (r == c ? 1e-9f : 0.0f);
            }
        }
        for (int axis = 0; axis < 3; ++axis) {
            float want[3] = {0, 0, 0};
            want[axis] = 1.0f;
            float y[3];
            if (!solve3(A, want, y)) continue;
            for (int j = 0; j < kJointCount; ++j) {
                basis_[l][axis][j] =
                    J[0][j] * y[0] + J[1][j] * y[1] + J[2][j] * y[2];
            }
        }
    }
}

void GaitPlan::targets(int leg, float phase, float out[kJointCount]) const {
    const float u = wrap01(phase) * kSamples;
    const int i0 = static_cast<int>(u);
    const int i1 = (i0 + 1) % kSamples;
    const float f = u - static_cast<float>(i0);
    for (int j = 0; j < kJointCount; ++j) {
        out[j] = table_[leg][i0][j] * (1.0f - f) + table_[leg][i1][j] * f;
    }
}

V3 GaitPlan::plannedFoot(int leg, float phase) const {
    const float u = wrap01(phase) * kSamples;
    const int i0 = static_cast<int>(u);
    const int i1 = (i0 + 1) % kSamples;
    const float f = u - static_cast<float>(i0);
    return foot_[leg][i0] * (1.0f - f) + foot_[leg][i1] * f;
}

}  // namespace fly
