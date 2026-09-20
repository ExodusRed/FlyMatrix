#pragma once

// Vectors, quaternions and 4x4 transforms.
//
// Small enough that a dependency would cost more than it saves, and keeping it
// here means the rotation conventions are written down in one place:
//
//   - Right-handed, column-major matrices, matching what glUniformMatrix4fv
//     expects with transpose = GL_FALSE.
//   - Quaternions are (w, x, y, z), unit length, rotating a vector as
//     q * v * conj(q).
//   - Transforms compose parent-first: world = parent * local.

#include <cmath>

namespace fly {

struct V3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    V3 operator+(const V3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(const V3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator*(float s) const { return {x * s, y * s, z * s}; }
    V3 operator-() const { return {-x, -y, -z}; }
    V3& operator+=(const V3& o) { x += o.x; y += o.y; z += o.z; return *this; }
};

inline float dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(const V3& a, const V3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float length(const V3& v) { return std::sqrt(dot(v, v)); }
inline V3 normalise(const V3& v) {
    const float len = length(v);
    return len > 1e-8f ? v * (1.0f / len) : V3{0, 0, 1};
}

struct Quat {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;

    static Quat axisAngle(const V3& axis, float radians) {
        const V3 a = normalise(axis);
        const float h = radians * 0.5f;
        const float s = std::sin(h);
        return {std::cos(h), a.x * s, a.y * s, a.z * s};
    }

    Quat operator*(const Quat& o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z,
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w};
    }

    V3 rotate(const V3& v) const {
        // v + 2w(q x v) + 2(q x (q x v)), the standard expansion that avoids
        // building a matrix for a single vector.
        const V3 q{x, y, z};
        const V3 t = cross(q, v) * 2.0f;
        return v + t * w + cross(q, t);
    }

    Quat normalised() const {
        const float n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n < 1e-8f) return {};
        const float inv = 1.0f / n;
        return {w * inv, x * inv, y * inv, z * inv};
    }
};

// Rigid transform: rotate, scale uniformly, then translate.
struct Transform {
    V3 position;
    Quat rotation;
    float scale = 1.0f;

    V3 apply(const V3& v) const { return rotation.rotate(v * scale) + position; }

    Transform operator*(const Transform& child) const {
        return {apply(child.position), (rotation * child.rotation).normalised(),
                scale * child.scale};
    }
};

struct M4 {
    float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

    static M4 perspective(float fovyRad, float aspect, float zNear, float zFar) {
        const float f = 1.0f / std::tan(fovyRad * 0.5f);
        M4 r;
        for (float& v : r.m) v = 0.0f;
        r.m[0] = f / aspect;
        r.m[5] = f;
        r.m[10] = (zFar + zNear) / (zNear - zFar);
        r.m[11] = -1.0f;
        r.m[14] = (2.0f * zFar * zNear) / (zNear - zFar);
        return r;
    }

    static M4 lookAt(const V3& eye, const V3& centre, const V3& up) {
        const V3 f = normalise(centre - eye);
        const V3 s = normalise(cross(f, up));
        const V3 u = cross(s, f);
        M4 r;
        r.m[0] = s.x;  r.m[4] = s.y;  r.m[8]  = s.z;
        r.m[1] = u.x;  r.m[5] = u.y;  r.m[9]  = u.z;
        r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
        r.m[3] = 0;    r.m[7] = 0;    r.m[11] = 0;
        r.m[12] = -dot(s, eye);
        r.m[13] = -dot(u, eye);
        r.m[14] = dot(f, eye);
        r.m[15] = 1.0f;
        return r;
    }

    // Non-uniform scale, for stretching a unit cylinder into a leg segment.
    static M4 fromTransform(const Transform& t, const V3& stretch = {1, 1, 1}) {
        const Quat& q = t.rotation;
        const float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
        const float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

        const float sx = t.scale * stretch.x;
        const float sy = t.scale * stretch.y;
        const float sz = t.scale * stretch.z;

        M4 r;
        r.m[0]  = (1 - 2 * (yy + zz)) * sx;
        r.m[1]  = (2 * (xy + wz)) * sx;
        r.m[2]  = (2 * (xz - wy)) * sx;
        r.m[3]  = 0;
        r.m[4]  = (2 * (xy - wz)) * sy;
        r.m[5]  = (1 - 2 * (xx + zz)) * sy;
        r.m[6]  = (2 * (yz + wx)) * sy;
        r.m[7]  = 0;
        r.m[8]  = (2 * (xz + wy)) * sz;
        r.m[9]  = (2 * (yz - wx)) * sz;
        r.m[10] = (1 - 2 * (xx + yy)) * sz;
        r.m[11] = 0;
        r.m[12] = t.position.x;
        r.m[13] = t.position.y;
        r.m[14] = t.position.z;
        r.m[15] = 1;
        return r;
    }

    M4 operator*(const M4& o) const {
        M4 r;
        for (int c = 0; c < 4; ++c) {
            for (int row = 0; row < 4; ++row) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k) sum += m[k * 4 + row] * o.m[c * 4 + k];
                r.m[c * 4 + row] = sum;
            }
        }
        return r;
    }
};

}  // namespace fly
