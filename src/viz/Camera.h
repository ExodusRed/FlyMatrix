#pragma once

// Just enough linear algebra and camera for a point cloud. Pulling in GLM for
// one matrix multiply and an orbit control would outweigh what it saves.

#include <cmath>

namespace fly::viz {

struct V3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    V3 operator+(const V3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(const V3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator*(float s) const { return {x * s, y * s, z * s}; }
};

inline V3 cross(const V3& a, const V3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 normalise(const V3& v) {
    const float len = std::sqrt(dot(v, v));
    return len > 1e-8f ? v * (1.0f / len) : V3{0, 0, 1};
}

// Column-major 4x4, matching what glUniformMatrix4fv expects with transpose
// set to GL_FALSE.
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
        r.m[0] = s.x; r.m[4] = s.y; r.m[8]  = s.z;
        r.m[1] = u.x; r.m[5] = u.y; r.m[9]  = u.z;
        r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
        r.m[3] = 0; r.m[7] = 0; r.m[11] = 0;
        r.m[12] = -dot(s, eye);
        r.m[13] = -dot(u, eye);
        r.m[14] = dot(f, eye);
        r.m[15] = 1.0f;
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

// Orbit camera: yaw/pitch around a target, with distance and panning.
class OrbitCamera {
public:
    V3 target;
    float distance = 1000.0f;
    float yaw = 0.0f;      // radians; 0 looks along +X
    float pitch = 0.18f;
    float fovy = 0.9f;

    void orbit(float dYaw, float dPitch) {
        yaw += dYaw;
        // Stop just short of the poles so `up` never becomes parallel to the
        // view direction and lookAt degenerates.
        constexpr float kLimit = 1.5533f;  // ~89 degrees
        pitch = std::fmin(kLimit, std::fmax(-kLimit, pitch + dPitch));
    }

    void zoom(float factor) {
        distance = std::fmin(20000.0f, std::fmax(5.0f, distance * factor));
    }

    // Pan in the camera's own screen plane, scaled so a drag moves roughly the
    // same screen distance regardless of zoom.
    void pan(float dx, float dy) {
        const V3 f = forward();
        const V3 right = normalise(cross(f, {0, 1, 0}));
        const V3 up = cross(right, f);
        const float scale = distance * 0.0015f;
        target = target + right * (-dx * scale) + up * (dy * scale);
    }

    // Y-up. The connectome's long axis is Z (brain to nerve cord, ~1 mm);
    // keeping Y as up lays that axis across a widescreen window instead of
    // standing it on end.
    V3 forward() const {
        const float cp = std::cos(pitch);
        return {cp * std::cos(yaw), std::sin(pitch), cp * std::sin(yaw)};
    }

    V3 eye() const { return target - forward() * distance; }

    M4 viewProjection(float aspect) const {
        // Near/far track distance so the depth range stays usable whether you
        // are inspecting one neuron or looking at the whole nervous system.
        const float zNear = std::fmax(0.5f, distance * 0.002f);
        const float zFar = distance * 8.0f + 4000.0f;
        return M4::perspective(fovy, aspect, zNear, zFar) *
               M4::lookAt(eye(), target, {0, 1, 0});
    }
};

}  // namespace fly::viz
