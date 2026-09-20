#pragma once

#include <cmath>

#include "engine/Math.h"

namespace fly::viz {

// The camera works in the engine vector and matrix types; these aliases keep
// existing fly::viz::V3 and fly::viz::M4 spellings valid.
using fly::cross;
using fly::dot;
using fly::M4;
using fly::normalise;
using fly::V3;

// Orbit camera: yaw/pitch around a target, with distance and panning.
class OrbitCamera {
public:
    V3 target;
    float distance = 1000.0f;
    float yaw = 0.0f;      // radians; 0 looks along +X
    float pitch = 0.18f;
    float fovy = 0.9f;
    // The connectome's long axis is Z, so that view wants Y as up. A
    // standing body wants Z. Both conventions are needed, so pick one.
    bool zUp = false;

    void orbit(float dYaw, float dPitch) {
        yaw += dYaw;
        // Stop just short of the poles so `up` never becomes parallel to the
        // view direction and lookAt degenerates.
        constexpr float kLimit = 1.5533f;  // ~89 degrees
        pitch = std::fmin(kLimit, std::fmax(-kLimit, pitch + dPitch));
    }

    void zoom(float factor) {
        distance = std::fmin(20000.0f, std::fmax(0.05f, distance * factor));
    }

    // Pan in the camera's own screen plane, scaled so a drag moves roughly the
    // same screen distance regardless of zoom.
    void pan(float dx, float dy) {
        const V3 f = forward();
        const V3 right = normalise(cross(f, upVector()));
        const V3 up = cross(right, f);
        const float scale = distance * 0.0015f;
        target = target + right * (-dx * scale) + up * (dy * scale);
    }

    V3 upVector() const { return zUp ? V3{0, 0, 1} : V3{0, 1, 0}; }

    V3 forward() const {
        const float cp = std::cos(pitch);
        const float sp = std::sin(pitch);
        if (zUp) return {cp * std::cos(yaw), cp * std::sin(yaw), sp};
        return {cp * std::cos(yaw), sp, cp * std::sin(yaw)};
    }

    V3 eye() const { return target - forward() * distance; }

    M4 viewProjection(float aspect) const {
        // Near/far track distance so the depth range stays usable whether you
        // are inspecting one neuron or looking at the whole nervous system.
        const float zNear = std::fmax(0.002f, distance * 0.002f);
        const float zFar = distance * 8.0f + 4000.0f;
        return M4::perspective(fovy, aspect, zNear, zFar) *
               M4::lookAt(eye(), target, upVector());
    }
};

}  // namespace fly::viz
