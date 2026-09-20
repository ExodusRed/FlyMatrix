#pragma once

#include <cstdint>
#include <vector>

#include "engine/Math.h"

namespace fly {

// An indexed triangle mesh on the GPU, with positions and normals.
//
// Geometry here is generated rather than loaded: a unit cylinder and a unit
// sphere, reused for every body part and stretched into place by the model
// matrix. Thirty-odd draw calls per frame is nothing, and it avoids needing an
// asset pipeline before there is anything to put in it.
class Mesh {
public:
    struct Vertex {
        V3 position;
        V3 normal;
    };

    // A cylinder of radius 1 along -Z, from z = 0 to z = -1, capped at both
    // ends. Leg segments run down this axis, matching FlyBody's convention.
    static Mesh cylinder(int radialSegments = 12);
    // A sphere of radius 1, for joints and the body.
    static Mesh sphere(int rings = 12, int sectors = 16);

    // Upload to the GPU. Requires a current GL context and fly::gl::load().
    void upload();
    void draw() const;
    void destroy();

    std::size_t triangleCount() const { return indices_.size() / 3; }

private:
    std::vector<Vertex> vertices_;
    std::vector<std::uint32_t> indices_;
    unsigned int vao_ = 0, vbo_ = 0, ebo_ = 0;
};

}  // namespace fly
