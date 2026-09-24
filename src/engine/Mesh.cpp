#include "Mesh.h"

#include <cmath>

#include "viz/GL.h"

namespace fly {

namespace {
constexpr float kPi = 3.14159265358979323846f;
}

Mesh Mesh::cylinder(int radialSegments) {
    Mesh m;
    const int n = radialSegments < 3 ? 3 : radialSegments;

    // Side wall: two rings of vertices, normals pointing outward.
    for (int ring = 0; ring < 2; ++ring) {
        const float z = ring == 0 ? 0.0f : -1.0f;
        for (int i = 0; i < n; ++i) {
            const float a = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(n);
            const float c = std::cos(a), s = std::sin(a);
            m.vertices_.push_back({{c, s, z}, {c, s, 0.0f}});
        }
    }
    for (int i = 0; i < n; ++i) {
        const std::uint32_t a = static_cast<std::uint32_t>(i);
        const std::uint32_t b = static_cast<std::uint32_t>((i + 1) % n);
        const std::uint32_t c = a + static_cast<std::uint32_t>(n);
        const std::uint32_t d = b + static_cast<std::uint32_t>(n);
        m.indices_.insert(m.indices_.end(), {a, c, b, b, c, d});
    }

    // Caps need their own vertices: the normal is along the axis, not outward,
    // so they cannot share the wall's ring.
    for (int cap = 0; cap < 2; ++cap) {
        const float z = cap == 0 ? 0.0f : -1.0f;
        const V3 normal = cap == 0 ? V3{0, 0, 1} : V3{0, 0, -1};
        const auto centre = static_cast<std::uint32_t>(m.vertices_.size());
        m.vertices_.push_back({{0, 0, z}, normal});
        for (int i = 0; i < n; ++i) {
            const float a = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(n);
            m.vertices_.push_back({{std::cos(a), std::sin(a), z}, normal});
        }
        for (int i = 0; i < n; ++i) {
            const std::uint32_t a = centre + 1 + static_cast<std::uint32_t>(i);
            const std::uint32_t b = centre + 1 + static_cast<std::uint32_t>((i + 1) % n);
            if (cap == 0) m.indices_.insert(m.indices_.end(), {centre, b, a});
            else m.indices_.insert(m.indices_.end(), {centre, a, b});
        }
    }
    return m;
}

Mesh Mesh::box() {
    // Six faces, four vertices each, so every face gets its own flat normal.
    // Sharing corner vertices between faces would average the normals and a
    // cube would shade like a ball.
    Mesh m;
    const V3 normals[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
                           {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (const V3& n : normals) {
        // Two directions spanning the face.
        const V3 u = (std::fabs(n.x) > 0.5f) ? V3{0, 1, 0} : V3{1, 0, 0};
        const V3 v = cross(n, u);
        const auto base = static_cast<std::uint32_t>(m.vertices_.size());
        m.vertices_.push_back({n - u - v, n});
        m.vertices_.push_back({n + u - v, n});
        m.vertices_.push_back({n + u + v, n});
        m.vertices_.push_back({n - u + v, n});
        for (const std::uint32_t k : {0u, 1u, 2u, 0u, 2u, 3u}) {
            m.indices_.push_back(base + k);
        }
    }
    return m;
}

Mesh Mesh::sphere(int rings, int sectors) {
    Mesh m;
    const int R = rings < 2 ? 2 : rings;
    const int S = sectors < 3 ? 3 : sectors;

    for (int r = 0; r <= R; ++r) {
        const float phi = kPi * static_cast<float>(r) / static_cast<float>(R);
        const float sp = std::sin(phi), cp = std::cos(phi);
        for (int s = 0; s <= S; ++s) {
            const float theta = 2.0f * kPi * static_cast<float>(s) / static_cast<float>(S);
            const V3 p{sp * std::cos(theta), sp * std::sin(theta), cp};
            m.vertices_.push_back({p, p});  // unit sphere: position is the normal
        }
    }
    for (int r = 0; r < R; ++r) {
        for (int s = 0; s < S; ++s) {
            const auto a = static_cast<std::uint32_t>(r * (S + 1) + s);
            const auto b = static_cast<std::uint32_t>(a + S + 1);
            m.indices_.insert(m.indices_.end(), {a, b, a + 1, a + 1, b, b + 1});
        }
    }
    return m;
}

void Mesh::upload() {
    gl::glGenVertexArrays(1, &vao_);
    gl::glBindVertexArray(vao_);

    gl::glGenBuffers(1, &vbo_);
    gl::glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    gl::glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(vertices_.size() * sizeof(Vertex)),
                     vertices_.data(), GL_STATIC_DRAW);

    gl::glGenBuffers(1, &ebo_);
    gl::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    gl::glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(indices_.size() * sizeof(std::uint32_t)),
                     indices_.data(), GL_STATIC_DRAW);

    gl::glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    gl::glEnableVertexAttribArray(0);
    gl::glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<const void*>(sizeof(V3)));
    gl::glEnableVertexAttribArray(1);

    gl::glBindVertexArray(0);
}

void Mesh::draw() const {
    if (!vao_) return;
    gl::glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()),
                   GL_UNSIGNED_INT, nullptr);
}

void Mesh::destroy() {
    if (ebo_) gl::glDeleteBuffers(1, &ebo_);
    if (vbo_) gl::glDeleteBuffers(1, &vbo_);
    if (vao_) gl::glDeleteVertexArrays(1, &vao_);
    ebo_ = vbo_ = vao_ = 0;
}

}  // namespace fly
