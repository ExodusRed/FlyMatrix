#include "body/FlyModel.h"

#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace fly {
namespace {

class Reader {
public:
    explicit Reader(std::vector<char> data) : d_(std::move(data)) {}
    template <typename T>
    T get() {
        need(sizeof(T));
        T v;
        std::memcpy(&v, d_.data() + o_, sizeof(T));
        o_ += sizeof(T);
        return v;
    }
    std::string str() {
        const auto n = get<std::uint8_t>();
        need(n);
        std::string s(d_.data() + o_, n);
        o_ += n;
        return s;
    }
    V3 v3() { const float x = get<float>(), y = get<float>(), z = get<float>(); return {x, y, z}; }
    const char* take(std::size_t n) {
        need(n);
        const char* p = d_.data() + o_;
        o_ += n;
        return p;
    }

private:
    void need(std::size_t n) const {
        if (o_ + n > d_.size()) throw std::runtime_error("fly model file is truncated");
    }
    std::vector<char> d_;
    std::size_t o_ = 0;
};

}  // namespace

FlyModel FlyModel::load(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open fly model " + path);
    Reader r(std::vector<char>(std::istreambuf_iterator<char>(f), {}));
    const char* magic = r.take(4);
    if (std::memcmp(magic, "FLYM", 4) != 0 || r.get<std::uint32_t>() != 2) {
        throw std::runtime_error(path + " is not a FlyMatrix model file");
    }
    FlyModel m;
    const auto nMat = r.get<std::uint32_t>();
    for (std::uint32_t i = 0; i < nMat; ++i) {
        std::array<float, 4> c{};
        for (float& v : c) v = r.get<float>();
        m.materials_.push_back(c);
    }
    const auto nBodies = r.get<std::uint32_t>();
    for (std::uint32_t i = 0; i < nBodies; ++i) {
        Body b;
        b.name = r.str();
        b.parent = r.get<std::int32_t>();
        b.pos = r.v3();
        const float w = r.get<float>(), x = r.get<float>(), y = r.get<float>(), z = r.get<float>();
        b.quat = Quat{w, x, y, z}.normalised();
        const auto nJ = r.get<std::uint32_t>();
        for (std::uint32_t k = 0; k < nJ; ++k) {
            Joint j;
            j.name = r.str();
            j.axis = normalise(r.v3());
            j.lo = r.get<float>();
            j.hi = r.get<float>();
            j.ref = r.get<float>();
            j.body = static_cast<int>(i);
            b.joints.push_back(static_cast<int>(m.joints_.size()));
            m.joints_.push_back(std::move(j));
        }
        m.bodies_.push_back(std::move(b));
    }
    const auto nMeshes = r.get<std::uint32_t>();
    for (std::uint32_t i = 0; i < nMeshes; ++i) {
        MeshData md;
        md.body = r.get<std::int32_t>();
        md.material = static_cast<int>(r.get<std::uint32_t>());
        const V3 lo = r.v3(), hi = r.v3();
        const auto nv = r.get<std::uint32_t>();
        const auto ni = r.get<std::uint32_t>();
        const auto* q = reinterpret_cast<const std::uint8_t*>(r.take(std::size_t{nv} * 6));
        md.positions.resize(nv);
        for (std::uint32_t v = 0; v < nv; ++v) {
            std::uint16_t c[3];
            std::memcpy(c, q + 6 * v, 6);
            md.positions[v] = {lo.x + (hi.x - lo.x) * c[0] / 65535.0f,
                               lo.y + (hi.y - lo.y) * c[1] / 65535.0f,
                               lo.z + (hi.z - lo.z) * c[2] / 65535.0f};
        }
        // The writer pads the normal block and the 16-bit index block each to
        // a multiple of 4 bytes (not to an absolute file offset), and puts no
        // padding after the positions.
        const std::size_t normalBytes = (std::size_t{nv} * 3 + 3) / 4 * 4;
        const auto* nb = reinterpret_cast<const std::int8_t*>(r.take(normalBytes));
        md.normals.resize(nv);
        for (std::uint32_t v = 0; v < nv; ++v) {
            md.normals[v] = normalise(V3{nb[3 * v] / 127.0f, nb[3 * v + 1] / 127.0f, nb[3 * v + 2] / 127.0f});
        }
        md.indices.resize(ni);
        if (nv < 65536) {
            const auto* ib = r.take((std::size_t{ni} + 1) / 2 * 4);
            for (std::uint32_t k = 0; k < ni; ++k) {
                std::uint16_t v;
                std::memcpy(&v, ib + 2 * k, 2);
                md.indices[k] = v;
            }
        } else {
            const auto* ib = r.take(std::size_t{ni} * 4);
            std::memcpy(md.indices.data(), ib, std::size_t{ni} * 4);
        }
        m.meshes_.push_back(std::move(md));
    }
    return m;
}

int FlyModel::bodyIndex(const std::string& name) const {
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (bodies_[i].name == name) return static_cast<int>(i);
    }
    return -1;
}

int FlyModel::jointIndex(const std::string& name) const {
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        if (joints_[i].name == name) return static_cast<int>(i);
    }
    return -1;
}

void FlyModel::pose(const Transform& root, const std::vector<float>& q, std::vector<Transform>& out) const {
    out.resize(bodies_.size());
    // Bodies are stored parent-first (the file is written depth-first), so a
    // single forward pass sees every parent before its children.
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        const Body& b = bodies_[i];
        const Transform& parent = b.parent >= 0 ? out[static_cast<std::size_t>(b.parent)] : root;
        Transform t = parent * Transform{b.pos, b.quat, 1.0f};
        for (const int j : b.joints) {
            const float angle = j < static_cast<int>(q.size()) ? q[static_cast<std::size_t>(j)] : 0.0f;
            if (angle != 0.0f) {
                t.rotation = (t.rotation * Quat::axisAngle(joints_[static_cast<std::size_t>(j)].axis, angle)).normalised();
            }
        }
        out[i] = t;
    }
}

}  // namespace fly
