#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "engine/Math.h"

namespace fly {

// The fruit fly body model from flybody (Vaxenburg, Siwanowicz, Merel et al.,
// Nature 2025; Google DeepMind and HHMI Janelia Research Campus; Apache-2.0),
// converted by tools/flymodel/import_flybody.mjs: millimetres, meshes baked
// into their bodies' frames and simplified for the web. See
// tools/flymodel/NOTICE.txt for attribution and the list of changes.
//
// This is geometry and kinematics only -- a tree of bodies connected by
// hinge joints, plus the meshes to draw them. No GL here, so native tools can
// use the kinematics without a window.
class FlyModel {
public:
    struct Joint {
        std::string name;
        V3 axis;           // in the owning body's frame
        float lo = 0.0f, hi = 0.0f;
        float ref = 0.0f;  // spring rest angle (e.g. the folded wing), 0 if none
        int body = -1;
    };
    struct Body {
        std::string name;
        int parent = -1;
        V3 pos;            // relative to parent, mm
        Quat quat;
        std::vector<int> joints;  // indices into joints()
    };
    struct MeshData {
        int body = -1;
        int material = 0;
        std::vector<V3> positions, normals;  // body frame, mm
        std::vector<std::uint32_t> indices;
    };

    // Throws std::runtime_error if the file is missing or malformed.
    static FlyModel load(const std::string& path);

    const std::vector<Body>& bodies() const { return bodies_; }
    const std::vector<Joint>& joints() const { return joints_; }
    const std::vector<MeshData>& meshes() const { return meshes_; }
    const std::vector<std::array<float, 4>>& materials() const { return materials_; }

    int bodyIndex(const std::string& name) const;   // -1 if absent
    int jointIndex(const std::string& name) const;  // -1 if absent

    // World transform of every body for joint angles `q` (one per joint, in
    // joints() order), with the root body placed by `root`. Each body is its
    // parent's frame, moved by its pos/quat, then rotated by its joints in
    // order -- the MuJoCo convention the model was authored in.
    void pose(const Transform& root, const std::vector<float>& q, std::vector<Transform>& out) const;

private:
    std::vector<std::array<float, 4>> materials_;
    std::vector<Body> bodies_;
    std::vector<Joint> joints_;
    std::vector<MeshData> meshes_;
};

}  // namespace fly
