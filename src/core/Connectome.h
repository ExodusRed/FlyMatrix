#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace fly {

// Neurotransmitter identity, indices matching the nt_table in cns_meta.json.
enum class Nt : std::uint8_t {
    Unknown = 0, Acetylcholine, Glutamate, Gaba, Histamine,
    Dopamine, Serotonin, Octopamine, Unclear,
};

struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;
};

// The connectome as a compressed sparse row graph: neuron i's outgoing
// connections are col[rowStart[i] .. rowStart[i+1]) with matching weights.
//
// Loaded from the single binary that tools/pack_cns.py produces. Arrays are
// views into one owned buffer, so construction is a single read with no
// per-neuron allocation.
class Connectome {
public:
    // Throws std::runtime_error if the file is missing, truncated, or was
    // written by a different packer version.
    static Connectome load(const std::string& binPath);

    std::uint32_t neuronCount() const { return nNeurons_; }
    std::uint64_t edgeCount() const { return nEdges_; }
    std::uint32_t minWeight() const { return minWeight_; }

    std::span<const std::int64_t> bodyIds() const { return bodyIds_; }
    std::span<const Vec3> positions() const { return positions_; }
    std::span<const std::uint8_t> ntCodes() const { return ntCodes_; }
    // +1 excitatory, -1 inhibitory, 0 modulatory or unresolved.
    std::span<const std::int8_t> signs() const { return signs_; }
    // bit0 set when the position came from a real soma rather than being
    // inferred from graph neighbours.
    std::span<const std::uint8_t> flags() const { return flags_; }

    std::span<const std::uint64_t> rowStart() const { return rowStart_; }
    std::span<const std::uint32_t> col() const { return col_; }
    std::span<const std::uint16_t> weight() const { return weight_; }

    // Outgoing connections of one neuron.
    std::span<const std::uint32_t> targetsOf(std::uint32_t i) const {
        return col_.subspan(rowStart_[i], rowStart_[i + 1] - rowStart_[i]);
    }
    std::span<const std::uint16_t> weightsOf(std::uint32_t i) const {
        return weight_.subspan(rowStart_[i], rowStart_[i + 1] - rowStart_[i]);
    }

    const Vec3& bboxMin() const { return bboxMin_; }
    const Vec3& bboxMax() const { return bboxMax_; }

    // Dense index for a neuPrint bodyId, or UINT32_MAX if absent.
    std::uint32_t indexOf(std::int64_t bodyId) const;

private:
    std::vector<std::byte> buffer_;

    std::uint32_t nNeurons_ = 0;
    std::uint64_t nEdges_ = 0;
    std::uint32_t minWeight_ = 0;
    Vec3 bboxMin_, bboxMax_;

    std::span<const std::int64_t> bodyIds_;
    std::span<const Vec3> positions_;
    std::span<const std::uint8_t> ntCodes_;
    std::span<const std::int8_t> signs_;
    std::span<const std::uint8_t> flags_;
    std::span<const std::uint64_t> rowStart_;
    std::span<const std::uint32_t> col_;
    std::span<const std::uint16_t> weight_;
};

}  // namespace fly
