#include "Connectome.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace fly {
namespace {

constexpr char kMagic[8] = {'F', 'L', 'Y', 'C', 'N', 'S', '0', '1'};
constexpr std::uint32_t kVersion = 2;

// Header is: magic[8], version u32, nNeurons u32, nEdges u64, minWeight u32,
// pad u32, bbox f32[6].
constexpr std::size_t kHeaderBytes = 8 + 4 + 4 + 8 + 4 + 4 + 24;

// Carve a typed view out of the buffer, advancing the cursor.
template <typename T>
std::span<const T> take(const std::vector<std::byte>& buf, std::size_t& off,
                        std::size_t count, const char* what) {
    const std::size_t bytes = count * sizeof(T);
    if (off + bytes > buf.size()) {
        throw std::runtime_error(std::string("cns.bin truncated while reading ") + what);
    }
    // The packer writes every array at its natural alignment in sequence, but
    // check rather than trust: a misaligned reinterpret_cast is UB.
    if (off % alignof(T) != 0) {
        throw std::runtime_error(std::string("cns.bin misaligned at ") + what);
    }
    const auto* p = reinterpret_cast<const T*>(buf.data() + off);
    off += bytes;
    return std::span<const T>(p, count);
}

}  // namespace

Connectome Connectome::load(const std::string& binPath) {
    std::ifstream f(binPath, std::ios::binary | std::ios::ate);
    if (!f) {
        throw std::runtime_error("cannot open " + binPath +
                                 " -- run: python tools/pack_cns.py");
    }
    const auto size = static_cast<std::size_t>(f.tellg());
    if (size < kHeaderBytes) throw std::runtime_error(binPath + " is too small to be valid");
    f.seekg(0);

    Connectome c;
    c.buffer_.resize(size);
    f.read(reinterpret_cast<char*>(c.buffer_.data()), static_cast<std::streamsize>(size));
    if (!f) throw std::runtime_error("short read on " + binPath);

    std::size_t off = 0;
    if (std::memcmp(c.buffer_.data(), kMagic, 8) != 0) {
        throw std::runtime_error(binPath + " has a bad magic number");
    }
    off += 8;

    std::uint32_t version = 0, pad = 0;
    std::memcpy(&version, c.buffer_.data() + off, 4); off += 4;
    if (version != kVersion) {
        throw std::runtime_error(binPath + " is packer version " + std::to_string(version) +
                                 ", this build expects " + std::to_string(kVersion) +
                                 " -- rerun tools/pack_cns.py");
    }
    std::memcpy(&c.nNeurons_, c.buffer_.data() + off, 4); off += 4;
    std::memcpy(&c.nEdges_,   c.buffer_.data() + off, 8); off += 8;
    std::memcpy(&c.minWeight_, c.buffer_.data() + off, 4); off += 4;
    std::memcpy(&pad, c.buffer_.data() + off, 4); off += 4;

    float bbox[6];
    std::memcpy(bbox, c.buffer_.data() + off, sizeof(bbox)); off += sizeof(bbox);
    c.bboxMin_ = {bbox[0], bbox[1], bbox[2]};
    c.bboxMax_ = {bbox[3], bbox[4], bbox[5]};

    const std::size_t n = c.nNeurons_;
    const std::size_t e = static_cast<std::size_t>(c.nEdges_);

    c.bodyIds_   = take<std::int64_t>(c.buffer_, off, n, "body_id");
    c.positions_ = take<Vec3>(c.buffer_, off, n, "pos");
    c.sizeRel_   = take<float>(c.buffer_, off, n, "size_rel");
    c.ntCodes_   = take<std::uint8_t>(c.buffer_, off, n, "nt_code");
    c.signs_     = take<std::int8_t>(c.buffer_, off, n, "sign");
    c.flags_     = take<std::uint8_t>(c.buffer_, off, n, "flags");
    take<std::uint8_t>(c.buffer_, off, n, "pad");
    c.rowStart_  = take<std::uint64_t>(c.buffer_, off, n + 1, "row_start");
    c.col_       = take<std::uint32_t>(c.buffer_, off, e, "col");
    c.weight_    = take<std::uint16_t>(c.buffer_, off, e, "weight");

    if (c.rowStart_[n] != c.nEdges_) {
        throw std::runtime_error("cns.bin CSR is inconsistent: row_start ends at " +
                                 std::to_string(c.rowStart_[n]) + " but header says " +
                                 std::to_string(c.nEdges_) + " edges");
    }
    return c;
}

std::uint32_t Connectome::indexOf(std::int64_t bodyId) const {
    // bodyIds_ is sorted ascending by the packer.
    const auto it = std::lower_bound(bodyIds_.begin(), bodyIds_.end(), bodyId);
    if (it == bodyIds_.end() || *it != bodyId) return UINT32_MAX;
    return static_cast<std::uint32_t>(it - bodyIds_.begin());
}

}  // namespace fly
