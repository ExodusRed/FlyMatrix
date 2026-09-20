#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace fly {

// Cell type and superclass labels, loaded from the cns_names.tsv sidecar that
// pack_cns.py writes alongside cns.bin.
//
// The binary stays fixed-width so it can be read in one go; the variable-length
// strings live here instead.
class NeuronNames {
public:
    // Returns an empty-but-valid instance if the file is missing, so callers
    // that only want labels for display can carry on unlabelled. `ok` reports
    // which happened.
    static NeuronNames load(const std::string& path, std::uint32_t n, bool* ok = nullptr);

    const std::string& type(std::uint32_t i) const { return type_[i]; }
    const std::string& superclass(std::uint32_t i) const { return superclass_[i]; }

    // Type name, or "(untyped)" for the ~7% of neurons with no assigned type.
    std::string label(std::uint32_t i) const;

    // Every neuron of a given cell type; empty if the name is unknown.
    std::span<const std::uint32_t> ofType(const std::string& t) const;

    const std::unordered_map<std::string, std::vector<std::uint32_t>>& byType() const {
        return byType_;
    }

private:
    std::vector<std::string> type_;
    std::vector<std::string> superclass_;
    std::unordered_map<std::string, std::vector<std::uint32_t>> byType_;
};

}  // namespace fly
