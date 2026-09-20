#include "NeuronNames.h"

#include <charconv>
#include <fstream>

namespace fly {

NeuronNames NeuronNames::load(const std::string& path, std::uint32_t n, bool* ok) {
    NeuronNames nm;
    nm.type_.resize(n);
    nm.superclass_.resize(n);

    std::ifstream f(path);
    if (!f) {
        if (ok) *ok = false;
        return nm;
    }

    std::string line;
    std::getline(f, line);  // header
    while (std::getline(f, line)) {
        // The packer writes LF, but a checkout with CRLF translation on will
        // leave the carriage return attached to the last field.
        if (!line.empty() && line.back() == '\r') line.pop_back();

        // index \t body_id \t type \t superclass
        const std::size_t a = line.find('\t');
        if (a == std::string::npos) continue;
        const std::size_t b = line.find('\t', a + 1);
        if (b == std::string::npos) continue;
        const std::size_t c = line.find('\t', b + 1);
        if (c == std::string::npos) continue;

        std::uint32_t idx = 0;
        const auto res = std::from_chars(line.data(), line.data() + a, idx);
        if (res.ec != std::errc{} || idx >= n) continue;

        nm.type_[idx] = line.substr(b + 1, c - b - 1);
        nm.superclass_[idx] = line.substr(c + 1);
        if (!nm.type_[idx].empty()) nm.byType_[nm.type_[idx]].push_back(idx);
    }

    if (ok) *ok = true;
    return nm;
}

std::string NeuronNames::label(std::uint32_t i) const {
    if (i < type_.size() && !type_[i].empty()) return type_[i];
    return "(untyped)";
}

std::span<const std::uint32_t> NeuronNames::ofType(const std::string& t) const {
    const auto it = byType_.find(t);
    if (it == byType_.end()) return {};
    return it->second;
}

}  // namespace fly
