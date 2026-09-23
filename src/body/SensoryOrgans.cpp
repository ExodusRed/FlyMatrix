#include "SensoryOrgans.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace fly {
namespace {

int legIndex(const std::string& name) {
    for (int l = 0; l < kLegCount; ++l) {
        if (name == legName(static_cast<LegId>(l))) return l;
    }
    return -1;
}

}  // namespace

SensoryOrgans SensoryOrgans::load(const std::string& tsvPath, Params params) {
    std::ifstream f(tsvPath);
    if (!f) {
        throw std::runtime_error(
            "cannot open " + tsvPath +
            "\nBuild it from the project root with: python tools/sensory_map.py");
    }

    SensoryOrgans so;
    so.p_ = params;

    std::string line;
    std::getline(f, line);  // header
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;

        std::istringstream ss(line);
        std::string organ, leg, joint, purity, idxStr;
        if (!std::getline(ss, organ, '\t') || !std::getline(ss, leg, '\t') ||
            !std::getline(ss, joint, '\t') || !std::getline(ss, purity, '\t') ||
            !std::getline(ss, idxStr, '\t')) {
            continue;
        }
        // `joint` is read past deliberately and never used; see the header.
        const int l = legIndex(leg);
        if (l < 0) continue;
        if (std::stof(purity) < params.minPurity) continue;

        const Organ o{static_cast<std::uint32_t>(std::stoul(idxStr)), l};
        if (organ == "chordotonal") so.chordotonal_.push_back(o);
        else if (organ == "campaniform") so.campaniform_.push_back(o);
    }

    if (so.chordotonal_.empty() && so.campaniform_.empty()) {
        throw std::runtime_error(tsvPath + " contained no usable rows");
    }
    return so;
}

void SensoryOrgans::reset() {
    chordotonalDrive_.fill(0.0f);
    campaniformDrive_.fill(0.0f);
    lastSpan_.fill(0.0f);
    haveLastSpan_ = false;
}

void SensoryOrgans::sense(const FlyPhysics& phys, LIFNetwork& net, float dtMs) {
    for (int l = 0; l < kLegCount; ++l) {
        const float span = phys.legSpan(l);
        const float rest = phys.legSpanRest(l);

        // Chordotonal: how far the leg has folded from its rest configuration,
        // plus a phasic term for how fast it is folding. Both are one-sided --
        // these report compression, which is what a leg bearing weight does,
        // and a leg extended past rest simply falls silent.
        float compression = rest > 1e-6f ? (rest - span) / rest : 0.0f;
        float rate = 0.0f;
        if (haveLastSpan_ && dtMs > 0.0f) {
            // Per second, normalised by rest span, so it is the same scale as
            // the positional term.
            rate = (lastSpan_[l] - span) / (rest * dtMs * 0.001f);
        }
        lastSpan_[l] = span;

        const float positional = compression / p_.fullCompression;
        const float phasic = rate * p_.velocityWeight / p_.fullCompression;
        chordotonalDrive_[l] =
            std::clamp(positional + phasic, 0.0f, 1.0f) * p_.maxDrive;

        // Campaniform: load carried by the foot. Zero while airborne, which is
        // the signal a fly uses to know it has left the ground.
        campaniformDrive_[l] =
            std::clamp(phys.footLoad(l) / p_.fullLoad, 0.0f, 1.0f) * p_.maxDrive;
    }
    haveLastSpan_ = true;

    for (const auto& o : chordotonal_) {
        net.setStimulus(o.neuron, chordotonalDrive_[o.leg]);
    }
    for (const auto& o : campaniform_) {
        net.setStimulus(o.neuron, campaniformDrive_[o.leg]);
    }
}

}  // namespace fly
