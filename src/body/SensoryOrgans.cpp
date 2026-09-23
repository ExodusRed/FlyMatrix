#include "SensoryOrgans.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_map>
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

void SensoryOrgans::loadSplit(const std::string& tsvPath) {
    std::ifstream f(tsvPath);
    if (!f) return;  // optional: without it the loop behaves as it did before

    std::unordered_map<std::uint32_t, int> side;
    std::string line;
    std::getline(f, line);  // header
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string leg, idxStr, group;
        if (!std::getline(ss, leg, '\t') || !std::getline(ss, idxStr, '\t') ||
            !std::getline(ss, group, '\t')) {
            continue;
        }
        side[static_cast<std::uint32_t>(std::stoul(idxStr))] =
            (group == "extend") ? +1 : -1;
    }

    splitAssigned_ = 0;
    for (auto& o : chordotonal_) {
        const auto it = side.find(o.neuron);
        o.side = (it == side.end()) ? 0 : it->second;
        if (o.side != 0) ++splitAssigned_;
    }
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

    if (p_.useSplit && splitAssigned_ > 0) {
        // Two opposed signals instead of one.
        //
        // The FeCO encodes tibia position, which is the FTi angle, so that is
        // what drives it -- not whole-leg compression, which cannot
        // distinguish a flexed tibia from a flexed femur and so cannot tell
        // antagonist afferents apart at all.
        //
        // A resistance reflex means extension excites flexors. So the cells
        // that drive flexor motor neurons are the ones a tibia *extension*
        // should excite, and vice versa. reflexSign flips that, and it is a
        // modelling choice the connectome cannot make for us.
        for (int l = 0; l < kLegCount; ++l) {
            const float dev = phys.jointAngle(l, static_cast<int>(Joint::FTi));
            const float norm =
                std::clamp(dev / p_.fullTibiaAngle, -1.0f, 1.0f);
            tibiaDrive_[l] = norm;
        }
        for (const auto& o : chordotonal_) {
            if (o.side == 0) {
                net.setStimulus(o.neuron, 0.0f);
                continue;
            }
            // side +1 drives extensors and is excited by flexion; side -1
            // drives flexors and is excited by extension.
            const float excite =
                -p_.reflexSign * static_cast<float>(o.side) * tibiaDrive_[o.leg];
            net.setStimulus(o.neuron,
                            std::max(0.0f, excite) * p_.maxDrive);
        }
    } else {
        for (const auto& o : chordotonal_) {
            net.setStimulus(o.neuron, chordotonalDrive_[o.leg]);
        }
    }
    for (const auto& o : campaniform_) {
        net.setStimulus(o.neuron, campaniformDrive_[o.leg]);
    }
}

}  // namespace fly
