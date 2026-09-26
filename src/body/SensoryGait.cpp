#include "SensoryGait.h"

#include <algorithm>
#include <cmath>

namespace fly {

namespace {
constexpr float kPi = 3.14159265358979f;
}  // namespace

int anteriorNeighbour(int leg) {
    switch (leg) {
        case 2: return 0;   // middle_L -> front_L
        case 3: return 1;   // middle_R -> front_R
        case 4: return 2;   // hind_L   -> middle_L
        case 5: return 3;   // hind_R   -> middle_R
        default: return -1; // the front legs have none
    }
}

int posteriorNeighbour(int leg) {
    switch (leg) {
        case 0: return 2;
        case 1: return 3;
        case 2: return 4;
        case 3: return 5;
        default: return -1;
    }
}

int contralateral(int leg) { return (leg % 2 == 0) ? leg + 1 : leg - 1; }

void SensoryGait::reset(const SensoryGaitParams& params) {
    p_ = params;
    timeouts_ = 0;
    anchored_ = false;
    // Start every leg in stance, spread along its stance travel. Starting
    // them all at the same point leaves the rules with nothing to break the
    // symmetry, and six legs that all reach their PEP on the same step try to
    // swing at once.
    //
    // The offsets are the tripod arrangement, which is where a walking fly
    // would already be. The rules are what *keep* it there; they are not
    // being asked to discover it from a standing start.
    const bool tripodA[kLegCount] = {true, false, false, true, true, false};
    for (int l = 0; l < kLegCount; ++l) {
        phase_[l] = Phase::Stance;
        footX_[l] = tripodA[l] ? p_.aep * 0.5f : p_.pep * 0.5f;
        swingFrom_[l] = footX_[l];
        swingU_[l] = 0.0f;
        zTrim_[l] = 0.0f;
        sinceTouchdown_[l] = 0.0f;
        inStanceFor_[l] = 0.0f;
        stanceTime_[l] = 0.0f;
        swingTime_[l] = 0.0f;
        steps_[l] = 0;
    }
}

bool SensoryGait::swingAllowed(int leg) const {
    const int ant = anteriorNeighbour(leg);
    const int post = posteriorNeighbour(leg);
    const int con = contralateral(leg);

    // Rule 1, and the contralateral form of it: do not lift while a
    // neighbour is already in the air.
    if (p_.rule1 && post >= 0 && phase_[post] == Phase::Swing) return false;
    if (p_.contra && phase_[con] == Phase::Swing) return false;

    // Rule 1 also acts on the leg in front: a swinging leg suppresses its
    // anterior neighbour.
    if (p_.rule1 && ant >= 0 && phase_[ant] == Phase::Swing) {
        // Suppression from behind is the weaker direction -- it delays rather
        // than forbids -- but with only six legs and a short swing, treating
        // it as a veto is what keeps two adjacent legs off the ground.
        return false;
    }

    // Rule 3: a leg may not swing until the leg in front of it has used up
    // most of its own stance travel. This sets the spacing along each side.
    if (p_.rule3 && ant >= 0 && phase_[ant] == Phase::Stance) {
        const float travel = p_.aep - p_.pep;
        if (travel > 1e-6f) {
            const float used = (p_.aep - footX_[ant]) / travel;
            if (used < p_.rule3Frac) return false;
        }
    }
    return true;
}

void SensoryGait::update(float dt, const FlyPhysics& phys,
                         const GaitPlan& plan,
                         float out[kLegCount][kJointCount]) {
    // --- load sharing, over the legs currently in stance ---
    {
        float load[kLegCount] = {};
        float total = 0.0f;
        int stanceCount = 0;
        for (int l = 0; l < kLegCount; ++l) {
            load[l] = phys.footLoad(l);
            if (phase_[l] == Phase::Stance) {
                total += load[l];
                ++stanceCount;
            }
        }
        if (stanceCount > 0 && total > 1e-9f) {
            const float want = total / static_cast<float>(stanceCount);
            for (int l = 0; l < kLegCount; ++l) {
                if (phase_[l] != Phase::Stance) continue;
                zTrim_[l] += p_.shareGain * ((load[l] - want) / want) * dt;
                zTrim_[l] = std::clamp(zTrim_[l], -p_.shareClamp,
                                       p_.shareClamp);
            }
        }
    }

    // Mean stance load, for deciding whether a foot has lost the ground.
    float meanLoad = 0.0f;
    int downCount = 0;
    for (int l = 0; l < kLegCount; ++l) {
        if (phase_[l] != Phase::Stance) continue;
        meanLoad += phys.footLoad(l);
        ++downCount;
    }
    if (downCount > 0) meanLoad /= static_cast<float>(downCount);

    // Where each foot actually is, fore and aft, in body coordinates.
    //
    // Measured rather than integrated. The reference offset is captured once,
    // on the first step, so that a measured position maps onto the same scale
    // the AEP and PEP are written in.
    float measured[kLegCount];
    {
        const V3 fwd = phys.thorax().orientation.rotate({1, 0, 0});
        const V3 origin = phys.thorax().position;
        for (int l = 0; l < kLegCount; ++l) {
            measured[l] = dot(phys.footPosition(l) - origin, fwd);
        }
        if (!anchored_) {
            // The animal starts at its rest pose, so whatever the feet read
            // on the first step *is* the zero of this scale. Subtracting the
            // initial command instead -- which the first version did -- biased
            // every leg by half a stride and left the measured position and
            // the command permanently disagreeing.
            for (int l = 0; l < kLegCount; ++l) refX_[l] = measured[l];
            anchored_ = true;
        }
        for (int l = 0; l < kLegCount; ++l) measured[l] -= refX_[l];
    }

    const float travel = std::max(1e-6f, p_.aep - p_.pep);
    const float nominalStance =
        (p_.speed > 1e-6f) ? travel / p_.speed : 1e9f;

    for (int l = 0; l < kLegCount; ++l) {
        if (phase_[l] == Phase::Stance) {
            // One backward rate, shared by every stance leg so they
            // cooperate rather than argue, and clamped to stay near the foot
            // so it cannot wind up.
            footX_[l] -= p_.speed * dt;
            footX_[l] = std::clamp(footX_[l], measured[l] - p_.maxLag,
                                   measured[l] + p_.maxLag);
            inStanceFor_[l] += dt;
            stanceTime_[l] += dt;

            const bool atPep = measured[l] <= p_.pep;
            const float used = (p_.aep - measured[l]) / travel;
            const bool unloaded =
                used > p_.minStanceFrac && meanLoad > 0.0f &&
                phys.footLoad(l) < p_.unloadFrac * meanLoad;
            const bool stuck =
                inStanceFor_[l] > p_.stanceTimeout * nominalStance;

            if ((atPep || unloaded) && swingAllowed(l)) {
                phase_[l] = Phase::Swing;
                swingFrom_[l] = footX_[l];
                swingU_[l] = 0.0f;
            } else if (stuck) {
                // The rules have deadlocked. Release the leg anyway and
                // count it, because a gait that leans on this is not
                // coordinating and the number should be visible.
                phase_[l] = Phase::Swing;
                swingFrom_[l] = footX_[l];
                swingU_[l] = 0.0f;
                ++timeouts_;
            }
        } else {
            // Swing is the one part that is timed, because in an insect swing
            // duration barely changes with walking speed while stance
            // duration does. Duty factor therefore falls out of speed instead
            // of being a number somebody picks.
            const float step = (p_.swingMs > 1e-6f)
                                   ? dt * 1000.0f / p_.swingMs
                                   : 1.0f;
            swingU_[l] = std::min(1.0f, swingU_[l] + step);
            swingTime_[l] += dt;
            footX_[l] = swingFrom_[l] +
                        (p_.aep - swingFrom_[l]) * swingU_[l];
            if (swingU_[l] >= 1.0f) {
                phase_[l] = Phase::Stance;
                footX_[l] = p_.aep;
                inStanceFor_[l] = 0.0f;
                sinceTouchdown_[l] = 0.0f;
                ++steps_[l];
            }
        }

        const float lift = (phase_[l] == Phase::Swing)
                               ? p_.liftMm * std::sin(kPi * swingU_[l])
                               : 0.0f;
        plan.jointsForFoot(l, footX_[l], lift + zTrim_[l], out[l]);
    }

    for (int l = 0; l < kLegCount; ++l) sinceTouchdown_[l] += dt;
}

}  // namespace fly
