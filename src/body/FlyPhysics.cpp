#include "FlyPhysics.h"

#include <algorithm>
#include <cmath>

namespace fly {
namespace {

// Rotation taking local -Z onto `dir`, matching how a segment is drawn.
Quat aimDownZ(const V3& dir) {
    const V3 d = normalise(dir);
    const V3 from{0, 0, -1};
    const float c = dot(from, d);
    if (c > 0.99999f) return {};
    if (c < -0.99999f) return Quat::axisAngle({1, 0, 0}, 3.14159265f);
    return Quat::axisAngle(cross(from, d), std::acos(std::clamp(c, -1.0f, 1.0f)));
}

}  // namespace

void FlyPhysics::build(const FlyBody& skeleton) {
    world = PhysicsWorld{};
    segments_.clear();
    probeLeg_.clear();

    // --- trunk ---
    RigidBody thorax;
    thorax.position = skeleton.root.position;
    thorax.orientation = skeleton.root.rotation;
    thorax.setBoxInertia(params.thoraxMass, anat::kThoraxHalf);
    thorax_ = world.addBody(thorax);

    RigidBody abdomen;
    abdomen.position =
        skeleton.root.apply({params.abdomenOffsetX, 0, anat::kAbdomenZ});
    abdomen.orientation = skeleton.root.rotation;
    abdomen.setBoxInertia(params.abdomenMass, anat::kAbdomenHalf);
    abdomen_ = world.addBody(abdomen);

    RigidBody head;
    head.position = skeleton.root.apply({anat::kHeadX, 0, anat::kHeadZ});
    head.orientation = skeleton.root.rotation;
    head.setBoxInertia(params.headMass, anat::kHeadHalf);
    head_ = world.addBody(head);

    // Abdomen and head are welded to the thorax.
    {
        HingeJoint j;
        j.a = thorax_;
        j.b = abdomen_;
        j.anchorA = {anat::kThoraxHalf.x * -0.8f, 0, -0.02f};
        // The shared anchor sits at -0.40 in the thorax's frame; expressed
        // in the abdomen's frame that is -0.40 minus the abdomen's own
        // offset, which for the default -0.78 gives +0.38.
        j.anchorB = {anat::kThoraxHalf.x * -0.8f - params.abdomenOffsetX, 0,
                     0.02f};
        j.axisA = j.axisB = {0, 1, 0};
        j.weld = true;
        world.joints.push_back(j);

        HingeJoint h;
        h.a = thorax_;
        h.b = head_;
        h.anchorA = {anat::kThoraxHalf.x * 0.85f, 0, 0.04f};
        h.anchorB = {anat::kThoraxHalf.x * 0.85f - anat::kHeadX, 0, -0.01f};
        h.axisA = h.axisB = {0, 1, 0};
        h.weld = true;
        world.joints.push_back(h);
    }

    // --- legs ---
    std::vector<FlyBody::SegmentPose> pose;
    skeleton.worldPose(pose);

    // Where each leg's tarsal chain continues from, captured while the main
    // segments are built and used by the second pass below.
    struct TarsusHead {
        std::uint32_t body = 0;
        V3 anchor;      // in the parent's frame
        V3 tip;         // world position of ta1's distal end
        Quat rot;       // ta1's orientation
        float full = 0.0f;    // full tarsus length for this leg
        float radius = 0.02f;
    };
    std::array<TarsusHead, kLegCount> tarsusHead{};

    for (int l = 0; l < kLegCount; ++l) {
        const Leg& leg = skeleton.legs()[l];
        std::uint32_t parent = thorax_;
        V3 parentAnchorLocal = leg.attach;

        for (int j = 0; j < kJointCount; ++j) {
            const FlyBody::SegmentPose& sp = pose[static_cast<std::size_t>(l) * kJointCount + j];
            const V3 delta = sp.b - sp.a;
            // The tarsus is five tarsomeres, not one rod. The skeleton still
            // describes it as a single segment -- that keeps the rest-pose
            // solver, the motor map and every l*kJointCount + j index working
            // -- and the physics splits it here. The TiTa motor joint drives
            // the first tarsomere, so the motor pool still moves the foot;
            // the rest follow on passive, compliant hinges.
            const bool isTarsus = (j == kJointCount - 1);
            const float fullLen = length(delta);
            const float len =
                isTarsus ? fullLen * anat::kTarsomereFrac[0] : fullLen;
            const Quat rot = aimDownZ(delta);

            RigidBody seg;
            // The body's origin sits at its proximal end, so the anchor to the
            // parent is simply the origin and the distal anchor is -Z * length.
            seg.position = sp.a;
            seg.orientation = rot;
            const float mass = std::max(params.minSegmentMass,
                                        len * 40.0f * params.segmentMassScale);
            seg.setCapsuleInertia(mass, std::max(sp.radius, 0.02f), len);
            const std::uint32_t id = world.addBody(seg);

            HingeJoint hj;
            hj.a = parent;
            hj.b = id;
            hj.anchorA = parentAnchorLocal;
            hj.anchorB = {0, 0, 0};
            // The hinge axis is the skeleton's joint axis, expressed in each
            // body's own frame.
            //
            // FlyBody::worldPose applies a joint's axis in the frame *before*
            // that joint rotates -- that is, in the parent segment's frame --
            // so the axis has to be taken into the world through the parent's
            // orientation, not the child's. Using the child's rotated every
            // hinge below ThC onto the wrong axis, which left the legs able to
            // splay but never to extend, and so unable to lift the body at all.
            //
            // ThC is the exception. Its parent is the thorax, and the leg is
            // mounted on it with a fixed outward tilt that is not a joint, so
            // it appears in no body's orientation. The tilt has to be folded
            // in here or ThC would hinge about the thorax's lateral axis
            // instead of the leg's, and the skeleton and the physics would
            // disagree about the one joint that takes a step.
            //
            // Only the world axis uses that frame. axisA is still expressed in
            // body A's own frame, because the solver recovers the world axis
            // with A.orientation.rotate(axisA).
            const Quat parentFrame =
                (j == 0) ? (world.bodies[parent].orientation * leg.mount).normalised()
                         : world.bodies[parent].orientation;
            const V3 worldAxis = parentFrame.rotate(leg.joints[j].axis);
            const M3 ra = M3::fromQuat(world.bodies[parent].orientation);
            const M3 rb = M3::fromQuat(rot);
            hj.axisA = ra.transposed() * worldAxis;
            hj.axisB = rb.transposed() * worldAxis;
            hj.maxTorque = params.postureTorque;
            hj.servoRate = params.servoRate;
            hj.damping = params.jointDamping;
            hj.minAngle = leg.joints[j].minAngle - leg.angle[j];
            hj.maxAngle = leg.joints[j].maxAngle - leg.angle[j];

            jointIndex_[l][j] = static_cast<std::uint32_t>(world.joints.size());
            restAngle_[l][j] = 0.0f;  // measured from the built pose
            world.joints.push_back(hj);

            segments_.push_back({id, len, sp.radius, sp.leg, sp.joint});

            if (isTarsus) {
                tarsusHead[l].full = fullLen;
                tarsusHead[l].radius = sp.radius;
                tarsusHead[l].rot = rot;
                tarsusHead[l].tip = sp.a + normalise(delta) * len;
            }

            parent = id;
            parentAnchorLocal = {0, 0, -len};
        }

        // `parent` and `parentAnchorLocal` now refer to ta1, which the main
        // loop left them pointing at.
        tarsusHead[l].body = parent;
        tarsusHead[l].anchor = parentAnchorLocal;
    }

    // --- tarsomeres 2..5 -------------------------------------------------
    //
    // Passive. A fly's individual tarsomeres have no motor neurons of their
    // own -- the motor map has one TiTa pool per leg and nothing below it --
    // and the foot works by draping over whatever it lands on. So these get
    // a weak servo back toward straight and a short range, which is what
    // makes the foot compliant instead of a spike.
    for (int l = 0; l < kLegCount; ++l) {
        std::uint32_t parent = tarsusHead[l].body;
        V3 anchor = tarsusHead[l].anchor;
        const Quat rot = tarsusHead[l].rot;
        V3 tip = tarsusHead[l].tip;
        const float radius = tarsusHead[l].radius;

        for (int t = 1; t < anat::kTarsomereCount; ++t) {
            const float len = tarsusHead[l].full * anat::kTarsomereFrac[t];
            const V3 dir = rot.rotate({0, 0, -1});

            RigidBody seg;
            seg.position = tip;
            seg.orientation = rot;
            const float mass = std::max(params.minTarsomereMass,
                                        len * 40.0f * params.segmentMassScale);
            // Tarsomeres taper toward the claw.
            const float r = radius * (1.0f - 0.10f * static_cast<float>(t));
            seg.setCapsuleInertia(mass, std::max(r, 0.014f), len);
            const std::uint32_t id = world.addBody(seg);

            HingeJoint hj;
            hj.a = parent;
            hj.b = id;
            hj.anchorA = anchor;
            hj.anchorB = {0, 0, 0};
            const V3 worldAxis =
                world.bodies[parent].orientation.rotate({0, 1, 0});
            const M3 ra = M3::fromQuat(world.bodies[parent].orientation);
            const M3 rb = M3::fromQuat(rot);
            hj.axisA = ra.transposed() * worldAxis;
            hj.axisB = rb.transposed() * worldAxis;
            hj.maxTorque = params.postureTorque * params.tarsusStiffness;
            hj.servoRate = params.servoRate;
            hj.damping = params.jointDamping;
            hj.minAngle = -params.tarsusRangeRad;
            hj.maxAngle = params.tarsusRangeRad;
            world.joints.push_back(hj);

            segments_.push_back({id, len, r, static_cast<LegId>(l),
                                 Joint::TiTa});
            footSegment_[l] = static_cast<std::uint32_t>(segments_.size() - 1);

            tip = tip + dir * len;
            parent = id;
            anchor = {0, 0, -len};
        }

        // The foot: a contact probe at the tip of the last tarsomere. Probes
        // are pushed in leg order, so probe index still equals leg index and
        // footLoad() is unaffected.
        const SegmentRef& last = segments_[footSegment_[l]];
        world.probes.push_back({last.body, {0, 0, -last.length}, last.radius});
    }

    // The trunk needs contacts too, or a collapse takes the body straight
    // through the floor instead of resting on it.
    // Placed on each part's underside, so they follow the anatomy rather
    // than being re-tuned by hand whenever the body changes shape.
    world.probes.push_back({thorax_, {0.0f, 0, -anat::kThoraxHalf.z}, 0.05f});
    world.probes.push_back({abdomen_, {-0.2f, 0, -anat::kAbdomenHalf.z}, 0.05f});
    world.probes.push_back({head_, {0.0f, 0, -anat::kHeadHalf.z}, 0.05f});

    // The tarsus now has five segments and, until here, one contact point at
    // the very tip. That would have made a jointed foot sink through the
    // floor more visibly than the single rod it replaced, so each tarsomere
    // gets its own probe. They go after the trunk probes, leaving the first
    // kLegCount probe indices as the foot tips in leg order.
    probeLeg_.assign(world.probes.size(), -1);
    for (int l = 0; l < kLegCount; ++l) probeLeg_[l] = l;
    for (int l = 0; params.tarsusProbes && l < kLegCount; ++l) {
        const std::uint32_t last = footSegment_[l];
        const std::uint32_t first = last - (anat::kTarsomereCount - 2);
        for (std::uint32_t k = first; k < last; ++k) {
            const SegmentRef& sref = segments_[k];
            world.probes.push_back({sref.body, {0, 0, -sref.length},
                                    sref.radius});
            probeLeg_.push_back(l);
        }
    }

    world.prepare();
    for (int l = 0; l < kLegCount; ++l) legSpanRest_[l] = legSpan(l);
    restHeight_ = world.bodies[thorax_].position.z;
    peakHeight_ = restHeight_;
    airborne_ = false;
}

float FlyPhysics::legSpan(int leg) const {
    // Coxa to foot, where the foot is the tip of the *last* tarsomere.
    //
    // This used to index segments_[leg * kJointCount + kJointCount - 1],
    // which was the whole tarsus until it was split into five. After the
    // split that index is ta1, so the span stopped at 40% of the way along
    // the foot and the proprioceptive compression signal in SensoryOrgans
    // was measuring a leg that ended in the wrong place.
    const std::size_t base = static_cast<std::size_t>(leg) * kJointCount;
    const RigidBody& coxa = world.bodies[segments_[base].body];
    return length(footPosition(leg) - coxa.position);
}

float FlyPhysics::footLoad(int leg) const {
    float load = 0.0f;
    for (const auto& c : world.contacts) {
        if (c.probe < probeLeg_.size() && probeLeg_[c.probe] == leg) {
            load += c.normalImpulse;
        }
    }
    return load;
}

V3 FlyPhysics::footPosition(int leg) const {
    const SegmentRef& s = segments_[footSegment_[leg]];
    const RigidBody& b = world.bodies[s.body];
    return b.position + b.orientation.rotate({0, 0, -s.length});
}

void FlyPhysics::reset(const FlyBody& skeleton) {
    build(skeleton);
}

void FlyPhysics::step(float dtSeconds) {
    if (dtSeconds <= 0.0f) return;
    for (int l = 0; l < kLegCount; ++l) {
        for (int j = 0; j < kJointCount; ++j) {
            HingeJoint& hj = world.joints[jointIndex_[l][j]];
            const bool legSelected =
                (params.forceLeg < 0 || params.forceLeg == l);
            float d = (params.forceJoint == j && legSelected)
                          ? params.forceDrive : 0.0f;
            if (params.useManualDrive) d = params.manualDrive[l][j];
            if (params.useManualTarget) {
                hj.targetAngle = restAngle_[l][j] + params.manualTarget[l][j];
                hj.maxTorque = params.postureTorque;
                hj.servoRate = params.servoRate;
                hj.muscleTorque = 0.0f;
                continue;
            }
            applyDrive(hj, restAngle_[l][j], d);
        }
    }
    advance(dtSeconds);
}

void FlyPhysics::step(float dtSeconds, const MotorPools& pools) {
    if (dtSeconds <= 0.0f) return;

    // Drive every joint from its two antagonist pools. Extensor minus flexor,
    // scaled to a torque -- the same difference that used to set an angle
    // directly now sets a force, and the body decides what happens.
    for (int l = 0; l < kLegCount; ++l) {
        for (int j = 0; j < kJointCount; ++j) {
            // drive() already sums each muscle weighted by its own strength,
            // so there is no per-joint fudge factor here any more: a jump
            // muscle is strong because the muscle is strong.
            HingeJoint& hj = world.joints[jointIndex_[l][j]];
            // An imposed pattern overrides the pools, the same way it does in
            // the mechanics-only step. Without this branch here, setting
            // useManualTarget on the path that has a nervous system attached
            // silently did nothing, and the "walking" in the 3D view was the
            // default giant-fibre stimulus landing askew.
            if (params.useManualTarget) {
                hj.targetAngle = restAngle_[l][j] + params.manualTarget[l][j];
                hj.maxTorque = params.postureTorque;
                hj.servoRate = params.servoRate;
                hj.muscleTorque = 0.0f;
                continue;
            }
            const float d = (params.forceJoint == j) ? params.forceDrive
                                                    : pools.drive(l, j);
            peakDrive_ = std::max(peakDrive_, std::fabs(d));
            applyDrive(hj, restAngle_[l][j], d);
        }
    }

    advance(dtSeconds);
}

// A muscle both shifts where the joint is trying to be and how hard it is
// willing to push to get there. Drive is unbounded -- a strong muscle at full
// activation returns a large number -- so the excursion saturates while the
// torque budget keeps growing, which is what makes a jump muscle different
// from a postural one rather than just louder.
// Hill force modifiers, both a function of where the joint is and how fast it
// is moving. Applied to the net drive rather than per muscle, which is coarse
// but captures the two effects that matter: a muscle at the end of its travel
// produces no force, and one shortening quickly produces less.
float FlyPhysics::hillFactor(const HingeJoint& hj, float drive) const {
    if (!params.hillMuscle || drive == 0.0f) return 1.0f;

    // Force-length. Taper toward zero as the joint approaches the limit the
    // muscle is pulling it toward. A muscle cannot keep contracting once it
    // has run out of travel, which is what let sustained drive push against
    // the joint stops indefinitely.
    const float angle = world.jointAngle(hj);
    const float toward = (drive > 0.0f) ? hj.maxAngle : hj.minAngle;
    const float span = std::fabs(toward - angle);
    const float taper = std::fabs(hj.maxAngle - hj.minAngle) * params.hillTaperFrac;
    const float lengthFactor = taper > 1e-6f ? std::clamp(span / taper, 0.0f, 1.0f)
                                             : 1.0f;

    // Force-velocity. Shortening against the direction of pull costs force,
    // hyperbolically as in Hill's relation; being stretched yields more.
    const float rate = world.jointRate(hj);
    peakJointRate_ = std::max(peakJointRate_, std::fabs(rate));
    const float shortening = (drive > 0.0f) ? rate : -rate;
    float velocityFactor;
    if (shortening > 0.0f) {
        velocityFactor = params.hillShorteningRate /
                         (params.hillShorteningRate + shortening);
    } else {
        const float g = params.hillLengtheningGain;
        velocityFactor = g - (g - 1.0f) * params.hillShorteningRate /
                                 (params.hillShorteningRate - shortening);
    }

    return lengthFactor * velocityFactor;
}

void FlyPhysics::applyDrive(HingeJoint& hj, float restAngle, float drive) const {
    // Posture and contraction are separate. The servo always holds the rest
    // angle with a fixed torque budget; the muscle adds a feed-forward torque
    // proportional to activation times its own strength, so a jump muscle
    // produces twenty times the force of a postural one rather than merely
    // holding the same target more firmly.
    hj.targetAngle = restAngle;
    // Antagonist relaxation: a joint being actively driven gives up its
    // postural hold in proportion to the drive. Without it, posture is a rigid
    // servo the muscle has to overpower, which makes the response a threshold
    // rather than a gradient -- below it nothing moves at all, and just above
    // it the fly launches sixteen millimetres. A real animal releases the
    // opposing muscles instead of fighting them.
    hj.maxTorque = params.postureTorque / (1.0f + std::fabs(drive));
    hj.servoRate = params.servoRate;
    hj.muscleTorque = drive * params.maxMuscleTorque * hillFactor(hj, drive);
}

void FlyPhysics::advance(float dtSeconds) {
    const float h = 1.0f / params.substepHz;
    int steps = static_cast<int>(std::ceil(dtSeconds / h));
    steps = std::clamp(steps, 1, 256);
    const float sub = dtSeconds / static_cast<float>(steps);
    for (int i = 0; i < steps; ++i) world.step(sub);

    const float z = world.bodies[thorax_].position.z;
    peakHeight_ = std::max(peakHeight_, z);
    airborne_ = world.contacts.empty();
}

float FlyPhysics::totalMass() const {
    float m = 0.0f;
    for (const auto& b : world.bodies) {
        if (b.invMass > 0.0f) m += 1.0f / b.invMass;
    }
    return m;
}

void FlyPhysics::readPose(std::vector<FlyBody::SegmentPose>& out) const {
    out.clear();
    out.reserve(segments_.size());
    for (const auto& s : segments_) {
        const RigidBody& b = world.bodies[s.body];
        const V3 a = b.position;
        const V3 tip = b.position + b.orientation.rotate({0, 0, -s.length});
        out.push_back({a, tip, s.radius, s.leg, s.joint});
    }
}

}  // namespace fly
