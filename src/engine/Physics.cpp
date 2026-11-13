#include "Physics.h"

#include <algorithm>
#include <cmath>

namespace fly {
namespace {

// Any unit vector perpendicular to `n`. Picking the axis `n` is least aligned
// with avoids the degenerate case where the cross product vanishes.
V3 perpendicular(const V3& n) {
    const V3 a = (std::fabs(n.x) < 0.57735f) ? V3{1, 0, 0}
               : (std::fabs(n.y) < 0.57735f) ? V3{0, 1, 0}
                                             : V3{0, 0, 1};
    return normalise(cross(n, a));
}

// RigidBody::applyImpulse / applyPseudoImpulse with the world-space inverse
// inertia passed in instead of recomputed from the orientation on every call.
// Same arithmetic, in the same order; only valid while the orientation is the
// one `invI` was computed from, i.e. inside one solver sweep.
inline void applyImpulseWith(RigidBody& b, const M3& invI, const V3& impulse, const V3& r) {
    b.velocity += impulse * b.invMass;
    b.angularVelocity += invI * cross(r, impulse);
}
inline void applyPseudoImpulseWith(RigidBody& b, const M3& invI, const V3& impulse, const V3& r) {
    b.pseudoVelocity += impulse * b.invMass;
    b.pseudoAngular += invI * cross(r, impulse);
}

}  // namespace

void PhysicsWorld::cacheInvInertia() {
    invInertia_.resize(bodies.size());
    for (std::size_t i = 0; i < bodies.size(); ++i) invInertia_[i] = bodies[i].invInertiaWorld();
}

void RigidBody::setBoxInertia(float mass, const V3& h) {
    invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass <= 0.0f) {
        invInertiaLocal = M3::zero();
        return;
    }
    const float k = mass / 3.0f;  // (1/12) * m * (2h)^2 == (1/3) * m * h^2
    const float ix = k * (h.y * h.y + h.z * h.z);
    const float iy = k * (h.x * h.x + h.z * h.z);
    const float iz = k * (h.x * h.x + h.y * h.y);
    invInertiaLocal = M3::diagonal(ix > 0 ? 1.0f / ix : 0.0f,
                                   iy > 0 ? 1.0f / iy : 0.0f,
                                   iz > 0 ? 1.0f / iz : 0.0f);
}

void RigidBody::setCapsuleInertia(float mass, float radius, float len) {
    // Approximated as a cylinder along local Z, which is how leg segments are
    // built. Exact capsule inertia is not worth the algebra at this scale.
    invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass <= 0.0f) {
        invInertiaLocal = M3::zero();
        return;
    }
    const float r2 = radius * radius;
    float along = 0.5f * mass * r2;
    const float across = mass * (3.0f * r2 + len * len) / 12.0f;
    // A thin segment's inertia about its own long axis is a hundredth of its
    // inertia across. Explicit damping is stable only for dt < 2I/c, so that
    // one tiny principal moment sets the timestep for the whole simulation --
    // and the trochanter-femur joint twists about exactly that axis. Keeping
    // the tensor within a 4:1 ratio costs a little realism in axial spin and
    // buys back a usable timestep.
    along = std::max(along, across * 0.25f);
    invInertiaLocal = M3::diagonal(across > 0 ? 1.0f / across : 0.0f,
                                   across > 0 ? 1.0f / across : 0.0f,
                                   along > 0 ? 1.0f / along : 0.0f);
}

std::uint32_t PhysicsWorld::addBody(const RigidBody& b) {
    bodies.push_back(b);
    return static_cast<std::uint32_t>(bodies.size() - 1);
}

void PhysicsWorld::prepare() {
    for (auto& j : joints) {
        // Measure the joint angle against a direction perpendicular to the
        // axis, captured in each body's own frame at the current pose. That
        // makes the present configuration angle zero, and the rest angle is
        // applied relative to it.
        j.refA = perpendicular(normalise(j.axisA));
        const RigidBody& A = bodies[j.a];
        const RigidBody& B = bodies[j.b];
        const V3 worldRefA = A.orientation.rotate(j.refA);
        // Express the same world direction in B's frame so the starting angle
        // reads as zero regardless of how the two bodies are oriented.
        const M3 rb = M3::fromQuat(B.orientation);
        j.refB = rb.transposed() * worldRefA;

        // A weld remembers the relative orientation it must maintain.
        const Quat conjA{A.orientation.w, -A.orientation.x, -A.orientation.y,
                         -A.orientation.z};
        j.weldRest = (conjA * B.orientation).normalised();
    }
}

float PhysicsWorld::jointAngle(const HingeJoint& j) const {
    const RigidBody& A = bodies[j.a];
    const RigidBody& B = bodies[j.b];
    const V3 axis = normalise(A.orientation.rotate(j.axisA));
    const V3 ra = A.orientation.rotate(j.refA);
    const V3 rb = B.orientation.rotate(j.refB);
    // Project both references into the plane perpendicular to the axis, then
    // take the signed angle between them.
    const V3 pa = normalise(ra - axis * dot(ra, axis));
    const V3 pb = normalise(rb - axis * dot(rb, axis));
    const float c = std::clamp(dot(pa, pb), -1.0f, 1.0f);
    const float s = dot(cross(pa, pb), axis);
    return std::atan2(s, c);
}

float PhysicsWorld::maxAnchorError() const {
    float worst = 0.0f;
    for (const auto& j : joints) {
        const RigidBody& A = bodies[j.a];
        const RigidBody& B = bodies[j.b];
        const V3 pA = A.position + A.orientation.rotate(j.anchorA);
        const V3 pB = B.position + B.orientation.rotate(j.anchorB);
        worst = std::max(worst, length(pB - pA));
    }
    return worst;
}

float PhysicsWorld::jointRate(const HingeJoint& j) const {
    const RigidBody& A = bodies[j.a];
    const RigidBody& B = bodies[j.b];
    const V3 axis = normalise(A.orientation.rotate(j.axisA));
    return dot(B.angularVelocity - A.angularVelocity, axis);
}

void PhysicsWorld::integrateVelocities(float dt) {
    for (auto& b : bodies) {
        if (b.invMass <= 0.0f) continue;
        b.velocity += (params.gravity + b.force * b.invMass) * dt;
        b.angularVelocity += b.invInertiaWorld() * b.torque * dt;
        b.velocity = b.velocity * (1.0f / (1.0f + params.linearDamping * dt));
        b.angularVelocity =
            b.angularVelocity * (1.0f / (1.0f + params.angularDamping * dt));

    }
    clampVelocities();
}

// The speed limit has to be the last thing applied before positions are
// integrated, not the first.
//
// It used to live at the end of integrateVelocities, which runs before the
// joint and contact solvers. Those solvers then applied impulses that nothing
// re-clamped, so the limit only ever caught velocity produced by gravity --
// by far the smallest contributor. Constraint impulses were unbounded, and
// measured joint rates during a jump reached 204,000 rad/s where a real fly's
// fastest leg joint does a few hundred.
void PhysicsWorld::clampVelocities() {
    for (auto& b : bodies) {
        if (b.invMass <= 0.0f) continue;
        const float lv = length(b.velocity);
        if (lv > params.maxLinearVelocity) {
            b.velocity = b.velocity * (params.maxLinearVelocity / lv);
        }
        const float av = length(b.angularVelocity);
        if (av > params.maxAngularVelocity) {
            b.angularVelocity = b.angularVelocity * (params.maxAngularVelocity / av);
        }
    }
}

void PhysicsWorld::solveJoints(float dt) {
    const float invDt = dt > 0.0f ? 1.0f / dt : 0.0f;

    for (auto& j : joints) {
        j.servoImpulse = 0.0f;
        j.muscleImpulse = 0.0f;
    }

    // Muscle contraction is applied inside the iteration loop below, as a
    // bounded impulse, not as a raw torque here.
    //
    // A feed-forward torque large enough to make the fly jump was also large
    // enough to pull the joint anchors apart faster than the solver could put
    // them back. The position correction then did work on the body every
    // substep, and the fly climbed steadily with no ground contact at all. A
    // real muscle cannot dislocate the joint it pulls on; making the muscle
    // one more constraint the solver has to satisfy enforces that.

    // Warm start: replay the impulse that held this joint together last step.
    // Without it a standing pose is re-solved from nothing every frame and the
    // legs visibly sag before the iterations catch up.
    for (auto& j : joints) {
        RigidBody& A = bodies[j.a];
        RigidBody& B = bodies[j.b];
        const V3 rA = A.orientation.rotate(j.anchorA);
        const V3 rB = B.orientation.rotate(j.anchorB);
        A.applyImpulse(-j.linearImpulse, rA);
        B.applyImpulse(j.linearImpulse, rB);
    }
    for (auto& j : joints) j.accumulated = {};

    // Geometry for the whole sweep (see JointPre in Physics.h). Each value is
    // computed with exactly the expression the loop below used to evaluate in
    // every iteration, so nothing about the solve changes except its cost.
    jointPre_.resize(joints.size());
    for (std::size_t n = 0; n < joints.size(); ++n) {
        const HingeJoint& j = joints[n];
        const RigidBody& A = bodies[j.a];
        const RigidBody& B = bodies[j.b];
        const M3& ia = invInertia_[j.a];
        const M3& ib = invInertia_[j.b];
        JointPre& p = jointPre_[n];

        p.axis = normalise(A.orientation.rotate(j.axisA));
        p.effAxis = dot(p.axis, ia * p.axis) + dot(p.axis, ib * p.axis);
        p.angle = jointAngle(j);

        if (j.weld) {
            const Quat want = (A.orientation * j.weldRest).normalised();
            const Quat conjWant{want.w, -want.x, -want.y, -want.z};
            Quat err = (conjWant * B.orientation).normalised();
            if (err.w < 0.0f) err = {-err.w, -err.x, -err.y, -err.z};  // shortest arc
            p.weldError = {2.0f * err.x, 2.0f * err.y, 2.0f * err.z};
            const V3 basis[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
            for (int k = 0; k < 3; ++k) {
                p.effWeld[k] = dot(basis[k], ia * basis[k]) + dot(basis[k], ib * basis[k]);
            }
        } else {
            const V3 axA = p.axis;  // identical to normalise(A.orientation.rotate(j.axisA))
            const V3 axB = normalise(B.orientation.rotate(j.axisB));
            p.tangent[0] = perpendicular(axA);
            p.tangent[1] = cross(axA, p.tangent[0]);
            p.axisError = cross(axA, axB);
            for (int k = 0; k < 2; ++k) {
                const V3& t = p.tangent[k];
                p.effTangent[k] = dot(t, ia * t) + dot(t, ib * t);
            }
        }

        p.rA = A.orientation.rotate(j.anchorA);
        p.rB = B.orientation.rotate(j.anchorB);
        p.anchorError = (B.position + p.rB) - (A.position + p.rA);
        const M3 sa = M3::skew(p.rA);
        const M3 sb = M3::skew(p.rB);
        M3 k = M3::diagonal(A.invMass + B.invMass, A.invMass + B.invMass,
                            A.invMass + B.invMass);
        k = k + (-(sa * ia * sa)) + (-(sb * ib * sb));
        p.kOk = k.invert(p.kInv);
    }

    for (int it = 0; it < params.iterations; ++it) {
        // Joints are swept in the order they were built, which is root to tip
        // along each leg. Alternating the sweep direction is the textbook way
        // to speed up Gauss-Seidel on a chain, and it was tried here: it made
        // things consistently worse (0.43 mm ride height down to 0.33 at 24
        // iterations). Warm starting already carries the previous step's
        // solution, and reversing appears to fight it. Left in the simple
        // order deliberately.
        for (std::size_t n = 0; n < joints.size(); ++n) {
            HingeJoint& j = joints[n];
            RigidBody& A = bodies[j.a];
            RigidBody& B = bodies[j.b];
            const JointPre& p = jointPre_[n];
            const M3& ia = invInertia_[j.a];
            const M3& ib = invInertia_[j.b];

            // --- servo: drive the hinge toward its target angle ---
            //
            // Solved as a velocity constraint with a bounded impulse rather
            // than applied as a torque. The bound is what gives a joint finite
            // strength: it can be made arbitrarily strong without the
            // integrator ever seeing a stiff force.
            if (j.maxTorque > 0.0f) {
                const V3& axis = p.axis;
                const float target = std::clamp(j.targetAngle, j.minAngle, j.maxAngle);
                const float angle = p.angle;
                const float eff = p.effAxis;
                if (eff > 1e-12f) {
                    const float relSpin =
                        dot(B.angularVelocity - A.angularVelocity, axis);

                    const float want = j.servoRate * (target - angle) * invDt;
                    float lambda = (want - relSpin * (1.0f + j.damping)) / eff;

                    const float maxImp = j.maxTorque * dt;
                    const float old = j.servoImpulse;
                    j.servoImpulse = std::clamp(old + lambda, -maxImp, maxImp);
                    lambda = j.servoImpulse - old;

                    const V3 imp = axis * lambda;
                    A.angularVelocity += ia * (-imp);
                    B.angularVelocity += ib * imp;
                }
            }

            // --- muscle: bounded contraction impulse about the hinge axis ---
            if (j.muscleTorque != 0.0f) {
                const V3& axis = p.axis;
                const float eff = p.effAxis;
                if (eff > 1e-12f) {
                    // The muscle delivers torque * dt of angular impulse per
                    // step, no more. Tracking what it has already delivered
                    // means the remainder is spent on the first iteration and
                    // the rest of the sweep is free to correct the damage in
                    // velocity space, where it costs no energy, instead of
                    // leaving it as position error for the correction pass.
                    const float target = j.muscleTorque * dt;
                    const float lambda = target - j.muscleImpulse;
                    j.muscleImpulse = target;

                    const V3 imp = axis * lambda;
                    A.angularVelocity += ia * (-imp);
                    B.angularVelocity += ib * imp;
                }
            }

            // --- hard joint limit ---
            //
            // The servo alone cannot hold a limit: it clamps its own target
            // but nothing stops the joint physically rotating past it, and a
            // bounded impulse loses to gravity eventually. Once a joint passes
            // +/-pi the atan2 angle wraps, the servo's error changes sign, and
            // it drives the joint further -- a runaway that scrambled the leg
            // in the first 20 ms and dropped the fly on its belly.
            //
            // This is a one-sided constraint: it only ever pushes back into
            // range, never pulls, and its impulse is unbounded because a joint
            // stop is structural rather than muscular.
            {
                const float angle = p.angle;
                float violation = 0.0f;
                if (angle < j.minAngle) violation = j.minAngle - angle;
                else if (angle > j.maxAngle) violation = j.maxAngle - angle;

                if (violation != 0.0f) {
                    const V3& axis = p.axis;
                    const float eff = p.effAxis;
                    if (eff > 1e-12f) {
                        const float relSpin =
                            dot(B.angularVelocity - A.angularVelocity, axis);

                        // Drive the relative spin *to* the recovery rate, not
                        // add the recovery rate to it.
                        //
                        // This used to read (bias - outward), which on the
                        // first iteration is right and on the next 63 is a
                        // disaster. jointAngle only changes when positions are
                        // integrated, at the end of the substep, so `violation`
                        // -- and therefore `bias` -- is constant across the
                        // whole Gauss-Seidel sweep. Once the first iteration
                        // had satisfied the constraint, `outward` was zero and
                        // every later iteration re-applied the full impulse
                        // again, so the joint left the sweep spinning at
                        // iterations * bias. At 64 iterations and a 125 us
                        // substep that turned a 0.5 rad overshoot into 180,000
                        // rad/s; measured peak joint rates during a jump were
                        // 204,000 rad/s, where a real fly's fastest leg joint
                        // does a few hundred. Targeting the velocity instead
                        // makes the constraint idempotent, which is what lets
                        // it be swept repeatedly.
                        float bias = violation * invDt * params.baumgarte;
                        bias = std::clamp(bias, -params.maxCorrectionVelocity,
                                          params.maxCorrectionVelocity);
                        const float lambda = (bias - relSpin) / eff;

                        // One-sided: a joint stop pushes back into range and
                        // never pulls. If the joint is already recovering
                        // faster than the bias asks for, leave it alone.
                        if ((violation > 0.0f) == (lambda > 0.0f)) {
                            const V3 imp = axis * lambda;
                            A.angularVelocity += ia * (-imp);
                            B.angularVelocity += ib * imp;
                        }
                    }
                }
            }

            // --- weld: lock all three rotational degrees of freedom ---
            if (j.weld) {
                // Orientation error as a rotation vector. For a unit error
                // quaternion the vector part is half the rotation angle times
                // the axis, so 2*xyz is the small-angle rotation vector.
                // (Computed once per substep, in JointPre.)
                const V3& errVec = p.weldError;
                const V3 relOmega = B.angularVelocity - A.angularVelocity;

                const V3 basis[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
                for (int k = 0; k < 3; ++k) {
                    const V3& t = basis[k];
                    const float eff = p.effWeld[k];
                    if (eff < 1e-12f) continue;
                    const float bias = params.baumgarte * invDt * dot(errVec, t);
                    const float lambda = -(dot(relOmega, t) + bias) / eff;
                    const V3 imp = t * lambda;
                    A.angularVelocity += ia * (-imp);
                    B.angularVelocity += ib * imp;
                }
            }

            // --- angular part: keep the two axes parallel ---
            if (!j.weld) {
                // Error is how far the axes have drifted apart, measured in
                // the two directions perpendicular to the hinge. (Tangents,
                // error and effective masses come from JointPre.)
                const V3& err = p.axisError;
                const V3 relOmega = B.angularVelocity - A.angularVelocity;

                for (int k = 0; k < 2; ++k) {
                    const V3& t = p.tangent[k];
                    const float eff = p.effTangent[k];
                    if (eff < 1e-12f) continue;
                    const float bias = params.baumgarte * invDt * dot(err, t);
                    const float lambda = -(dot(relOmega, t) + bias) / eff;
                    const V3 imp = t * lambda;
                    A.angularVelocity += ia * (-imp);
                    B.angularVelocity += ib * imp;
                }
            }

            // --- linear part: keep the anchors coincident ---
            {
                // K = (1/mA + 1/mB) I - skew(rA) IA skew(rA) - skew(rB) IB skew(rB),
                // inverted once per substep in JointPre.
                const V3& rA = p.rA;
                const V3& rB = p.rB;
                const V3& err = p.anchorError;
                if (!p.kOk) continue;
                const M3& kInv = p.kInv;

                // Velocity constraint, with no positional bias: this impulse
                // is pure momentum exchange and adds no energy.
                const V3 relVel = B.pointVelocity(rB) - A.pointVelocity(rA);
                const V3 impulse = kInv * (-relVel);
                applyImpulseWith(A, ia, -impulse, rA);
                applyImpulseWith(B, ib, impulse, rB);
                j.accumulated += impulse;

                // Position correction, into the pseudo-velocities. These move
                // the bodies during integration and are then thrown away, so
                // the correction never becomes momentum.
                const V3 pseudoRel =
                    B.pseudoPointVelocity(rB) - A.pseudoPointVelocity(rA);
                const V3 bias = err * (params.baumgarte * invDt);
                const V3 pseudo = kInv * (-(pseudoRel + bias));
                applyPseudoImpulseWith(A, ia, -pseudo, rA);
                applyPseudoImpulseWith(B, ib, pseudo, rB);
            }
        }
    }

    for (auto& j : joints) j.linearImpulse = j.accumulated;
}

void PhysicsWorld::buildGroundContacts() {
    contacts.clear();

    // Each arena plane as an inward normal and an offset: a point p is inside
    // when dot(n, p) >= d, so penetration is d + radius - dot(n, p). Writing
    // all six the same way means the floor is not a special case, which is
    // what it used to be.
    struct PlaneDef { V3 n; float d; bool on; };
    const float gz = params.groundZ;
    const PlaneDef planes[kPlaneCount] = {
        {{ 0,  0,  1},  gz,                       params.groundOn},
        {{ 0,  0, -1}, -(gz + params.arenaHeight), params.ceilingOn},
        {{ 1,  0,  0}, -params.arenaHalfX,        params.wallsOn},
        {{-1,  0,  0}, -params.arenaHalfX,        params.wallsOn},
        {{ 0,  1,  0}, -params.arenaHalfY,        params.wallsOn},
        {{ 0, -1,  0}, -params.arenaHalfY,        params.wallsOn},
    };

    for (const auto& p : probes) {
        const RigidBody& b = bodies[p.body];
        const V3 world = b.position + b.orientation.rotate(p.localPoint);
        const auto probeIdx = static_cast<std::uint32_t>(&p - probes.data());

        for (int k = 0; k < kPlaneCount; ++k) {
            if (!planes[k].on) continue;
            const float depth =
                planes[k].d + p.radius - dot(planes[k].n, world);
            if (depth <= 0.0f) continue;

            Contact c;
            c.body = p.body;
            c.probe = probeIdx;
            c.plane = static_cast<std::uint8_t>(k);
            c.localPoint = p.localPoint;
            c.normal = planes[k].n;
            c.penetration = depth;
            // One probe against one plane is a stable identity between
            // frames, so its impulse can be carried across.
            const std::size_t key = probeIdx * kPlaneCount + k;
            if (key < probeImpulse.size()) c.normalImpulse = probeImpulse[key];
            contacts.push_back(c);
        }
    }
}

void PhysicsWorld::solveContacts(float dt) {
    const float invDt = dt > 0.0f ? 1.0f / dt : 0.0f;

    // Keyed on (probe, plane): a foot can touch the floor and a wall at
    // the same time and each contact needs its own carried impulse.
    probeImpulse.assign(probes.size() * kPlaneCount, 0.0f);

    // Contact geometry is fixed for the sweep, like the joints' (JointPre).
    contactPre_.resize(contacts.size());
    for (std::size_t n = 0; n < contacts.size(); ++n) {
        const Contact& c = contacts[n];
        const RigidBody& b = bodies[c.body];
        const M3& ii = invInertia_[c.body];
        ContactPre& p = contactPre_[n];
        p.r = b.orientation.rotate(c.localPoint);
        const V3 rn = cross(p.r, c.normal);
        p.effN = b.invMass + dot(rn, ii * rn);
        p.tangent[0] = perpendicular(c.normal);
        p.tangent[1] = cross(c.normal, p.tangent[0]);
        for (int k = 0; k < 2; ++k) {
            const V3 rt = cross(p.r, p.tangent[k]);
            p.effT[k] = b.invMass + dot(rt, ii * rt);
        }
    }

    for (int it = 0; it < params.iterations; ++it) {
        for (std::size_t n = 0; n < contacts.size(); ++n) {
            Contact& c = contacts[n];
            const ContactPre& p = contactPre_[n];
            RigidBody& b = bodies[c.body];
            const M3& ii = invInertia_[c.body];
            const V3& r = p.r;
            const V3 v = b.pointVelocity(r);

            // Normal: stop the point sinking, and push it back out slowly.
            const float effN = p.effN;
            if (effN < 1e-12f) continue;

            float lambda = -dot(v, c.normal) / effN;

            // Accumulated impulse is clamped, not the increment: a contact may
            // pull less than before but the total can never become adhesive.
            const float old = c.normalImpulse;
            c.normalImpulse = std::max(0.0f, old + lambda);
            lambda = c.normalImpulse - old;
            applyImpulseWith(b, ii, c.normal * lambda, r);

            // Push the body back out of the ground through the pseudo
            // velocities, so resting on the floor does not slowly trickle
            // energy into the fly.
            const float push = std::max(0.0f, c.penetration - params.slop);
            if (push > 0.0f) {
                const float pv = dot(b.pseudoPointVelocity(r), c.normal);
                const float bias = params.baumgarte * invDt * push;
                const float pl = std::max(0.0f, (bias - pv) / effN);
                applyPseudoImpulseWith(b, ii, c.normal * pl, r);
            }

            // Friction, in two tangent directions, clamped by Coulomb.
            for (int k = 0; k < 2; ++k) {
                const V3& t = p.tangent[k];
                const V3 vv = b.pointVelocity(r);
                const float effT = p.effT[k];
                if (effT < 1e-12f) continue;
                float lt = -dot(vv, t) / effT;
                const float maxT = params.friction * c.normalImpulse;
                const float oldT = c.tangentImpulse[k];
                c.tangentImpulse[k] = std::clamp(oldT + lt, -maxT, maxT);
                lt = c.tangentImpulse[k] - oldT;
                applyImpulseWith(b, ii, t * lt, r);
            }
        }
    }

    for (const auto& c : contacts) {
        const std::size_t key =
            static_cast<std::size_t>(c.probe) * kPlaneCount + c.plane;
        if (key < probeImpulse.size()) probeImpulse[key] = c.normalImpulse;
    }
}

void PhysicsWorld::integratePositions(float dt) {
    for (auto& b : bodies) {
        if (b.invMass <= 0.0f) continue;
        // Real velocity plus the position-correction velocity. The latter is
        // cleared below, so it moves the body without ever being momentum.
        // Cap the correction before it is used, so a persistent constraint
        // error cannot drive the body indefinitely.
        const float pv = length(b.pseudoVelocity);
        if (pv > params.maxCorrectionVelocity) {
            b.pseudoVelocity = b.pseudoVelocity * (params.maxCorrectionVelocity / pv);
        }
        const float pa = length(b.pseudoAngular);
        if (pa > params.maxAngularVelocity) {
            b.pseudoAngular = b.pseudoAngular * (params.maxAngularVelocity / pa);
        }
        b.position += (b.velocity + b.pseudoVelocity) * dt;
        const V3 spin = b.angularVelocity + b.pseudoAngular;
        // Quaternion derivative: q' = 0.5 * omega * q, integrated explicitly
        // and renormalised, which is accurate enough at these timesteps.
        const Quat w{0.0f, spin.x, spin.y, spin.z};
        const Quat dq = w * b.orientation;
        b.orientation = Quat{b.orientation.w + 0.5f * dq.w * dt,
                             b.orientation.x + 0.5f * dq.x * dt,
                             b.orientation.y + 0.5f * dq.y * dt,
                             b.orientation.z + 0.5f * dq.z * dt}
                            .normalised();
        b.force = {};
        b.torque = {};
        b.pseudoVelocity = {};
        b.pseudoAngular = {};
    }
}

void PhysicsWorld::step(float dt) {
    if (dt <= 0.0f) return;
    integrateVelocities(dt);
    buildGroundContacts();
    // Orientations are now fixed until integratePositions(): cache what the
    // solvers derive from them.
    cacheInvInertia();
    solveJoints(dt);
    solveContacts(dt);
    clampVelocities();
    integratePositions(dt);
}

}  // namespace fly
