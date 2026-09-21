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

}  // namespace

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

void PhysicsWorld::integrateVelocities(float dt) {
    for (auto& b : bodies) {
        if (b.invMass <= 0.0f) continue;
        b.velocity += (params.gravity + b.force * b.invMass) * dt;
        b.angularVelocity += b.invInertiaWorld() * b.torque * dt;
        b.velocity = b.velocity * (1.0f / (1.0f + params.linearDamping * dt));
        b.angularVelocity =
            b.angularVelocity * (1.0f / (1.0f + params.angularDamping * dt));

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

    for (auto& j : joints) j.servoImpulse = 0.0f;

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

            // --- servo: drive the hinge toward its target angle ---
            //
            // Solved as a velocity constraint with a bounded impulse rather
            // than applied as a torque. The bound is what gives a joint finite
            // strength: it can be made arbitrarily strong without the
            // integrator ever seeing a stiff force.
            if (j.maxTorque > 0.0f) {
                const V3 axis = normalise(A.orientation.rotate(j.axisA));
                const float target = std::clamp(j.targetAngle, j.minAngle, j.maxAngle);
                const float angle = jointAngle(j);

                const M3 ia = A.invInertiaWorld();
                const M3 ib = B.invInertiaWorld();
                const float eff = dot(axis, ia * axis) + dot(axis, ib * axis);
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
                const float angle = jointAngle(j);
                float violation = 0.0f;
                if (angle < j.minAngle) violation = j.minAngle - angle;
                else if (angle > j.maxAngle) violation = j.maxAngle - angle;

                if (violation != 0.0f) {
                    const V3 axis = normalise(A.orientation.rotate(j.axisA));
                    const M3 ia = A.invInertiaWorld();
                    const M3 ib = B.invInertiaWorld();
                    const float eff = dot(axis, ia * axis) + dot(axis, ib * axis);
                    if (eff > 1e-12f) {
                        const float relSpin =
                            dot(B.angularVelocity - A.angularVelocity, axis);
                        // Push back toward the limit, and cancel any velocity
                        // still carrying the joint outward.
                        const float bias = violation * invDt * params.baumgarte;
                        const float outward =
                            (violation > 0.0f) ? std::min(relSpin, 0.0f)
                                               : std::max(relSpin, 0.0f);
                        const float lambda = (bias - outward) / eff;

                        const V3 imp = axis * lambda;
                        A.angularVelocity += ia * (-imp);
                        B.angularVelocity += ib * imp;
                    }
                }
            }

            // --- weld: lock all three rotational degrees of freedom ---
            if (j.weld) {
                // Orientation error as a rotation vector. For a unit error
                // quaternion the vector part is half the rotation angle times
                // the axis, so 2*xyz is the small-angle rotation vector.
                const Quat conjA{A.orientation.w, -A.orientation.x,
                                 -A.orientation.y, -A.orientation.z};
                const Quat want = (A.orientation * j.weldRest).normalised();
                const Quat conjWant{want.w, -want.x, -want.y, -want.z};
                Quat err = (conjWant * B.orientation).normalised();
                if (err.w < 0.0f) {  // shortest arc
                    err = {-err.w, -err.x, -err.y, -err.z};
                }
                const V3 errVec{2.0f * err.x, 2.0f * err.y, 2.0f * err.z};
                (void)conjA;

                const M3 ia = A.invInertiaWorld();
                const M3 ib = B.invInertiaWorld();
                const V3 relOmega = B.angularVelocity - A.angularVelocity;

                const V3 basis[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
                for (const V3& t : basis) {
                    const float eff = dot(t, ia * t) + dot(t, ib * t);
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
                const V3 axA = normalise(A.orientation.rotate(j.axisA));
                const V3 axB = normalise(B.orientation.rotate(j.axisB));
                const V3 t1 = perpendicular(axA);
                const V3 t2 = cross(axA, t1);
                // Error is how far the axes have drifted apart, measured in
                // the two directions perpendicular to the hinge.
                const V3 err = cross(axA, axB);

                const M3 ia = A.invInertiaWorld();
                const M3 ib = B.invInertiaWorld();
                const V3 relOmega = B.angularVelocity - A.angularVelocity;

                for (const V3& t : {t1, t2}) {
                    const float eff = dot(t, ia * t) + dot(t, ib * t);
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
                const V3 rA = A.orientation.rotate(j.anchorA);
                const V3 rB = B.orientation.rotate(j.anchorB);
                const V3 pA = A.position + rA;
                const V3 pB = B.position + rB;
                const V3 err = pB - pA;

                const M3 ia = A.invInertiaWorld();
                const M3 ib = B.invInertiaWorld();
                const M3 sa = M3::skew(rA);
                const M3 sb = M3::skew(rB);
                // K = (1/mA + 1/mB) I - skew(rA) IA skew(rA) - skew(rB) IB skew(rB)
                M3 k = M3::diagonal(A.invMass + B.invMass,
                                    A.invMass + B.invMass,
                                    A.invMass + B.invMass);
                k = k + (-(sa * ia * sa)) + (-(sb * ib * sb));

                M3 kInv;
                if (!k.invert(kInv)) continue;

                const V3 relVel = B.pointVelocity(rB) - A.pointVelocity(rA);
                const V3 bias = err * (params.baumgarte * invDt);
                const V3 impulse = kInv * (-(relVel + bias));

                A.applyImpulse(-impulse, rA);
                B.applyImpulse(impulse, rB);
                j.accumulated += impulse;
            }
        }
    }

    for (auto& j : joints) j.linearImpulse = j.accumulated;
}

void PhysicsWorld::buildGroundContacts() {
    contacts.clear();
    for (const auto& p : probes) {
        const RigidBody& b = bodies[p.body];
        const V3 world = b.position + b.orientation.rotate(p.localPoint);
        const float depth = params.groundZ + p.radius - world.z;
        if (depth <= 0.0f) continue;
        Contact c;
        c.body = p.body;
        c.probe = static_cast<std::uint32_t>(&p - probes.data());
        c.localPoint = p.localPoint;
        c.normal = {0, 0, 1};
        c.penetration = depth;
        // One probe is one foot, so a contact keeps its identity between
        // frames and its impulse can be carried across.
        if (c.probe < probeImpulse.size()) c.normalImpulse = probeImpulse[c.probe];
        contacts.push_back(c);
    }
}

void PhysicsWorld::solveContacts(float dt) {
    const float invDt = dt > 0.0f ? 1.0f / dt : 0.0f;

    probeImpulse.assign(probes.size(), 0.0f);

    for (int it = 0; it < params.iterations; ++it) {
        for (auto& c : contacts) {
            RigidBody& b = bodies[c.body];
            const V3 r = b.orientation.rotate(c.localPoint);
            const V3 v = b.pointVelocity(r);

            // Normal: stop the point sinking, and push it back out slowly.
            const M3 ii = b.invInertiaWorld();
            const V3 rn = cross(r, c.normal);
            const float effN = b.invMass + dot(rn, ii * rn);
            if (effN < 1e-12f) continue;

            const float push = std::max(0.0f, c.penetration - params.slop);
            const float bias = params.baumgarte * invDt * push;
            float lambda = -(dot(v, c.normal) - bias) / effN;

            // Accumulated impulse is clamped, not the increment: a contact may
            // pull less than before but the total can never become adhesive.
            const float old = c.normalImpulse;
            c.normalImpulse = std::max(0.0f, old + lambda);
            lambda = c.normalImpulse - old;
            b.applyImpulse(c.normal * lambda, r);

            // Friction, in two tangent directions, clamped by Coulomb.
            const V3 t1 = perpendicular(c.normal);
            const V3 t2 = cross(c.normal, t1);
            const V3 tangents[2] = {t1, t2};
            for (int k = 0; k < 2; ++k) {
                const V3 t = tangents[k];
                const V3 vv = b.pointVelocity(r);
                const V3 rt = cross(r, t);
                const float effT = b.invMass + dot(rt, ii * rt);
                if (effT < 1e-12f) continue;
                float lt = -dot(vv, t) / effT;
                const float maxT = params.friction * c.normalImpulse;
                const float oldT = c.tangentImpulse[k];
                c.tangentImpulse[k] = std::clamp(oldT + lt, -maxT, maxT);
                lt = c.tangentImpulse[k] - oldT;
                b.applyImpulse(t * lt, r);
            }
        }
    }

    for (const auto& c : contacts) {
        if (c.probe < probeImpulse.size()) probeImpulse[c.probe] = c.normalImpulse;
    }
}

void PhysicsWorld::integratePositions(float dt) {
    for (auto& b : bodies) {
        if (b.invMass <= 0.0f) continue;
        b.position += b.velocity * dt;
        // Quaternion derivative: q' = 0.5 * omega * q, integrated explicitly
        // and renormalised, which is accurate enough at these timesteps.
        const Quat w{0.0f, b.angularVelocity.x, b.angularVelocity.y,
                     b.angularVelocity.z};
        const Quat dq = w * b.orientation;
        b.orientation = Quat{b.orientation.w + 0.5f * dq.w * dt,
                             b.orientation.x + 0.5f * dq.x * dt,
                             b.orientation.y + 0.5f * dq.y * dt,
                             b.orientation.z + 0.5f * dq.z * dt}
                            .normalised();
        b.force = {};
        b.torque = {};
    }
}

void PhysicsWorld::step(float dt) {
    if (dt <= 0.0f) return;
    integrateVelocities(dt);
    buildGroundContacts();
    solveJoints(dt);
    solveContacts(dt);
    integratePositions(dt);
}

}  // namespace fly
