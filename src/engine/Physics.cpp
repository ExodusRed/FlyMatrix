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

    for (auto& j : joints) {
        RigidBody& A = bodies[j.a];
        RigidBody& B = bodies[j.b];

        // Motor and passive spring act about the hinge axis. This is where a
        // muscle's force enters the simulation.
        const V3 axis = normalise(A.orientation.rotate(j.axisA));
        const float angle = jointAngle(j);
        const float relSpin = dot(B.angularVelocity - A.angularVelocity, axis);
        float t = j.motorTorque
                  + j.stiffness * (j.restAngle - angle)
                  - j.damping * relSpin;
        // A joint driven past its limit gets pushed back hard, which is
        // cheaper and steadier than adding a separate limit constraint.
        if (angle < j.minAngle) t += j.stiffness * 8.0f * (j.minAngle - angle);
        if (angle > j.maxAngle) t += j.stiffness * 8.0f * (j.maxAngle - angle);

        const V3 torque = axis * t;
        A.angularVelocity += A.invInertiaWorld() * (-torque) * dt;
        B.angularVelocity += B.invInertiaWorld() * torque * dt;

        // Re-clamp here as well: the motor is applied after integrateVelocities
        // has already clamped, and it is the largest single impulse in the step.
        for (RigidBody* b : {&A, &B}) {
            const float av = length(b->angularVelocity);
            if (av > params.maxAngularVelocity) {
                b->angularVelocity = b->angularVelocity * (params.maxAngularVelocity / av);
            }
        }
    }

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
        for (auto& j : joints) {
            RigidBody& A = bodies[j.a];
            RigidBody& B = bodies[j.b];

            // --- angular part: keep the two axes parallel ---
            {
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
