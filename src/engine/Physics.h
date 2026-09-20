#pragma once

#include <cstdint>
#include <vector>

#include "engine/Math.h"

namespace fly {

// A maximal-coordinate rigid body solver.
//
// Every segment is a free body with its own position and orientation, and
// joints are constraints that pull them back together. This is how Box2D and
// Bullet work, and it is the more general choice: the alternative, reduced
// coordinates, stores joint angles directly and cannot come apart, but only
// handles tree-shaped linkages.
//
// The cost is conditioning. A chain of bodies connected by constraints is
// solved iteratively, and error propagates along the chain, so long chains go
// soft and large mass ratios between neighbouring bodies make it worse. A fly's
// thorax against a tarsus segment is a ratio of hundreds to one. Two things
// keep it stable here: distal segment masses are inflated well past anatomy
// (see FlyPhysics), and the solver warm-starts from the previous frame's
// impulses so a standing pose does not have to be rediscovered every step.

struct RigidBody {
    float invMass = 0.0f;                  // 0 means immovable
    M3 invInertiaLocal = M3::zero();       // in body frame
    V3 position;
    Quat orientation;
    V3 velocity;
    V3 angularVelocity;
    V3 force;
    V3 torque;

    // Inverse inertia rotated into world space, which is what the solver needs.
    M3 invInertiaWorld() const {
        const M3 r = M3::fromQuat(orientation);
        return r * invInertiaLocal * r.transposed();
    }

    void applyImpulse(const V3& impulse, const V3& relativePoint) {
        velocity += impulse * invMass;
        angularVelocity += invInertiaWorld() * cross(relativePoint, impulse);
    }

    V3 pointVelocity(const V3& relativePoint) const {
        return velocity + cross(angularVelocity, relativePoint);
    }

    void setBoxInertia(float mass, const V3& halfExtents);
    void setCapsuleInertia(float mass, float radius, float length);
};

// A hinge: the two anchor points must coincide, and the two axes must stay
// parallel, leaving one rotational degree of freedom.
struct HingeJoint {
    std::uint32_t a = 0, b = 0;
    V3 anchorA, anchorB;   // in each body's local frame
    V3 axisA, axisB;       // in each body's local frame

    // Muscle torque about the hinge axis, set each step from activation.
    float motorTorque = 0.0f;
    // Passive stiffness and damping, standing in for the joint's own elasticity
    // and for the muscles that are not currently being driven.
    float restAngle = 0.0f;
    float stiffness = 0.0f;
    float damping = 0.0f;
    float minAngle = -3.2f, maxAngle = 3.2f;

    // Reference directions perpendicular to the axis, used to measure the
    // joint angle. Filled in by PhysicsWorld::prepare().
    V3 refA, refB;

    // Warm-start state, carried between frames.
    V3 linearImpulse;
    V3 accumulated;
};

struct Contact {
    std::uint32_t body = 0;
    std::uint32_t probe = 0;
    V3 localPoint;       // contact point in the body's frame
    V3 normal{0, 0, 1};  // world, pointing out of the ground
    float penetration = 0.0f;
    float normalImpulse = 0.0f;
    float tangentImpulse[2] = {0.0f, 0.0f};
};

class PhysicsWorld {
public:
    struct Params {
        V3 gravity{0, 0, -9810.0f};  // mm/s^2
        int iterations = 24;
        // Fraction of position error corrected per step. Too high and the
        // solver injects energy and jitters; too low and joints visibly sag.
        float baumgarte = 0.2f;
        // Penetration tolerated before any correction is applied, which stops
        // resting contacts from buzzing.
        float slop = 0.002f;  // mm
        float friction = 0.9f;
        float restitution = 0.0f;
        float linearDamping = 0.02f;
        float angularDamping = 0.04f;
        float groundZ = 0.0f;

        // Velocity ceilings. A large muscle torque on a light distal segment
        // produces an angular velocity per substep that no number of solver
        // iterations can reconcile, and the scene explodes. Clamping is crude
        // -- it silently discards momentum -- but it bounds the damage to one
        // frame instead of destroying the simulation, and the ceilings are set
        // well above anything a fly actually does (a leg segment sweeping at
        // 400 rad/s moves its tip at ~200 mm/s).
        float maxAngularVelocity = 400.0f;   // rad/s
        float maxLinearVelocity = 3000.0f;   // mm/s
    };

    Params params;
    std::vector<RigidBody> bodies;
    std::vector<HingeJoint> joints;
    std::vector<Contact> contacts;

    std::uint32_t addBody(const RigidBody& b);
    // Must be called after joints are added and bodies placed: caches the
    // reference directions used to measure joint angles.
    void prepare();

    void step(float dt);

    // Current angle of a hinge, in radians, measured from its rest reference.
    float jointAngle(const HingeJoint& j) const;

private:
    void integrateVelocities(float dt);
    void solveJoints(float dt);
    void solveContacts(float dt);
    void integratePositions(float dt);
    void buildGroundContacts();

    // Normal impulse per probe, kept between frames for warm starting.
    std::vector<float> probeImpulse;

    // Contact points are supplied by the caller each step via these.
public:
    // Candidate contact points: body index plus a point in its local frame,
    // usually a foot. Cleared and refilled by the owner each step.
    struct ContactProbe {
        std::uint32_t body;
        V3 localPoint;
        float radius;
    };
    std::vector<ContactProbe> probes;
};

}  // namespace fly
