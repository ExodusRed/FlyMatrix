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

    // Split impulse: position error is corrected through a second, parallel
    // set of velocities that move the bodies but are discarded afterwards.
    //
    // Folding position correction into real velocity, the usual Baumgarte
    // approach, leaves that correction behind as momentum. At a low
    // coefficient it is a slow leak; at the 0.7 needed here to stop a
    // five-link leg compressing, it accumulated until the fly launched itself
    // 26 mm into the air and stayed there.
    V3 pseudoVelocity;
    V3 pseudoAngular;

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

    void applyPseudoImpulse(const V3& impulse, const V3& relativePoint) {
        pseudoVelocity += impulse * invMass;
        pseudoAngular += invInertiaWorld() * cross(relativePoint, impulse);
    }

    V3 pseudoPointVelocity(const V3& relativePoint) const {
        return pseudoVelocity + cross(pseudoAngular, relativePoint);
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

    // The joint is driven as a position servo solved at the velocity level,
    // not as an explicit spring torque.
    //
    // An explicit spring cannot be made stiff enough to hold a body up before
    // it goes unstable: raising the stiffness seventeen-fold made the fly's
    // proximal joints sag *further* (0.35 rad to 0.87 rad) and set the feet
    // sliding, because the spring had passed the timestep's stability limit
    // and was injecting energy instead of resisting. Solving it as a
    // constraint is unconditionally stable, and the joint's strength becomes
    // an impulse bound rather than a gain that can explode.
    float targetAngle = 0.0f;
    // Fraction of the remaining angle error the servo tries to erase each
    // substep, like a Baumgarte coefficient. Dimensionless on purpose: an
    // absolute rate in 1/s does not scale with the timestep, so it corrects a
    // fixed amount per second while gravity disturbs the joint every substep,
    // and the error grows no matter how much torque the joint is allowed.
    float servoRate = 0.25f;
    // Largest torque this joint can exert to reach its target. Posture holding
    // and muscle contraction both draw on this.
    float maxTorque = 0.0f;
    // Feed-forward torque from muscle contraction, applied once per step
    // on top of the posture servo. Kept separate on purpose: folding the
    // muscle into the servo's target angle made a muscle's strength
    // almost irrelevant to the movement it produced, because the target
    // shift saturated. A weak muscle and a jump muscle then differed only
    // in how hard they held a target they both reached, and whole-body
    // activation from anywhere in the brain out-jumped the escape circuit.
    float muscleTorque = 0.0f;
    float damping = 0.0f;
    float minAngle = -3.2f, maxAngle = 3.2f;

    // Rigid attachment: locks all three rotational degrees of freedom rather
    // than leaving one free. Two hinges sharing an anchor were standing in for
    // this, which does not actually lock rotation -- the abdomen sagged 79 um
    // below the thorax, the largest single contributor to the fly settling
    // onto its belly.
    bool weld = false;

    // Reference directions perpendicular to the axis, used to measure the
    // joint angle. Filled in by PhysicsWorld::prepare().
    V3 refA, refB;
    // Relative orientation the weld holds, captured at prepare().
    Quat weldRest;

    // Warm-start state, carried between frames.
    V3 linearImpulse;
    V3 accumulated;
    float servoImpulse = 0.0f;
    float muscleImpulse = 0.0f;
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
        // A 5-link leg needs far more Gauss-Seidel passes than a typical
        // game scene: at 24 the legs behaved as if compressible and the
        // fly sank 0.1 mm into them.
        int iterations = 64;
        // Fraction of position error corrected per step. Too high and the
        // solver injects energy and jitters; too low and joints visibly sag.
        // 0.2 is the usual starting value and was far too soft here -- the
        // accumulated anchor error down a five-joint chain cost 0.1 mm of ride
        // height and tipped the fly onto its abdomen. 0.7 costs nothing
        // measurable in stability at this timestep.
        float baumgarte = 0.7f;
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

        // Ceiling on the split-impulse correction velocity.
        //
        // Position correction moves a body without that motion ever being
        // momentum, which is the whole point -- but it also means gravity
        // cannot oppose it. Left uncapped, a joint error that is recreated
        // every substep becomes a steady upward teleport: the fly climbed
        // past 300 mm at a constant 940 mm/s with no ground contact at all,
        // never decelerating, because it was not actually moving under its
        // own velocity.
        // Swept against both failure modes it sits between: too low and a
        // body that has sunk into the floor cannot climb back out, too
        // high and persistent joint error drives the fly upward.
        float maxCorrectionVelocity = 150.0f;  // mm/s
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

    // Largest distance by which any joint's two anchor points have come
    // apart. A constraint solver is only as good as this number: if it
    // grows under load the linkage is stretching, and every downstream
    // measurement is describing a body that is quietly falling apart.
    float maxAnchorError() const;

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
