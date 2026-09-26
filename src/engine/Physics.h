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
    // Cached, because it is an invariant of the solve and was being rebuilt
    // as though it were not.
    //
    // R * I_local * R^T depends only on orientation, and orientation does not
    // change while the solver iterates -- positions are integrated once, at
    // the end of the substep. The solver asks for it about four times per
    // joint per iteration, which at 64 iterations, 56 joints and 8 kHz
    // substeps is some 229 million quaternion-to-matrix conversions and
    // 3x3 products per two seconds of simulated time, all of them returning
    // the same answer.
    //
    // refreshInertiaWorld() is called for every body at the top of
    // PhysicsWorld::step, so anything inside a step sees a current value. A
    // caller that rotates a body by hand and then asks must refresh it.
    M3 invInertiaWorld() const { return invInertiaW_; }

    void refreshInertiaWorld() {
        const M3 r = M3::fromQuat(orientation);
        invInertiaW_ = r * invInertiaLocal * r.transposed();
    }

    M3 invInertiaW_ = M3::zero();

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

// The six planes of the arena, in the order buildGroundContacts tests them.
// A probe can touch more than one at a time -- a foot in a corner touches
// three -- so the plane index is part of a contact's identity, not just the
// probe.
enum class Plane : std::uint8_t {
    Floor = 0, Ceiling, MinX, MaxX, MinY, MaxY, Count
};
constexpr int kPlaneCount = static_cast<int>(Plane::Count);

struct Contact {
    std::uint32_t body = 0;
    std::uint32_t probe = 0;
    // Which arena plane this contact is against. Warm starting is keyed on
    // (probe, plane): keyed on the probe alone, a foot touching the floor and
    // a wall would have the two contacts overwrite each other's stored
    // impulse every frame and neither would converge.
    std::uint8_t plane = 0;
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
        // Alternate one joint sweep with one contact sweep, instead of
        // running every joint iteration and then every contact one.
        bool interleave = false;

        // The arena: an axis-aligned box the fly is contained by.
        //
        // The floor is what the fly has always stood on. The walls and
        // ceiling are off by default so that every existing measurement is
        // unchanged -- a jump reaches 7 mm and would start bouncing off a
        // ceiling that was not there when the numbers were taken.
        //
        // Extents are half-widths from the origin, in millimetres, and the
        // fly is about 2.5 mm long.
        bool groundOn = true;
        bool wallsOn = false;
        bool ceilingOn = false;
        float arenaHalfX = 15.0f;
        float arenaHalfY = 15.0f;
        float arenaHeight = 18.0f;  // above groundZ

        // Velocity ceilings. A large muscle torque on a light distal segment
        // produces an angular velocity per substep that no number of solver
        // iterations can reconcile, and the scene explodes. Clamping is crude
        // -- it silently discards momentum -- but it bounds the damage to one
        // frame instead of destroying the simulation, and the ceilings are set
        // well above anything a fly actually does (a leg segment sweeping at
        // 400 rad/s moves its tip at ~200 mm/s).
        float maxAngularVelocity = 400.0f;   // rad/s

        // Fastest relative spin the position servo is allowed to ask a joint
        // for, rad/s.
        //
        // The servo's target rate was servoRate * error * invDt, which at
        // servoRate 0.9 and 8 kHz substeps is 7200 rad/s for every radian of
        // error: it demands the whole error be erased inside a fraction of
        // one substep. Standing, the errors are small enough that nothing
        // notices. Walking, they reach a few tenths of a radian, the servo
        // asks for thousands of rad/s, and the body speed ceiling above then
        // fires -- 12,249 times in two seconds -- rescaling one body's
        // velocity without its neighbours and pulling the joints apart that
        // the solver had just satisfied. The linkage burst that ended every
        // long run was this, not a loss of balance: the anchor error went
        // from 0.047 mm to 1.42 mm in a single step while height, pitch and
        // contacts were all still normal.
        //
        // Tying controller stiffness to the integrator's timestep is the
        // actual error. It is also why finer substeps made things worse
        // rather than better, which had been read as the model being
        // chaotic.
        // Left wide. Bounding it to 200 measured far worse -- peak spin
        // went from 29,000 rad/s to 1.4 million -- because the stiff servo
        // was the thing holding the tarsal chain together, and the runaway
        // was in the unclamped angular constraints below rather than here.
        // Kept as a knob because the coupling to invDt is still wrong.
        float maxServoSpin = 20000.0f;  // rad/s

        // Time constant of the position servo, seconds. Zero keeps the old
        // behaviour.
        //
        // The servo asked for servoRate * error * invDt, which is a gain of
        // 7200 rad/s per radian at 8 kHz: it demands the whole error be
        // erased inside a fraction of one substep, and it demands more of it
        // the finer the substep. That is a controller tuned by the
        // integrator. Capping the demand does not fix it, because small
        // errors still see the full gain -- measured, a cap of 300 rad/s only
        // moved the failure later. Dividing by a real time constant lowers
        // the gain everywhere and makes the servo mean the same thing at any
        // timestep.
        //
        // 1 ms. At the old, timestep-derived 0.1125 ms the distal tarsomere
        // reached 2.2 million rad/s and the linkage tore itself apart inside
        // four seconds every time; at 1 ms the peak is 3,005 rad/s and it
        // does not. Slower than about 3 ms and the servo is too soft to
        // carry a stride.
        float servoTau = 0.001f;  // seconds
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

    // How many times the speed ceiling has actually fired.
    //
    // The clamp runs after the solvers, which is where it has to be if it is
    // to bound constraint impulses at all -- but it scales one body's
    // velocity without touching its neighbours, so every time it fires it
    // breaks the joint the solver has just satisfied. A linkage that comes
    // apart in a single step is exactly what that looks like from outside.
    std::size_t clampHits = 0;
    std::size_t clampLinear = 0;
    std::size_t clampAngular = 0;
    float peakAngular = 0.0f;      // largest speed that was clamped
    float peakAngularSeen = 0.0f;  // largest seen at all
    std::uint32_t peakAngularBody = 0;

    // Which constraint block put the spin there.
    //
    // Six different places apply angular impulses and every guess about which
    // one was responsible for the distal tarsomere reaching 13,000 rad/s was
    // wrong -- the servo, the speed ceiling, the Baumgarte terms, the tarsal
    // stiffness and the tarsomere mass were each ruled out by experiment.
    // This records the largest change in spin each block actually applies.
    enum Blame { kServo, kMuscle, kLimit, kWeld, kAxis, kAnchor, kBlameCount };
    float blame[kBlameCount] = {};

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
    // Index of the joint carrying that error, for locating it.
    std::size_t worstAnchorJoint() const;

    // Rate of change of a hinge's angle, rad/s, positive in the same
    // direction the angle is measured.
    float jointRate(const HingeJoint& j) const;

private:
    void integrateVelocities(float dt);
    void clampVelocities();
    void beginJoints();
    void jointPass(float dt);
    void beginContacts();
    void contactPass(float dt);
    void endContacts();
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
