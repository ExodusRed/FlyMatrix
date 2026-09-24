"""Solve leg rest angles so the fly stands with all six feet on the ground.

Mirrors the forward kinematics in src/body/FlyBody.cpp and runs damped least
squares (Levenberg-Marquardt) on the five joint angles of each leg until the
foot reaches a target contact point. Hand-tuning five coupled angles per leg to
a common ground height is tedious and easy to get subtly wrong; this just
solves it.

Prints the angles to paste back into FlyBody.cpp.

    python tools/solve_rest_pose.py
"""
from __future__ import annotations

import numpy as np

# Must match FlyBody.cpp.
BODY_Z = 0.62
BODY_RADIUS = 0.38
GROUND_Z = 0.0

# Mirrors src/body/Anatomy.h. If these disagree the solved rest pose is
# for a different animal than the one that gets built.
SEG = dict(coxa=0.26, troch=0.09, femur=0.54, tibia=0.50, tarsus=0.55)

LEGS = [
    # name, attachX, lengthScale, target foot (x, y, z) for the LEFT leg,
    # fixed outward mount tilt (radians, about the fore-aft axis)
    ("front",  0.34, 0.88, (0.92, 0.72, GROUND_Z), 0.657),
    ("middle", 0.02, 1.00, (0.05, 0.92, GROUND_Z), 0.909),
    ("hind",  -0.30, 1.12, (-0.92, 0.88, GROUND_Z), 0.893),
]

# Axes for a left leg, in the order ThC, CTr, TrF, FTi, TiTa.
AXES = np.array([
    [0.0, 1.0, 0.0],   # ThC  lateral axis: protracts and retracts -- the step
    [0.0, 1.0, 0.0],   # CTr  lateral axis: depresses the femur -- the power joint
    [0.0, 0.0, 1.0],   # TrF  vertical: twists the femur
    [0.0, 1.0, 0.0],   # FTi  lateral: the knee
    [0.0, 1.0, 0.0],   # TiTa lateral: the ankle
])

# A fly's legs are long relative to its ride height -- roughly 2 mm of leg
# holding the body half a millimetre up -- so a standing pose needs a deeply
# folded knee and plenty of fore-aft swing. Narrow limits make the target
# unreachable and the solver stalls against them.
# ThC is kept well short of horizontal. Left free it saturates, and the solver
# then reaches the foot target by routing the femur up over the thorax -- a
# valid solution to the equations and a nonsense one for a fly.
#
# FTi is allowed to bend either way. A real fly's front legs fold with the
# tibia swinging back and the hind legs with it swinging forward, so forcing
# one sign made the hind leg's knee undo its own backward reach.
LIMITS = np.array([
    [-0.9, 0.9],
    [-1.6, 1.6],
    [-0.9, 0.9],
    [-2.6, 2.6],
    [-1.2, 1.5],
])

JOINT_NAMES = ["ThC", "CTr", "TrF", "FTi", "TiTa"]


def quat_axis_angle(axis, angle):
    a = axis / (np.linalg.norm(axis) + 1e-12)
    h = angle * 0.5
    s = np.sin(h)
    return np.array([np.cos(h), a[0] * s, a[1] * s, a[2] * s])


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_rotate(q, v):
    qv = q[1:]
    t = 2.0 * np.cross(qv, v)
    return v + q[0] * t + np.cross(qv, t)


def foot_position(angles, attach, lengths, mount):
    """Forward kinematics, matching FlyBody::footPosition."""
    pos = np.array(attach, dtype=float)
    # The leg is mounted with a fixed outward tilt before any joint applies.
    rot = quat_axis_angle(np.array([1.0, 0.0, 0.0]), mount)
    for i in range(5):
        rot = quat_mul(rot, quat_axis_angle(AXES[i], angles[i]))
        pos = pos + quat_rotate(rot, np.array([0.0, 0.0, -1.0])) * lengths[i]
    return pos


def solve(attach, lengths, target, seed, mount, limits):
    angles = seed.copy()
    lam = 1e-3
    for _ in range(1500):
        err = foot_position(angles, attach, lengths, mount) - target
        if np.linalg.norm(err) < 1e-6:
            break
        # Numerical Jacobian: 5 columns, cheap enough at this size.
        J = np.zeros((3, 5))
        eps = 1e-5
        for k in range(5):
            bumped = angles.copy()
            bumped[k] += eps
            J[:, k] = (foot_position(bumped, attach, lengths, mount) - err - target) / eps
        # Damped least squares, with a pull toward the seed so the five
        # redundant joints settle on a natural posture instead of drifting.
        A = J.T @ J + (lam + 0.002) * np.eye(5)
        b = J.T @ err + 0.002 * (angles - seed)
        angles = np.clip(angles - np.linalg.solve(A, b), limits[:, 0], limits[:, 1])
    return angles


def solve_leg(attach_x, scale, target, mount, branch):
    """Best posture for one leg, restricted to a given joint configuration.

    `branch` gives the required sign of CTr and FTi. Restricting the limits to
    one side of zero is what forces the choice: the equations are happy either
    way, and left free each leg picks its own.
    """
    attach = (attach_x, BODY_RADIUS * 0.75, BODY_Z - BODY_RADIUS * 0.35)
    lengths = np.array([SEG["coxa"], SEG["troch"], SEG["femur"],
                        SEG["tibia"], SEG["tarsus"]]) * scale

    limits = LIMITS.copy()
    for joint, sign in ((1, branch[0]), (3, branch[1])):
        if sign > 0:
            limits[joint, 0] = max(limits[joint, 0], 0.05)
        else:
            limits[joint, 1] = min(limits[joint, 1], -0.05)

    # Five joints reaching a three-dimensional target is redundant, so the
    # solution found depends on where the search starts. Try a spread of
    # postures and keep whichever lands closest. A leg that ends up with three
    # joints jammed against their limits means the starting posture was wrong,
    # not the target.
    best, best_score = None, float("inf")
    for thc in (-0.5, 0.0, 0.5):
        for ctr in (-1.2, -0.6, 0.6, 1.2):
            for knee in (-2.2, -1.2, 1.2, 2.2):
                seed = np.clip(np.array([thc, ctr, 0.0, knee, 0.4]),
                               limits[:, 0], limits[:, 1])
                angles = solve(attach, lengths, np.array(target, float), seed,
                               mount, limits)
                err = np.linalg.norm(
                    foot_position(angles, attach, lengths, mount)
                    - np.array(target))
                margin = np.min(np.minimum(angles - limits[:, 0],
                                           limits[:, 1] - angles))
                # Penalise solutions pressed against a joint limit.
                score = err + max(0.0, 0.05 - margin) * 2.0
                if score < best_score:
                    best, best_score = angles, score
    return best, best_score, attach, lengths


def main() -> None:
    print(f"body at z = {BODY_Z}, ground at z = {GROUND_Z}")

    # All six legs must end up in the same joint configuration.
    #
    # The previous version solved each leg independently and scored it on foot
    # error alone. Nothing asked the three leg types to agree, and they did
    # not: the front legs came out with CTr positive and FTi negative, the
    # middle and hind legs the other way round. That is a valid solution per
    # leg and a broken animal, because the motor pools apply one sign per joint
    # to all six legs, so a single "extend" command extended some legs and
    # flexed others. flyphys test 3 measures this directly.
    #
    # So the branch is chosen once, globally, and every leg is solved inside
    # it. Whichever branch reaches all three targets best is the one that ships.
    best_branch, best_total, best_results = None, float("inf"), None
    for ctr_sign in (+1, -1):
        for fti_sign in (+1, -1):
            branch = (ctr_sign, fti_sign)
            results, total = {}, 0.0
            for name, attach_x, scale, target, mount in LEGS:
                angles, score, attach, lengths = solve_leg(
                    attach_x, scale, target, mount, branch)
                results[name] = (angles, attach, lengths, target, mount)
                total += score
            sgn = lambda v: "+" if v > 0 else "-"
            print(f"  branch CTr {sgn(ctr_sign)}  FTi {sgn(fti_sign)}"
                  f"   total score {total:.4f}")
            if total < best_total:
                best_branch, best_total, best_results = branch, total, results

    sgn = lambda v: "+" if v > 0 else "-"
    print()
    print(f"chosen branch: CTr {sgn(best_branch[0])}, "
          f"FTi {sgn(best_branch[1])}")
    print()

    results = {}
    for name, (angles, attach, lengths, target, mount) in best_results.items():
        foot = foot_position(angles, attach, lengths, mount)
        err = np.linalg.norm(foot - np.array(target))
        margin = np.min(np.minimum(angles - LIMITS[:, 0], LIMITS[:, 1] - angles))
        results[name] = angles
        print(f"{name:<7} foot = ({foot[0]:+.3f}, {foot[1]:+.3f}, {foot[2]:+.3f})"
              f"   error {err*1000:6.1f} um   limit margin {margin:.3f} rad")

    print("\nrest angles, radians:")
    print(f"  {'leg':<8}" + "".join(f"{j:>9}" for j in JOINT_NAMES))
    for name, angles in results.items():
        print(f"  {name:<8}" + "".join(f"{a:>9.3f}" for a in angles))

    print("\npaste into FlyBody.cpp kLayout as restAngles:")
    for name, angles in results.items():
        vals = ", ".join(f"{a:.3f}f" for a in angles)
        print(f"  // {name}")
        print(f"  {{{vals}}},")


if __name__ == "__main__":
    main()
