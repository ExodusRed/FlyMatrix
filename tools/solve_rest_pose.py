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

import pathlib
import re
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent

# Must match FlyBody.cpp.
BODY_Z = 0.62
BODY_RADIUS = 0.38
GROUND_Z = 0.0
# The ankle rides a tarsus-radius above the floor so the tarsus lying flat
# rests on it rather than through it.
ANKLE_Z = 0.020
# Every joint down the leg must sit at least this far above the floor.
GROUND_CLEARANCE = 0.015
# Segment radii, matching the capsules FlyBody builds. A leg segment is not a
# line: a femur whose axis sits 20 um up is 16 um through the floor, because
# the femur is 36 um thick. The clearance test compared joint heights against
# a single 15 um figure and so passed poses that were plainly underground.
RADIUS = dict(coxa=0.045, troch=0.040, femur=0.036, tibia=0.028, tarsus=0.020)
# Only the upper leg is held clear. The ankle and the tarsus are supposed to
# be on the floor -- that is what standing is -- and their heights are set by
# ANKLE_Z instead.
CLEAR_RADII = [RADIUS["coxa"], RADIUS["troch"], RADIUS["femur"]]
# Millimetres of fore-aft foot travel each leg must have in hand, each way,
# before a joint reaches a stop. The gait that walks at 7.9 mm/s uses a
# 0.475 mm stride, so 0.30 mm each way is that with room to spare.
WANT_STRIDE = 0.30

# Mirrors src/body/Anatomy.h. If these disagree the solved rest pose is
# for a different animal than the one that gets built.
SEG = dict(coxa=0.26, troch=0.09, femur=0.54, tibia=0.50, tarsus=0.55)

LEGS = [
    # name, attachX, lengthScale, target foot (x, y, z) for the LEFT leg,
    # fixed outward mount tilt (radians, about the fore-aft axis)
    ("front",  0.34, 0.88, (0.48, 0.64, ANKLE_Z), 0.657),
    ("middle", 0.02, 1.00, (-0.15, 0.92, ANKLE_Z), 0.909),
    ("hind",  -0.30, 1.12, (-1.12, 0.88, ANKLE_Z), 0.893),
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
def _limits_from_header():
    """Read the joint limits out of src/body/Anatomy.h.

    Parsed rather than copied. These were duplicated here as literals and
    drifted from the C++ on four joints out of five, so the solver produced
    poses the physics rejected on the first step -- every leg's knee started
    outside its limit and the fly stood 0.37 mm too high on four feet.
    """
    src = (ROOT / "src" / "body" / "Anatomy.h").read_text(encoding="utf-8")
    body = src[src.index("kJointLimit[5][2] = {"):]
    body = body[:body.index("};")]
    rows = re.findall(r"\{\s*(-?[0-9.]+)f\s*,\s*(-?[0-9.]+)f\s*\}", body)
    if len(rows) != 5:
        sys.exit("could not parse kJointLimit from Anatomy.h")
    return np.array([[float(a), float(b)] for a, b in rows])


LIMITS = _limits_from_header()

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


def joint_positions(angles, attach, lengths, mount):
    """Every joint position down the leg, for checking ground clearance."""
    out = []
    pos = np.array(attach, dtype=float)
    rot = quat_axis_angle(np.array([1.0, 0.0, 0.0]), mount)
    for i in range(5):
        rot = quat_mul(rot, quat_axis_angle(AXES[i], angles[i]))
        pos = pos + quat_rotate(rot, np.array([0.0, 0.0, -1.0])) * lengths[i]
        out.append(pos.copy())
    return out


def chain(angles, attach, lengths, mount, n):
    """Walk the first n joints, returning the position and frame after them."""
    pos = np.array(attach, dtype=float)
    # The leg is mounted with a fixed outward tilt before any joint applies.
    rot = quat_axis_angle(np.array([1.0, 0.0, 0.0]), mount)
    for i in range(n):
        rot = quat_mul(rot, quat_axis_angle(AXES[i], angles[i]))
        pos = pos + quat_rotate(rot, np.array([0.0, 0.0, -1.0])) * lengths[i]
    return pos, rot


def foot_position(angles, attach, lengths, mount):
    """Forward kinematics, matching FlyBody::footPosition."""
    return chain(angles, attach, lengths, mount, 5)[0]


def ankle_position(angles, attach, lengths, mount):
    """Position of the tibia-tarsus joint: the chain through FTi only."""
    return chain(angles, attach, lengths, mount, 4)[0]


def flat_tarsus_angle(angles, attach, lengths, mount):
    """TiTa angle that lays the tarsus flat along the ground.

    A real fly does not stand on the tip of its tarsus. The tarsus lies
    *along* the substrate -- that is what the five tarsomeres are for, and
    what the adhesive pads act through. Solving the rest pose for a single
    point foot put the rest of a jointed tarsus below the floor, which is
    both why the legs appeared to clip through it and why switching on
    per-tarsomere contact probes lifted the body to 0.94 mm.

    After the first four joints the frame is fixed, and the tarsus points
    along R * R_y(t) * (0,0,-1). Setting the vertical component of that to
    zero is one equation in one unknown.
    """
    _, rot = chain(angles, attach, lengths, mount, 4)
    # Columns of R applied to the two basis directions the hinge sweeps.
    ez = quat_rotate(rot, np.array([0.0, 0.0, -1.0]))
    ex = quat_rotate(rot, np.array([-1.0, 0.0, 0.0]))
    # dir(t) = cos(t)*ez + sin(t)*ex; want dir.z == 0.
    t = np.arctan2(-ez[2], ex[2])
    # Two solutions half a turn apart. Take the one that carries on in the
    # direction the leg is already heading, rather than always pointing
    # forward: a fly's front tarsi point forward and its hind tarsi trail
    # back, and forcing them all forward jams the hind leg's TiTa against
    # its limit and lifts the tarsus 77 um off the floor.
    ankle, _ = chain(angles, attach, lengths, mount, 4)
    away = np.array([ankle[0] - attach[0], ankle[1] - attach[1], 0.0])
    if np.linalg.norm(away) > 1e-9:
        away = away / np.linalg.norm(away)
        d = np.cos(t) * ez + np.sin(t) * ex
        if np.dot(np.array([d[0], d[1], 0.0]), away) < 0:
            t += np.pi
    return np.arctan2(np.sin(t), np.cos(t))


def foot_jacobian(angles, attach, lengths, mount):
    """d(foot) / d(angle), 3x5, millimetres per radian."""
    eps = 1e-5
    f0 = foot_position(angles, attach, lengths, mount)
    J = np.zeros((3, 5))
    for k in range(5):
        bumped = angles.copy()
        bumped[k] += eps
        J[:, k] = (foot_position(bumped, attach, lengths, mount) - f0) / eps
    return J


def stride_travel(angles, attach, lengths, mount, limits):
    """How far the foot can slide fore and aft before a joint hits a stop.

    Standing on six feet is necessary and not sufficient: the leg then has to
    be able to walk from there, which means translating its foot along the
    ground -- not lifting it, not swinging it sideways -- in both directions.

    Nothing asked for this and the pose that shipped fails it badly. The front
    leg could move its foot 0.541 mm forward and 0.031 mm *backward*, because
    its ankle sat 0.057 rad from its limit. Backward is the power stroke. A
    front leg with thirty-one micrometres of it cannot push the animal along,
    and measured at the joint it did the opposite: once the height correction
    was applied its foot travelled forwards during stance while the other four
    travelled back.

    Returns millimetres of travel each way along the least-norm direction that
    slides the foot forward at constant height.
    """
    J = foot_jacobian(angles, attach, lengths, mount)
    want = np.array([1.0, 0.0, 0.0])
    q = J.T @ np.linalg.solve(J @ J.T + 1e-9 * np.eye(3), want)
    if np.linalg.norm(J @ q - want) > 1e-3:
        return 0.0, 0.0
    fwd = back = np.inf
    for k in range(5):
        if abs(q[k]) < 1e-9:
            continue
        up = (limits[k, 1] - angles[k]) / q[k]
        dn = (limits[k, 0] - angles[k]) / q[k]
        fwd = min(fwd, max(up, dn))
        back = min(back, -min(up, dn))
    return max(0.0, fwd), max(0.0, back)


def solve(attach, lengths, target, seed, mount, limits):
    """Place the ankle, then lay the tarsus flat.

    Only the first four joints move the ankle, so the Jacobian is 3x4 and
    TiTa drops out of the search entirely -- it is then set analytically to
    whatever lays the tarsus along the ground.
    """
    angles = seed.copy()
    lam = 1e-3
    for _ in range(500):
        err = ankle_position(angles, attach, lengths, mount) - target
        if np.linalg.norm(err) < 1e-6:
            break
        J = np.zeros((3, 4))
        eps = 1e-5
        for k in range(4):
            bumped = angles.copy()
            bumped[k] += eps
            J[:, k] = (ankle_position(bumped, attach, lengths, mount)
                       - err - target) / eps
        # Damped least squares, with a pull toward the seed so the redundant
        # joints settle on a natural posture instead of drifting.
        A = J.T @ J + (lam + 0.002) * np.eye(4)
        b = J.T @ err + 0.002 * (angles[:4] - seed[:4])
        angles[:4] = np.clip(angles[:4] - np.linalg.solve(A, b),
                             limits[:4, 0], limits[:4, 1])
    angles[4] = np.clip(flat_tarsus_angle(angles, attach, lengths, mount),
                        limits[4, 0], limits[4, 1])
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
    for thc in (-1.0, -0.5, 0.0, 0.5, 1.0):
        for ctr in (-2.2, -1.6, -1.2, -0.6, -0.2, 0.2, 0.6, 1.2, 1.6, 2.2):
            for knee in (-2.6, -2.2, -1.2, -0.4, 0.4, 1.2, 2.2, 2.6, 3.0):
                seed = np.clip(np.array([thc, ctr, 0.0, knee, 0.4]),
                               limits[:, 0], limits[:, 1])
                angles = solve(attach, lengths, np.array(target, float), seed,
                               mount, limits)
                err = np.linalg.norm(
                    ankle_position(angles, attach, lengths, mount)
                    - np.array(target))
                margin = np.min(np.minimum(angles - limits[:, 0],
                                           limits[:, 1] - angles))
                # Penalise a knee, or any other joint, that ends up below the
                # floor.
                #
                # Nothing used to stop this. The solve constrains the ankle and
                # the foot target and says nothing about the joints in between,
                # so it happily produced a stance whose femur ended 85 um under
                # the ground. The physics then had no collision on the upper leg
                # either, so the femur and tibia simply ran underground and the
                # leg looked broken in half on screen.
                below = 0.0
                for q, r in zip(joint_positions(angles, attach, lengths, mount),
                                CLEAR_RADII):
                    below += max(0.0, r + 0.005 - q[2])
                # Penalise solutions pressed against a joint limit.
                #
                # The floor used to be 0.05 rad, which is not room to move in,
                # only room to exist in. The front leg came out at 0.057 and
                # so paid nothing, while being jammed hard enough that it
                # could not take a step at all.
                # Require a stride in both directions, not just a stance.
                fwd, back = stride_travel(angles, attach, lengths, mount,
                                          limits)
                short = (max(0.0, WANT_STRIDE - fwd) +
                         max(0.0, WANT_STRIDE - back))
                # Clearance is not tradeable. Weighted at 20 against a
                # stride term worth up to 3 it was simply outbid, and the
                # solver bought a longer stride with a hind femur 61 um under
                # the floor -- which the physics then stood the animal on, so
                # the hind legs carried 95% of it on their knees while the
                # tarsi hung 90 um in the air.
                score = (err + max(0.0, 0.30 - margin) * 3.0 + short * 5.0
                         + (1000.0 if below > 1e-6 else 0.0) + below * 20.0)
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
        ankle = ankle_position(angles, attach, lengths, mount)
        foot = foot_position(angles, attach, lengths, mount)
        err = np.linalg.norm(ankle - np.array(target))
        margin = np.min(np.minimum(angles - LIMITS[:, 0], LIMITS[:, 1] - angles))
        results[name] = angles
        print(f"{name:<7} ankle = ({ankle[0]:+.3f}, {ankle[1]:+.3f}, "
              f"{ankle[2]:+.3f})  tarsus tip = ({foot[0]:+.3f}, {foot[1]:+.3f}, "
              f"{foot[2]:+.3f})")
        fwd, back = stride_travel(angles, attach, lengths, mount, LIMITS)
        print(f"        error {err*1000:6.1f} um   tarsus rise "
              f"{(foot[2] - ankle[2])*1000:+6.1f} um   "
              f"limit margin {margin:.3f} rad")
        print(f"        stride available: {fwd:.3f} mm forward, "
              f"{back:.3f} mm backward")
        lowest = min(
            (q[2] - r) for q, r in
            zip(joint_positions(angles, attach, lengths, mount), CLEAR_RADII))
        print(f"        upper leg clears the floor by {lowest*1000:+6.1f} um")

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
