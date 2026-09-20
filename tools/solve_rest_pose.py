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

SEG = dict(coxa=0.26, troch=0.08, femur=0.52, tibia=0.48, tarsus=0.50)

LEGS = [
    # name, attachX, lengthScale, target foot (x, y, z) for the LEFT leg
    ("front",  0.34, 0.88, (0.92, 0.72, GROUND_Z)),
    ("middle", 0.02, 1.00, (0.05, 0.92, GROUND_Z)),
    ("hind",  -0.30, 1.12, (-0.92, 0.88, GROUND_Z)),
]

# Axes for a left leg, in the order ThC, CTr, TrF, FTi, TiTa.
AXES = np.array([
    [0.0, 1.0, 0.0],   # ThC  lateral: swings fore and aft
    [1.0, 0.0, 0.0],   # CTr  fore-aft: lifts and lowers
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
    [-1.05, 1.05],
    [-0.4, 1.6],
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


def foot_position(angles, attach, lengths):
    """Forward kinematics, matching FlyBody::footPosition."""
    pos = np.array(attach, dtype=float)
    rot = np.array([1.0, 0.0, 0.0, 0.0])
    for i in range(5):
        rot = quat_mul(rot, quat_axis_angle(AXES[i], angles[i]))
        pos = pos + quat_rotate(rot, np.array([0.0, 0.0, -1.0])) * lengths[i]
    return pos


def solve(attach, lengths, target, seed):
    angles = seed.copy()
    lam = 1e-3
    for _ in range(1500):
        err = foot_position(angles, attach, lengths) - target
        if np.linalg.norm(err) < 1e-6:
            break
        # Numerical Jacobian: 5 columns, cheap enough at this size.
        J = np.zeros((3, 5))
        eps = 1e-5
        for k in range(5):
            bumped = angles.copy()
            bumped[k] += eps
            J[:, k] = (foot_position(bumped, attach, lengths) - err - target) / eps
        # Damped least squares, with a pull toward the seed so the five
        # redundant joints settle on a natural posture instead of drifting.
        A = J.T @ J + (lam + 0.002) * np.eye(5)
        b = J.T @ err + 0.002 * (angles - seed)
        angles = np.clip(angles - np.linalg.solve(A, b), LIMITS[:, 0], LIMITS[:, 1])
    return angles


def main() -> None:
    # A plausible starting posture: leg swung out, knee bent.
    seed = np.array([0.0, 0.62, 0.0, -0.95, 0.45])
    # Front knees fold one way, hind knees the other.
    knee_seed = {"front": -1.6, "middle": -1.9, "hind": 1.9}

    print(f"body at z = {BODY_Z}, ground at z = {GROUND_Z}\n")
    results = {}
    for name, attach_x, scale, target in LEGS:
        attach = (attach_x, BODY_RADIUS * 0.75, BODY_Z - BODY_RADIUS * 0.35)
        lengths = np.array([SEG["coxa"], SEG["troch"], SEG["femur"],
                            SEG["tibia"], SEG["tarsus"]]) * scale
        s = seed.copy()
        s[0] = {"front": 0.2, "middle": 0.6, "hind": 0.9}[name]
        s[3] = knee_seed[name]
        angles = solve(attach, lengths, np.array(target, dtype=float), s)
        foot = foot_position(angles, attach, lengths)
        err = np.linalg.norm(foot - np.array(target))
        results[name] = angles
        print(f"{name:<7} foot = ({foot[0]:+.3f}, {foot[1]:+.3f}, {foot[2]:+.3f})"
              f"   target error {err*1000:.2f} um")

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
