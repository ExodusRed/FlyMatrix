"""How well can each leg take a step, given the pose it stands in?

A rest pose that puts all six feet on the ground is necessary and not
sufficient. The leg also has to be able to *walk* from there: translate its
foot fore and aft, along the ground, without lifting it, without swinging it
sideways, and without any joint running into a stop.

Nothing measured that, and the pose that shipped fails it. The front leg's
ankle sits 0.057 rad from its limit and the only joint left to hold the foot
level is CTr, which costs the front leg so much fore-aft travel that its foot
ends up moving *forwards* during the stance where the other four move back.

Reads the rest angles straight out of FlyBody.cpp so it describes the animal
that is actually built.

    python tools/leg_authority.py
"""
from __future__ import annotations

import pathlib
import re
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from solve_rest_pose import (  # noqa: E402
    BODY_RADIUS, BODY_Z, LEGS, LIMITS, JOINT_NAMES, SEG,
    foot_position,
)

ROOT = pathlib.Path(__file__).resolve().parent.parent


def rest_angles_from_cpp():
    """Pull the shipped rest angles out of the kLayout table."""
    src = (ROOT / "src" / "body" / "FlyBody.cpp").read_text(encoding="utf-8")
    out = {}
    for name, tag in (("front", "FrontL"), ("middle", "MiddleL"),
                      ("hind", "HindL")):
        m = re.search(
            r"LegId::" + tag + r",[^{]*\{([^}]*)\}", src)
        if not m:
            sys.exit("could not find " + tag + " in FlyBody.cpp")
        vals = [float(v) for v in re.findall(r"(-?[0-9.]+)f", m.group(1))]
        if len(vals) != 5:
            sys.exit("expected 5 rest angles for " + tag)
        out[name] = np.array(vals)
    return out


def jacobian(angles, attach, lengths, mount):
    """d(foot) / d(angle), 3x5, in millimetres per radian."""
    eps = 1e-5
    f0 = foot_position(angles, attach, lengths, mount)
    J = np.zeros((3, 5))
    for k in range(5):
        b = angles.copy()
        b[k] += eps
        J[:, k] = (foot_position(b, attach, lengths, mount) - f0) / eps
    return J


def step_cost(J, free):
    """Joint motion needed to slide the foot 1 mm forward along the ground.

    Least-norm solution of J q = (1, 0, 0) over the joints in `free`. The
    answer is in radians per millimetre: how hard the leg has to work to take
    a step that neither lifts the foot nor drags it sideways. `resid` says
    whether it can be done at all.
    """
    Jf = J[:, free]
    want = np.array([1.0, 0.0, 0.0])
    q = Jf.T @ np.linalg.solve(Jf @ Jf.T + 1e-9 * np.eye(3), want)
    resid = np.linalg.norm(Jf @ q - want)
    full = np.zeros(5)
    full[free] = q
    return full, resid


def main() -> None:
    rest = rest_angles_from_cpp()
    print("joint limit margins, radians (how far the rest pose is from a stop)")
    print(f"  {'leg':<8}" + "".join(f"{j:>9}" for j in JOINT_NAMES) + "    worst")
    worst_all = {}
    for name, angles in rest.items():
        margins = np.minimum(angles - LIMITS[:, 0], LIMITS[:, 1] - angles)
        worst_all[name] = margins.min()
        flag = "  <-- jammed" if margins.min() < 0.3 else ""
        print(f"  {name:<8}" + "".join(f"{m:>9.3f}" for m in margins)
              + f" {margins.min():>8.3f}{flag}")

    print()
    print("stepping authority: joint motion to slide the foot 1 mm forward")
    print("along the ground, holding height and lateral position")
    print(f"  {'leg':<8}" + "".join(f"{j:>9}" for j in JOINT_NAMES)
          + f"{'|q| rad/mm':>12}{'resid':>9}")
    for name, attach_x, scale, _target, mount in LEGS:
        angles = rest[name]
        attach = (attach_x, BODY_RADIUS * 0.75, BODY_Z - BODY_RADIUS * 0.35)
        lengths = np.array([SEG["coxa"], SEG["troch"], SEG["femur"],
                            SEG["tibia"], SEG["tarsus"]]) * scale
        J = jacobian(angles, attach, lengths, mount)
        q, resid = step_cost(J, [0, 1, 2, 3, 4])
        print(f"  {name:<8}" + "".join(f"{v:>9.3f}" for v in q)
              + f"{np.linalg.norm(q):>12.3f}{resid:>9.2e}")

    print()
    print("travel available before the first joint hits a stop, mm of stride")
    for name, attach_x, scale, _target, mount in LEGS:
        angles = rest[name]
        attach = (attach_x, BODY_RADIUS * 0.75, BODY_Z - BODY_RADIUS * 0.35)
        lengths = np.array([SEG["coxa"], SEG["troch"], SEG["femur"],
                            SEG["tibia"], SEG["tarsus"]]) * scale
        J = jacobian(angles, attach, lengths, mount)
        q, _ = step_cost(J, [0, 1, 2, 3, 4])
        # How far along q, in millimetres of foot travel, until some joint
        # reaches its limit -- forwards and backwards.
        fwd = back = np.inf
        for k in range(5):
            if abs(q[k]) < 1e-9:
                continue
            up = (LIMITS[k, 1] - angles[k]) / q[k]
            dn = (LIMITS[k, 0] - angles[k]) / q[k]
            hi, lo = max(up, dn), min(up, dn)
            fwd = min(fwd, hi)
            back = min(back, -lo)
        limiter_f = min(range(5), key=lambda k: abs(
            ((LIMITS[k, 1] if q[k] > 0 else LIMITS[k, 0]) - angles[k]) / q[k])
            if abs(q[k]) > 1e-9 else np.inf)
        print(f"  {name:<8} forward {fwd:6.3f} mm   backward {back:6.3f} mm"
              f"   first stop: {JOINT_NAMES[limiter_f]}")


if __name__ == "__main__":
    main()
