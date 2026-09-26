"""Search for a rest pose that can stand *and* walk, with consistent legs.

solve_rest_pose.py picks the pose by reaching a foot target and then keeping
every leg on the same side of zero for CTr and FTi. That second rule was added
so that one motor command does the same thing to all six legs, and it does not
work: with the rule enforced, the shipped pose has

    dz/dCTr   front -0.321   middle +0.585   hind +1.036

so raising CTr lowers the front foot and raises the other four. The rule
constrains the *sign of an angle*, which is a coordinate convention, when what
has to agree is the *effect of the joint on the foot*, which is geometry. It
costs a great deal to enforce: the front leg's rest pose ends up 0.057 rad from
its ankle limit and can slide its foot 0.031 mm backward -- the power stroke --
against 0.54 mm for poses on the other side.

So this searches the other way round. Generate many poses per leg that reach
the target and keep the upper leg out of the floor, measure what each joint
does to the foot, and then choose the combination of *functional* signs that
leaves every leg the most room to walk.

    python tools/pose_search.py
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import solve_rest_pose as S  # noqa: E402

RESTARTS = 400
# Millimetres of foot travel the worst leg should have, each way.
REPORT_WANT = 0.30


def leg_geometry(legname):
    name, ax, scale, target, mount = [l for l in S.LEGS if l[0] == legname][0]
    attach = (ax, S.BODY_RADIUS * 0.75, S.BODY_Z - S.BODY_RADIUS * 0.35)
    lengths = np.array([S.SEG["coxa"], S.SEG["troch"], S.SEG["femur"],
                        S.SEG["tibia"], S.SEG["tarsus"]]) * scale
    return attach, lengths, np.array(target, float), mount


def dz_per_joint(angles, attach, lengths, mount):
    """How much the foot rises per radian of each joint."""
    eps = 1e-5
    out = np.zeros(5)
    for k in range(5):
        hi = angles.copy()
        hi[k] += eps
        lo = angles.copy()
        lo[k] -= eps
        out[k] = (S.foot_position(hi, attach, lengths, mount)[2]
                  - S.foot_position(lo, attach, lengths, mount)[2]) / (2 * eps)
    return out


def candidates(legname, rng):
    attach, lengths, target, mount = leg_geometry(legname)
    found = []
    for _ in range(RESTARTS):
        seed = np.array([rng.uniform(*S.LIMITS[k]) for k in range(5)])
        a = S.solve(attach, lengths, target, seed, mount, S.LIMITS)
        if np.linalg.norm(S.ankle_position(a, attach, lengths, mount)
                          - target) > 0.01:
            continue
        pts = S.joint_positions(a, attach, lengths, mount)
        clear = min(q[2] - r for q, r in zip(pts, S.CLEAR_RADII))
        if clear <= 0.0:
            continue
        fwd, back = S.stride_travel(a, attach, lengths, mount, S.LIMITS)
        marg = float(np.min(np.minimum(a - S.LIMITS[:, 0],
                                       S.LIMITS[:, 1] - a)))
        dz = dz_per_joint(a, attach, lengths, mount)
        found.append(dict(angles=a, clear=clear, fwd=fwd, back=back,
                          margin=marg, dz=dz,
                          sig=(int(np.sign(dz[1])), int(np.sign(dz[3])))))
    return found


def main() -> None:
    rng = np.random.default_rng(7)
    pool = {}
    for leg in ("front", "middle", "hind"):
        pool[leg] = candidates(leg, rng)
        print(f"{leg:<7} {len(pool[leg]):4d} poses reach the target and clear "
              f"the floor, of {RESTARTS} restarts")

    print()
    print("best pose per leg for each combination of functional signs")
    print("(sign of dz/dCTr, dz/dFTi -- what the joint does to the foot)")
    print(f"  {'CTr':>4}{'FTi':>4}  " + "".join(f"{n:>26}" for n in
                                                ("front", "middle", "hind"))
          + f"{'worst back':>12}")

    best = None
    for ctr_sign in (+1, -1):
        for fti_sign in (+1, -1):
            sig = (ctr_sign, fti_sign)
            picks, ok = {}, True
            for leg in ("front", "middle", "hind"):
                same = [c for c in pool[leg] if c["sig"] == sig]
                if not same:
                    ok = False
                    break
                # The power stroke is the backward one; prefer the pose with
                # the most of it, then the one furthest from a joint stop.
                same.sort(key=lambda c: (-min(c["fwd"], c["back"]),
                                         -c["margin"]))
                picks[leg] = same[0]
            cells = ""
            for leg in ("front", "middle", "hind"):
                if not ok:
                    cells += f"{'--':>26}"
                else:
                    c = picks[leg]
                    cells += (f"{c['back']:8.3f} back{c['margin']:7.3f} marg"
                              f"{c['clear']*1000:7.0f}um")
            worst = (min(min(p["fwd"], p["back"]) for p in picks.values())
                     if ok else -1.0)
            sgn = lambda v: "+" if v > 0 else "-"
            print(f"  {sgn(ctr_sign):>4}{sgn(fti_sign):>4}  {cells}"
                  f"{worst:12.3f}")
            if ok and (best is None or worst > best[0]):
                best = (worst, sig, picks)

    if best is None:
        sys.exit("no sign combination is reachable on all three legs")

    worst, sig, picks = best
    print()
    print(f"chosen: dz/dCTr {'+' if sig[0] > 0 else '-'}, "
          f"dz/dFTi {'+' if sig[1] > 0 else '-'}   "
          f"worst leg has {worst:.3f} mm of travel each way "
          f"(wanted {REPORT_WANT})")
    print()
    for leg in ("front", "middle", "hind"):
        c = picks[leg]
        print(f"{leg:<7} margin {c['margin']:.3f} rad   clears "
              f"{c['clear']*1000:.0f} um   stride {c['fwd']:.3f} forward / "
              f"{c['back']:.3f} backward")
        print("        dz per joint: " + "  ".join(
            f"{n}={v:+.3f}" for n, v in zip(S.JOINT_NAMES, c["dz"])))

    print()
    print("paste into FlyBody.cpp kLayout as restAngles:")
    for leg in ("front", "middle", "hind"):
        vals = ", ".join(f"{a:.3f}f" for a in picks[leg]["angles"])
        print(f"  // {leg}")
        print(f"  {{{vals}}},")


if __name__ == "__main__":
    main()
