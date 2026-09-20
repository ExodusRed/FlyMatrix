"""Derive a leg/joint/direction map for the VNC motor neurons.

The connectome names every motor neuron after the muscle it drives, and
somaNeuromere plus somaSide say which of the six legs it belongs to. That is
enough to wire each joint of a simulated leg to its real motor pool, rather
than inventing a mapping.

    T1/T2/T3  x  L/R            -> front / middle / hind leg, left or right
    "Ti flexor MN"              -> femur-tibia joint, flexor
    "Sternal posterior rotator" -> thorax-coxa joint, retractor

Writes data/bin/motor_map.tsv:  leg, joint, direction, neuron_index, type

Not every motor neuron in a leg neuromere drives a leg. The wing steering
muscles (b1-b3, i1/i2, iii1/iii3, hg1-hg4, ps1/ps2, tp1/tp2), the flight power
muscles (DLMn, DVMn) and the haltere muscles all attach to the thorax and are
correctly left out. What is genuinely missing is the MNhl*/MNml* neurons: those
are leg motor neurons, but they are numbered rather than named after a muscle,
so there is nothing to map them by.

The muscle-to-joint assignments below are anatomy, not data: the dataset says
which muscle, and this table says which joint that muscle crosses. They follow
the standard description of the Drosophila leg (Azevedo et al. 2020) but are a
best effort, and coverage is reported so what is left unmapped is visible.

Usage:  python tools/motor_map.py
"""
from __future__ import annotations

import collections
import pathlib

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
RAW = ROOT / "data" / "raw"
BIN = ROOT / "data" / "bin"

# The five joints of a fly leg, proximal to distal.
JOINTS = ["ThC", "CTr", "TrF", "FTi", "TiTa"]

# Substring of the cell type -> (joint, direction). Order matters: the first
# match wins, so the more specific patterns come first.
#
# Direction is the sign of the joint angle the muscle drives. "flex" folds the
# leg up, "extend" straightens it; for the thorax-coxa joint the pair is
# protract (swing forward) and retract (swing back).
MUSCLE_TO_JOINT = [
    ("Acc. ti flexor",              ("FTi", "flex")),
    ("Ti flexor",                   ("FTi", "flex")),
    ("Ti extensor",                 ("FTi", "extend")),

    ("Acc. tr flexor",              ("CTr", "flex")),
    ("Tr flexor",                   ("CTr", "flex")),
    ("Tr extensor",                 ("CTr", "extend")),
    # The tergotrochanteral muscle is the jump muscle: one explosive
    # trochanter extension, driven by TTMn off the giant fibre.
    ("Tergotr.",                    ("CTr", "extend")),
    # TTMn is the same jump muscle under its own name -- this is the neuron the
    # giant fibre drives, so missing it would lose the escape jump entirely.
    ("TTMn",                        ("CTr", "extend")),
    ("STTMm",                       ("CTr", "extend")),
    ("Sternotrochanter",            ("CTr", "extend")),

    ("Fe reductor",                 ("TrF", "flex")),

    ("Ta depressor",                ("TiTa", "flex")),
    ("Ta levator",                  ("TiTa", "extend")),
    ("ltm1-tibia",                  ("TiTa", "flex")),
    ("ltm2-femur",                  ("TiTa", "flex")),
    ("ltm",                         ("TiTa", "flex")),

    ("Sternal anterior rotator",    ("ThC", "protract")),
    ("Tergopleural/Pleural promotor", ("ThC", "protract")),
    ("Pleural promotor",            ("ThC", "protract")),
    ("Sternal posterior rotator",   ("ThC", "retract")),
    ("Pleural remotor/abductor",    ("ThC", "retract")),
    ("Sternal adductor",            ("ThC", "retract")),
]

LEG_FROM_NEUROMERE = {"T1": "front", "T2": "middle", "T3": "hind"}


def classify(cell_type: str):
    for pattern, jd in MUSCLE_TO_JOINT:
        if pattern in cell_type:
            return jd
    return None


def main() -> None:
    nz = np.load(RAW / "neurons.npz", allow_pickle=False)
    types = nz["type"]
    supers = nz["superclass"]
    neuromere = nz["neuromere"]
    side = nz["soma_side"]

    motor = np.flatnonzero(supers == "vnc_motor")
    print(f"{len(motor)} vnc_motor neurons")

    rows = []
    unmapped = collections.Counter()
    no_leg = 0
    for i in motor:
        jd = classify(str(types[i]))
        if jd is None:
            unmapped[str(types[i])] += 1
            continue
        leg_seg = LEG_FROM_NEUROMERE.get(str(neuromere[i]))
        s = str(side[i])
        if leg_seg is None or s not in ("L", "R"):
            # Wing, haltere and abdominal motor neurons live in the same
            # superclass but do not belong to a leg.
            no_leg += 1
            continue
        joint, direction = jd
        rows.append((f"{leg_seg}_{s}", joint, direction, int(i), str(types[i])))

    tab, nl = chr(9), chr(10)
    BIN.mkdir(parents=True, exist_ok=True)
    out = BIN / "motor_map.tsv"
    with open(out, "w", encoding="utf-8", newline=nl) as f:
        f.write(tab.join(["leg", "joint", "direction", "neuron_index", "type"]) + nl)
        for r in sorted(rows):
            f.write(tab.join(str(x) for x in r) + nl)

    print(f"mapped {len(rows)} motor neurons to a leg joint")
    print(f"  {no_leg} skipped: right muscle name, but not in a leg neuromere")
    print(f"  {sum(unmapped.values())} skipped: muscle not in the joint table")

    by_leg = collections.Counter(r[0] for r in rows)
    print(nl + "per leg:")
    for leg in sorted(by_leg):
        per_joint = collections.Counter(r[1] for r in rows if r[0] == leg)
        detail = "  ".join(f"{j}={per_joint.get(j, 0)}" for j in JOINTS)
        print(f"  {leg:<10} {by_leg[leg]:3d}   {detail}")

    if unmapped:
        print(nl + "unmapped muscle types (largest first):")
        for name, c in unmapped.most_common(12):
            print(f"  {name or '(untyped)':<40} {c:3d}")

    print(nl + f"wrote {out}")


if __name__ == "__main__":
    main()
