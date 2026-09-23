"""Assign leg proprioceptors to a leg, and where possible to a joint.

The motor map could be read off the neuron names: the connectome calls a motor
neuron after the muscle it drives, and somaNeuromere says which leg. Sensory
neurons give us neither. Their cell bodies sit out in the leg itself, outside
the imaged volume, so they carry no neuromere and no side, and their names
(SNpp50, SNpp60, ...) say nothing about what they measure.

What they do carry is their wiring. A chordotonal neuron in one leg synapses
onto that leg's premotor circuits, so following it forward two hops and asking
which leg's motor neurons it reaches recovers the assignment the annotation is
missing. The same walk, tallied by joint instead of by leg, gives a weaker but
usable hint about which joint it senses.

Writes data/bin/sensory_map.tsv:
    organ, leg, joint, purity, neuron_index, type

`purity` is the fraction of a neuron's reach landing on its assigned leg, so a
consumer can discard the ambiguous ones. Intersegmental proprioceptors are real
and will legitimately look impure.

The `joint` column is reported but should NOT be trusted, and nothing in the
simulation uses it. Tallied across all placed neurons it reproduces the size of
each motor pool almost exactly -- ThC 17% of sensory against 19% of motor, FTi
26% against 29%, TiTa 15% against 17% -- which is what you would get by
assigning at random in proportion to pool size. The walk finds which leg a
proprioceptor serves, not which joint it measures. Resolving the joint would
need the position of the sensory terminal, which this dataset does not give for
neurons whose somas lie outside the imaged volume.

Usage:  python tools/sensory_map.py
"""
from __future__ import annotations

import collections
import csv
import glob
import pathlib

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
RAW = ROOT / "data" / "raw"
BIN = ROOT / "data" / "bin"

# Connections weaker than this are mostly segmentation noise and would blur the
# leg assignment.
MIN_WEIGHT = 5
# Below this fraction on one leg, the assignment is not trusted.
PURITY = 0.8


def build_csr(n, edges):
    src = np.searchsorted(np.arange(n), edges[:, 0])
    return src


def main() -> None:
    nz = np.load(RAW / "neurons.npz", allow_pickle=False)
    body_id = nz["body_id"]
    types = nz["type"]
    supers = nz["superclass"]
    subs = nz["subclass"]
    n = len(body_id)

    # Motor neurons, already mapped to leg and joint by tools/motor_map.py.
    motor_leg, motor_joint = {}, {}
    with open(BIN / "motor_map.tsv", encoding="utf-8") as f:
        for r in csv.DictReader(f, delimiter=chr(9)):
            idx = int(r["neuron_index"])
            motor_leg[idx] = r["leg"]
            motor_joint[idx] = r["joint"]

    print("loading edges")
    edges = np.concatenate([np.load(p) for p in sorted(glob.glob(str(RAW / "edges" / "chunk_*.npy")))])
    src = np.searchsorted(body_id, edges[:, 0])
    dst = np.searchsorted(body_id, edges[:, 1])
    w = edges[:, 2]
    keep = w >= MIN_WEIGHT
    src, dst, w = src[keep], dst[keep], w[keep]
    order = np.argsort(src, kind="stable")
    src, dst, w = src[order], dst[order], w[order]
    starts = np.searchsorted(src, np.arange(n + 1))
    print(f"  {len(src):,} edges at weight >= {MIN_WEIGHT}")

    def outgoing(i):
        a, b = starts[i], starts[i + 1]
        return dst[a:b], w[a:b]

    # Leg proprioceptors: chordotonal organs report joint angle and movement,
    # campaniform sensilla report load on the cuticle.
    want = ("chordotonal organ", "campaniform sensilla")
    sens = [i for i in range(n)
            if supers[i] == "vnc_sensory" and str(subs[i]) in want]
    print(f"{len(sens)} leg proprioceptors to place")

    rows = []
    stats = collections.Counter()
    for i in sens:
        legs = collections.Counter()
        joints = collections.Counter()
        # Two hops: straight onto a motor neuron, or via one interneuron. Most
        # proprioceptive feedback in the fly is at least disynaptic.
        first, fw = outgoing(i)
        for m, mw in zip(first, fw):
            if m in motor_leg:
                legs[motor_leg[m]] += int(mw)
                joints[motor_joint[m]] += int(mw)
            second, sw = outgoing(m)
            for k, kw in zip(second, sw):
                if k in motor_leg:
                    # Discount the indirect path so a direct connection counts
                    # for more than one routed through an interneuron.
                    legs[motor_leg[k]] += int(mw * kw) // 64
                    joints[motor_joint[k]] += int(mw * kw) // 64

        total = sum(legs.values())
        if total == 0:
            stats["no motor reach"] += 1
            continue
        leg, hits = legs.most_common(1)[0]
        purity = hits / total
        joint = joints.most_common(1)[0][0] if joints else ""
        organ = "chordotonal" if str(subs[i]) == "chordotonal organ" else "campaniform"
        stats["placed" if purity >= PURITY else "ambiguous"] += 1
        rows.append((organ, leg, joint, round(purity, 3), int(i), str(types[i])))

    tab, nl = chr(9), chr(10)
    out = BIN / "sensory_map.tsv"
    with open(out, "w", encoding="utf-8", newline=nl) as f:
        f.write(tab.join(["organ", "leg", "joint", "purity",
                          "neuron_index", "type"]) + nl)
        for r in sorted(rows, key=lambda r: (r[0], r[1], r[2])):
            f.write(tab.join(str(x) for x in r) + nl)

    print()
    for k, v in stats.most_common():
        print(f"  {k:<16} {v:4d}")

    clean = [r for r in rows if r[3] >= PURITY]
    print(f"{nl}{len(clean)} placed at purity >= {PURITY}")
    per_leg = collections.Counter((r[1], r[0]) for r in clean)
    for leg in sorted({r[1] for r in clean}):
        ch = per_leg[(leg, "chordotonal")]
        ca = per_leg[(leg, "campaniform")]
        print(f"  {leg:<10} {ch + ca:3d}   chordotonal={ch:3d}  campaniform={ca:3d}")

    print(f"{nl}The joint column is not trustworthy -- see the note at the top of")
    print("this file. Only the leg assignment should be used.")

    print(f"{nl}wrote {out}")


if __name__ == "__main__":
    main()
