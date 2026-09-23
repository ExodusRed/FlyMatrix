"""Can the chordotonal population be split by what it drives?

The sensorimotor loop fails for a reason findings.md section 1 names precisely:
all 254 chordotonal neurons are driven with one signal, leg compression. The
femoral chordotonal organ has five subtypes, and in particular contains both
extension-encoding and flexion-encoding cells, so feeding them the same input
excites antagonist pathways together. The loop then does not stabilise the leg,
it fights itself -- with it on, Kenyon cells move the fly 12 mm.

The subtype labels come from axon morphology, which male-cns:v1.0 does not give
us for these neurons, and their type names (SNpp50, SNpp60, ...) carry no
functional information. So the labels are out of reach.

But the labels may not be what we need. What the loop needs is to know which
sensory neurons should be driven by one phase of the movement and which by the
opposite phase, and that is a question about what each neuron *does* downstream.
A cell that drives tibia flexors and a cell that drives tibia extensors belong
on opposite sides of the signal whatever they are called.

So this measures, for each chordotonal neuron, its weighted two-hop reach to
FTi flexor motor neurons against its reach to FTi extensor motor neurons, and
reports the distribution of the resulting bias. FTi is the right joint: the
femoral chordotonal organ encodes tibia position, which is the FTi angle.

The test is whether that distribution is bimodal. If the population separates
into extensor-biased and flexor-biased groups, the split is real and the loop
can be given two signals instead of one. If it is a single lump around zero,
this proxy does not work and the failure should be recorded rather than
papered over.

Two hops because chordotonal afferents reach motor neurons through premotor
interneurons; direct sensory-to-motor connections are the minority.

Usage:  python tools/feco_split.py [--joint FTi]
"""
from __future__ import annotations

import argparse
import collections
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from cpg_probe import BIN, load_graph, load_motor_map  # noqa: E402


def load_chordotonal():
    """Chordotonal neuron indices, per leg."""
    out = collections.defaultdict(list)
    with (BIN / "sensory_map.tsv").open() as f:
        next(f)
        for line in f:
            p = line.rstrip("\n").split("\t")
            if len(p) < 5 or p[0] != "chordotonal":
                continue
            out[p[1]].append(int(p[4]))
    return out


def reach_two_hop(row_start, col, weight, sign, sources, target_mask, n):
    """Signed two-hop weight from each source onto a target set.

    Hop one is scaled by the intermediate neuron's sign, so a sensory cell that
    reaches a motor pool through an inhibitory interneuron counts against it.
    Glutamate is inhibitory in Drosophila and the packed sign array already
    reflects that.
    """
    deg = np.diff(row_start).astype(np.int64)
    src = np.repeat(np.arange(n, dtype=np.int64), deg)
    w = weight.astype(np.float64)

    # Hop two: each neuron's signed weight onto the target set.
    onto = np.bincount(src, weights=np.where(target_mask[col], w, 0.0),
                       minlength=n)
    onto *= np.where(sign >= 0, 1.0, -1.0)

    # Hop one: each neuron's weight onto those, weighted by what they reach.
    reach = np.bincount(src, weights=w * onto[col], minlength=n)
    return {s: float(reach[s]) for s in sources}


def write_all_legs(row_start, col, weight, sign, n, joint):
    """Emit the split for every leg, for SensoryOrgans to read."""
    chord = load_chordotonal()
    out = BIN / "feco_split.tsv"
    rows = 0
    with out.open("w", encoding="utf-8", newline="\n") as f:
        f.write("leg\tneuron_index\tgroup\tbias\n")
        for leg in sorted(chord):
            groups = load_motor_map(joint, leg)
            names = sorted(groups)
            if len(names) < 2:
                print(f"{leg}: {joint} has fewer than two directions, skipped")
                continue
            a_name, b_name = names[0], names[1]
            mask_a = np.zeros(n, dtype=bool); mask_a[list(groups[a_name])] = True
            mask_b = np.zeros(n, dtype=bool); mask_b[list(groups[b_name])] = True
            cells = chord[leg]
            ra = reach_two_hop(row_start, col, weight, sign, cells, mask_a, n)
            rb = reach_two_hop(row_start, col, weight, sign, cells, mask_b, n)
            kept = 0
            for c in cells:
                a, b = ra[c], rb[c]
                denom = abs(a) + abs(b)
                if denom < 1e-9:
                    continue
                bias = (a - b) / denom
                # Cells near zero drive both antagonists about equally, which
                # is exactly the confusion the split exists to remove. Leaving
                # them out is better than assigning them arbitrarily.
                if abs(bias) < 0.2:
                    continue
                f.write(f"{leg}\t{c}\t{a_name if bias > 0 else b_name}\t"
                        f"{bias:+.4f}\n")
                kept += 1
                rows += 1
            print(f"{leg:<10} {len(cells):4d} chordotonal -> {kept:4d} assigned")
    print(f"\nwrote {out} ({rows} rows)")
    return 0


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="FTi")
    ap.add_argument("--leg", default="middle_L")
    ap.add_argument("--all-legs", action="store_true",
                    help="run every leg and write the split to "
                         "data/bin/feco_split.tsv")
    args = ap.parse_args()

    row_start, col, weight, sign, n = load_graph()
    if args.all_legs:
        return write_all_legs(row_start, col, weight, sign, n, args.joint)
    groups = load_motor_map(args.joint, args.leg)
    names = sorted(groups)
    if len(names) < 2:
        sys.exit(f"{args.joint} on {args.leg} has fewer than two directions")
    a_name, b_name = names[0], names[1]

    mask_a = np.zeros(n, dtype=bool)
    mask_a[list(groups[a_name])] = True
    mask_b = np.zeros(n, dtype=bool)
    mask_b[list(groups[b_name])] = True

    chord = load_chordotonal()
    cells = chord.get(args.leg, [])
    if not cells:
        sys.exit(f"no chordotonal neurons mapped to {args.leg}")

    print(f"\n{args.leg}: {len(cells)} chordotonal neurons, "
          f"{len(groups[a_name])} {a_name} MNs, {len(groups[b_name])} {b_name} MNs")
    print("computing two-hop reach (this walks the whole graph twice)...")

    ra = reach_two_hop(row_start, col, weight, sign, cells, mask_a, n)
    rb = reach_two_hop(row_start, col, weight, sign, cells, mask_b, n)

    bias = []
    for c in cells:
        a, b = ra[c], rb[c]
        denom = abs(a) + abs(b)
        bias.append(0.0 if denom < 1e-9 else (a - b) / denom)
    bias = np.array(bias)

    # A neuron that reaches only one pool scores exactly +/-1 for free, which
    # would manufacture bimodality out of sparse connectivity. Count those
    # separately from cells that reach both and genuinely prefer one.
    both, only_one = [], 0
    for c in cells:
        a, b = ra[c], rb[c]
        if abs(a) > 1e-9 and abs(b) > 1e-9:
            both.append((a - b) / (abs(a) + abs(b)))
        elif abs(a) > 1e-9 or abs(b) > 1e-9:
            only_one += 1
    both = np.array(both)

    live = bias[np.abs(bias) > 1e-9]
    print(f"\n{len(live)} of {len(cells)} reach either pool within two hops\n")
    if len(live) == 0:
        print("VERDICT: no chordotonal neuron reaches these motor pools in two "
              "hops. The split cannot be made this way.")
        return

    # Histogram, so the shape is visible rather than summarised away.
    print(f"bias toward {a_name} (+1) against {b_name} (-1):")
    edges = np.linspace(-1, 1, 21)
    hist, _ = np.histogram(live, bins=edges)
    peak = max(hist.max(), 1)
    for i, cnt in enumerate(hist):
        lo, hi = edges[i], edges[i + 1]
        bar = "#" * int(40 * cnt / peak)
        print(f"  {lo:+.1f}..{hi:+.1f} {cnt:4d} {bar}")

    # Bimodality: compare the population against a single central lump. A
    # cheap and honest check is what fraction sits out at the extremes versus
    # near zero.
    extreme = int((np.abs(live) > 0.5).sum())
    middle = int((np.abs(live) <= 0.2).sum())
    print(f"\n  |bias| > 0.5 : {extreme:4d}  ({extreme / len(live):.0%})")
    print(f"  |bias| <= 0.2: {middle:4d}  ({middle / len(live):.0%})")
    print(f"  mean {live.mean():+.3f}   sd {live.std():.3f}")

    print(f"\n  reach exactly one pool: {only_one:4d}  (bias is +/-1 by "
          f"construction)")
    print(f"  reach both pools:       {len(both):4d}")
    if len(both):
        ext_b = int((np.abs(both) > 0.5).sum())
        mid_b = int((np.abs(both) <= 0.2).sum())
        print(f"    of those, |bias| > 0.5 : {ext_b:4d}  ({ext_b / len(both):.0%})")
        print(f"    of those, |bias| <= 0.2: {mid_b:4d}  ({mid_b / len(both):.0%})")

    print()
    if extreme > middle:
        print("VERDICT: the population is mostly polarised, so a functional "
              "split is available.\nThe loop can be given two opposed signals "
              "without needing subtype labels.")
    else:
        print("VERDICT: the population is concentrated near zero, so this "
              "proxy does not\nseparate it. Most chordotonal neurons reach "
              "both antagonist pools about\nequally, which is what makes "
              "driving them with one signal excite both sides.")


if __name__ == "__main__":
    main()
