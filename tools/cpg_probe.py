"""Is there a half-centre oscillator wired into the leg motor circuits?

A walking animal needs a rhythm, and the model does not produce one. Driving
MDN -- the moonwalker descending neuron, the best-characterised walking command
cell in the fly -- gives a sustained, segmentally organised posture: front legs
retract, hind legs protract, held steady for as long as the stimulus lasts. It
is a coherent motor command and it never alternates.

The textbook way an animal makes a rhythm out of non-rhythmic drive is a half
centre: two premotor populations that each excite one side of a joint and
inhibit the other, plus some form of adaptation. Tonic drive in, alternation
out. Whether the fly's ThC circuits are wired that way is a question about the
connectome rather than about our simulation, so it can be asked directly.

This asks it. For the ThC joint, whose motor neurons the dataset names promotor
and remotor:

  1. collect the protractor and retractor motor neurons from the motor map
  2. find their presynaptic partners -- the premotor layer
  3. split that layer into cells that favour protractors and cells that favour
     retractors
  4. count the connections *between* those two groups, and their signs

If the two premotor groups inhibit each other, the substrate for a half centre
is present and the missing piece is in our neuron model. If they do not, the
rhythm has to come from somewhere else -- sensory feedback, neuromodulation, or
a mechanism the wiring diagram alone does not show.

Glutamate is inhibitory in Drosophila (GluCl-alpha), which the packed sign
array already accounts for.

Usage:  python tools/cpg_probe.py [--joint ThC]
"""
from __future__ import annotations

import argparse
import collections
import pathlib
import struct
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
BIN = ROOT / "data" / "bin"
MAGIC = b"FLYCNS01"


def load_graph():
    """Read cns.bin into CSR arrays plus the per-neuron sign."""
    raw = (BIN / "cns.bin").read_bytes()
    if raw[:8] != MAGIC:
        sys.exit(f"{BIN / 'cns.bin'} is not a FLYCNS binary")
    version, n, e, _minw, _pad = struct.unpack_from("<IIQII", raw, 8)
    off = 8 + struct.calcsize("<IIQII") + 6 * 4  # header + bbox

    def take(dtype, count):
        nonlocal off
        a = np.frombuffer(raw, dtype=dtype, count=count, offset=off)
        off += a.nbytes
        return a

    take(np.int64, n)        # body_id
    take(np.float32, n * 3)  # pos
    take(np.float32, n)      # size_rel
    take(np.uint8, n)        # nt_code
    sign = take(np.int8, n)
    take(np.uint8, n)        # flags
    take(np.uint8, n)        # pad
    row_start = take(np.uint64, n + 1)
    col = take(np.uint32, e)
    weight = take(np.uint16, e)
    print(f"cns.bin v{version}: {n:,} neurons, {e:,} edges")
    return row_start, col, weight, sign, n


def load_motor_map(joint, leg=None):
    """Motor neuron indices for one joint, split by direction.

    Restricting to a single leg matters: each leg has its own premotor circuit
    in its own neuromere, and pooling all six mixes six separate networks
    together, which washes out any structure that is there.
    """
    groups = collections.defaultdict(set)
    path = BIN / "motor_map.tsv"
    with path.open() as f:
        next(f)
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) < 5 or parts[1] != joint:
                continue
            if leg is not None and parts[0] != leg:
                continue
            groups[parts[2]].add(int(parts[3]))
    return groups


def presynaptic(row_start, col, weight, targets, n, min_syn=0):
    """Map presynaptic neuron -> summed synapse weight onto `targets`.

    min_syn drops weak connections. A premotor neuron is one that actually
    drives the pool; counting every cell with a single synapse onto a motor
    neuron pulls in thousands of cells that are premotor in name only, and
    then the excitatory/inhibitory balance just reconverges on the network
    average.
    """
    want = np.zeros(n, dtype=bool)
    want[list(targets)] = True
    hit = want[col] & (weight >= min_syn)
    if not hit.any():
        return {}
    # Which source row each surviving edge came from.
    src = np.repeat(np.arange(n, dtype=np.int64), np.diff(row_start).astype(np.int64))
    out = collections.defaultdict(int)
    for s, w in zip(src[hit], weight[hit]):
        out[int(s)] += int(w)
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="ThC")
    ap.add_argument("--leg", default=None,
                    help="restrict to one leg, e.g. middle_L")
    ap.add_argument("--min-syn", type=int, default=20,
                    help="minimum synapses for a connection to count as "
                         "premotor")
    ap.add_argument("--bias", type=float, default=2.0,
                    help="how strongly a premotor cell must favour one side "
                         "to be counted as belonging to it")
    args = ap.parse_args()

    row_start, col, weight, sign, n = load_graph()
    groups = load_motor_map(args.joint, args.leg)
    if len(groups) < 2:
        sys.exit(f"{args.joint} does not have two directions in the motor map")

    names = sorted(groups)
    a_name, b_name = names[0], names[1]
    a_mn, b_mn = groups[a_name], groups[b_name]
    print(f"\n{args.joint}: {len(a_mn)} {a_name} MNs, {len(b_mn)} {b_name} MNs\n")

    pre_a = presynaptic(row_start, col, weight, a_mn, n, args.min_syn)
    pre_b = presynaptic(row_start, col, weight, b_mn, n, args.min_syn)

    # Split the premotor layer by which side it favours. A cell driving both
    # sides about equally is shared, not part of a half centre.
    allpre = set(pre_a) | set(pre_b)
    group_a, group_b, shared = [], [], []
    for p in allpre:
        wa, wb = pre_a.get(p, 0), pre_b.get(p, 0)
        if wa >= args.bias * max(wb, 1):
            group_a.append(p)
        elif wb >= args.bias * max(wa, 1):
            group_b.append(p)
        else:
            shared.append(p)

    print(f"premotor layer: {len(allpre)} cells")
    print(f"  favour {a_name:<9} {len(group_a)}")
    print(f"  favour {b_name:<9} {len(group_b)}")
    print(f"  shared{'':<11}{len(shared)}\n")

    # Connections between the two premotor groups, by sign.
    setb = np.zeros(n, dtype=bool)
    setb[group_b] = True
    seta = np.zeros(n, dtype=bool)
    seta[group_a] = True

    def cross(src_list, dst_mask, label):
        exc = inh = 0
        exc_w = inh_w = 0
        for s in src_list:
            lo, hi = int(row_start[s]), int(row_start[s + 1])
            tgt = col[lo:hi]
            w = weight[lo:hi]
            keep = dst_mask[tgt]
            if not keep.any():
                continue
            tot = int(w[keep].sum())
            if sign[s] < 0:
                inh += int(keep.sum())
                inh_w += tot
            elif sign[s] > 0:
                exc += int(keep.sum())
                exc_w += tot
        print(f"  {label}: {exc} excitatory edges ({exc_w:,} synapses), "
              f"{inh} inhibitory ({inh_w:,})")
        return exc_w, inh_w

    # Null baseline. Any large subgraph of this connectome will come out
    # near the network-wide excitatory share, so a result only means
    # something if it departs from this.
    deg = np.diff(row_start).astype(np.int64)
    src_all = np.repeat(np.arange(n, dtype=np.int64), deg)
    s_all = sign[src_all]
    w64 = weight.astype(np.int64)
    net_exc = int(w64[s_all > 0].sum())
    net_inh = int(w64[s_all < 0].sum())
    net_frac = net_exc / max(net_exc + net_inh, 1)
    print(f"network-wide baseline: {net_frac:.0%} excitatory by synapse\n")

    print("connections between the two premotor groups:")
    ae, ai = cross(group_a, setb, f"{a_name}-premotor -> {b_name}-premotor")
    be, bi = cross(group_b, seta, f"{b_name}-premotor -> {a_name}-premotor")

    print()
    total_inh, total_exc = ai + bi, ae + be
    if total_inh == 0 and total_exc == 0:
        print("VERDICT: the two premotor groups do not talk to each other at "
              "all. No half centre here.")
    else:
        frac_exc = total_exc / (total_inh + total_exc)
        delta = frac_exc - net_frac
        print(f"VERDICT: mutual connection is {frac_exc:.0%} excitatory, "
              f"against a network\nbaseline of {net_frac:.0%}.")
        if delta < -0.10:
            print("  Markedly more inhibitory than the network at large, so "
                  "the substrate for\n  a half centre is present and the "
                  "missing piece is in the neuron model.")
        elif delta > 0.10:
            print("  Markedly more excitatory than the network at large. "
                  "Mutual excitation\n  synchronises rather than alternates, "
                  "so this is not a half centre.")
        else:
            print("  Indistinguishable from the network baseline, so this "
                  "measurement says\n  nothing either way. Reciprocal "
                  "inhibition between antagonist pools is\n  not detectable at "
                  "this resolution -- which is a limit of the method, not\n"
                  "  evidence that the fly lacks a rhythm generator.")


if __name__ == "__main__":
    main()
