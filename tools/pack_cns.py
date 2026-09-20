"""Pack data/raw/ into a single binary the C++ engine loads with one read.

Layout of data/bin/cns.bin -- a header, then tightly packed arrays in this
order.  Everything is little-endian; offsets are absolute byte positions so the
loader can mmap and point at each array without parsing.

    magic      char[8]   "FLYCNS01"
    version    u32       = 2
    n_neurons  u32
    n_edges    u64
    min_weight u32
    _pad       u32
    bbox       f32[6]    xmin,ymin,zmin,xmax,ymax,zmax  (micrometres)
    arrays:
      body_id   i64[N]     neuPrint bodyId, for round-tripping to the web UI
      pos       f32[N*3]   soma position in micrometres
      size_rel  f32[N]     segmentation volume / median, proxy for capacitance
      nt_code   u8[N]      index into the neurotransmitter table
      sign      i8[N]      +1 excitatory, -1 inhibitory, 0 modulatory/unknown
      flags     u8[N]      bit0: real soma (0 = inferred)
                           bit1: graded -- releases continuously, never spikes
      _pad      u8[N]
      row_start u64[N+1]   CSR offsets into the edge arrays
      col       u32[E]     target neuron index
      weight    u16[E]     synapse count, saturated at 65535

Array order matters: size_rel sits before the byte arrays so every array lands
on its natural alignment regardless of N, which the C++ loader checks.

Neuron names/types live alongside in cns_meta.json so the binary stays
fixed-width.

Usage:  python tools/pack_cns.py
        python tools/pack_cns.py --graded-superclass ol_intrinsic
"""
from __future__ import annotations

import argparse
import json
import pathlib
import struct
import sys
import time

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
RAW = ROOT / "data" / "raw"
BIN = ROOT / "data" / "bin"

# male-cns voxels are 8 nm on a side; convert to micrometres for sane camera
# units in the renderer.
VOXEL_NM = 8.0
NM_PER_UM = 1000.0
SCALE = VOXEL_NM / NM_PER_UM

MAGIC = b"FLYCNS01"
VERSION = 2

# Cell types modelled as graded (non-spiking): they release transmitter in
# proportion to membrane depolarisation rather than emitting discrete spikes.
#
# This is a judgement call, not a dataset field. The lamina monopolar cells
# L1-L5 are the best-established graded neurons present in this volume --
# photoreceptors, the other classic example, sit in the retina and were not
# imaged. Much of the rest of the optic lobe is graded or mixed, but where the
# line falls is an open question, so the default stays conservative. Pass
# --graded-superclass ol_intrinsic to treat the whole optic lobe as graded.
DEFAULT_GRADED_TYPES = ["L1", "L2", "L3", "L4", "L5"]


def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def load_edges() -> np.ndarray:
    meta_path = RAW / "edges" / "_meta.json"
    if not meta_path.exists():
        sys.exit("data/raw/edges/_meta.json missing -- run fetch_cns.py first")
    chunks = sorted((RAW / "edges").glob("chunk_*.npy"))
    log(f"loading {len(chunks)} edge chunks")
    parts = [np.load(p) for p in chunks]
    parts = [p for p in parts if len(p)]
    edges = np.concatenate(parts) if parts else np.empty((0, 3), np.int64)
    log(f"  {len(edges):,} edges loaded")
    return edges


def infer_missing_positions(pos: np.ndarray, col: np.ndarray,
                            row_start: np.ndarray, rounds: int = 6) -> np.ndarray:
    """Place soma-less neurons at the centroid of their connected partners.

    ~20% of neurons (optic-lobe intrinsics and sensory afferents) have no soma
    in the imaged volume.  Averaging over graph neighbours, repeated a few
    times so positions propagate inward, gives something spatially plausible to
    draw.  These are flagged so the renderer can tell them apart -- they are a
    layout convenience, not anatomy.
    """
    n = len(pos)
    known = np.isfinite(pos[:, 0])
    log(f"inferring positions for {int((~known).sum()):,} soma-less neurons")

    # Undirected neighbour list, so a sensory afferent with only outgoing
    # connections still gets pulled toward its targets.
    # row_start is uint64; np.repeat refuses unsigned repeat counts.
    src = np.repeat(np.arange(n, dtype=np.int64),
                    np.diff(row_start).astype(np.int64))
    dst = col.astype(np.int64)
    a = np.concatenate([src, dst])
    b = np.concatenate([dst, src])

    out = pos.copy()
    out[~known] = 0.0
    for r in range(rounds):
        valid = np.isfinite(pos[:, 0]) | (out[:, 0] != 0.0)
        contrib = valid[b]
        if not contrib.any():
            break
        sums = np.zeros((n, 3), np.float64)
        counts = np.zeros(n, np.int64)
        np.add.at(sums, a[contrib], out[b[contrib]])
        np.add.at(counts, a[contrib], 1)
        fill = (~known) & (counts > 0)
        out[fill] = (sums[fill] / counts[fill, None]).astype(np.float32)
        remaining = int(((~known) & (out[:, 0] == 0.0)).sum())
        log(f"  round {r+1}: {remaining:,} still unplaced")
        if remaining == 0:
            break

    # Anything still unplaced (fully disconnected) goes to the centroid.
    stranded = (~known) & (out[:, 0] == 0.0)
    if stranded.any():
        out[stranded] = np.nanmean(pos[known], axis=0)
        log(f"  {int(stranded.sum()):,} disconnected neurons parked at centroid")
    return out, known


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--graded-types", nargs="*", default=DEFAULT_GRADED_TYPES,
                    metavar="TYPE",
                    help="cell types to model as non-spiking "
                         f"(default: {' '.join(DEFAULT_GRADED_TYPES)}); "
                         "pass with no values to disable")
    ap.add_argument("--graded-superclass", nargs="*", default=[], metavar="SC",
                    help="whole superclasses to model as non-spiking, "
                         "e.g. ol_intrinsic for the entire optic lobe")
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    nz = np.load(RAW / "neurons.npz", allow_pickle=False)
    body_id = nz["body_id"]
    n = len(body_id)
    log(f"{n:,} neurons")

    edges = load_edges()

    # Map bodyId -> dense index. body_id is already sorted by the fetcher.
    order = np.argsort(body_id, kind="stable")
    assert np.all(order == np.arange(n)), "body_id must be sorted"

    src_idx = np.searchsorted(body_id, edges[:, 0])
    dst_idx = np.searchsorted(body_id, edges[:, 1])
    ok = ((src_idx < n) & (dst_idx < n)
          & (body_id[np.clip(src_idx, 0, n - 1)] == edges[:, 0])
          & (body_id[np.clip(dst_idx, 0, n - 1)] == edges[:, 1]))
    dropped = int((~ok).sum())
    if dropped:
        log(f"dropping {dropped:,} edges touching neurons outside the Neuron set")
    src_idx, dst_idx = src_idx[ok].astype(np.uint32), dst_idx[ok].astype(np.uint32)
    weight = np.clip(edges[ok, 2], 0, 65535).astype(np.uint16)
    e = len(src_idx)

    # Sort into CSR order (by source, then target).
    log("building CSR")
    perm = np.lexsort((dst_idx, src_idx))
    col = dst_idx[perm]
    weight = weight[perm]
    counts = np.bincount(src_idx, minlength=n)
    row_start = np.zeros(n + 1, np.uint64)
    np.cumsum(counts, out=row_start[1:])

    pos_vox = nz["pos"]
    pos_um = pos_vox * SCALE
    pos_um, had_soma = infer_missing_positions(pos_um, col, row_start)

    # Relative cell size. Neurons with no recorded volume sit at the median so
    # they are neither favoured nor penalised.
    size = nz["size"].astype(np.float64)
    median_size = float(np.median(size[size > 0])) if (size > 0).any() else 1.0
    size_rel = np.where(size > 0, size / median_size, 1.0).astype(np.float32)

    types = nz["type"]
    supers = nz["superclass"]
    graded = np.isin(types, args.graded_types)
    if args.graded_superclass:
        graded |= np.isin(supers, args.graded_superclass)

    flags = had_soma.astype(np.uint8) | (graded.astype(np.uint8) << 1)
    finite = np.isfinite(pos_um).all(axis=1)
    bbox = np.concatenate([pos_um[finite].min(axis=0), pos_um[finite].max(axis=0)])

    log(f"median cell size {median_size:,.0f} voxels; "
        f"size_rel spans {size_rel.min():.3g}..{size_rel.max():.3g}")
    log(f"{int(graded.sum()):,} neurons flagged graded "
        f"({', '.join(args.graded_types) or 'none'}"
        f"{' + superclass ' + ','.join(args.graded_superclass) if args.graded_superclass else ''})")

    BIN.mkdir(parents=True, exist_ok=True)
    out = BIN / "cns.bin"
    min_weight = json.loads((RAW / "edges" / "_meta.json").read_text())["min_weight"]

    with open(out, "wb") as f:
        f.write(MAGIC)
        f.write(struct.pack("<IIQII", VERSION, n, e, min_weight, 0))
        f.write(bbox.astype(np.float32).tobytes())
        f.write(body_id.astype(np.int64).tobytes())
        f.write(pos_um.astype(np.float32).tobytes())
        f.write(size_rel.tobytes())
        f.write(nz["nt_code"].astype(np.uint8).tobytes())
        f.write(nz["sign"].astype(np.int8).tobytes())
        f.write(flags.tobytes())
        f.write(np.zeros(n, np.uint8).tobytes())
        f.write(row_start.astype(np.uint64).tobytes())
        f.write(col.astype(np.uint32).tobytes())
        f.write(weight.astype(np.uint16).tobytes())

    meta = {
        "dataset": "male-cns:v1.0",
        "n_neurons": int(n),
        "n_edges": int(e),
        "n_synapses": int(weight.astype(np.int64).sum()),
        "min_weight": int(min_weight),
        "units": "micrometres",
        "bbox_um": [round(float(v), 1) for v in bbox],
        "nt_table": ["unknown", "acetylcholine", "glutamate", "gaba", "histamine",
                     "dopamine", "serotonin", "octopamine", "unclear"],
        "median_size_voxels": median_size,
        "graded_types": list(args.graded_types),
        "graded_superclasses": list(args.graded_superclass),
        "n_graded": int(graded.sum()),
    }
    (BIN / "cns_meta.json").write_text(json.dumps(meta))

    # Fixed-column sidecar so the C++ side can label neurons without pulling in
    # a JSON parser. One line per neuron, in dense-index order.
    tab, nl = chr(9), chr(10)
    with open(BIN / "cns_names.tsv", "w", encoding="utf-8", newline=nl) as f:
        f.write(tab.join(["index", "body_id", "type", "superclass"]) + nl)
        for i in range(n):
            f.write(tab.join([str(i), str(body_id[i]), types[i], supers[i]]) + nl)

    log(f"wrote {out} ({out.stat().st_size/1e6:.1f} MB)")
    log(f"  {n:,} neurons, {e:,} edges, {meta['n_synapses']:,} synapses")
    log(f"  bbox (um): x {bbox[0]:.0f}..{bbox[3]:.0f}  "
        f"y {bbox[1]:.0f}..{bbox[4]:.0f}  z {bbox[2]:.0f}..{bbox[5]:.0f}")
    log(f"  mean out-degree {e/n:.1f}, max {int(counts.max()):,}")


if __name__ == "__main__":
    main()
