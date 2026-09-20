"""Download the male-cns:v1.0 connectome into data/raw/.

Two stages, both resumable -- rerun after an interruption and it picks up
where it stopped:

  neurons  ->  data/raw/neurons.npz   (one shot, ~8 requests)
  edges    ->  data/raw/edges/*.npy   (chunked by source neuron, ~180 requests)

Usage:
    python tools/fetch_cns.py                 # both stages, weight >= 2
    python tools/fetch_cns.py --min-weight 5  # lighter download
    python tools/fetch_cns.py --stage edges
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import np_client as nc

ROOT = pathlib.Path(__file__).resolve().parent.parent
RAW = ROOT / "data" / "raw"
EDGE_DIR = RAW / "edges"

NEURON_CHUNK = 25_000
EDGE_CHUNK = 1_000

# Drosophila sign convention. Glutamate is *inhibitory* in the fly (GluCl-alpha
# receptors), unlike in vertebrates -- this follows Shiu et al. 2024.
NT_SIGN = {
    "acetylcholine": +1,
    "glutamate": -1,
    "gaba": -1,
    "histamine": -1,
    "dopamine": 0,
    "serotonin": 0,
    "octopamine": 0,
    "unclear": 0,
}
NT_CODES = {
    name: i
    for i, name in enumerate(
        ["unknown", "acetylcholine", "glutamate", "gaba", "histamine",
         "dopamine", "serotonin", "octopamine", "unclear"]
    )
}


def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def fetch_neurons() -> None:
    out = RAW / "neurons.npz"
    if out.exists():
        log(f"neurons.npz already present ({out.stat().st_size/1e6:.1f} MB) -- skipping")
        return

    total = nc.cypher("MATCH (n:Neuron) RETURN count(n)")[0][0]
    log(f"fetching {total:,} neurons in {-(-total // NEURON_CHUNK)} chunks")

    rows = []
    for skip in range(0, total, NEURON_CHUNK):
        batch = nc.cypher(
            f"""MATCH (n:Neuron)
                RETURN n.bodyId, n.somaLocation.x, n.somaLocation.y, n.somaLocation.z,
                       n.consensusNt, n.predictedNt, n.type, n.class, n.superclass,
                       n.somaSide, n.pre, n.post
                ORDER BY n.bodyId SKIP {skip} LIMIT {NEURON_CHUNK}"""
        )
        rows.extend(batch)
        log(f"  neurons {len(rows):,}/{total:,}")

    n = len(rows)
    body_id = np.empty(n, np.int64)
    pos = np.full((n, 3), np.nan, np.float32)
    nt_code = np.zeros(n, np.uint8)
    sign = np.zeros(n, np.int8)
    pre = np.zeros(n, np.int32)
    post = np.zeros(n, np.int32)
    types: list[str] = []
    classes: list[str] = []
    superclasses: list[str] = []
    soma_side: list[str] = []

    for i, r in enumerate(rows):
        body_id[i] = r[0]
        if r[1] is not None:
            pos[i] = (r[1], r[2], r[3])
        # consensusNt is the curated call where it exists; fall back to the
        # per-cell-type prediction.
        nt = (r[4] or r[5] or "unknown").lower()
        nt_code[i] = NT_CODES.get(nt, 0)
        sign[i] = NT_SIGN.get(nt, 0)
        types.append(r[6] or "")
        classes.append(r[7] or "")
        superclasses.append(r[8] or "")
        soma_side.append(r[9] or "")
        pre[i] = r[10] or 0
        post[i] = r[11] or 0

    RAW.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out,
        body_id=body_id, pos=pos, nt_code=nt_code, sign=sign, pre=pre, post=post,
        type=np.array(types), cls=np.array(classes),
        superclass=np.array(superclasses), soma_side=np.array(soma_side),
    )
    have_soma = int(np.isfinite(pos[:, 0]).sum())
    log(f"wrote {out.name}: {n:,} neurons, {have_soma:,} with soma "
        f"({100*have_soma/n:.0f}%), {int((sign!=0).sum()):,} with a signed NT")


def fetch_edges(min_weight: int) -> None:
    EDGE_DIR.mkdir(parents=True, exist_ok=True)
    meta = EDGE_DIR / "_meta.json"
    if meta.exists():
        prev = json.loads(meta.read_text())["min_weight"]
        if prev != min_weight:
            sys.exit(
                f"data/raw/edges/ was downloaded at min_weight={prev}, you asked for "
                f"{min_weight}.\nDelete the directory to re-download, or rerun with "
                f"--min-weight {prev}."
            )

    ids = np.load(RAW / "neurons.npz")["body_id"]
    batches = list(nc.chunks(ids.tolist(), EDGE_CHUNK))
    log(f"fetching edges (weight >= {min_weight}) for {len(ids):,} source neurons "
        f"in {len(batches)} chunks")

    done = total_edges = 0
    t0 = time.time()
    for idx, batch in enumerate(batches):
        path = EDGE_DIR / f"chunk_{idx:05d}.npy"
        if path.exists():
            total_edges += len(np.load(path))
            done += 1
            continue

        id_list = "[" + ",".join(str(b) for b in batch) + "]"
        rows = nc.cypher(
            f"""MATCH (a:Neuron)-[w:ConnectsTo]->(b:Neuron)
                WHERE a.bodyId IN {id_list} AND w.weight >= {min_weight}
                RETURN a.bodyId, b.bodyId, w.weight"""
        )
        arr = np.array(rows, dtype=np.int64) if rows else np.empty((0, 3), np.int64)
        # Write via a temp file so an interrupted run never leaves a short chunk
        # that the resume logic would happily accept.
        # np.save appends .npy to any name lacking it, so the temp name must
        # already end in .npy or the rename below has nothing to find.
        tmp = path.with_name(path.name + ".tmp.npy")
        np.save(tmp, arr)
        tmp.replace(path)

        total_edges += len(arr)
        done += 1
        if done % 10 == 0 or done == len(batches):
            rate = done / max(time.time() - t0, 1e-9)
            eta = (len(batches) - done) / max(rate, 1e-9)
            log(f"  chunk {done}/{len(batches)}  edges={total_edges:,}  "
                f"eta={eta/60:.1f} min")

    meta.write_text(json.dumps({"min_weight": min_weight, "chunks": len(batches),
                                "edges": total_edges}, indent=2))
    log(f"edge download complete: {total_edges:,} edges in {len(batches)} chunks")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--min-weight", type=int, default=2,
                    help="drop connections with fewer synapses (default: 2)")
    ap.add_argument("--stage", choices=["neurons", "edges", "all"], default="all")
    args = ap.parse_args()

    if args.stage in ("neurons", "all"):
        fetch_neurons()
    if args.stage in ("edges", "all"):
        fetch_edges(args.min_weight)


if __name__ == "__main__":
    main()
