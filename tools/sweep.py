"""Measure how wide the usable band of synaptic gain is, per model variant.

A connectome LIF network has a narrow range of synaptic gain between "the
evoked cascade dies out" and "the whole network saturates". How wide that
range is matters more than where it sits: a wider band means the model is not
balanced on a knife edge, and a published result is less likely to be an
artefact of one lucky parameter.

Band edges are found by bisection rather than by sampling a grid -- a grid
fine enough to separate these variants needs hundreds of runs, and a coarse
one just reports its own spacing back at you.

    python tools/sweep.py
    python tools/sweep.py --lo 0.5 --hi 5.0 --duration 200

Mean rate is very nearly monotonic in gain (there is a small dip where
inhibition catches up), so bisection finds a representative edge even though
it is not strictly guaranteed to be the only crossing.
"""
from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
EXE = ROOT / "build" / "Release" / "flysim.exe"
if not EXE.exists():
    EXE = ROOT / "build" / "flysim"

RATE_RE = re.compile(r"mean rate ([0-9.]+) Hz")

VARIANTS = [
    ("baseline (neither)",      ["--adapt", "0",   "--size-exp", "0"]),
    ("adaptation only",         ["--adapt", "0.4", "--size-exp", "0"]),
    ("size scaling only",       ["--adapt", "0",   "--size-exp", "0.5"]),
    ("both (current default)",  ["--adapt", "0.4", "--size-exp", "0.5"]),
]


def rate_at(epsp: float, extra: list[str], args) -> float:
    cmd = [str(EXE), "--stim-body", str(args.stim), "--duration", str(args.duration),
           "--epsp", f"{epsp:.5f}", "--top", "0", *extra]
    out = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT).stdout
    m = RATE_RE.search(out)
    return float(m.group(1)) if m else float("nan")


def find_crossing(target: float, extra: list[str], args,
                  lo: float = 0.01, hi: float = 2.0, iters: int = 16) -> float:
    """Smallest gain whose mean rate reaches `target`, by bisection."""
    if rate_at(hi, extra, args) < target:
        return float("nan")  # never gets there
    if rate_at(lo, extra, args) >= target:
        return lo
    for _ in range(iters):
        mid = (lo * hi) ** 0.5  # geometric midpoint
        if rate_at(mid, extra, args) >= target:
            hi = mid
        else:
            lo = mid
    return (lo * hi) ** 0.5


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stim", type=int, default=10001, help="bodyId to drive")
    ap.add_argument("--duration", type=float, default=100.0)
    ap.add_argument("--lo", type=float, default=0.5, help="lower bound of plausible Hz")
    ap.add_argument("--hi", type=float, default=5.0, help="upper bound of plausible Hz")
    args = ap.parse_args()

    if not EXE.exists():
        sys.exit(f"{EXE} not found -- build first")

    print(f"driving bodyId {args.stim} for {args.duration:.0f} ms")
    print(f"plausible band = mean network rate between {args.lo} and {args.hi} Hz\n")
    print(f"{'variant':<26} {'band lower':>11} {'band upper':>11} {'width':>8}")
    print("-" * 60)

    for name, extra in VARIANTS:
        lower = find_crossing(args.lo, extra, args)
        upper = find_crossing(args.hi, extra, args)
        if lower != lower or upper != upper:
            print(f"{name:<26} {'--':>11} {'--':>11} {'n/a':>8}")
            continue
        print(f"{name:<26} {lower:>11.4f} {upper:>11.4f} {upper/lower:>7.2f}x")

    print("\nWidth is the ratio of the two edges: how far gain can drift before")
    print("the network leaves a plausible regime. Higher is more robust.")


if __name__ == "__main__":
    main()
