# Research

Written for whoever picks this project up next — including me, months from
now, with none of the context.

This directory is the durable record: what was learned, why it is true, and
what it cost to find out. It is deliberately separate from `docs/findings.md`,
which is a running narrative in the order things happened. Here the material
is sorted by subject and written to be reused.

## The project in four sentences

A leaky integrate-and-fire simulation of the Janelia **male-cns:v1.0**
*Drosophila* connectome — 176,422 neurons, ~25.9M synaptic connections — wired
to a physically simulated fly body in a custom C++ sandbox (SDL3 + OpenGL, own
maximal-coordinate rigid-body solver). The long-term goal is a fly whose legs
are moved by its own motor neurons. The body and the nervous system are
developed separately because each is hard enough alone, and they meet at
`src/body/MotorPools.*`. Units throughout are **millimetres, micrograms,
seconds**, so a torque of 1 is 1e-15 N·m and the whole animal weighs about
1300 µg.

## What to read, in order

| File | Subject |
|---|---|
| [01-solver-physics.md](01-solver-physics.md) | The rigid-body solver. The most reusable material here — several of these bugs are generic to any constraint solver. |
| [02-leg-geometry.md](02-leg-geometry.md) | Leg kinematics, the foot Jacobian, and what makes a rest pose able to walk rather than only stand. |
| [03-gait-control.md](03-gait-control.md) | The gait controller: foot-path planning, load sharing, and the measured tuning. |
| [04-connectome.md](04-connectome.md) | The nervous-system side: what the connectome has and has not produced. |
| [05-method.md](05-method.md) | How to test this thing, and the traps that have cost whole nights. |
| [06-open-questions.md](06-open-questions.md) | What is still wrong, ranked, and everything already ruled out. |

`docs/walking.md` is the short current-state summary. `docs/findings.md` is the
chronological narrative with the full working.

## Current state, one table

```
standing                    stable indefinitely, anchor error 0.022 mm
walking, 12 s, 5 trials     4 of 5 upright, 1 of 5 linkage burst
walking speed               3.89 mm/s   (a real fly: 10-20)
worst pitch                 13 deg      (a real fly: a few)
duty factor                 ~0.9        (a real fly: ~0.5)
gait source                 an imposed tripod, NOT the connectome
```

That last line matters and is easy to lose: **nothing in this project has ever
walked from the connectome.** The gait is a hand-written pattern. See
[04-connectome.md](04-connectome.md).

## Building and running

```sh
cmake -B build -S .
cmake --build build --config Release --target flyphys   # mechanics only, fast
cmake --build build --config Release --target flybody   # 3D view
```

`flyphys` touches no connectome data and runs in seconds, so it is where all
body work happens. `flybody` needs the packed data (`tools/fetch_cns.py` then
`tools/pack_cns.py`) but will run the body alone.

```sh
build/Release/flyphys.exe --stand-only              # quick regression
build/Release/flyphys.exe --gait --seconds 12 --trials 5
build/Release/flyphys.exe --gait --from 2.0 --to 2.2 --trials 1   # per-frame
build/Release/flyphys.exe --jacobian                # what joints do to feet
build/Release/flybody.exe --gait 60 --film out/w --film-every 25 --frames 1500
```

**Kill the binary before rebuilding.** A held `.exe` makes the link fail and
the old binary runs, producing byte-identical sweep rows that look like a real
null result. This has cost hours, repeatedly:

```sh
taskkill //F //IM flyphys.exe
```
