# FlyBrain

A leaky integrate-and-fire simulation of the adult *Drosophila* central nervous
system, built on the Janelia **male-cns:v1.0** connectome — 176,422 neurons and
~25.9M synaptic connections spanning brain and ventral nerve cord.

Two programs: `flysim` runs the model headless and reports who fired, and
`flyviz` draws all 176,422 neurons in their real anatomical positions and
lights them as they fire.

## Layout

```
tools/       Python, run once: pull the connectome, pack it, derive the motor map
src/core/    simulation engine (no dependencies)
src/engine/  maths and mesh generation
src/body/    fly skeleton and the motor-neuron-to-muscle bridge
src/viz/     3D renderer (SDL3 + OpenGL 3.3)
data/        downloaded + packed data (gitignored)
```

Three programs: `flysim` runs the model headless, `flyviz` draws the nervous
system, and `flybody` drives a fly's legs from its own motor neurons.

## Getting the data

Janelia serves `male-cns:v1.0` over neuPrint's Cypher endpoint without
authentication, so no token or account is needed.

```sh
python tools/fetch_cns.py     # ~5 min, ~300 MB into data/raw/
python tools/pack_cns.py      # packs into data/bin/cns.bin
```

`fetch_cns.py` is resumable — it writes one file per chunk of source neurons
and skips what is already on disk, so an interrupted run just needs rerunning.
Pass `--min-weight 5` for a lighter download that keeps ~72% of synaptic mass.

## Building

```sh
cmake -S . -B build
cmake --build build --config Release
./build/Release/flysim --help
```

Requires a C++20 compiler and CMake 3.21+. No external libraries.

## Running

Drive a cell type and see what lights up downstream:

```sh
./build/Release/flysim --stim-type DNp01 --duration 200
./build/Release/flysim --list-types DN     # find cell types by substring
```

`DNp01` is the giant fibre descending neuron that triggers the escape reflex —
a good first target, because its downstream path into the leg motor neurons is
well described in the literature and you can check the simulation against it.


## The 3D view

```sh
./build/Release/flyviz                       # giant fibre, the default
./build/Release/flyviz --stim-type L2        # drive the lamina instead
```

Both programs locate `cns.bin` by searching upward from the working directory
and from their own location, so they run from anywhere in the tree -- including
double-clicked from a file manager -- without needing `--data`.

| | |
|---|---|
| left drag | orbit |
| right drag | pan |
| scroll | zoom |
| space | fire the stimulus again |
| `[` `]` | fewer / more simulation steps per frame |
| `r` | reset |
| `s` | hide resting neurons, show only active ones |
| `esc` | quit |

What you see is slow motion by default -- eight simulation steps per frame, so
about 0.8 ms of fly time per frame. That is deliberate. At true speed the
synaptic delay is 1.8 ms and the whole escape response would be over in three
frames.

Neurons are drawn as point sprites with additive blending and no depth test, so
the cloud reads as a volume and density becomes brightness rather than the
nearest point hiding everything behind it. Resting neurons sit at a low alpha;
firing ones get both brighter and larger, because a response involving a few
hundred neurons has to be visible against 176,000 resting ones. Graded neurons
never spike, so they are drawn from their continuous output instead.

### Checking that it draws what the model does

`--frames N --screenshot FILE` renders N frames and saves the last one, which
makes the renderer testable without a human watching the window. Diffing those
frames against an unstimulated baseline tracks where activity actually is:

| sim time | activity centroid | region |
|---------:|------------------:|--------|
| 1.2 ms   | (nothing yet)     | -- |
| 2.4 ms   | x = 560           | brain |
| 9.6 ms   | x = 894           | nerve cord |

Nothing happens before 1.8 ms because that is the synaptic delay; activity then
appears in the brain and reaches the ventral nerve cord about 7 ms later, which
is roughly four synaptic hops. The descending wave is real, not a lighting
effect.

Measuring this needs the baseline subtraction. A naive "count bright pixels"
threshold returns exactly the same number every frame, because the densest
regions of the optic lobe accumulate past any fixed threshold while at rest.

### Dependencies

SDL3 and OpenGL 3.3. CMake looks for an SDL3 checkout at `../Library/SDL` by
default and builds it into this project's own build tree; point it elsewhere
with `-DFLYBRAIN_SDL_DIR=<path>`, or install SDL3 and it will be found. Build
with `-DFLYBRAIN_VIZ=OFF` to skip the renderer entirely.

The ~27 OpenGL entry points beyond 1.1 are resolved through
`SDL_GL_GetProcAddress` in [src/viz/GL.h](src/viz/GL.h) rather than pulling in
GLAD or GLEW, and the matrix maths is a few dozen lines in
[src/viz/Camera.h](src/viz/Camera.h) rather than GLM. Nothing else is needed.

## Model

A clock-driven LIF network. Each neuron integrates a leak toward rest plus
incoming voltage steps; crossing threshold emits a spike, resets, and enters a
refractory period. Spikes arrive at their targets after a uniform delay,
carried through a ring buffer so cost scales with the out-degree of neurons
that actually fired rather than with network size.

Connection sign comes from the presynaptic neuron's predicted
neurotransmitter. **Glutamate is treated as inhibitory** — in *Drosophila* it
gates GluClα chloride channels, unlike in vertebrates. Acetylcholine (54% of
neurons) is excitatory, GABA and histamine inhibitory. The modulators
(dopamine, serotonin, octopamine — about 3% of neurons) act on timescales this
model does not represent, so their connections carry no current by default.

### Beyond plain LIF

Three additions go past the baseline model. One earned its place, one is
useful but limited, and one is off by default because it failed its own test.

**Spike-frequency adaptation** (on). Each spike raises that neuron's own
threshold; the raise decays with a 150 ms time constant. Without it nothing
stops a neuron pinning at the refractory ceiling — 410 Hz in the first version
of this model, which no fly neuron does. It also measurably widens the usable
range of synaptic gain (below). It does *not* rescue a badly over-set gain: at
`--epsp 0.40` the network still saturates whatever the adaptation strength,
because a neuron receiving 40 mV per presynaptic spike does not care about
12 mV of accumulated threshold.

**Graded transmission** (on, conservative). Neurons flagged graded release
continuously in proportion to depolarisation and never spike. Output is
expressed as the firing rate a spiking neuron would need to deliver the same
current, so it shares the `--epsp` scale. Propagation is lazy — a graded
neuron only touches its out-edges when its output moves more than
`gradedDeltaHz` — which keeps it sparse instead of costing every edge every
timestep.

Only L1–L5 are flagged by default. That is a judgement call, not a dataset
field: those lamina monopolar cells are the best-established graded neurons in
this volume, photoreceptors were not imaged, and how far graded transmission
extends into the medulla is genuinely unsettled. Pass
`--graded-superclass ol_intrinsic` to `pack_cns.py` to treat the whole optic
lobe that way.

**Size-scaled excitability** (off). A bigger cell has more membrane to charge,
so the same current should move it less; incoming steps get scaled by
`size^-0.5`. It is implemented correctly — 96.5% of neurons are scaled
smoothly, only 3.5% hit a clamp — and it is plausible physics. It just doesn't
help. Enable it with `--size-exp 0.5`.

### Calibration

`--epsp`, the depolarisation per synapse, is the parameter that matters. What
matters more than its value is how much room there is around it. `tools/sweep.py`
bisects for the gain at which the network enters and leaves a plausible regime
(0.5–5 Hz mean rate), driving DNp01 for 100 ms:

| variant | band lower | band upper | width |
|---------|-----------:|-----------:|------:|
| baseline (neither) | 0.0616 | 0.0986 | 1.60× |
| **adaptation only (default)** | **0.0648** | **0.1160** | **1.79×** |
| size scaling only | 0.1959 | 0.2766 | 1.41× |
| both | 0.1967 | 0.2857 | 1.45× |

Adaptation widens the band by 12%. Size scaling narrows it by 12% — it shifts
where the band sits without making the network any less precarious, which is
why it ships disabled. The default `--epsp 0.085` is the geometric centre of
the winning row.

Note how narrow all of these are. Even the best is under 2×: double the
synaptic gain and the model goes from silent to saturated. That fragility is
worth keeping in mind before reading much into any single run, and it is a
real property of connectome LIF models rather than a defect of this one.

None of these numbers transfer. They depend on the weight threshold used at
download time and would change for a different dataset.

## Validating against known anatomy

Driving the giant fibre and inspecting its direct targets reproduces the escape
circuit:

```sh
./build/Release/flysim --stim-body 10001 --targets 12 --top 0
```

```
rank  bodyId       type          superclass       synapses   spikes
1     800146       TTMn          vnc_motor              70       14
2     800178       IN11A001      vnc_intrinsic          32        7
3     802471       IN18B031      vnc_intrinsic          26        9
7     802799       GFC2          vnc_intrinsic          21        9
9     802478       GFC2          vnc_intrinsic          20        7
```

TTMn is the tergotrochanteral motor neuron that fires the jump muscle, and GFC2
the giant-fibre coupled interneurons — which is what the giant fibre is
supposed to drive.

Ranking the whole network by spike count tells the same story from the other
direction. The top of that list is five **DLMn** — the dorsal longitudinal
muscle motor neurons that depress the wings — alongside lateral horn and
ascending neurons. Jump plus wing depression is the takeoff sequence, and none
of it was put in by hand.

Two things worth knowing about this readout:

- **Rank by spike count can mislead, and did.** Before spike-frequency
  adaptation was added, the top 25 for this run was dominated by AVLP neurons
  that are *not* downstream of DNp01 — the anterior ventrolateral
  protocerebrum is recurrent enough to sustain itself once seeded, and it
  drowned out the real pathway. Adaptation damped that, and the ranking now
  agrees with the anatomy. The lesson survives the fix: `--targets` walks the
  out-edges, so a silent target reads as a zero rather than being absent, and
  it does not care what else is loud.
- **One spike is not enough.** DNp01's strongest connection is 70 synapses, or
  about 6 mV at the default gain, against a 7 mV threshold. TTMn only fires
  because DNp01 is driven repeatedly. That is a real property of the model, and
  it is why coincidence matters here.

A second check, on the graded side: driving L2 (cholinergic, non-spiking)
excites Tm1 and Tm2, which is the well-described first stage of the motion
vision pathway. Driving L1 instead produces silence, because L1 is
glutamatergic and therefore inhibitory under the sign rule above.

Stimulating TTMn itself produces nothing downstream, which is the correct
negative control: motor neurons terminate on muscle, and muscle is not in the
connectome.

See `LifParams` in [LIFNetwork.h](src/core/LIFNetwork.h) for the full parameter
set.


## The body

```sh
python tools/motor_map.py                       # once, after pack_cns.py
./build/Release/flybody                         # drives the knee flexors
./build/Release/flybody --stim-type "Tr extensor MN"
./build/Release/flybody --dump-pose             # check the skeleton, no window
```

The connectome names every motor neuron after the muscle it drives, and
`somaNeuromere` plus `somaSide` say which of the six legs it belongs to:

```
Ti flexor MN_R    neuromere=T1  side=R    right front leg, knee flexor
Ti flexor MN_L    neuromere=T3  side=L    left hind leg, knee flexor
```

`tools/motor_map.py` turns that into a wiring table: 334 of the 708 VNC motor
neurons, across all six legs and all five leg joints. The rest are wing
steering muscles, flight power muscles, halteres and abdominal muscles, which
sit in leg neuromeres because those structures attach to the thorax but do not
move a leg. A further group, the `MNhl*`/`MNml*` neurons, *are* leg motor
neurons but are numbered rather than named after a muscle, so there is nothing
to map them by.

Each joint gets two antagonist pools. Spikes are low-pass filtered into a
muscle activation with a 30 ms time constant, and the difference between the
antagonists sets the joint angle. The filter is not smoothing for its own sake:
muscle force develops over tens of milliseconds, so firing *rate* is what a
muscle responds to, and it also bridges the gap between a 0.1 ms neural step
and a display frame.

Driving one named pool moves one joint:

```
$ flybody --stim-type "Ti flexor MN" --frames 60
leg        joint      flex   extend angle-rest
front_L    FTi       0.392    0.000     -0.275
middle_L   FTi       0.392    0.000     -0.236
hind_L     FTi       0.392    0.000     -0.275
...
```

Only FTi, the knee, on every leg that has those neurons, in the flexor
direction. `Tr extensor MN` moves CTr the other way instead.

### The skeleton

Six legs of five joints each, at *Drosophila* proportions, in millimetres. The
rest pose is not hand-tuned: `tools/solve_rest_pose.py` mirrors the forward
kinematics and runs damped least squares on the five angles of each leg until
the foot reaches the ground. Five coupled angles per leg, to a common ground
height, is not something to fit by eye.

`--dump-pose` checks the result without opening a window, including that left
and right are exact mirrors. That check earns its place: an earlier version had
the mirroring rule backwards and every foot still passed a "below the body and
out to the side" test while the two sides sat in visibly different poses.

Two things the solver taught us about fly legs. The hind knee has to bend the
*opposite way* to the others -- forcing all six to fold alike leaves the hind
leg unable to reach behind the body at all. And thorax-coxa has to be kept
short of horizontal, because given free rein the solver reaches the target by
routing the femur up over the thorax: a valid solution to the equations and a
nonsense one for a fly.

### What this is not

There is no physics yet. Joint angles are set directly, the body floats at a
fixed height, and nothing collides with anything. Ground contact and rigid-body
dynamics are the next stage.

It will also not walk. Coordinated locomotion depends on central pattern
generators whose dynamics live in neuromodulation and intrinsic membrane
properties that a LIF model does not have -- this model sets modulatory
transmitters to zero gain outright. Connectome-driven walking is an open
research problem; NeuroMechFly, the state of the art for a neuromechanical fly,
gets its gaits from optimised CPGs and motion capture rather than from
simulating the connectome. Reflexes and single muscle actions are in reach.
Gait is not.

## Caveats

- **Positions.** 141,781 neurons (80%) have a real soma in the imaged volume.
  The rest are optic-lobe intrinsics and sensory afferents whose cell bodies
  sit outside it; the packer places them at the centroid of their connected
  partners so they can be drawn. Bit 0 of the `flags` array marks the real
  ones. Inferred positions are a layout convenience, not anatomy.
- **Connection strength.** Synapse count is a proxy for synaptic weight, not a
  measurement of it.
- **No compartments.** Each neuron is a single point. Real fly neurons compute
  within their dendrites, and graded-potential neurons — much of the optic
  lobe — do not spike at all.

## Data

Janelia FlyEM, `male-cns:v1.0`, via [neuprint.janelia.org](https://neuprint.janelia.org).
The model follows the approach of Shiu et al. 2024, *A leaky integrate-and-fire
computational model based on the connectome of the entire adult Drosophila
brain*, which validated the same style of model against taste-evoked activity
in FlyWire.
