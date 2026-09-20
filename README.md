# FlyBrain

A leaky integrate-and-fire simulation of the adult *Drosophila* central nervous
system, built on the Janelia **male-cns:v1.0** connectome — 176,422 neurons and
~25.9M synaptic connections spanning brain and ventral nerve cord.

The end goal is an interactive 3D sandbox in C++. This repo gets there in
stages; stage 1 and 2 are working.

## Layout

```
tools/       Python, run once: pull the connectome and pack it to binary
src/core/    C++ simulation engine (no dependencies)
src/viz/     C++ 3D renderer (SDL3 + OpenGL) -- stage 3, not yet built
data/        downloaded + packed data (gitignored)
```

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

### Calibration

`--epsp`, the depolarisation contributed per synapse, is the parameter that
matters. Swept against this connectome with DNp01 driven for 100 ms:

| epsp (mV) | network rate | neurons active |
|-----------|--------------|----------------|
| 0.05      | 0.02 Hz      | 0.1%           |
| 0.06      | 0.03 Hz      | 0.1%           |
| **0.08**  | **1.80 Hz**  | **4.5%**       |
| 0.10      | 9.33 Hz      | 12.1%          |
| 0.275     | 37.09 Hz     | 25.7%          |

The transition is sharp. Below ~0.06 an evoked cascade dies out; above ~0.1 the
network runs away and neurons pin at the refractory ceiling (~450 Hz), which is
well outside anything a fly does. 0.08 is the default. Note the dip at 0.085 in
a finer sweep — inhibition biting back is a sign the E/I balance is doing real
work rather than the network simply scaling with gain.

This number is not transferable. It depends on the weight threshold used at
download time, and it would change for a different dataset.

## Validating against known anatomy

Driving the giant fibre and inspecting its direct targets reproduces the escape
circuit:

```sh
./build/Release/flysim --stim-body 10001 --targets 12 --top 0
```

```
rank  bodyId       type          superclass       synapses   spikes
1     800146       TTMn          vnc_motor              70       14
2     800178       IN11A001      vnc_intrinsic          32        9
7     802799       GFC2          vnc_intrinsic          21       14
```

TTMn is the tergotrochanteral motor neuron that fires the jump muscle, and GFC2
the giant-fibre coupled interneurons — which is what the giant fibre is
supposed to drive.

Two things worth knowing about this readout:

- **Rank by spike count is misleading in a recurrent network.** The overall
  top-25 for this run is dominated by AVLP neurons, which are *not* downstream
  of DNp01 — the anterior ventrolateral protocerebrum is densely recurrent, so
  once seeded it sustains itself and drowns out the actual escape pathway.
  `--targets` walks the out-edges instead, so a silent target reads as a zero
  rather than simply being absent.
- **One spike is not enough.** DNp01's strongest connection is 70 synapses, or
  5.6 mV at the default gain, against a 7 mV threshold. TTMn only fires because
  DNp01 is driven repeatedly. That is a real property of the model, and it is
  why coincidence matters here.

Stimulating TTMn itself produces nothing downstream, which is the correct
negative control: motor neurons terminate on muscle, and muscle is not in the
connectome.

See `LifParams` in [LIFNetwork.h](src/core/LIFNetwork.h) for the full parameter
set.

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
