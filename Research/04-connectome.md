# The connectome side

`src/core/`, `src/body/MotorPools.*`, `src/body/SensoryOrgans.*`,
`tools/cpg_probe.py`, `tools/feco_split.py`.

This summarises the nervous-system work; the full working is in
`docs/findings.md` sections 1–5, 9, 10 and 15, and the section numbers are
given so you can go back to the primary record.

---

## The headline, stated plainly

**Nothing in this project has ever walked from the connectome.** Every walking
figure anywhere in this repository comes from a hand-written alternating
tripod. That caveat is printed by `flyphys --gait` itself, and it should stay
printed.

This is not a failure of the implementation. As of the work in findings §4,
**nobody drives walking from a connectome** — the field does not have a
worked example. The connectome gives connectivity and sign, not dynamics.

---

## What the connectome does not contain

**The sign of a reflex is not in the wiring** (§2). Knowing that neuron A
contacts neuron B, and with what weight, does not tell you whether the loop
that results is negative feedback or positive. Getting this wrong is not a
subtle error — it inverts the behaviour.

One thing that *is* recoverable and matters: in *Drosophila*, **glutamate is
inhibitory** at the relevant synapses (GluCl), not excitatory as vertebrate
intuition suggests. A model that treats it the vertebrate way will produce a
nervous system that cannot stop itself.

**Tonic drive is not in the wiring** (§3). Posture in a real fly is held by
the continuous firing of slow motor neurons. A controller needs a baseline
output to modulate; a network at rest produces zero, and zero is not a
posture. The body model currently substitutes `postureTorque` — an engineering
servo standing in for those neurons — and as long as that servo is stiffer than
the muscles it is the servo doing the walking, not the model. See
[03-gait-control.md](03-gait-control.md).

---

## Three mechanisms tried for rhythm, none produced a gait

The central question is where a stepping rhythm could come from.

1. **Bulk premotor wiring.** Probed for half-centre structure — mutually
   inhibitory pairs that could oscillate — with `tools/cpg_probe.py`. Did not
   yield a rhythm.
2. **Tonic drive.** Adding a baseline output to modulate (findings §5, acting
   on §3) changed the posture but did not produce oscillation.
3. **Cellular adaptation.** Spike-frequency adaptation is the classic way to
   make a half-centre alternate. Implemented in the LIF model; findings §15
   records that it **does not make a gait** here.

**MDN produces a posture, not a rhythm** (§9). MDN is the descending command
neuron associated with backward walking, and stimulating it moves the model
into a configuration rather than starting an oscillation — consistent with
descending neurons selecting behaviour rather than generating the step cycle.

The honest reading: the step rhythm in a real fly is thought to arise from
local VNC circuitry combined with sensory feedback, and neither the local
dynamics nor the feedback loop are recoverable from connectivity alone.

---

## Sensory feedback: the FeCO

The femoral chordotonal organ is the leg's main proprioceptor, and it is where
the sensorimotor loop would close.

**It has five functional subtypes and the model drove all of them
identically** (§1) — claw, hook and club subtypes signal different things
(position, movement direction, vibration), and collapsing them loses the
information the loop needs.

**Splitting it by what it drives helps and is not enough** (§10).
`tools/feco_split.py` separates the FeCO by downstream target. That cut the
measured violation by 3.2×, but the loop remained unstable — the club subtype
still displaced the fly by 3.88 mm.

The sensorimotor loop is, as of now, **not stable**. This is the main open
problem on the nervous-system side and it is coupled to the body side: an
unstable plant makes an unstable loop much harder to diagnose, which is part of
why the body work was prioritised.

---

## Practical notes

**Data.** Janelia serves `male-cns:v1.0` over neuPrint's Cypher endpoint with
no authentication. `tools/fetch_cns.py` (resumable, ~5 min, ~300 MB) then
`tools/pack_cns.py`. `--min-weight 5` gives a lighter download keeping ~72% of
synaptic mass.

**Do not edit the data to make results come out.** Stated as policy: the
target is a working implementation, not adjusted inputs. Where the model
disagrees with the animal, that is a finding.

**Derived maps** live in `data/bin/` — `motor_map.tsv` is the motor-neuron to
joint mapping, whose `joint` column is the authority for the joint names used
throughout the body code.

**Naming is evidence.** The ThC motor neurons are named "Tergopleural/Pleural
promotor MN", "Pleural remotor/abductor MN", "Sternal anterior rotator MN" and
"Sternal posterior rotator MN" — 62 motor neurons whose job is swinging the leg
fore and aft. The body model originally hinged ThC as an abduction axis, which
left it with **no joint that could take a step** (§7). The connectome's own
naming caught that.

But see [02-leg-geometry.md](02-leg-geometry.md) for the limit of this: a
muscle's name tells you its anatomical action at a neutral pose, not what the
joint does to the foot at the pose the animal stands in. Those turn out to
differ enough to invert signs.
