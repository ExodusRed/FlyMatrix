# What the literature says about the problems we hit

Notes from reading around the two things that blocked this project: the
proprioceptive loop making the fly worse rather than better, and the nervous
system producing no tonic drive to hold posture.

Each section separates **what the literature says** from **what we measured in
our own data**, because in two cases they agree and in one they do not.

---

## 1. The FeCO has five subtypes, and we drive all of them identically

### Literature

The femoral chordotonal organ is about 150 cholinergic sensory neurons per leg,
in five functionally distinct subtypes ([Mamiya et al. 2018][mamiya],
[Chen et al. 2025][divergent]):

| subtype | encodes |
|---------|---------|
| claw, extension-encoding | tibia **position** |
| claw, flexion-encoding | tibia **position** |
| hook, extension-encoding | tibia **movement**, directional |
| hook, flexion-encoding | tibia **movement**, directional |
| club | **bidirectional movement and vibration** (<1 µm, high frequency) |

They are named for axon shape, and that is how they were sorted in the
connectome — "based on axon morphology and comparison with light microscopy
images" ([Chen et al. 2025][divergent]). Not from connectivity, and not from
any annotation field.

Their connectivity differs sharply. For claw and hook, "the majority of
synapses ... are onto local VNC interneurons and leg motor neurons". For club,
"more than half of all synapses ... are onto intersegmental neurons", and they
project to brain auditory regions. **Claw and hook are proprioceptive; club is
exteroceptive.** In one reconstruction of ~50% of a front-left FeCO: 8 claw
extension, 13 claw flexion, 9 hook extension, 13 hook flexion, 35 club — club
is the single largest group.

### What we measured

Our `SensoryOrgans` drives all 254 chordotonal neurons with one signal: leg
compression. That conflates five subtypes into one, and in particular drives
extension-encoding and flexion-encoding neurons with the *same* input, which
means exciting antagonist pathways simultaneously. This is the most likely
reason the loop produces incoherent motor output.

We cannot fix it from this dataset. The subtype labels come from axon
morphology, which `male-cns:v1.0` does not give us for these neurons, and their
type names (`SNpp50`, `SNpp60`, …) carry no functional information.

The connectivity signature is the only handle available, and we tested whether
our existing purity filter already exploits it. It does, but weakly:

```
kept by the purity filter (254 neurons)   17.2% of output to ascending neurons
rejected                  (149 neurons)   27.5%
```

Club neurons should be above 50%. Neither group is, so the filter enriches for
claw/hook without separating them, and the set we drive is still contaminated
with vibration-sensing neurons being fed a position signal.

---

## 2. The sign of the reflex is not in the connectome

This is the most important thing we found, and it reframes the whole problem.

A subset of FeCO neurons sense tibia extension and excite flexor motor neurons
— a resistance reflex that opposes the movement and stabilises the joint. But
**during walking that reflex reverses**, so that the same sensory input promotes
the cyclic flexion and extension walking requires
([Wikipedia summary][feco-wiki], and reflex reversal is long established in
stick insect and locust work — [Bässler][bassler], [Skorupski &
Sillar][skorupski]).

Reflex reversal has been reported in *Drosophila* directly: tibia extension
usually excites flexor motor neurons, but when the fly is actively moving the
same extension can produce an *inhibitory* response.

The consequence for us is blunt. **The same anatomy implements a stabilising
reflex or a destabilising one depending on behavioural state, and that state is
not in the connectome.** A wiring diagram plus a linear neuron model cannot
tell you which sign is in force. We were not merely missing a parameter; we
were missing the variable that selects between two opposite controllers.

This is the sharpest form of the difficulty: **the connectome underdetermines
the controller.**

---

## 3. Posture is held by tonically firing slow motor neurons

### Literature

*Drosophila* leg motor neurons follow a size principle ([Azevedo et al.
2020][size]). Slow motor neurons are recruited first, then intermediate, then
fast. Crucially for us: **the resting spike rate of slow motor neurons
maintains constant force and is used to maintain posture**, while fast motor
neurons are transiently recruited for movement and are not sustained.

Force per spike spans roughly three orders of magnitude between the extremes:
slow motor neurons produce under 0.1 µN per spike, fast ones about 10 µN.

### What we measured

This agrees with our data, and it is the one place where our hand-tuning turns
out to have been approximately right for the wrong reason.

Sizes of the 334 mapped leg motor neurons:

```
p5    377,412,195 voxels
p50 1,566,399,266
p95 4,094,458,520        p95/p5 = 11x

largest   14,551,198,550   TTMn                 CTr
          10,156,209,474   TTMn                 CTr
           6,769,864,809   Sternotrochanter MN  CTr
smallest     207,656,386   Fe reductor MN       TrF
```

**The two largest leg motor neurons in the entire dataset are both TTMn** — the
jump muscle — and the smallest are a small postural muscle. Largest to smallest
is about 70x, the right order for the ~100x force range the literature reports.

Our `MotorPools` assigns muscle strength from a hand-written table (TTM 22,
postural 1, ratio 22x). That is recoverable from measured neuron size instead,
which would be principled rather than tuned, and would extend to muscles we
never named.

It also explains a failure. We currently hold posture with an engineering
crutch — `postureTorque`, a servo that collapses the fly below about 2e6 — and
the literature says what it stands in for: tonic firing of slow motor neurons.
Our LIF network is silent at rest, so it produces no such drive. **A controller
needs a baseline output to modulate, and ours is zero.**

---

## 4. Nobody drives walking from a connectome yet

NeuroMechFly v2 ([Wang-Chen et al., Nature Methods 2024][nmf2]) is the state of
the art for an embodied *Drosophila*. It is worth being precise about what it
does with connectome data: it uses a **connectome-constrained visual network**
(T1–T5, Tm, TmY cell types) for object detection, and builds its locomotion
controllers from biologically inspired designs plus reinforcement learning.

So the connectome drives vision there, not walking. Our earlier statement to
this effect holds up: connectome-driven locomotion is an open problem, not an
engineering task someone has already completed.

---

## What this changes

Ordered by what the evidence now supports, which is not the order we guessed
before doing the reading:

1. ~~**Derive muscle strength from motor neuron size.**~~ **Done � see below.**
   It works, and it is not uniformly an improvement.
2. **Give the network tonic drive.** Until slow motor neurons fire at rest
   there is no baseline to modulate and `postureTorque` is doing the nervous
   system's job. Neuromodulation is the biologically honest source and we
   currently set its gain to zero.
3. **Stop driving all chordotonal neurons alike.** Even without subtype labels,
   splitting extension- from flexion-encoding drive would stop us exciting
   antagonists together. How to split them without morphology is unsolved.
4. **Accept that the reflex sign is a free variable.** It is not recoverable
   from the connectome. Any closed loop we build has to choose it, and that
   choice should be explicit and labelled as an assumption rather than buried.

---

## Sources

- [Mamiya, Gurung & Tuthill, *Neural coding of leg proprioception in Drosophila*, Neuron 2018][mamiya]
- [Chen et al., *Divergent neural circuits for proprioceptive and exteroceptive sensing of the Drosophila leg*, Nature Communications 2025][divergent] ([preprint][divergent-pre])
- [Azevedo et al., *A size principle for recruitment of Drosophila leg motor neurons*, eLife 2020][size]
- [Wang-Chen et al., *NeuroMechFly v2: simulating embodied sensorimotor control in adult Drosophila*, Nature Methods 2024][nmf2]
- [Femoral chordotonal organ — overview and reflex reversal][feco-wiki]
- [Bässler, *Effects of afference sign reversal on motor activity in walking stick insects*, J Exp Biol 1981][bassler]
- [Skorupski & Sillar, *Reflex reversal in the walking systems of mammals and arthropods*][skorupski]

[mamiya]: https://pmc.ncbi.nlm.nih.gov/articles/PMC6481666/
[divergent]: https://www.nature.com/articles/s41467-025-59302-3
[divergent-pre]: https://www.biorxiv.org/content/10.1101/2024.04.23.590808v2.full
[size]: https://elifesciences.org/articles/56754
[nmf2]: https://www.nature.com/articles/s41592-024-02497-y
[feco-wiki]: https://en.wikipedia.org/wiki/Femoral_chordotonal_organ
[bassler]: https://journals.biologists.com/jeb/article/91/1/179/22868/Effects-of-Afference-Sign-Reversal-on-Motor
[skorupski]: https://link.springer.com/chapter/10.1007/978-1-4615-1985-0_18

---

## 5. What happened when we acted on finding 3

Muscle strength now comes from the summed segmentation volume of a muscle's
motor neurons, in units of the median single leg motor neuron, replacing a
hand-written table. The result is instructive and only partly good.

### The jump got closer to the real animal

    giant fibre jump   3.75 mm  ->  4.96 mm      (real takeoff ~4.6 mm ballistic)

and the non-motor controls stayed perfectly silent at 0.62 mm: MBON01, Kenyon
cells, APL. Specificity survived the change intact.

### But it redistributed force, and not only for the better

| neuron | hand table | from size |
|--------|-----------:|----------:|
| DNp01, giant fibre | 3.75 mm | **4.96 mm** |
| DNp03 | 3.19 mm | 0.62 mm |
| DNp04 | 0.93 mm | 0.62 mm |
| DNp09 | 24.8 mm | **63.0 mm** |

DNp09 was already wrong and is now much more wrong. Two descending neurons
stopped moving the fly at all.

The mechanism is visible in the strength distributions:

```
hand table   median 1.00   max 22.0     one special muscle over a flat floor
from size    median 3.61   max 11.6     many comparably strong muscles
```

The table I wrote encoded a belief -- that the jump muscle towers over
everything else. The data says the fly has many substantial leg muscles and
the tergotrochanteral one is not an outlier in *force*. Per neuron TTMn is the
largest cell in the dataset, but a muscle's force is the sum over its motor
units, and the tibia flexor has many. What makes the jump special is speed and
the catapult mechanism, neither of which is force.

Raising the floor 3.6x is why broad activation got worse. DNp09 reaching 63 mm
is the same failure the README already records under supraphysiological drive,
amplified: nothing in this model prevents every muscle contracting maximally
at once, and now every muscle is strong.

### Kept anyway

The measured version ships, because the alternative is a table of numbers I
invented. Replacing invention with measurement is worth a worse DNp09, and it
makes the next problem legible rather than hidden: the model needs something
that stops simultaneous maximal activation of everything. In a real animal
that is reciprocal inhibition and recruitment order -- the size principle
again, which says small units fire first and large ones only under strong
drive. Our muscles have no recruitment order at all: every motor unit in a
muscle shares one activation.

One free parameter remains, `maxMuscleTorque`, which converts segmentation
volume into torque. Nothing measures that conversion. Its usable window is
narrow and not monotonic -- 4e5 and 6e5 both diverge sooner than the 5e5 we
ship -- which is a real weakness and not a tuned success.
