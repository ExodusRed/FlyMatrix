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



1. ~~**Derive muscle strength from motor neuron size.**~~ **Done — see below.**

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



---

## 6. The solver was amplifying every joint-limit impulse 64 times

This one invalidated a good deal of earlier work, so it is worth stating plainly.

The hard joint limit computed its corrective impulse as `(bias - outward)`.
That is correct on the first Gauss-Seidel iteration and wrong on the next 63.
`jointAngle` only changes when positions are integrated, at the *end* of the
substep, so the violation -- and therefore the bias -- is constant across the
whole sweep. Once the first iteration had satisfied the constraint in velocity,
`outward` was zero, and every later iteration re-applied the full impulse. A
joint left the sweep spinning at `iterations * bias`.

### What we measured

```
peak joint rate, giant fibre jump    204,359 rad/s
peak joint rate, DNp09               438,127 rad/s
a real fly's fastest leg joint         a few hundred rad/s
```

204,000 rad/s is 32,000 revolutions per second. At a 125 us substep that is 25
radians of rotation in a single step.

After targeting the velocity instead of adding to it, the same run peaks at
**791 rad/s**.

Every other constraint in the solver already had the idempotent form
`-(relVel + bias)/eff` and was fine. Contacts and friction correctly clamp
accumulated impulse. The limit was the only one that accumulated.

### What it had been hiding

- **The velocity clamp was decorative.** It lived in `integrateVelocities`,
  which runs *before* the joint and contact solvers, so it only ever caught
  velocity produced by gravity -- by far the smallest contributor. Moved to
  just before `integratePositions`. It is now genuinely inert: results are
  bit-identical for `maxAngularVelocity` anywhere from 400 to 100,000.

- **Parameter response was chaotic.** Jump height against `maxMuscleTorque` was
  non-monotonic at every scale, which is why tuning it always produced a narrow
  band and a "tuned success" rather than a calibration. It is now monotonic
  across two decades.

- **The jump was powered by the bug.** With it fixed, the torque that used to
  produce a 5.13 mm jump produces 0.79 mm.

The recalibration that followed finally has a physical anchor instead of a
target height. Units are micrograms, millimetres and seconds, so a torque of 1
is 1e-15 N m. The body masses 1344 ug and weighs 13.2 uN -- a real fly is about
10 uN -- and a 4.6 mm ballistic takeoff needs roughly 300 mm/s of launch
velocity, which is 6 to 7 body weights of leg thrust sustained over the 5 ms of
a real escape.

---

## 7. ThC was hinged about the wrong axis, and it cost us walking

### What the data says

The connectome names the ThC motor neurons:

| direction | muscle |
|-----------|--------|
| protract | Sternal anterior rotator MN |
| protract | Tergopleural/Pleural **promotor** MN |
| retract | Pleural **remotor**/abductor MN |
| retract | Sternal adductor MN |
| retract | Sternal posterior rotator MN |

Promotor and remotor. 62 motor neurons whose job is swinging the leg forward
and backward. That is the step cycle.

### What we had

`FlyBody` hinged ThC about the fore-aft axis, so it abducted the leg sideways.
The model had **no joint that could take a step**, and it was feeding the step
pool a degree of freedom that splays.

### The damage was not confined to ThC

With no fore-aft joint, the only way to reach a foot target ahead of its
attachment was to fold CTr and FTi the opposite way round from a leg reaching
behind. `solve_rest_pose.py` solved each leg independently and scored it on
foot error alone, so nothing asked the three leg types to agree -- and they did
not. The front legs came out in one joint configuration, the middle and hind
legs in the mirror of it.

`MotorPools` applies **one sign per joint to all six legs**. So a single
"extend" command extended some legs and flexed others.

`flyphys` test 3, added to measure this, drives one joint on one leg at a time:

```
before    the six legs disagreed about which way a joint lifts
          on 5 of 10 joint/direction rows
after     1 of 10
```

This is why the nervous system's output looked broken when it was not. Driving
the giant fibre, the CTr drive reaches +8.16 within 5 ms -- strong, correctly
signed, early -- and the fly *sinks*. The escape command was arriving fine and
the body was cancelling it against itself.

### The fix

ThC becomes the lateral-axis swing. Splay becomes a fixed mount rotation, which
is what it is in the animal: the coxae project ventrolaterally as structure,
not posture. The mount is seeded from the ThC rest angles it replaces, so the
standing pose is unchanged.

`solve_rest_pose.py` now picks one joint configuration for the whole animal.
This is not a constraint fought against the geometry:

```
branch CTr +  FTi +   total score 0.8208
branch CTr +  FTi -   0.1165
branch CTr -  FTi +   0.0014   <- chosen, 83x better than the runner-up
branch CTr -  FTi -   1.1706
```

All three foot targets are reached to under a micron. The new ThC rest angles
run -0.841 front, -0.515 middle, -0.061 hind: the front legs reach forward and
the hind legs sit back, from the joint that is supposed to do it.

Two drive signs, CTr and TiTa, were re-measured and both flipped. The old
values had been measured against the pre-fix skeleton *and* the pre-fix solver.

---

## 8. The body can walk

`flyphys --gait` imposes an alternating tripod by hand and measures whether the
body goes anywhere. **This is not connectome-driven walking** -- the rhythm is a
sine wave and no neuron is involved. It answers a question that had to come
first: given a correct gait signal, can this body walk at all? While that was
unknown, any failure of the nervous system was unfalsifiable, because the body
might not have been able to walk however it was driven.

It can:

```
travelled +7.480 mm in 2.0 s (3.74 mm/s)
height 0.620 -> 0.594 mm, worst pitch 9.0 deg, 5-6 feet down
```

Travel is linear to three figures across the whole two seconds -- 0.74 mm per
199 ms, every interval -- so this is steady locomotion and not a fall dressed
up as progress. A real fly walks at 10-25 mm/s, so this is about four times
slow, at a physiological 16.7 Hz step frequency and a 0.3 rad coxa swing.

Two things were needed beyond the ThC fix.

**Position control.** `applyDrive` gives the neural path feed-forward torque and
*reduces* the postural hold in proportion to drive. That is right for a jump and
useless for placing a foot: a large command becomes a hard shove with a weak
servo, and the fly covered ground while tumbling at 75-88 degrees of pitch. The
distinction matters for what comes next -- **the nervous system drives torque,
so a torque-only controller walking is a strictly harder problem than this
result**, and this does not claim that one is solved.

**ThC range.** At +/-0.9 rad the front legs, resting at -0.841, had 0.06 rad of
protraction available against 1.74 of retraction. They could only ever drag
backwards, which is what pitched the body.

Known imperfect: contacts stay at 5-6 where a true tripod alternates 3 and 6,
so the swing legs are not fully clearing the ground. The fly walks, but scuffs.

---

## 9. MDN produces a posture, not a rhythm

MDN -- the moonwalker descending neuron, the best-characterised walking command
cell in the fly -- is in this dataset, four copies. Driving it continuously and
reading the per-leg ThC drive:

```
  t (ms)   spikes   front_L   front_R  middle_L  middle_R    hind_L    hind_R
      10     1579    -1.901    -3.241    +2.124    -0.798    +8.414    +3.927
      50      392    -3.293    -6.084    -1.084    -1.218    +4.697    +1.591
      90      194    -4.583    -5.402    -3.472    -3.111    +2.361    +2.479
     119      645    -5.524    -2.783    -4.074    -3.052    +1.032    +2.752
```

This is a real, specific, segmentally organised motor command: front legs
retract, hind legs protract, sustained without saturating for as long as the
stimulus lasts. The control is clean -- Kenyon cells produce **exactly zero**
drive on all six legs while the fly stands.

It is also completely non-rhythmic. The pattern settles into a fixed posture
with a slow transient and never alternates.

### So we asked the wiring whether a half centre is there

The textbook way an animal makes a rhythm out of tonic drive is two premotor
populations that inhibit each other. `tools/cpg_probe.py` collects the ThC
protractor and retractor motor neurons for one leg, finds their strong
presynaptic partners, splits that premotor layer by which side it favours, and
counts the connections between the two groups by sign.

```
leg         premotor cells    mutual connection, excitatory share
middle_L    371               60%
front_L     423               61%
hind_L      406               58%
network-wide baseline                                61%
```

**Indistinguishable from chance on all three legs.** Reciprocal inhibition
between antagonist premotor pools is not detectable at this resolution.

That is a limit of the method, not evidence that the fly lacks a rhythm
generator, and the distinction matters. The premotor layer is 370-420 cells per
leg, densely interconnected; a functional half centre made of a handful of
identified interneurons would be invisible inside it. Insect CPGs are also
distributed rather than two clean populations, and there is a real possibility
that in flies the rhythm is substantially sensory-driven -- which lands us back
on finding 2, the reflex sign the connectome does not contain.

### Where this leaves walking

The mechanics are solved and the command layer works. What is missing is
specifically a rhythm, and we now know three things about it that we did not:

1. It is not going to fall out of tonic drive through this premotor layer as
   wired, because nothing in the bulk connectivity favours alternation.
2. The body does not need much of one -- 16.7 Hz and a 0.3 rad coxa swing is
   enough.
3. Anything we add to produce it is a modelling choice and has to be labelled
   as one, exactly like the reflex sign in finding 2.


---

## 10. The FeCO can be split by what it drives, and it is not enough

Finding 1 said the sensorimotor loop fails because all 254 chordotonal neurons
are driven with one signal, which excites extension-encoding and
flexion-encoding cells together, and that we could not fix it because the
subtype labels come from axon morphology and are not in this dataset.

The labels turn out not to be what was needed.

What the loop needs is to know which sensory cells belong on opposite sides of
the signal. That is a question about what each cell *does* downstream, and it
is recoverable from connectivity. `tools/feco_split.py` measures each
chordotonal neuron's signed two-hop reach to the FTi flexor motor pool against
its reach to the FTi extensor pool. FTi is the right joint: the femoral
chordotonal organ encodes tibia position, which is the FTi angle.

### The population is sharply bimodal

For the left middle leg, 71 of 87 chordotonal neurons reach either pool within
two hops:

```
bias toward extend (+1) against flex (-1)
  -1.0..-0.9   24 ##############################
  -0.9..-0.5    4 ####
  -0.5..+0.0    3 ###
  +0.0..+0.5    4 ####
  +0.5..+0.9    4 ####
  +0.9..+1.0   32 ########################################

  |bias| > 0.5 : 64  (90%)
  |bias| <= 0.2:  4  (6%)
```

A cell that reaches only one pool scores exactly +/-1 for free, which would
manufacture bimodality out of sparse connectivity, so that was checked
separately. Only 9 cells are in that position. Of the **62 that reach both
pools**, 89% still favour one by a margin and 6% sit near the middle. The
split is real.

Across all six legs, 284 of 403 chordotonal neurons get an assignment: 156
extensor-driving, 128 flexor-driving. Cells with |bias| < 0.2 are deliberately
left unassigned rather than forced to a side, because a cell that drives both
antagonists equally is precisely the confusion the split exists to remove.

### Wired in, it helps substantially and does not fix the loop

The two groups are now driven by opposite phases of tibia movement, from the
FTi angle rather than whole-leg compression. Whole-leg compression could never
have worked: it cannot distinguish a flexed tibia from a flexed femur, so it
cannot tell antagonist afferents apart at all.

```
                loop off    old loop    split loop
giant fibre       7.2044     28.1023        7.7625
Kenyon cells      0.6200     12.2878        3.8834
APL               0.6200      7.6635        3.6407
```

Specificity violation falls by 3.2x, and the giant fibre jump stops being
destroyed. But 3.88 mm is not 0.62 mm: a Kenyon cell still moves the fly.

Sweeping the gain does not rescue it, and the shape of the failure is
informative:

```
gain      GF       KC      APL
  90   7.7625   3.8834   3.6407
  40  10.9711   5.9991   4.9253
  15  13.9431   5.3028   7.8749
   5  21.8265   4.7083   3.0308
   2  12.7311   3.9244   3.4689
```

There is no value that works, and the giant fibre jump gets *larger* as the
sensory gain gets smaller, which is not something a well-behaved feedback loop
does. The loop is still positive feedback through a network with no gain
control: activity produces motor output, motor output moves the body, body
movement produces sensory drive, and nothing anywhere limits the round trip.

So the loop stays off by default, for a reason that has moved one level down.
It is no longer "we cannot tell the afferents apart". We can. What is missing
is whatever normally keeps the loop stable, and the candidates -- presynaptic
inhibition, gain scaling with behavioural state, and the reflex sign of finding
2 that the connectome does not contain -- are all things a wiring diagram plus
a linear neuron model does not supply.

The reflex sign is now an explicit parameter, `SensoryOrgans::Params::
reflexSign`, rather than an accident of the code. +1 is resistance, which is the
standing case; -1 is the assistance reflex that the same anatomy implements
during walking.


---

## 11. An anatomy audit, and what the model was missing

The body was assembled from plausible-looking numbers rather than from the
literature, and three copies of it existed: leg geometry in `FlyBody`, masses
and collision extents in `FlyPhysics`, and what actually gets drawn in
`body_main`. They could disagree, and they did -- the renderer drew the trunk
from a transform the physics never wrote to.

Everything now lives in `src/body/Anatomy.h`, and each block says whether its
numbers are **measured** (published measurements of real flies),
**proportional** (a measured total divided by published ratios), or
**estimated** (nobody measured it for us).

### What was wrong

| part | real fly | had | status |
|------|----------|-----|--------|
| tarsus | **five tarsomeres** ta1-ta5 plus a pretarsus with claws | one rigid rod | fixed |
| wings | two, about as long as the body | none | added |
| halteres | two, the fly's gyroscopes | none | added |
| proboscis | rostrum, haustellum, labellum | none | added |
| antennae | pedicel, funiculus, arista | none | added |
| body length | 2.5-3 mm | 2.04 mm | 2.50 mm |
| coxa/femur/tibia | -- | -- | already about right |

The leg *proportions* turned out to be close: coxa 0.26, femur 0.52, tibia
0.48 needed only small adjustments. The error was the tarsus. A fly's foot is
a jointed chain that drapes over what it stands on, and we had a spike.

### The tarsus, and why it is only half fixed

The five tarsomeres are in, below the TiTa joint. They are **passive**, which
is right: the motor map has one TiTa pool per leg and nothing below it, so
individual tarsomeres have no motor neurons of their own. The TiTa joint drives
ta1 and the rest follow on compliant hinges.

The skeleton still describes the tarsus as one segment, so the rest-pose
solver, the motor map and every `l * kJointCount + j` index keep working, and
`FlyPhysics` splits it during build. 33 bodies became 57.

Two honest caveats.

**The compliance barely acts at the shipped stiffness.** Results are identical
from 1.0 to 20.0, meaning the servo holds the tarsomeres straight and the chain
behaves near-rigidly while walking. Below 1.0 the compliance is real and the
fly cannot walk: at 0.35 the gait tips it to 77 degrees of pitch. So what ships
is an articulated tarsus that mostly acts like a stiff one.

**Only the tip has a contact probe.** A real tarsus lies *along* the ground
rather than touching at a point, so probing every tarsomere is what the anatomy
calls for, and `tarsusProbes` does it. It is off, because the rest pose was
solved to put a single point foot on the floor, which leaves the rest of a
jointed tarsus below it -- switching the probes on pushes the body from 0.55 mm
to 0.94 and leaves one foot down. Making that work needs the rest pose
re-solved against a segmented foot. That is the leg-clipping problem, still
open.

### What the new geometry did to the physics

```
                        before      after
standing height        0.5711 mm   0.5283 mm
standing body pitch    +0.79 deg   +0.01 deg
imposed tripod         3.74 mm/s   11.47 mm/s
giant fibre jump        7.20 mm     4.35 mm
```

The pitch is the satisfying one: with the legs re-solved for their true lengths
the fly stands level rather than nose-up.

The gait tripled and is now inside the real range of 10-25 mm/s, at a 40 ms
period. The sweet spot moved with the geometry -- at the old 60 ms period the
new body does 1.87 mm/s -- which is a reminder that the gait numbers are a
property of body and pattern together, not of the body alone.

The jump had to be recalibrated, and lands better than before: 4.35 mm against
a real escape takeoff of about 4.6 mm ballistic, at `maxMuscleTorque` 3e6.
Specificity is untouched, with Kenyon cells and APL at exactly 0.6200 mm.

### The model's largest physical inaccuracy

The fly masses **1494 ug where a real one is about 1000**, and its legs are
754 ug of that where a real fly's are perhaps 8% of its body.

This is `minSegmentMass`, a solver-stability floor. A huge mass ratio between
neighbouring bodies is what an iterative solver handles worst, so leg segments
are floored well above their true mass. Splitting the tarsus into five took the
leg count from 30 segments to 54 and the floor now dominates the animal.

Lowering it for tarsomeres specifically was tried, on the reasoning that the
ratio argument applies to a coxa hanging off a 330 ug thorax but not between
one tiny tarsomere and the next. The measurement disagreed:

```
floor    total mass    standing height
  4 ug      1302 ug      0.447 mm   FAIL
  8 ug      1398 ug      0.513 mm   FAIL
 10 ug      1446 ug      0.521 mm   pass
 12 ug      1494 ug      0.528 mm   pass
```

It only stands from 10 upward, which saves 48 ug and no margin. The floor
stays, and this is the number to distrust most in the model.

### Sources

- [Appendometer: high-throughput morphometry of Drosophila legs and wings, bioRxiv 2025](https://www.biorxiv.org/content/10.1101/2025.01.21.634122v1.full)
- [An anatomical atlas of Drosophila melanogaster -- the wild-type, Genetics 2024](https://academic.oup.com/genetics/article/228/2/iyae129/7750380)
- [A leg model based on anatomical landmarks for 3D joint kinematics of walking, PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC11233710/)
- [The NeuroMechFly model -- body parts and degrees of freedom](https://nely-epfl.github.io/flygym-gymnasium/neuromechfly.html)
- [Subdivision of the tarsal region into five tarsal segments, Cytologia 2019](https://www.jstage.jst.go.jp/article/cytologia/84/2/84_840202/_html/-char/en)
- [Drosophila melanogaster, Animal Diversity Web](https://animaldiversity.org/accounts/Drosophila_melanogaster/)

### Still simplified

The abdomen is one ellipsoid where a fly has six segments. The head does not
articulate on the neck. The thorax-coxa joint is a single hinge plus a fixed
mount, where the real joint is a ball-and-socket with three degrees of freedom
(yaw, pitch, roll). The proboscis is drawn retracted and cannot extend, so the
fly has mouthparts but still cannot feed. Wings and halteres are drawn and
carry no mass and do not articulate.
