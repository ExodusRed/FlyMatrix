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


---

## 12. Why the fly was slow, and it was not what I expected

Walking had reached 11.47 mm/s against a real *Drosophila*'s 10-25 mm/s, and
the obvious question was what was holding it at the bottom of the range. Three
hypotheses, measured in order, and the first two were wrong.

### It is not cadence

```
ours        stride 0.459 mm at 25.0 Hz
real fly    stride ~1.3 mm  at ~15 Hz
```

We step **faster** than a real fly and still travel more slowly. Speed is
stride times frequency, and ours is short-stride, high-cadence -- which is the
signature of scuffing rather than walking.

### It is not grip

The natural guess was slip. A real fly has adhesive tarsal pads and claws on
the pretarsus and we model a point foot with Coulomb friction, so it seemed
obvious. Sweeping friction over a fiftyfold range:

```
friction   0.4    0.9    2.0    5.0   20.0
stride    0.344  0.420  0.347  0.395  0.426
```

Nothing. Whatever the foot is doing, it is not sliding for want of friction.
Recorded because the adhesion story is plausible enough to be worth not
re-deriving.

### It was the swing leg never leaving the ground

Measuring the foot's fore-aft travel relative to the body, and how much of it
happens while the foot is carrying load:

```
foot sweep 1.192 mm relative to body, of which 1.189 mm loaded
stride 0.420 mm, duty 0.77
```

The foot sweeps 1.19 mm and is loaded for **1.19 mm of it**. The leg is on the
ground through essentially the whole cycle, so the return stroke pushes the
body backwards nearly as hard as the power stroke pushes it forwards, and the
1.19 mm sweep nets 0.42 mm of travel. Stride efficiency 35%.

A real fly walking at speed has a duty factor near 0.5: three legs down, three
in the air. Ours was 0.77.

### Why it could not lift its feet

Commanding a bigger foot lift did produce a proper duty factor -- 0.46 at a
lift of 0.9 rad, which is biologically right -- and the fly fell over, pitching
83 to 88 degrees. So the constraint was not the command. It was that the body
could not stay upright on three legs.

That traces straight back to the mass problem recorded in finding 11. The legs
massed 754 ug against a 740 ug trunk, where a real fly's legs are perhaps 8% of
it. Half the animal's mass was in its legs, and swinging that around is what
tipped it over. The `minSegmentMass` floor -- a solver-stability number, not an
anatomical one -- was setting the walking speed.

### What fixed it

Lowering the floor from 12 ug to 8 ug for the proximal segments, which became
possible only once the rest pose was re-solved for the corrected leg lengths:
at 12 ug the same value had failed to stand.

```
                       before     after
stride efficiency        35%       70%
duty factor             0.77      0.69
walking speed         11.47      14.94 mm/s
standing pitch        +0.89      +0.09 deg
```

Squarely inside the real range now, and four times the 3.74 mm/s this started
at.

One counter-intuitive detail worth keeping: the **tarsomeres need to stay
heavier than the segments above them**. Dropping all floors to 8 ug makes the
fly sag and fail to stand; keeping the tarsomeres at 12 while the rest go to 8
is what works. They sit at the end of the longest lever and the solver appears
to need the mass there.

### Why a real fly is fast

Worth stating alongside, because our numbers only mean something against the
animal's.

A fly is quick for reasons of scale before anything else. Muscle force goes
with cross-sectional area and mass with volume, so force per unit mass goes as
1/length: a 1 mg animal accelerates on a scale a large one cannot. Rotational
inertia falls faster still, which is why a fly can turn in a wingbeat or two.
Its whole nervous system is under a millimetre across, so conduction delays are
tens of microseconds where ours as vertebrates are tens of milliseconds.

For specific speeds it has dedicated machinery. The escape is a giant-fibre
pathway with the fewest synapses evolution could manage, taking about 5 ms from
looming stimulus to takeoff. Flight runs on asynchronous fibrillar muscle that
oscillates mechanically at around 200 Hz, far above any rate its motor neurons
fire at -- the muscle is stretch-activated and the nervous system only gates it.
And the jump uses a catapult: energy stored in cuticle and released faster than
muscle alone could deliver it.

None of that is what limits our walking. Ours is limited by the mass
distribution of the model and by a rhythm we do not yet have.

### How fast the simulation itself runs

A separate sense of slow, and worth having a number for:

```
mechanics only        0.03x real time
with the connectome   0.025x real time
```

About 33x slower than the animal it models. 64 solver iterations at an 8 kHz
substep across 57 bodies, plus 176,422 LIF neurons at 10 kHz. Neither has been
optimised at all -- there is no spatial partitioning, no SIMD, no threading,
and the substep rate was set by what kept the solver stable rather than by what
it needs.


---

## 13. The legs stopped clipping through the floor, and walking got worse

The reported defect was legs passing through the ground. The cause was not a
contact bug: **only the foot tip had a contact probe at all**. Nothing else on
the leg was ever tested against the floor, so a tibia or a mid-tarsus could
pass straight through it because nothing was asking.

Finding 11 added the five tarsomeres and left this half done, because probing
each of them pushed the body from 0.55 mm to 0.94 and left one foot down. The
reason was the rest pose: it had been solved to put a *point* foot on the
floor, which leaves the rest of a jointed tarsus below it.

### Laying the tarsus flat

A real fly does not stand on the tip of its tarsus. The tarsus lies along the
substrate -- that is what five tarsomeres and adhesive pads are for.

`solve_rest_pose.py` now solves for the **ankle** rather than the foot. Only
the first four joints move the ankle, so the search is a 3x4 problem and TiTa
drops out of it entirely; TiTa is then set analytically to whatever lays the
tarsus horizontal, which is one equation in one unknown.

Two attempts failed before it worked, and both were the same mistake in
different places. Forcing every tarsus to point forward jammed the hind leg
against its TiTa limit and left its tarsus 77 um off the floor. Letting each
tarsus continue in the direction its own leg is already heading fixed the hind
leg and jammed the front one instead, at 201 um. Widening TiTa from +/-1.2 to
+/-2.3 and pulling the front stance in from x = 0.72 to 0.62 got all three:

```
front   ankle (+0.619, +0.700, +0.019)   tarsus tip (+1.103, +0.694, +0.019)
middle  ankle (-0.150, +0.920, +0.020)   tarsus tip (-0.698, +0.960, +0.020)
hind    ankle (-1.121, +0.881, +0.019)   tarsus tip (-1.736, +0.897, +0.019)

tarsus rise: 0.0 um on all three
```

The front tarsus points forward and the hind one trails back, which is what a
real fly's do. The branch search still picks one joint configuration for the
whole animal, and still prefers it by a wide margin: 0.0035 against 0.1028.

### Standing is now right

```
              before        after
height       0.5215 mm     0.5501 mm
contacts      6             22
feet down     6             6
body pitch   +0.09 deg     -0.09 deg
```

22 contacts because the tarsomeres are resting on the ground along their
length, as they should. The legs no longer pass through the floor.

A stale measurement had to be fixed to see this: the harness counted a leg as
down only if the probe at its very tip was touching, and with the tarsus flat
a leg commonly rests on its middle tarsomeres. It reported a correctly
standing fly as having four feet down. Feet are now counted per leg.

### And walking regressed

```
              point foot    flat tarsus
speed         17.04 mm/s     8.37 mm/s
pitch         11.0 deg      18.6 deg
duty          0.69          0.65
```

This is a real trade, not a tuning artefact, and it is worth being clear that
the anatomically correct model walks worse.

A flat foot cannot be planted and lifted straight up the way a point can.
Lifting at the ankle alone leaves the far end dragging, so the gait needed a
third joint: TiTa now curls during swing to bring the tarsus up with the leg.
That works, and it works dramatically -- with the curl the fly reaches

```
stride 1.204 mm, duty 0.48, speed 26.76 mm/s
```

which is real *Drosophila* territory on every count; a real fly's stride is
about 1.3 mm at a duty factor near 0.5. It also pitches to 87 degrees and
falls over. Across the sweep, every setting that produced a biological stride
tumbled, and every setting that stayed upright produced a short one. The
shipped default is the quickest gait that keeps pitch under 20 degrees.

### What that is actually telling us

The flat foot did not make the mechanics worse. It made them more realistic
and therefore more demanding, and it exposed the limit of what an open-loop
sine wave can do.

A real fly does not walk by replaying a fixed pattern. It is continuously
correcting, and the faster it goes the more correction it needs. Our gait has
no feedback at all: the same six sinusoids play whether the body is level or
rolling over. With a point foot that was survivable because the foot could not
catch on anything. With a tarsus lying along the ground it is not.

So the stride is there and the stability is not, and closing that gap is not a
matter of better sine waves. It wants a controller -- which is what the
nervous system in this project is supposed to eventually be.

### Note on the jump

The giant fibre now peaks at 9.96 mm where the same `maxMuscleTorque` gave
4.43 before the pose change. The sampled table in `FlyPhysics.h` was measured
against the previous rest pose and should be read as illustrating the chaos
rather than as current values. It is further evidence for finding 12's
conclusion: the jump height is not a reproducible quantity in this model, and
anything that perturbs timing moves it.


---

## 14. Postural feedback: the mechanism works, the tuning does not converge

Finding 13 ended on a clear diagnosis. Every gait setting that produced a
biological stride fell over and every setting that stayed upright produced a
short one, so the fly is not short of stride but short of correction. The
obvious next move was to give it some.

`flyphys --gait` now takes two feedback gains, both off by default. The rule is
the simplest thing that could work: if the body is pitching nose-up, extend the
front legs less and the hind legs more; roll does the same across left and
right.

### Proportional feedback works, and costs everything it gains

Applied to the fast-but-tumbling setting (period 45, swing 0.2, lift 0.3, toe
curl 0.3), which open-loop reaches 26.76 mm/s at 87 degrees of pitch:

```
gain     speed   stride   duty   pitch
0        26.76    1.204   0.48   86.9
0.5      -4.33   -0.195   0.58   34.9
1.5       4.78    0.215   0.67   18.7
3.0      -1.20   -0.054   0.49   83.6
6.0      16.33    0.735   0.24   85.1
```

At a gain of 1.5 the pitch falls from 87 degrees to 18.7, which is a real
effect and says the mechanism is sound. It also drops the speed from 26.76 to
4.78, which is worse than doing nothing. And the response is not monotonic in
the gain: 3.0 is worse than 1.5 and 6.0 is worse again.

One sign error found on the way, worth recording because it is easy to make
twice. CTr negative *extends* the leg and raises the body at that corner
(flyphys test 2 reports CTr -15 as LIFTS), so correcting a nose-up pitch means
making the front legs' CTr more positive, not less. Backwards, the controller
drove the fly at -48 mm/s with a duty factor of 0.13.

### The haltere term did not rescue it

A proportional controller with no damping fights an error only once the error
exists, so the natural addition is a rate term -- and that is also the
biologically correct signal. A haltere is a gyroscope. It reports the body's
angular velocity, not its angle, which is exactly the derivative term a
proportional controller is missing.

It did not help. At rate gains large enough to matter the fly is flung rather
than walked (95 mm/s at a duty factor of 0.17 and 89 degrees of pitch, which is
a tumble with forward momentum, not locomotion), and at gains small enough to
stay upright it settles at 4 to 7 mm/s and 21 to 35 degrees -- no better than
the 8.37 mm/s at 18.6 degrees that the open-loop gait already manages.

```
rate      speed   stride   duty   pitch
0.0002     6.57    0.296   0.60   31.4
0.0005     4.20    0.189   0.57   34.7
0.001      6.62    0.298   0.66   20.9
0.002      5.89    0.265   0.61   34.6
```

### Why this stopped here rather than continuing to sweep

Two gains against a response surface already known to be chaotic will always
produce a best point, and it will not mean anything. This project has the
evidence for that already: caching the world inertia, a change that alters no
physics at all, moved the gait's worst pitch from 12.8 degrees to 43.5, and
sampling `maxMuscleTorque` finely enough dissolved what had looked like a
smooth plateau into noise.

Any gain pair picked from these tables would be a number that a recompile could
undo. So the controller ships off, with the tables above as the record of what
it does, and the honest position is that this needs either a principled design
-- deriving the gains from the body's inertia and step period rather than
searching for them -- or an evaluation that scores across many conditions
rather than one two-second run.

What is not in doubt is that the mechanism is the right one and the model needs
it. An open-loop pattern cannot walk this body quickly and stay upright, and no
amount of better sine waves will change that.


---

## 15. Spike-frequency adaptation does not make a gait

Finding 9 established that MDN produces a sustained posture and no rhythm, and
that no reciprocal inhibition is detectable between the ThC antagonist premotor
pools. The obvious remaining candidate was a mechanism already in the model:
spike-frequency adaptation.

It is the standard way a population makes a rhythm out of tonic drive without a
half centre. Neurons fatigue, the population falls silent, the fatigue decays,
it fires again. Our LIF neurons have it -- each spike raises that neuron's own
threshold by `adaptIncrement`, decaying with `tauAdapt` -- so this was a
question about parameter values rather than about adding machinery.

### Measuring rhythm instead of eyeballing it

`flybody --probe-joint` now reports, per leg, the mean drive, its full swing,
and a frequency taken from how often the signal crosses its own mean with a 5%
deadband. A tonic signal crosses almost never; an oscillation crosses twice per
cycle.

At the shipped adaptation, driving MDN:

```
leg              mean      swing         Hz
front_L        -3.755      5.784      21.87
front_R        -3.885      7.513      10.00
middle_L       -2.903      8.725       5.62
middle_R       -0.083     10.706       6.87
hind_L          1.382      9.890       4.37
hind_R          4.556      8.484       7.50
```

The drive is not flat -- swings of 5.8 to 10.7 are large against means of the
same order. But every leg does something different, between 4.4 and 21.9 Hz,
and a gait is six legs at **one** frequency in fixed phase.

### No setting synchronises them

Sweeping adaptation strength and time constant, and reporting the mean
frequency across the six legs with its spread:

```
adapt  tau      meanHz     sdHz    swing
0.4    150        9.37     5.85     8.52
0.4    60         7.60     2.76     8.77
0.4    30         6.45     2.97     9.20
1.5    150        8.23     5.36     8.25
1.5    60        10.31     8.63     8.02
1.5    30        13.12     9.90     9.02
4.0    150        9.16     5.91     5.81
4.0    60        13.33     9.72     6.37
4.0    30        16.14     7.35     6.46
```

The spread is comparable to the mean everywhere. The best case, 0.4 mV with a
60 ms constant, still has a standard deviation more than a third of the mean.
Stronger and faster adaptation raises the frequency and makes the spread worse,
not better.

**Adaptation alone does not produce a coordinated gait in this network.** It
makes the output fluctuate faster; it never makes the six legs agree.

That is what should have been expected on reflection, and it agrees with
finding 9 from the other direction. Adaptation is a property of individual
cells and can make a population burst. Coordinating six limbs in fixed phase
needs *coupling* between them, and the premotor connectivity measurement found
no sign of the coupling a half centre would need. Two independent lines of
evidence, one from the wiring and one from the dynamics, land in the same
place.

### A caveat on the metric

Crossing rate cannot fully separate an oscillation from a noisy tonic signal,
since a noisy signal also crosses its mean. The deadband suppresses small
noise but not large. So the frequencies above should be read as "how fast the
drive fluctuates", not as established oscillation.

It does not change the conclusion. Whether the fluctuation is rhythm or noise,
it is not synchronised across legs, and an unsynchronised signal is not a gait
either way.

### Where the rhythm has to come from, then

Three candidates are now ruled out or measured as insufficient: the bulk
premotor connectivity has no detectable half centre (finding 9), tonic drive
through it produces posture (finding 9), and cellular adaptation produces
incoherent fluctuation (here).

What is left is coupling that the connectome does contain but our model does
not use well -- the intersegmental interneurons that finding 1 noted club
chordotonal neurons project to heavily -- or sensory feedback, which in insects
is substantially what coordinates stepping, and which this model cannot yet run
stably (finding 10). Both routes lead back to the sensorimotor loop.


---

## 16. Correction: the walking figures were single trajectories

**This section retracts a headline result reported in sections 8, 11, 12 and
13.** Every walking speed quoted there came from one two-second run from one
exact starting state, and one run is not a measurement of a chaotically
sensitive system.

The warning signs were all present and were written down without the
conclusion being drawn. Three separate changes that altered no physics
whatever each moved the headline figure by a factor of two or more:

- caching the world inertia moved the worst pitch from 12.8 to 43.5 degrees
- splitting `solveJoints` into passes moved the speed from 8.37 to 6.62 mm/s
- re-solving the rest pose moved it from 5.65 to 1.87 mm/s

Each time the response was to re-sweep and pick a new default. The right
response was to stop trusting a single run.

### What repeating it shows

`flyphys --gait` now runs several trials from starting heights displaced by a
few micrometres and reports the spread. At the best settings found, with a
perturbation of +/-8 um against a ride height of 550 um -- about 1.5% --

```
  jitter      speed     stride     duty    pitch
  -0.008      -1.27     -0.038     0.48     61.8
  -0.004       3.08      0.093     0.63     34.7
  +0.000       4.82      0.145     0.66     16.3
  +0.004       5.51      0.165     0.63     25.0
  +0.008       3.70      0.111     0.66     14.5

median speed 3.70 mm/s (range -1.27 to 5.51)
median pitch 25.0 deg (range 14.5 to 61.8)
upright in 2 of 5 trials
```

The fly sometimes walks backwards. It falls over more often than not.

Across every gait setting previously reported as good, none keeps the body
upright in a majority of trials:

```
period  swing  lift  toe   median speed   median pitch   upright
45      0.2    0.5   0.4   5.88 mm/s      71.5 deg       1/5
45      0.2    0.3   0.0   3.26           34.2           0/5
30      0.3    0.7   0.0   1.94           37.1           0/5
60      0.2    0.3   0.2   7.05           37.8           0/5
30      0.2    0.5   0.2   3.70           25.0           2/5
```

### What is and is not true

**True:** the mechanics permit walking. Trajectories exist in which the body
travels 16 mm in two seconds and stays upright, and they are not flukes of a
broken solver -- they are real solutions of a body that now has correct leg
proportions, a jointed tarsus lying flat on the ground, and six legs that
agree about which way a joint lifts.

**Not true, and previously claimed:** that the body *can walk* in any useful
sense. It does not walk repeatably. The imposed tripod produces locomotion
from some starting states and a fall from most.

The earlier sections are left as written rather than silently edited, because
the sequence of reasoning is the useful part and quietly correcting numbers
would hide how the error survived so long.

### Why this strengthens rather than overturns finding 14

Finding 14 concluded that an open-loop pattern cannot walk this body and that
it wants a controller. That conclusion was right and is now much better
supported: a gait that falls over under a 1.5% perturbation is not marginally
stable, it is unstable, and no choice of sine-wave parameters fixes it.

It also explains why the postural controller in finding 14 looked like it was
failing. It was being scored on single runs of a process whose single runs
mean nothing. Whether feedback helps is still open -- the question simply has
not been asked properly yet, and now it can be.

### The harness change

`flyphys --gait` defaults to five trials and reports the median with its
range, and its verdict is "not a repeatable gait" unless the body stays
upright in a majority. A single trial is still available as the last argument
and prints the familiar detailed trace, but it now says "one trial only --
run several" rather than "the body can walk when driven correctly".

The default test currently fails. That is correct: the thing it tests does not
yet work.


---

## 17. The diagnostic that the sign tables come from was measuring a launch

Tests 2, 3 and 4 probe a joint by driving it and watching what the body does.
The drive was +/-15 throughout, chosen when `maxMuscleTorque` was 5e5. It is
now 2.7e6, which makes +/-15 about ten body weights on a single joint.

The tests stopped measuring which way a joint lifts and started measuring how
far it throws the animal. Height changes of 3.4, 10.3 and 19.8 mm on a fly
that stands 0.55 mm off the ground are not lifts.

Worse, it inverted the answers:

```
drive +/-15        CTr -15 all sink,  CTr +15 LEGS DISAGREE
drive +/-3         CTr -3  all lift,  CTr +3  all sink
```

At +/-3, roughly two body weights on the joint, every leg agrees and the sign
is the opposite of what +/-15 reported. `kJointDriveSign` and the postural
controller both read their signs off this test, so both were being set from a
fly being thrown into the air.

The default became 3, with `--probe-drive` to change it -- and 3 turned out to
be wrong too. Section 18 measures the usable window and settles on 1.5. Anyone
altering `maxMuscleTorque` should re-check that this still lands where the
response is a lift rather than a launch.

One consolation: the postural controller of finding 14 had the right sign
after all. Its failure, re-measured across trials in finding 16, is real and
not a sign error.

This also cost an hour to find because a held .exe meant `--probe-drive` was
silently doing nothing across three separate sweeps, all of which returned
identical numbers. The project has been caught by stale binaries before; the
tell is results that do not move when they certainly should.


---

## 18. The rest pose had no room to move, and the probe had too much

Two faults in the same place, found by chasing why the joint diagnostic gave
answers that contradicted each other.

### The rest pose sat on its limits

Joint limits were symmetric -- CTr +/-1.6, FTi +/-2.6, TiTa +/-2.3 -- and the
flat-tarsus rest pose puts the front leg at CTr -1.538, FTi +2.129, TiTa
-2.219. That leaves:

```
CTr    0.062 rad of travel in one direction
FTi    0.471
TiTa   0.081
```

Three joints of five effectively jammed against a stop. The rest-pose solver
printed a limit margin of 0.062 rad and it was read and not acted on.

The symptom was a diagnostic that made no sense: at a drive of +/-3 both
directions of a joint were reported as lifting, because one direction could
only push against its own stop. Limits are asymmetric now, as a real leg's are
-- a knee has far more flexion than hyperextension -- and sized so the rest
pose sits inside them with room either way.

It did not fix walking. Repeatability stayed at two upright trials in five, so
the jamming was real and was not what makes the gait fall over.

### The probe was launching the fly

Tests 2, 3 and 4 drove a joint at +/-15, a figure chosen when
`maxMuscleTorque` was 5e5. At 2.7e6 that is around ten body weights on one
joint. The tests now report whether the fly left the ground during a probe,
and at +/-3 **five of ten rows in test 2 are flight paths**, not lifts.

Sweeping the drive shows how narrow the usable window is:

```
drive 1.0    most joints do not move at all
drive 1.5    nothing takes off, every joint moves,
             nine of ten per-leg verdicts agree
drive 3.0    half of test 2 is flight, four of ten legs disagree
```

1.5 is now the default. The window is narrow because joint drive is not
graded: below a threshold the posture servo simply holds, and above it the
joint slams to its stop. That is a property of the muscle model worth
remembering, not an artefact of the test.

### And the sign it was supposed to settle is still unsettled

`kJointDriveSign[CTr]` was measured against the pre-flat-tarsus pose. At the
usable drive the two tests disagree about it: test 2, driving all six legs
together, says CTr +1.5 lifts, which argues for +1; test 3, driving one leg at
a time, says both directions sink on every leg, which argues for neither.

Flipping it was tried. The giant fibre jump moves from 7.37 mm to 6.99, a
difference far inside the chaotic spread of that measurement, and Kenyon cells
and APL sit at exactly 0.6200 mm either way. There is no evidence to justify
the change, so it is left as measured and the ambiguity is recorded in the
table rather than resolved by preference.


---

## 19. The fly was not falling over. It was launching itself.

Finding 16 established that the gait was not repeatable and finding 14 that it
wanted a controller. Three controllers were tried and every one made it worse,
which is a strong enough signal to stop building controllers and look at what
actually happens.

### The failure is a takeoff

Tracing a single failing trial, with contacts and height:

```
  t (ms)      x (mm)   height   contacts
    1393       5.500    0.531      7
    1592       3.746    0.676      0     <- airborne
    1791      -5.976    6.143      0     <- 6.1 mm up
    1990      19.205    0.718     19     <- lands 19 mm further on
```

The body leaves the ground entirely and reaches 6.1 mm, eleven times its ride
height, on a fly whose legs are driven by nothing but a sine wave. It is not
losing its balance. **The gait is pumping energy into the body until it
catapults.**

That explains the three failed controllers at a stroke. Every one of them
worked by adding more motion to the joint targets -- extending a leg further,
placing a foot differently -- and more target motion is more energy into a
system that was already overfilled. They were not failing to stabilise; they
were feeding the thing that throws the fly.

### Why the joints pump

The joints are position-servoed with a large torque budget, and the gait moves
their targets at 33 Hz. A servo whose target jumps while its foot is planted
does not gently reposition the leg -- it shoves against the ground. Finding 18
had already noted that joint drive is not graded: below a threshold the servo
holds, above it the joint slams to its stop. At gait frequencies that is a
pump.

A real muscle is not a position servo. It is compliant, and it dissipates.

### What works

Slowing the step frequency, which gives the servo time to reach its target
before the target moves again:

```
period    median speed   median pitch   upright
 30 ms      2.82 mm/s      28.3 deg       2/5
 60         4.85           30.1           2/5
100         6.24           77.1           0/5
160         1.67           17.3           5/9
190         4.42           34.0           2/9
```

Confirmed as a basin rather than a spike, at nine trials each: 130 ms gives
5/9 and 160 ms gives 5/9, with 190 falling away to 2/9.

**This is the first configuration in the project that walks in a majority of
trials.** The default is now 160 ms.

Speed cannot be bought back through stride, either: at that period a swing of
0.35 rad gives 0 of 9 upright and 0.5 rad likewise. Amplitude pumps energy the
same way frequency does.

### What it costs, stated plainly

```
median speed    1.67 mm/s   (a real fly walks at 10-25)
median pitch    17.3 deg
step frequency  6.25 Hz     (a real fly steps at 10-20)
upright         5 of 9 trials
```

So: about a sixth of a real fly's speed, at half its step frequency, staying
upright a little over half the time. That is a long way from an animal, and it
is the first honest "it walks" this project has been able to write.

The verdict line reports the fraction now -- "walks in a majority of trials
(5 of 9 upright)" -- rather than rounding a bare majority up to a capability.

### Where the ceiling actually is

The ceiling is not control and not the gait pattern. It is that
`applyDrive` gives the model a stiff position servo where an animal has a
compliant, dissipative actuator, and a stiff servo driven cyclically is a
pump. Everything else follows from that: why fast stepping launches the fly,
why bigger strides launch it, why adding corrective motion makes it worse, and
why the usable probe window in finding 18 is so narrow.

The Hill muscle model added in an earlier session is exactly the right idea and
is currently bypassed on the walking path, which uses `useManualTarget` and the
posture servo directly. Giving the gait a force-based, force-velocity-damped
actuator instead of a position target is the obvious next thing to try, and it
is a change to the muscle model rather than to the controller.


---

## 20. Force-driven muscles instead of a position servo: equivalent, and the reason why matters

Finding 19 concluded that the ceiling on walking was the actuator -- a stiff
position servo can only add energy, and at gait frequencies that pumps the
body into the air, whereas a muscle's force-velocity relation dissipates. The
Hill model had been sitting in `FlyPhysics` for three sessions with the
walking path bypassing it.

So the walking path was given a force mode: the gait writes muscle drives
instead of joint angles, through `applyDrive`, with the Hill force-length and
force-velocity factors active.

### The damping is real and it is not enough

Within force mode, turning Hill off costs reliability, so the damping does
what it is supposed to:

```
                      median speed   upright
Hill on  (vmax 12)      2.64 mm/s      2/5
Hill off                1.16           1/5
```

Stronger damping helps further, and a sweep of the shortening rate lands on
0.6 rad/s. But measured properly against the position servo, at nine trials
each:

```
                 median speed   median pitch   upright
force + Hill       1.99 mm/s      22.9 deg       5/9
position servo     1.67           17.3           5/9
```

**Identical reliability.** Force mode is marginally quicker and marginally
less steady. There is no evidence to prefer either, so the default does not
move and force mode ships behind `--force`, available and documented.

### Why the actuator change did not break through

Because the actuator was not the whole story. `applyDrive` gives a joint a
feed-forward muscle torque *and* leaves the posture servo holding the rest
angle underneath it, at `postureTorque` 6e6 against a muscle that peaks
around 2.7e6. The servo is stiffer than the muscle, so even in "force mode"
the servo is doing most of the walking.

The obvious next move was to weaken the servo and let the muscles carry the
body, which needs a tonic baseline -- exactly what finding 3 identified as
missing three sessions ago: *a controller needs a baseline output to modulate,
and ours is zero.* A `--tonic` term was added for that.

It does not work:

```
posture   tonic   median speed   upright
6e6       0        1.99 mm/s      4/5
6e6       1.5     -0.42           1/5
6e6       3       -4.72           1/5
2e6       0       -0.62           0/5
2e6       1.5     -0.60           0/5
2e6       3        0.04           0/5
```

Weakening the servo collapses the fly whatever the tonic drive, and adding
tonic drive at full servo stiffness makes things worse rather than better.

The reason is straightforward once stated. A constant extensor drive pushes;
it does not *restore*. Posture needs a force that grows with deviation, and
that is precisely what the servo supplies and a tonic activation does not. In
a real fly the restoring term comes from proprioceptive feedback modulating
those tonic motor neurons -- the loop this project cannot yet run stably
(finding 10).

### What this actually says

The ceiling was correctly identified as an implementation artefact rather than
physics, and it was still wrongly located. It is not the position servo as
such. It is that **posture is held by an engineering servo because there is no
working proprioceptive loop to hold it**, and any change to the actuator runs
into that fact.

That unifies three findings that had looked separate. Finding 3: the network
produces no tonic drive, so `postureTorque` stands in for slow motor neurons.
Finding 10: the sensory loop is unstable and switched off. Finding 19: the
servo pumps energy and caps walking speed. They are one problem seen from
three sides, and the sensorimotor loop is the thing at the bottom of it.

A hand-built controller cannot substitute, either: findings 14 and 19 show
every added correction term feeding the pump. The restoring force has to come
from something that measures the body's state, and in this animal that is the
campaniform sensilla and the chordotonal organs.
