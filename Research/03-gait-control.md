# The gait controller

`src/body/GaitPlan.{h,cpp}`, driven from `src/phys_test.cpp` (the harness) and
`src/body_main.cpp` (the 3D view). Both use the same plan — see the warning at
the end.

**None of this is neural.** It is a hand-written alternating tripod. Every
figure produced by it describes the *body* walking, not the connectome walking
it.

---

## Design: state the gait in feet, not in joints

Every earlier version drove one joint per effect — ThC a sine because its
motor neurons are called promotor and remotor, CTr a half-wave because that is
the joint the jump muscle acts on. [02-leg-geometry.md](02-leg-geometry.md)
shows why that cannot work: the same command produces different, sometimes
opposite, foot motion on each leg pair.

`GaitPlan` instead states the gait as a **foot path in millimetres**:

```
stance (u < duty):   foot slides straight back along the ground at rest height
swing  (u > duty):   foot arcs forward and up, lift * sin(pi * w)
```

and solves the whole cycle by damped-least-squares inverse kinematics
**before the run starts**, 256 samples per leg. The control loop is a table
lookup. Per-leg sign and scale differences fall out of the geometry instead of
being rediscovered by hand.

Two implementation details that matter:

- **Warm start each sample from the previous one.** Five joints reaching a 3D
  point is redundant; without continuity, consecutive samples pick unrelated
  postures and the tabulated path becomes discontinuous, which the physics then
  absorbs as an impulse.
- **Pull toward the rest pose in the null space** to settle the two redundant
  degrees of freedom, so the leg does not wander into odd configurations.

`GaitPlan::basis(leg, axis)` exposes the joint change that moves a foot one
millimetre along a body axis. That is what lets a controller say *"carry less
load, so drop 20 µm"* instead of *"add 0.14 rad of CTr, except on the front
legs where it is TiTa and the sign is the other way"*.

---

## Load sharing

Planning the feet to a fixed height immediately exposed a problem the old
bouncing gait had been hiding: **six rigid legs under a rigid body is
statically indeterminate.** Nothing in the geometry decides how the weight
divides, so whichever feet are commanded lowest take all of it. Measured, the
hind pair carried 95% of the animal while the front legs touched the floor
bearing one per cent each.

A real fly does not leave this to chance — campaniform sensilla report leg load
continuously. The controller does the same thing: each stance leg raises its
foot slightly if it carries more than its share, lowers it if less.

```
shareGain    0.4 mm/s per unit of relative load error
shareClamp   0.04 mm of trim
```

Worth 7.93 → 9.02 mm/s and duty 0.95 → 0.88 when it was added.

---

## Height levelling (superseded, kept for the joint-space path)

Before `GaitPlan` there was a narrower fix for the same problem: a CTr offset
that cancels the foot lift ThC introduces, tabulated exactly by bisection on
the forward kinematics. `--level`, default 0.9.

```
level   speed   pitch   upright
0.0      6.86    13.9    9 of 9
0.6      7.61    11.1    9 of 9
0.9      7.90    10.2    9 of 9
1.0      7.11    10.2    8 of 9
```

Note it peaks at 0.9, not 1.0. The exact table is within 6% of the linear
estimate so that is not the linearisation running out — a foot held at
*precisely* constant height has no vertical give, and the last tenth of the
correction is what lets a stance leg absorb a bad step instead of levering
against it.

Still reachable with `--joint-gait`. Useful as a control.

---

## Current tuning

```
period      60 ms      16.7 Hz
stride      0.24 mm    either side of rest, so a 0.48 mm stride
lift        0.09 mm
duty        0.55       commanded
shareGain   0.4
levelGain   0.9        (joint-space path only)
```

Measured at the defaults:

```
 8 s, 3 trials    3 of 3 upright, 0 of 3 burst, 3.24 mm/s, worst pitch 11.5
12 s, 5 trials    4 of 5 upright, 1 of 5 burst, 3.89 mm/s, worst pitch 13.0
```

The stability envelope is **narrow and not monotone**:

```
stride 0.24 period 60    3 of 3 upright, 0 of 3 burst    3.24 mm/s
stride 0.32 period 60    0 of 3 upright, 3 of 3 burst
stride 0.42 period 60    0 of 3 upright, 3 of 3 burst
stride 0.24 period 45    2 of 3 upright, 1 of 3 burst    3.83 mm/s
stride 0.32 period 80    2 of 3 upright, 1 of 3 burst    4.57 mm/s
```

And at 4 s, strides of 0.02 and 0.10 survive while 0.05 and 0.16 do not.
**That is not a dose-response.** It is a latent solver divergence that the gait
merely decides when to trigger — see [01-solver-physics.md](01-solver-physics.md)
item 5. Do not read these sweeps as if stride were a physical parameter with a
monotone effect.

---

## Controllers that did not work

Recorded so they are not rebuilt from scratch.

**Postural feedback** (`--pgain`, `--prate`): extend the front legs less and
the hind more when pitching nose-up, roll likewise. The rate term is the
biologically correct signal — a haltere is a gyroscope and reports angular
*velocity*, not angle. Tested at gains 0, 0.5, 1.5 and 4.0 over 12 s: **0 of 3
upright at every gain**, and high gain actively worse (−67 mm/s, −58° pitch).
It was aimed at a pitch drift that turned out not to be the failure mode.

**Foot placement / Raibert stepping** (`--step`): shift the swing target by
`k * forward velocity`, so a body carrying momentum puts its next foot further
ahead. Applied to every leg at every phase it was steadily worse; restricted to
the swing leg it is still inert at the current speeds. A planted foot cannot be
placed anywhere, so applying it in stance just drags the body along the ground.

**Stride-differential steering** (`--yaw`, `--yawrate`): shorten the stride on
one side in proportion to heading error and yaw rate, which is how a fly
actually turns. Worse at every gain tried. The heading problem it targets is
real — yaw reaches 6° in the first half-second and 8.7° by 1.5 s, after which
the body crabs at that angle — but the model is built bilaterally symmetric, so
the source of the asymmetry should be found before a controller is bolted on.

---

## Warning: keep the harness and the viewer on the same controller

`flybody` had its own older two-joint sinusoid — no height correction, no foot
planning, no load sharing. **Three sessions of measured walking improvements
had never once reached the 3D view**, which is the thing the work is actually
judged by. Both now call the same `driveGait()` over the same `GaitPlan`.

If you add a controller, add it in one place and make both callers use it.
This is the fourth copy-divergence in the project; see
[05-method.md](05-method.md).
