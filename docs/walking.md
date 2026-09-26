# Walking: where it stands

Consolidated after the session that found the linkage burst. `docs/findings.md`
is the running narrative; this is the current state, meant to be read first.

## Headline

The fly now walks for twelve seconds without falling. It never had before.

```
                        before tonight     now
12 s upright                    0 of 5    4 of 5
12 s linkage burst              5 of 5    1 of 5
peak spin, rad/s             2,222,792     3,005
worst pitch, deg                  ~88        13
walking speed, mm/s              6.86      3.89
```

The speed is *lower* than the figure this project used to quote, and that is
not a regression. The old number was measured on a model that was tearing its
own legs off, and the same run ended on its back a second later.

## The three faults fixed, in the order they were found

### 1. The stepping joint was lifting the foot

ThC takes the step. At the rest pose no leg sits anywhere near ThC zero, so
the foot's height depends on ThC to first order. Measured, per radian:

```
leg        dx       dy       dz
front   -0.589   +0.380   -0.492
middle  -0.820   -0.568   +0.443
hind    -0.772   -1.122   +0.903
```

`dz` is opposite in sign front to hind, so every step drove the front feet
down while the hind feet came up: a pitch oscillator at 16.7 Hz. The hind
leg's stepping joint also moved its foot further sideways than forwards.

Cancelled with a CTr offset read off the forward kinematics
(`flyphys --jacobian`, `--level`). Because ThC and CTr are parallel hinges it
cancels the lateral motion too, to three decimals.

### 2. The gait was written in joints, not in feet

`GaitPlan` (`src/body/GaitPlan.{h,cpp}`) now states the gait as a foot path in
millimetres -- straight back along the ground through stance, an arc forward
through swing -- and solves the whole cycle by inverse kinematics before the
run starts. The control loop is a table lookup. Per-leg sign and scale
differences fall out of the geometry instead of being rediscovered.

That exposed a second problem immediately: six rigid legs under a rigid body is
statically indeterminate, so with the feet planned to a fixed height the hind
pair took 95% of the weight and the front legs touched the floor carrying one
per cent each. The old bouncing gait had been spreading the load by accident.
Fixed with a load-sharing trim (`--share`) that raises or lowers each stance
foot until it carries its share -- what campaniform sensilla do.

### 3. The position servo was tuned by the integrator

This was the one that mattered. The servo solved for a relative spin of

```
want = servoRate * error * invDt
```

which at `servoRate` 0.9 and 8 kHz substeps is **7200 rad/s per radian of
error**: erase the whole error inside a fraction of one substep, and demand
more of it the finer the substep gets.

A leg joint never achieves that -- bounded torque cannot move that much
inertia in 125 us. A distal tarsomere has almost none, so it achieves it
exactly, and reached 2.2 million rad/s. The body speed ceiling then fired
624,554 times in four seconds, and a ceiling that rescales one body's velocity
without touching its neighbours breaks the joint the solver has just
satisfied.

The failure signature, printed every frame:

```
  t (ms)   height   pitch   legs down   anchor error
    2163   0.6038    4.46           6         0.0470
    2164   0.6005    3.50           6         1.4211
```

One step. Height, pitch and contacts all still normal. **The fall was the
wreckage, not the event.** Four earlier explanations -- pitch drift, foot
placement, postural gain, heading drift -- were all describing the wreckage,
each argued from a trace sampled ten times in twelve seconds.

Fix: divide by a real time constant, so the servo means the same thing at any
substep rate. `PhysicsWorld::Params::servoTau`, default 1 ms.

```
tau (ms)   12 s upright   bursts
0.1125            0 of 5    5 of 5      <- the old, timestep-derived value
0.6               0 of 5    5 of 5
0.8               1 of 5    3 of 5
1.0               4 of 5    1 of 5
1.5               2 of 5    3 of 5
2.0               0 of 5    5 of 5
```

## Current defaults

```
servoTau           1.0 ms     Physics.h
stride             0.24 mm    either side of rest, so a 0.48 mm stride
period             60 ms      16.7 Hz
lift               0.09 mm
duty               0.55       commanded; ~0.9 measured, see below
levelGain          0.9
shareGain          0.4        clamped to 0.04 mm of trim
```

Measured at those defaults: 8 s, 3 trials -- 3 of 3 upright, 0 of 3 burst,
3.24 mm/s, worst pitch 11.5 deg. 12 s, 5 trials -- 4 of 5 upright, 1 of 5
burst, 3.89 mm/s, worst pitch 13.0 deg.

## Ruled out -- do not re-try these

Each was tested against the linkage burst and made no difference or made it
worse. Listed so the next session does not spend the night on them again.

```
solver iterations 64 / 128 / 256        3 of 3 burst at every value
substep rate 8 / 16 / 24 kHz            3 of 3 burst; finer is worse
posture torque 6e6 / 3e6 / 1.5e6        3 of 3 burst
tarsomere mass floor 12..400 ug         helps peak spin 100x, not survival
tarsal stiffness 1.0 / 0.3 / 0.1        worse
tarsus-specific servo rate 0.9..0.03    no effect on survival
body spin ceiling 400..50,000 rad/s     much worse -- it was protecting us
servo spin cap 300..20,000 rad/s        moves the failure later, does not stop it
Baumgarte clamps on weld + axis bias    no effect on survival (kept anyway)
postural feedback, pgain 0..4           0 of 3 upright at every gain
stride-differential steering            worse at every gain
```

Two of those point at the answer in hindsight. Raising the speed ceiling was
far worse, so the ceiling was protecting against the divergence rather than
causing it. And stride versus failure is **not monotone** -- 0.02 and 0.10
survive four seconds while 0.05, 0.16 and 0.24 do not -- which is a latent
divergence the gait merely triggers, not a dose-response. Surviving runs still
had a tarsomere at 13,000 rad/s.

## Open problems, in the order worth attacking

1. **The divergence is still latent.** Stride 0.24 is stable; 0.32 bursts 3 of
   3. The tau fix raised the threshold, it did not remove the conditioning
   problem. The suspect remains the distal tarsomere -- always link 8, the free
   end of a nine-link chain of near-zero inertia driven by a stiff servo. The
   untested idea is to make the tarsal chain genuinely **passive**: no servo at
   all, a weak spring and real damping, which is closer to a real tarsus
   anyway. A sweep of tarsal stiffness *after* the tau fix was started and not
   finished.

2. **Speed.** 3.89 mm/s against a real fly's 10-20. Stride is capped by (1),
   so (1) is the way in. `--period 45` gave 3.83 at 2 of 3, `--period 80` with
   stride 0.32 gave 4.57 at 2 of 3 -- both marginal.

3. **Duty factor ~0.9** where a walking fly is near 0.5. The legs are barely
   leaving the ground; this is a shuffle. Suspect the tarsus lying flat means a
   leg counts as down while only its proximal tarsomeres touch.

4. **Pitch 13 deg** is still large for a walking animal.

5. **Heading.** Yaw reaches 6 deg in the first half second and 8.7 by 1.5 s,
   then the body crabs at that angle. Nothing measures or corrects heading. The
   naive stride-differential controller made things worse; it needs doing
   properly, or the source of the asymmetry needs finding first, since the
   model is built bilaterally symmetric.

6. **The rest pose still cannot really walk.** `tools/leg_authority.py` says
   the front leg can slide its foot 1.45 mm forward and **0.031 mm backward**
   before a joint hits a stop -- backward is the power stroke. Its ankle sits
   0.057 rad from its limit. `tools/pose_search.py` finds front-leg poses with
   0.54 mm of backward travel, but they need CTr on the opposite side of zero
   from the hind leg, which the global-branch rule in
   `tools/solve_rest_pose.py` forbids. That rule exists so one motor command
   does the same thing to all six legs, and it **does not achieve that**: with
   it enforced, `dz/dCTr` is -0.321 front and +1.036 hind. It constrains the
   sign of an angle when what has to agree is the effect of the joint on the
   foot. Replacing it with the functional test is a real and unfinished piece
   of work -- an attempt to re-solve the pose put the hind femur 61 um under
   the floor and was reverted.

## Tools

```
flyphys --gait                  the gait test; --seconds --trials for duration
        --from A --to B         dense per-frame trace between two times
        --jacobian              what each joint does to the foot, mm/rad
        --foot N                per-segment report for one leg
        --stand-only            quick standing regression
flybody --gait MS --film PREFIX --film-every N --frames M
tools/leg_authority.py          can this pose walk, not just stand
tools/pose_search.py            poses by functional sign, not angle sign
tools/solve_rest_pose.py        the rest pose itself
```

The gait test now reports worst anchor separation and flags a linkage burst
above 0.5 mm. **That number is the health check.** Standing is 0.022 mm; a
healthy walk stays under about 0.06; anything approaching a millimetre means
the legs have come apart and every other number in the run is meaningless.

## Method notes that keep earning their place

- **A passing metric is not evidence.** Every number said the fly was walking
  while it was visibly in pieces (finding 21), and again while its legs were
  being torn off (finding 22). Both were found by looking -- once at a
  rendered frame, once at a per-frame trace instead of ten samples.
- **If a number exists in two places, it is already wrong.** Four times now:
  the anatomy, the abdomen offset, the joint limits, and this session the 3D
  view running its own older gait, so that three sessions of measured walking
  improvements had never once reached the picture the work is judged by.
- **Watch for results that do not move when they certainly should.** A held
  `.exe` makes the link fail silently and the old binary runs. It produced
  byte-identical sweep rows three times tonight. `taskkill //F //IM
  flyphys.exe` before building.
- Tying a controller's gain to `1/dt` makes the simulation a different
  dynamical system at every timestep. It is also why finer substeps made
  walking worse, which had been recorded as evidence that the model was
  chaotic.
