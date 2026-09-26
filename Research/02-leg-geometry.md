# Leg geometry and the rest pose

`src/body/Anatomy.h`, `src/body/FlyBody.cpp`, `tools/solve_rest_pose.py`,
`tools/leg_authority.py`, `tools/pose_search.py`.

---

## The chain

Five joints per leg, proximal to distal, matching the `joint` column of
`data/bin/motor_map.tsv`:

```
ThC    thorax-coxa        swings the whole leg fore and aft -- the step
CTr    coxa-trochanter    lifts and lowers
TrF    trochanter-femur   rotates the femur about its own axis
FTi    femur-tibia        the knee
TiTa   tibia-tarsus       the ankle
```

Plus the tarsus, which is **not** a point foot: five tarsomeres
(fractions 0.40 / 0.17 / 0.14 / 0.12 / 0.17 of tarsus length) that lie *along*
the ground. Modelling it as a point foot put the rest of a jointed tarsus
below the floor and was a source of years of confusion.

Body axes are **+X forward, +Y left, +Z up**. Left and right legs are mirror
images through XZ, so axes lying in the mirror plane flip and axes along Y do
not: joints hinging about the lateral axis (ThC, CTr, FTi, TiTa) share one axis
across both sides, while the leg mount and TrF get multiplied by side. Getting
this backwards leaves the two sides in visibly different poses.

The leg has a **fixed outward mount tilt** (`splay`) applied before any joint
angle, because a fly's coxae project ventrolaterally as a matter of structure.
Modelling that as a joint angle cost ThC its only degree of freedom.

---

## The foot Jacobian: measure it, never assume it

This is the most important single idea in this file. **Joint names describe
what a muscle does at a neutral pose the fly is nowhere near.** Run
`flyphys --jacobian`. Millimetres of foot movement per radian, at the pose the
animal actually stands in:

```
leg        joint      dx       dy       dz      |d|
front_L    ThC    -0.589   +0.380   -0.492    0.857
front_L    CTr    -0.514   +0.248   -0.321    0.655
front_L    TrF    +0.000   -0.012   -0.009    0.015
front_L    FTi    -0.087   +0.031   -0.042    0.102
front_L    TiTa   -0.003   +0.293   -0.385    0.484
middle_L   ThC    -0.820   -0.568   +0.443    1.092
middle_L   CTr    -0.701   -0.750   +0.584    1.181
middle_L   FTi    -0.078   -0.828   +0.641    1.050
hind_L     ThC    -0.772   -1.122   +0.903    1.634
hind_L     CTr    -0.573   -1.287   +1.036    1.749
hind_L     FTi    -0.054   -0.902   +0.752    1.176
```

Read the consequences off that table:

- **`dz` flips sign front to hind.** One ThC command sent to all six legs
  drives the front feet *into* the floor while the hind feet come *off* it.
  That is a pitch oscillator at the step frequency, and it is exactly what the
  body was visibly doing — 8° peak to peak, 16.7 times a second.
- **The hind leg's "stepping" joint moves its foot further sideways than
  forwards** (dy −1.12 vs dx −0.77). Half the step is lateral scrubbing.
- **The front leg's knee is nearly useless** for moving the foot: |d| = 0.102
  against 1.18 for the middle leg's CTr.
- **ThC, CTr, FTi are near-parallel hinges** — their dz/dy ratios agree to
  three decimals (−0.780, −0.779, −0.774 on the middle leg). Each leg is
  therefore effectively a **planar** manipulator. TrF is the only joint that
  moves the foot out of that plane.

That last point has a useful corollary: cancelling a ThC-induced height change
with CTr *automatically* cancels the lateral motion too, because the two
vectors are parallel. That is why the height correction in
[03-gait-control.md](03-gait-control.md) works as well as it does.

**How to diagnose the sign of anything: perturb the joint, run forward
kinematics, look at the foot.** Never reason from the muscle's name. This
project set a whole table of drive signs from a diagnostic that was actually
measuring the fly taking off (findings §17).

---

## Standing is not walking

`tools/leg_authority.py` asks a question nobody had asked: given this rest
pose, how far can each foot slide **along the ground** — not lifting, not
swinging sideways — before some joint hits a stop?

For the pose currently shipped:

```
leg       forward   backward   worst joint margin
front     1.450 mm    0.031 mm       0.057 rad  (TiTa)
middle    1.575 mm    1.214 mm       0.508 rad
hind      1.309 mm    0.875 mm       0.436 rad
```

**Backward is the power stroke.** The front leg has thirty-one micrometres of
it, because its ankle sits 0.057 rad from its limit. It cannot push the animal
along. Apply the height correction and its net foot travel actually *reverses*:
+0.199 mm/rad forward while the other four go −0.28 mm/rad back. The front legs
fight the other four on every stride.

A rest pose needs to be scored on **travel available in both directions**, not
only on getting six feet onto the ground. `solve_rest_pose.py` now includes a
`stride_travel()` term for this.

---

## Two traps in the rest-pose solver

**Segment radius.** The ground-clearance test compared joint heights against a
flat 15 µm. The femur is 36 µm thick, so a femur whose axis sits at 20 µm is
16 µm *through* the floor and passed the test. Clearance must be checked per
segment against that segment's own radius. Related: the hind leg was once
solved with its femur 61 µm underground, and the physics then stood the animal
on its knees — hind legs carrying 95% of the weight while the tarsi hung 90 µm
in the air.

**Clearance must be a hard constraint.** When it was one weighted term among
several, a stride-authority term simply outbid it and bought a longer stride
with a buried femur.

---

## The global-branch rule is wrong and still in place

`solve_rest_pose.py` forces all six legs onto the same side of zero for CTr and
FTi. The stated reason is that motor pools apply one sign per joint to all six
legs, so one "extend" command must extend every leg.

**It does not achieve that.** With the rule enforced:

```
dz/dCTr    front -0.321    middle +0.585    hind +1.036
```

Raising CTr lowers the front foot and raises the other four. The rule
constrains the **sign of an angle**, which is a coordinate convention, when
what has to agree is the **effect of the joint on the foot**, which is
geometry.

It is also expensive. `tools/pose_search.py` samples poses per leg and sorts
them by functional sign rather than angle sign, and finds front-leg poses with
**0.54 mm of backward travel instead of 0.031** — seventeen times more — that
clear the floor by 280 µm. They need CTr on the opposite side of zero from the
hind leg, which the rule forbids.

Of 400 random restarts per leg: front 230 poses reach the target and clear the
floor, middle 119, **hind only 10**. The hind leg is the genuinely constrained
one and it sits essentially on the clearance boundary. Any replacement rule
has to be built around what the hind leg can do.

This is unfinished work. An attempt to re-solve the whole pose with the new
scoring produced the buried-femur pose above and was reverted.

---

## Anatomy provenance

`src/body/Anatomy.h` is the single source of truth for every dimension, with
each value marked *measured*, *proportional* or *estimated*. **Joint limits
live there too** (`kJointLimit`), and `solve_rest_pose.py` parses them out of
the header rather than keeping its own copy.

That parsing exists because the two copies had drifted on four joints out of
five, so the solver produced poses the physics rejected on the first step —
every leg's knee started outside its limit and the fly stood 0.37 mm too high
on four feet. See [05-method.md](05-method.md); this has now happened four
times in this project.
