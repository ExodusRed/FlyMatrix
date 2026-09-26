# The rigid-body solver

`src/engine/Physics.{h,cpp}`. Maximal coordinates, sequential impulses,
Gauss-Seidel, warm starting, split impulse for position correction. A fly leg
is a nine-link chain (five leg segments plus four tarsomeres) whose distal
bodies are a thousand times lighter than its proximal ones, which is a far
harder conditioning problem than a typical game scene and is the source of
most of what follows.

Most of this file is generic. If you are writing any constraint solver, items
1, 2 and 3 are worth knowing.

---

## 1. Never tie a controller's gain to the timestep

**This was the single most damaging bug in the project.** The position servo
solved for a relative joint spin of

```cpp
want = servoRate * (target - angle) * invDt;
```

At `servoRate` 0.9 and 8 kHz substeps that is **7200 rad/s per radian of
error**. It asks for the entire error to be erased inside a fraction of one
substep, and it asks for *more* the finer the substep becomes.

A leg joint never achieves it — a bounded torque cannot accelerate that much
inertia in 125 µs. A distal tarsomere has almost no inertia, so it achieves it
exactly, and was measured at **2.2 million rad/s**.

The fix is to divide by a real time constant, so the controller means the same
thing at any substep rate:

```cpp
const float rate = params.servoTau > 0.0f ? 1.0f / params.servoTau
                                          : j.servoRate * invDt;
```

`servoTau` is 1 ms. The response to it is sharply peaked — this is not a knob
where more is better:

```
tau (ms)   12 s upright   bursts
0.1125            0 of 5    5 of 5   <- the old, timestep-derived value
0.8               1 of 5    3 of 5
1.0               4 of 5    1 of 5
1.5               2 of 5    3 of 5
2.0               0 of 5    5 of 5
```

Below ~0.8 ms it diverges; above ~1.5 ms the servo is too soft to carry a
stride and the fly sags.

**The wider lesson.** A gain containing `1/dt` makes the simulation a
different dynamical system at every timestep. It also means *refining the
timestep makes the physics worse*, which is the opposite of the usual
intuition and had been recorded in this project as evidence that the model was
chaotic. It was not chaos. It was `invDt` in a gain.

---

## 2. A post-solve velocity clamp tears linkages apart

`clampVelocities()` rescales any body exceeding a speed ceiling, and it runs
*after* the constraint solve. That placement is deliberate and documented —
put before the solve, it only ever caught velocity from gravity, and
constraint impulses went unbounded.

But a clamp that scales **one body's** velocity without touching its
neighbours breaks whatever joint the solver has just satisfied. During walking
it fired **624,554 times in four seconds** — roughly 40% of all body-substeps.
Each firing prised the anchors apart a little more.

The failure looks like this, printed every frame:

```
  t (ms)   height   pitch   legs down   anchor error
    2163   0.6038    4.46           6         0.0470
    2164   0.6005    3.50           6         1.4211
```

The linkage separates by 1.4 mm in **one step**, while height, pitch and
contact count are all still perfectly normal. The body is thrown afterwards.

**Do not "fix" this by raising the ceiling.** That was tested: 400 → 50,000
rad/s took the worst anchor error from 36 mm to 1332 mm. The ceiling is
protecting against a genuine divergence; the divergence itself is item 1.

The right diagnostic is to count clamp firings. Standing fires it **zero**
times with a peak spin of 183 rad/s. If it is firing during normal operation,
something upstream is wrong.

---

## 3. Clamp every Baumgarte bias, not just the one you remember

Position error is fed back as a velocity bias, `baumgarte * invDt * error`,
which at 8 kHz is a factor of ~5600. A quarter radian of misalignment asks for
1400 rad/s.

Only the joint-limit constraint clamped this. The weld constraint and the
hinge parallel-axis constraint applied it **unclamped, directly to real
angular velocity**, and the contact push did too. They are all clamped to
`maxCorrectionVelocity` now.

This did not on its own fix the burst, but an unbounded energy injection
proportional to `1/dt` is not something to leave sitting in a solver.

Note the asymmetry worth copying: the *linear* anchor correction already went
into pseudo-velocities (split impulse), which are clamped globally before
integration. Position correction that becomes real momentum is the thing to
avoid — this project once had the fly climb to 300 mm at a constant 940 mm/s
with no ground contact, purely on accumulated correction velocity.

---

## 4. Idempotence: a constraint swept 64 times must be safe to sweep 64 times

Recorded here because it is the same class of error as item 1 and was found
earlier (findings §6). The joint-limit constraint read:

```cpp
const float lambda = (bias - outward) / eff;     // WRONG
```

`bias` is constant across the whole Gauss-Seidel sweep, because `jointAngle`
only changes when positions are integrated at the end of the substep. So once
the first iteration satisfied the constraint, `outward` was zero and every
subsequent iteration re-applied the **full** impulse. The joint left the sweep
spinning at `iterations * bias`. Measured peak joint rates reached 204,000
rad/s where a real fly's fastest leg joint does a few hundred.

The correct form targets the velocity rather than adding to it:

```cpp
const float lambda = -(relVel + bias) / eff;     // idempotent
```

**Rule: write every constraint so that applying it to an already-satisfied
state is a no-op.**

---

## 5. Conditioning: tiny distal bodies are where solvers die

Every divergence in this project traced to the same body — link 8, the most
distal tarsomere, the free end of a nine-link chain.

```
tarsomere mass      12 µg
tarsomere length    ~0.06 mm
rotational inertia  ~4e-3 µg·mm²   (~1/1300 of the thorax)
```

With `maxTorque` 6e6 and dt 125 µs, one substep of full servo torque gives it
**190,000 rad/s**. The torque was sized for the leg, not for the thing it
drives.

Things that did **not** fix it (all measured):

- more iterations (64 / 128 / 256)
- finer substeps (8 / 16 / 24 kHz) — *made it worse*, see item 1
- raising the tarsomere mass floor (12 → 400 µg) — cut peak spin 100× but did
  not improve survival
- softening the tarsal joints (`tarsusStiffness` 1.0 / 0.3 / 0.1) — worse
- a tarsus-specific servo rate — no effect

`setCapsuleInertia` already floors the tensor's aspect ratio at 4:1, because a
thin segment's axial moment is a hundredth of its transverse one and that one
tiny principal moment otherwise sets the timestep for the whole simulation.
There is still **no absolute inertia floor**, and adding one is untested.

The untried idea, and the most promising: make the tarsal chain genuinely
**passive** — no servo at all, a weak spring and real damping. A real tarsus is
largely passive, driven through the retractor unguis tendon. See
[06-open-questions.md](06-open-questions.md).

---

## 6. Damping that does nothing

`params.angularDamping` is 0.04, applied as `1/(1 + 0.04*dt)`. With
dt = 125 µs that factor is `1/(1 + 5e-6)` — utterly negligible. The global
damping in this solver is, in practice, **off**. Anything relying on it for
stability is relying on nothing.

Per-joint damping (`j.damping`, 0.6) is applied inside the servo term and is
therefore bounded by the servo's impulse clamp. It is real but it only acts
where the servo acts.

---

## 7. Numbers worth keeping

```
substep rate            8 kHz
solver iterations       64        (a 5-link leg needs far more than a game
                                   scene; at 24 the legs behaved as if
                                   compressible and the fly sank 0.1 mm)
baumgarte               0.7
maxAngularVelocity      400 rad/s
maxLinearVelocity       3000 mm/s
maxCorrectionVelocity   150 mm/s
servoTau                1 ms
postureTorque           6e6       (= 6 nN·m)
```

Alternating the Gauss-Seidel sweep direction — the textbook trick for chains —
was tried and made things **consistently worse** (ride height 0.43 → 0.33 mm at
24 iterations). Warm starting already carries the previous solution and
reversing appears to fight it. The sweep is left in build order deliberately.

Caching the world-space inverse inertia was a 1.94× speedup: `R I R^T` depends
only on orientation, and orientation does not change while the solver
iterates.
