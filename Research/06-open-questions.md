# Open questions and what is already ruled out

Ranked by what is worth attacking next. The ruled-out list at the bottom is
the more valuable half of this file — it is a night's work each time somebody
re-tries one of them.

---

## 1. The solver divergence is still latent

**Status:** raised the threshold, did not remove the cause.

Stride 0.24 is stable (4 of 5 upright, 1 of 5 burst at 12 s). Stride 0.32
bursts 5 of 5 at 12 s, as does 0.42. Even the *surviving* runs have a tarsomere spinning at 13,000
rad/s against 183 rad/s when standing. The failure body is always the same:
**link 8, the most distal tarsomere**, the free end of a nine-link chain of
near-zero inertia driven by a stiff servo.

Everything about stride, speed and duty is capped by this, so it is first.

**The untried idea, and the most promising one:** make the tarsal chain
genuinely **passive**. No servo at all — a weak spring and real damping. A real
tarsus is largely passive, driven through the retractor unguis tendon, so this
is also the more faithful model. Note that softening `tarsusStiffness` was
tried and was worse, but that was *before* the `servoTau` fix and it only
softened the servo rather than removing it. A sweep of tarsal stiffness after
the fix was started and never finished.

**Second idea:** an absolute inertia floor. `setCapsuleInertia` floors the
tensor's *aspect ratio* at 4:1 but there is no absolute floor, and the distal
tarsomere's rotational inertia is ~1/1300 of the thorax's.

**Third idea:** bound the servo impulse by the joint's own effective inertia
rather than by a fixed torque, i.e. `maxImp = min(maxTorque*dt, maxSpin/eff)`.
Light joints would then get proportionally less impulse automatically. A crude
version of this (a global spin cap) only moved the failure later, but it was
applied to the demand rather than to the impulse.

---

## 2. Speed: 3.89 mm/s against a real fly's 10–20

Capped by (1), so (1) is the way in. Both of the near-misses are marginal:

```
stride 0.24 period 45    3.83 mm/s   2 of 3 upright
stride 0.32 period 80    4.57 mm/s   2 of 3 upright
```

Do not read the stride sweeps as a dose-response — they are not monotone.

---

## 3. Duty factor ~0.9 where a walking fly is ~0.5

The legs barely leave the ground; this is a shuffle rather than a tripod.

**Specific suspicion worth checking first:** a leg is counted as "down" if
*any* of its probes touch, and the tarsus lies flat with probes at both ends of
every tarsomere. So a leg whose distal tarsomeres have lifted still counts as
down. The measurement may be wrong before the behaviour is. Check by counting
*loaded* legs (`footLoad(l) > 0`) rather than contacting ones.

If the behaviour is genuinely wrong, three levers were measured against duty
before the solver fix and all three traded it for falling over (more lift, more
toe curl, more tarsal compliance). Worth re-running now that the fall had a
different cause than was assumed.

---

## 4. Heading drift

Yaw reaches 6° in the first half-second and 8.7° by 1.5 s, after which the body
crabs at that angle — which means every stance foot is being dragged across the
ground rather than along it, since the planned foot paths run fore-aft in
*body* coordinates.

A stride-differential steering controller made it worse at every gain. **Find
the source before building another controller.** The model is constructed
bilaterally symmetric — mirrored rest angles, mirrored tripod — so a persistent
yaw offset is either a symmetry-breaking instability (plausible: it saturates
around 9° rather than growing, which looks like a pitchfork bifurcation) or an
actual asymmetry somewhere in the build. Check the latter first: it is cheap.
Compare left and right leg builds element by element.

---

## 5. Pitch 13° is still large

Was 88° (fallen), then 13.9°, now 13.0°. A walking fly's body is much steadier
than that. Probably downstream of (1) and (3); revisit after those.

---

## 6. The rest pose still cannot really walk

The front leg has **0.031 mm of backward travel** — the power stroke —
because its ankle sits 0.057 rad from its limit. Poses with 0.54 mm exist but
need CTr on the opposite side of zero from the hind leg, which the
global-branch rule in `tools/solve_rest_pose.py` forbids.

That rule does not achieve what it exists for: with it enforced, `dz/dCTr` is
−0.321 front and +1.036 hind, so one command already does opposite things to
different legs. Replacing it with a **functional**-sign test is real,
well-defined, unfinished work. `tools/pose_search.py` is the start of it.

Two hazards, both already hit:
- The hind leg is the constrained one — only 10 of 400 random restarts both
  reach the target and clear the floor. Build the rule around it.
- Clearance must be a hard constraint checked against each segment's own
  radius, or the solver buys stride with a buried femur.

---

## 7. The sensorimotor loop is unstable

See [04-connectome.md](04-connectome.md). Splitting the FeCO by downstream
target cut the violation 3.2× but the club subtype still displaces the fly
3.88 mm. Coupled to the body problems above — an unstable plant makes an
unstable loop very hard to diagnose — so it is reasonable to keep prioritising
the body.

---

## 8. The connectome still produces no rhythm

Three mechanisms ruled out (bulk premotor wiring, tonic drive, cellular
adaptation). This is the actual point of the project and it is genuinely open.
It is also open in the field — nobody drives walking from a connectome yet.

---

# Ruled out — do not re-try

Each tested against the linkage burst, each either no effect or worse.

```
solver iterations 64 / 128 / 256         3 of 3 burst at every value
substep rate 8 / 16 / 24 kHz             3 of 3 burst; finer is WORSE
posture torque 6e6 / 3e6 / 1.5e6         3 of 3 burst
tarsomere mass floor 12..400 ug          peak spin 100x better, survival same
tarsal stiffness 1.0 / 0.3 / 0.1         worse (but see item 1 -- pre-fix)
tarsus-specific servo rate 0.9..0.03     no effect on survival
body spin ceiling 400..50,000 rad/s      MUCH worse; it was protecting us
servo spin cap 300..20,000 rad/s         moves failure later, does not stop it
Baumgarte clamps on weld + axis bias     no survival effect (kept anyway)
postural feedback pgain 0 / 0.5 / 1.5 / 4    0 of 3 upright at every gain
stride-differential steering             worse at every gain
Raibert foot placement                   inert or worse
Gauss-Seidel sweep direction alternation ride height 0.43 -> 0.33 mm
```

Two of these point at the answer in hindsight, and that reasoning is worth
copying:

- **Raising the speed ceiling was far worse** (anchor error 36 mm → 1332 mm).
  So the ceiling was *protecting* against the divergence, not causing it —
  which redirected the search from the clamp to whatever it was clamping.
- **Stride versus failure is not monotone.** 0.02 and 0.10 survive where 0.05
  and 0.16 do not. A real physical parameter does not behave like that; a
  latent divergence with a stochastic trigger does.
