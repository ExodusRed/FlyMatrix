# A motor architecture that resembles the animal

A design proposal, not yet built. The case for changing approach, and what the
replacement should look like.

---

## Why change approach

The current gait is **a clock**. `phase = 2*pi*t/period`, legs assigned to two
tripods, joint targets read from a table. A real fly has nothing like this, and
the differences are not cosmetic:

| The model | The animal |
|---|---|
| Stance ends when the clock says so | Stance ends when the leg reaches its posterior extreme position, or unloads |
| Fixed tripod, fixed phase | A continuum of gaits — wave, tetrapod, tripod — selected by walking speed |
| Six legs on one global clock | Six largely independent leg controllers, coupled by local rules between neighbours |
| Perturbation is ignored | Perturbation changes when the next step happens |

That last row is the important one. A clock-driven leg will lift while still
loaded and plant while still unloaded, because the clock does not know. Those
are impulsive events, and impulsive events are what the solver has been
choking on. **A sensory-triggered gait is not only more faithful, it is
plausibly easier on the physics** — worth testing explicitly, since it would
mean the biology and the numerics want the same thing.

---

## The shape: three layers, one interface

```
  behaviour          descending neurons select an action
      |              (MDN, DNa02, giant fibre, ...)
      v
  motor programs     walking net | flight net | grooming | escape
      |              produce muscle activations, not joint angles
      v
  plant              muscles -> joints -> physics -> sensors
                              ^                        |
                              +------------------------+
                                   local reflexes
```

**The rule that makes this worth doing:** every layer exposes the same
interface the connectome would. A phenomenological walking net and a
connectome-derived one are interchangeable at the same seam. That gives the
connectome work something it has never had — *a working reference to be
measured against*. At present the connectome produces no rhythm and there is
no target implementation to compare it to, so a null result is uninformative.

---

## Layer 2a: walking, done as the animal does it

Replace the clock with **per-leg state machines plus local coordination
rules** — the Cruse/Walknet formulation, which is the best-validated
phenomenological model of insect walking and reproduces the gait continuum and
its transitions with speed.

Each leg is an independent unit with two states:

```
STANCE   foot planted, moves backward relative to body at the current
         walking speed; ends when the foot reaches its PEP (posterior
         extreme position) OR load drops below threshold
SWING    foot lifts, arcs forward to its AEP (anterior extreme position);
         ends on ground contact, not on a timer
```

Coordination rules act between neighbouring legs (ipsilateral neighbours and
the contralateral partner), each either inhibiting or promoting the start of a
swing:

1. Suppress a neighbour's swing while this leg is swinging (prevents adjacent
   legs lifting together — the thing that drops an insect)
2. Excite a neighbour's swing when this leg begins stance
3. Excite a neighbour's swing as this leg approaches its PEP
4. Correcting rules: a leg that has slipped or missed its target shifts its
   neighbours' targets toward it

Gait emerges. Nobody assigns a tripod. At high speed rule 1 and rule 3
together produce tripod; slow down and it relaxes into tetrapod and wave. That
is the behaviour to test for, and it is a **falsifiable prediction** of the
architecture rather than something tuned in.

What already exists and is reusable:
- `GaitPlan`'s foot-path IK and `basis(leg, axis)` — the swing trajectory and
  the "move this foot 1 mm in x" primitive
- the load-sharing trim — becomes rule 4 and the stance→swing load trigger
- `SensoryOrgans` — the FeCO gives leg position, i.e. the PEP/AEP detector

What has to be built: the per-leg state machine, the rule network, and a
walking-speed input that sets AEP/PEP separation rather than a period.

---

## Layer 2b: flight, which is a different machine entirely

This is the part most often got wrong, and getting it right is *simpler* than
the naive approach, not harder.

**Do not neurally time the wingbeat.** *Drosophila* power comes from
asynchronous, stretch-activated indirect flight muscles. They are myogenic —
the ~200 Hz wingbeat is set by the **mechanical resonance of the thorax**, and
the motor neurons fire far below that frequency and only set activation level.
Simulating a neuron per wingbeat is both expensive and wrong.

So:

```
power muscles     a resonant oscillator. Activation sets amplitude.
                  Frequency comes from thorax stiffness and wing inertia.
steering muscles  ~12 pairs of small SYNCHRONOUS muscles at the wing hinge
                  (basalares b1-b3, axillaries i1/i2/iii1/iii3, hg1-hg4).
                  These fire phase-locked, one spike per cycle for b1, and
                  the SPIKE PHASE encodes the steering command. They modulate
                  the hinge, not the power.
halteres          modified hindwings beating antiphase, sensing Coriolis
                  force. This is the gyroscope, and it is why a fly corrects
                  in tens of milliseconds. Fast inner loop, sub-wingbeat.
```

Aerodynamics at Reynolds number ~100–250 is not the steady kind. The standard
tractable model is **quasi-steady blade element** with three terms beyond
translational lift:

- delayed stall / leading-edge vortex (the reason a fly gets far more lift
  than steady theory allows)
- rotational circulation (wing rotation at stroke reversal)
- added mass

Integrate per wing element per aerodynamic substep. The body physics runs at
1 kHz; the wingbeat is 200 Hz, so aerodynamics needs its own finer substep or
an averaged-per-cycle force model, depending on what is being asked.

**The strategic argument for doing flight next: it has no ground contact at
all.** The solver divergence that has cost this project several nights lives
entirely in the tarsal chain hitting the floor. In flight the legs are tucked
and the only external forces are aerodynamic. The flight track sidesteps the
current blocker completely, and the aerodynamics is additive — it does not
touch the existing solver.

---

## Layer 1: behaviour as action selection

Fly behaviour is well described as a **library of motor programs selected by
descending neurons**, not as continuous control. Roughly 350 DN types carry
the brain's output to the ventral nerve cord, and several are known to be
command-like:

```
MDN          backward walking
DNa02        turning
DNp09        stopping / freezing
giant fibre  escape takeoff
pIP10        courtship song
aDN          grooming
```

**This project can check these against its own data rather than trusting the
list** — `src/core/NeuronNames.*` and the packed connectome are right there,
and `tools/motor_map.py` already builds the motor-neuron mapping. Verify
before building on any of them.

The layer itself is small: a state with hysteresis, DN activity as the input,
a motor program as the output, plus sensible transition rules. Its value is
that it gives the connectome's descending neurons **something to command**. At
present MDN stimulation produces a posture because there is no walking program
for it to select.

---

## What this buys, concretely

1. **The connectome gets a target.** Layer-by-layer replacement, each measured
   against a working reference.
2. **Gait transitions become a test.** Does walking speed move the model
   through wave → tetrapod → tripod? That is a real prediction, checkable
   against published *Drosophila* gait data, and the current model cannot even
   be asked the question.
3. **The duty-factor problem may dissolve.** Duty is ~0.9 because the clock
   holds legs down. Sensory-triggered stance ends when the leg runs out of
   travel, which is what sets duty in an animal.
4. **Flight is a clean room.** No contact, no tarsal chain, no linkage burst.

---

## Risks, honestly

- **Walknet is phenomenological.** It reproduces insect walking well; it is not
  a claim about *Drosophila* circuitry. It must be labelled as a reference
  implementation, never reported as the connectome walking. The project has
  made the reverse error before (findings §16).
- **Flight aerodynamics is a real subsystem**, not a weekend. Quasi-steady
  blade element is tractable but wing hinge kinematics, the resonant thorax
  and haltere feedback are each their own piece of work.
- **The solver still diverges.** Flight avoids it; walking does not. If the
  walking track is chosen, [06-open-questions.md](06-open-questions.md) item 1
  is still on the critical path.
- **Scope.** These are three subsystems. Doing all three at once is how the
  body and the nervous system both ended up half-finished.
