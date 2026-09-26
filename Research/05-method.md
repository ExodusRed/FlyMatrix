# Method: how to test this, and what keeps going wrong

The most expensive mistakes in this project have not been physics mistakes.
They have been measurement mistakes — believing a number that was not
measuring what it appeared to measure. This file is the accumulated list.

---

## 1. A passing metric is not evidence the behaviour is right

Twice now, every number in the harness said the fly was walking correctly while
it was visibly broken.

**Finding 21.** The femur ended 85 µm *below* the floor and the tibia ran
underground for its whole length, because only the tarsus had contact probes.
On screen the leg looked snapped in half. No measurement asked where the leg
segments actually were. The worst anchor error, 0.0616 mm, had been noted twice
and dismissed as small — at 370 px/mm it renders as a 23-pixel break. It was
never small.

**Finding 22.** The legs were being torn off by the solver, and height, pitch
and contact count all looked normal on the frame it happened.

Both were found by **looking**: once at a rendered frame, once at a per-frame
trace. Neither was findable from the summary statistics that existed.

> If you have never watched the thing move, you do not know what it is doing.

Tools that exist for this and should be used before believing anything:

```
flybody --gait 60 --film out/w --film-every 25 --frames 1500
flyphys --foot N                  per-segment report, one leg
flyphys --from A --to B           every frame between two times
flyphys --jacobian                what each joint does to the foot
```

---

## 2. Ten samples in twelve seconds will not show you a one-frame event

The default verbose trace printed ten rows per run. Read from those rows, the
failure looked like a slow accumulating pitch drift, and **four separate
explanations were built on that reading** — pitch drift, foot placement,
postural gain, heading drift — each of which was then tested and failed.

Printing every frame showed the truth immediately:

```
2163 ms  anchor 0.047     <- normal
2164 ms  anchor 1.4211    <- linkage torn apart
```

A sampled trace shows you trends. If the thing you are hunting is an *event*,
sampling will actively mislead you by making it look like a trend.

---

## 3. If a number exists in two places, it is already wrong

Four occurrences, all costing real time:

| What | Symptom |
|---|---|
| Anatomy dimensions | model built to different dimensions than were solved for |
| Abdomen offset | trunk geometry disagreed with itself |
| Joint limits | drifted on **4 of 5 joints**; solver produced poses the physics rejected on step one, every knee starting outside its limit, fly standing 0.37 mm too high |
| The gait controller | `flybody` ran its own older gait, so three sessions of measured improvements never reached the picture the work is judged by |

The fix used for joint limits is the pattern to copy: **one owner, everyone
else parses it.** `src/body/Anatomy.h` holds `kJointLimit`, and
`tools/solve_rest_pose.py` reads the limits out of the header with a regex
rather than keeping a copy.

---

## 4. Watch for results that do not move when they certainly should

A held `.exe` makes the link fail — `LNK1104` — and the **old binary runs**.
The output is byte-identical sweep rows that look exactly like a genuine null
result. This happened at least five times across sessions, including three
times in one night: a `--probe-drive` sweep, a tarsal-servo-rate sweep, and a
posture sweep all "showed no effect" while testing nothing.

```sh
taskkill //F //IM flyphys.exe     # before every build
```

Related: an **invented flag**. A whole tonic/posture sweep returned identical
rows because the flag was `--posture` and the sweep passed `--stiffness`,
which the parser ignored silently. Unknown flags should be rejected loudly;
they currently are not.

Related: a **flag-parsing bug**. `for (int k = 1; k < argc - 1; ++k)` skips a
valueless flag in last position, so `--no-hill` was silently inert wherever it
was written last.

**The tell in all three cases is the same: a result that does not move when it
certainly should.** Treat that as a bug in the harness until proven otherwise.

---

## 5. One trial is not a measurement

This model was — and to a degree still is — sensitive enough that a single
trajectory means nothing. Three changes that altered no physics at all
(caching world inertia, splitting the joint solve into passes, re-solving the
rest pose) each moved the headline walking figure by a factor of two or more.

Findings §16 is a **retraction** of earlier walking figures on exactly these
grounds. It stands.

The harness therefore runs N trials from jittered starting heights
(±16 µm) and reports median, range and upright fraction:

```sh
flyphys --gait --seconds 12 --trials 5
```

Report the fraction, not a verdict that rounds a bare majority up to "it
walks". **Five of nine is not the same claim as nine of nine.**

A caveat learned late: even this can mislead. If the underlying problem is a
latent divergence rather than a continuous sensitivity, sweeps come out
**non-monotone** — stride 0.02 and 0.10 survive while 0.05 and 0.16 do not —
and reading that as a dose-response will send you after the wrong variable.

---

## 6. Health checks worth running before believing a run

```
anchor separation    standing 0.022 mm; a healthy walk stays under ~0.06 mm;
                     approaching 1 mm means the legs have come apart and every
                     other number in the run is meaningless
velocity clamp hits  standing fires it ZERO times. If it fires during normal
                     operation, something upstream is diverging
peak angular vel     standing peaks at 183 rad/s. Thousands means trouble even
                     if the run looks fine
trunk contact        the thorax, abdomen and head should never touch the floor
```

`flyphys --stand-only` is the fast regression: six feet down, trunk clear,
ride height > 0.52 mm, pitch within 2°.

---

## 7. Editing C++ from Python heredocs mangles escapes

A practical annoyance that has cost real time roughly ten times. Writing
`"\\n"` inside a quoted shell heredoc can arrive at Python as `"\n"` and be
written into the C++ source as a literal newline inside a string literal —
`error C2001: newline in constant`.

Reliable approaches, in order of preference:

1. Write the patch to a `.py` file with the Write tool, then run it.
2. Build the escape explicitly: `NL = chr(92) + "n"`.
3. Use a placeholder in the template and substitute:
   `"...text@N@".replace("@N@", NL)`.

Do not fight it inline.
