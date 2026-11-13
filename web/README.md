# FlyMatrix in the browser

`web/` is a static site that shows FlyMatrix without installing anything:

- **Brain**: all 176,571 neurons at their real positions, replaying a giant-fibre
  run. A WebGL 2 port of `flyviz` with the same shaders, colours, camera and
  activity trace.
- **Body**: the real `FlyBody` + `FlyPhysics` C++ code compiled to WebAssembly,
  stepping the physics live in the browser every millisecond.

## Why part of it is a recording

The connectome does not fit in a web page: `cns.bin` is ~100 MB and the network
needs several hundred MB of RAM. So the nervous system runs **natively**, and
`flyrecord` saves what the browser needs:

| file | contents | size |
|---|---|---|
| `data/brain.bin` | neuron positions (16-bit) + per-frame spikes and graded output (delta-coded varints) | ~1.2 MB |
| `data/drive.bin` | activation of all 334 leg motor units, every 1 ms for 400 ms | ~0.3 MB |
| `data/motor_map.tsv` | the motor map the drive was recorded against | 18 KB |

The body is not an animation of that run. It receives the recorded **muscle
activations** and the physics computes the movement live. With the sensory loop
off (the native default) nothing flows from the body back to the brain, so the
replayed drive is identical to the live one.

### The jump is sensitive to rounding

The drive is deterministic: a native `/fp:fast` build and a strict build
record bit-identical `drive.bin` files. The physics is not robust to rounding,
though. Fed that identical drive, the same solver code peaks at:

| build | peak height |
|---|---:|
| native Release (MSVC `/fp:fast`) | 2.14 mm (2.89 before the solver speed-up) |
| native RelWithDebInfo (strict FP) | 4.46 mm (unchanged: that build is bit-identical) |
| WebAssembly (Clang, strict FP, musl libm) | 2.70 mm (7.38 before) |

Last-bit differences in the contact and joint solve during takeoff grow into a
different jump, which is worth knowing before quoting any single jump height
(the main README's 4.96 mm and 7.20 mm figures are two more points on that
spread). `flyrecord` prints its native reference; in the browser,
`Module.ccall('fly_selftest', 'string', [], [])` replays the full recording
headlessly and returns the peak and the cost per physics step.

The solver speed-up (per-substep geometry computed once instead of in all 64
iterations, bit-identical without fast-math) moved the fast-math figures,
which is the sensitivity again rather than a change in the physics.

It is still not real time: about 6 ms of computation per simulated ms in the
browser for the jump (64 solver iterations), less for walking (32), so both
play in slow motion with a ~20 ms CPU budget per frame to keep the tab
responsive.

## Walking

The Walk mode (the default on the Body tab) drives the same live physics with
`WalkController`: six coupled phase oscillators locked into an alternating
tripod, a constant-speed stance stroke and a lifted swing, duty factor and lift
that change with speed, turning by stride asymmetry, and a heading hold. The
rhythm is a modelling choice, not the connectome -- see `docs/findings.md`
section 12 for why, how the parameters were chosen and what it measures.

## Rebuilding

**1. Record** (native, needs `data/bin/` from `tools/pack_cns.py` and `tools/motor_map.py`):

```sh
cmake -S . -B build
cmake --build build --config Release --target flyrecord
./build/Release/flyrecord            # writes web/data/
```

**2. Compile the body to WebAssembly** (needs the [Emscripten SDK](https://emscripten.org)
and the SDL3 checkout at `../Library/SDL`):

```sh
emcmake cmake -S . -B build-web
cmake --build build-web              # writes web/body/flybody.{js,wasm,data}
```

**3. Serve** `web/` with any static server, e.g. `python -m http.server -d web`.
Opening `index.html` straight from disk does not work: browsers block `fetch`
and WebAssembly from `file://`.

## Licences and attribution

- **Data** (`data/*`): derived from the FlyEM Male CNS connectome v0.9, which
  is licensed CC BY 4.0. [`DATA_NOTICE.txt`](DATA_NOTICE.txt) carries the
  attribution, the licence link, the list of changes and the no-endorsement
  statement that CC BY 4.0 requires; the page shows the same in its footer.
  Keep both when redistributing.
- **Compiled-in software**: Emscripten runtime and musl (MIT), LLVM libc++
  (Apache-2.0 with LLVM exception), SDL 3 (zlib). Their licence texts are in
  [`THIRD_PARTY_NOTICES.txt`](THIRD_PARTY_NOTICES.txt), served next to the
  page and linked from its footer. Regenerate it if the toolchain changes.

## What changed in the C++ for this

- `src/viz/GL.h`: under Emscripten the GL entry points come straight from
  WebGL 2 (`GLES3/gl3.h`) instead of `SDL_GL_GetProcAddress`. Native is unchanged.
- `MotorPools::unitCount()` / `setActivations()`: lets the web build feed
  recorded activations into the same muscles.
- `src/web/record_main.cpp` (`flyrecord`) and `src/web/body_web.cpp` (the
  browser entry point; the scene drawing follows `body_main.cpp`, with
  GLSL ES 3.00 shaders and an `emscripten_set_main_loop` frame loop).
- `src/core/Connectome.cpp`: skips the 4 alignment bytes the packer writes when
  the neuron count is odd (male-cns:v0.9 has 176,571). Taken from the stashed
  work on `physics-servo`.
