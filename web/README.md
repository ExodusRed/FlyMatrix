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
| native Release (MSVC `/fp:fast`) | 2.89 mm |
| native RelWithDebInfo (strict FP) | 4.46 mm |
| WebAssembly (Clang, strict FP, musl libm) | 7.38 mm |

Last-bit differences in the contact and joint solve during takeoff grow into a
different jump, which is worth knowing before quoting any single jump height
(the main README's 4.96 mm and 7.20 mm figures are two more points on that
spread). `flyrecord` prints its native reference; in the browser,
`Module.ccall('fly_selftest', 'string', [], [])` replays the full recording
headlessly and returns the peak and the cost per physics step.

It is also slow, natively as well as here: `flybody --drop 400` takes ~40 s,
so the page plays the jump in slow motion (at most 1 ms of fly time a frame,
with a ~20 ms CPU budget per frame so the tab stays responsive).

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
