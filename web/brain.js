// FlyMatrix brain viewer -- a WebGL 2 port of flyviz.
//
// Draws all neurons at their real positions and replays a run recorded by
// flyrecord. Shaders, colours, point sizing, blending and camera are the same
// as src/viz/viz_main.cpp and src/viz/Camera.h; the activity trace is rebuilt
// from the recorded spikes with the same decay flyviz applies, so a frame here
// shows what flyviz showed at that moment of the run.

const VERT = `#version 300 es
precision highp float;
layout(location = 0) in vec3 aPos;
layout(location = 1) in float aActivity;
uniform mat4 uViewProj;
uniform vec3 uEye;
uniform float uPointScale;
uniform float uDimAlpha;
out float vActivity;
out float vAlpha;
void main() {
  gl_Position = uViewProj * vec4(aPos, 1.0);
  float dist = length(aPos - uEye);
  float size = uPointScale * (1.0 + 5.0 * aActivity) / max(dist, 1.0);
  gl_PointSize = clamp(size, 1.0, 64.0);
  vActivity = aActivity;
  vAlpha = mix(uDimAlpha, 1.0, aActivity);
}`;

const FRAG = `#version 300 es
precision highp float;
in float vActivity;
in float vAlpha;
out vec4 fragColour;
void main() {
  vec2 d = gl_PointCoord - vec2(0.5);
  float r2 = dot(d, d);
  if (r2 > 0.25) discard;
  float falloff = 1.0 - smoothstep(0.0, 0.25, r2);
  vec3 rest = vec3(0.22, 0.34, 0.62);
  vec3 mid  = vec3(0.20, 0.85, 0.95);
  vec3 hot  = vec3(1.00, 0.96, 0.80);
  float a = clamp(vActivity, 0.0, 1.0);
  vec3 colour = a < 0.5 ? mix(rest, mid, a * 2.0) : mix(mid, hot, (a - 0.5) * 2.0);
  fragColour = vec4(colour * falloff, vAlpha * falloff);
}`;

// ---------------------------------------------------------------- recording

/** Parses brain.bin (see src/web/record_main.cpp for the format). */
export function parseBrain(buffer) {
  const dv = new DataView(buffer);
  const bytes = new Uint8Array(buffer);
  const tag = String.fromCharCode(...bytes.subarray(0, 4));
  if (tag !== 'FLYB' || dv.getUint32(4, true) !== 1) throw new Error('not a flyrecord brain file');
  let o = 8;
  const n = dv.getUint32(o, true); o += 4;
  const frames = dv.getUint32(o, true); o += 4;
  const msPerFrame = dv.getFloat32(o, true); o += 4;
  const decay = dv.getFloat32(o, true); o += 4;
  const pulseMs = dv.getFloat32(o, true); o += 4;
  const box = []; for (let i = 0; i < 6; i++) { box.push(dv.getFloat32(o, true)); o += 4; }

  const positions = new Float32Array(n * 3);
  for (let i = 0; i < n * 3; i++) {
    const axis = i % 3;
    const lo = box[axis], hi = box[axis + 3];
    positions[i] = lo + (dv.getUint16(o, true) / 65535) * (hi - lo);
    o += 2;
  }

  const varint = () => {
    let v = 0, shift = 0, b;
    do { b = bytes[o++]; v |= (b & 0x7f) << shift; shift += 7; } while (b & 0x80);
    return v >>> 0;
  };
  const spikes = [], gradedIdx = [], gradedVal = [];
  for (let f = 0; f < frames; f++) {
    const ns = varint();
    const s = new Uint32Array(ns);
    let prev = 0;
    for (let k = 0; k < ns; k++) { prev += varint(); s[k] = prev; }
    spikes.push(s);
    const ng = varint();
    const gi = new Uint32Array(ng), gv = new Uint8Array(ng);
    prev = 0;
    for (let k = 0; k < ng; k++) { prev += varint(); gi[k] = prev; gv[k] = bytes[o++]; }
    gradedIdx.push(gi);
    gradedVal.push(gv);
  }
  return { n, frames, msPerFrame, decay, pulseMs, box, positions, spikes, gradedIdx, gradedVal };
}

// ---------------------------------------------------------------- matrices (column-major, as M4)

function perspective(fovy, aspect, near, far) {
  const f = 1 / Math.tan(fovy / 2), m = new Float32Array(16);
  m[0] = f / aspect; m[5] = f; m[10] = (far + near) / (near - far); m[11] = -1;
  m[14] = (2 * far * near) / (near - far);
  return m;
}
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (a) => { const l = Math.hypot(...a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
function lookAt(eye, centre, up) {
  const f = norm(sub(centre, eye)), s = norm(cross(f, up)), u = cross(s, f), m = new Float32Array(16);
  m[0] = s[0]; m[4] = s[1]; m[8] = s[2];
  m[1] = u[0]; m[5] = u[1]; m[9] = u[2];
  m[2] = -f[0]; m[6] = -f[1]; m[10] = -f[2];
  m[12] = -dot(s, eye); m[13] = -dot(u, eye); m[14] = dot(f, eye); m[15] = 1;
  return m;
}
function mul(a, b) {
  const r = new Float32Array(16);
  for (let c = 0; c < 4; c++)
    for (let row = 0; row < 4; row++) {
      let v = 0;
      for (let k = 0; k < 4; k++) v += a[k * 4 + row] * b[c * 4 + k];
      r[c * 4 + row] = v;
    }
  return r;
}

/** Same behaviour as fly::viz::OrbitCamera with zUp = false. */
class OrbitCamera {
  target = [0, 0, 0]; distance = 1000; yaw = 0; pitch = 0.18; fovy = 0.9;
  orbit(dy, dp) { this.yaw += dy; this.pitch = Math.min(1.5533, Math.max(-1.5533, this.pitch + dp)); }
  zoom(f) { this.distance = Math.min(20000, Math.max(0.05, this.distance * f)); }
  forward() {
    const cp = Math.cos(this.pitch), sp = Math.sin(this.pitch);
    return [cp * Math.cos(this.yaw), sp, cp * Math.sin(this.yaw)];
  }
  pan(dx, dy) {
    const f = this.forward(), right = norm(cross(f, [0, 1, 0])), up = cross(right, f), s = this.distance * 0.0015;
    for (let i = 0; i < 3; i++) this.target[i] += right[i] * -dx * s + up[i] * dy * s;
  }
  eye() { const f = this.forward(); return this.target.map((t, i) => t - f[i] * this.distance); }
  viewProjection(aspect) {
    const near = Math.max(0.002, this.distance * 0.002), far = this.distance * 8 + 4000;
    return mul(perspective(this.fovy, aspect, near, far), lookAt(this.eye(), this.target, [0, 1, 0]));
  }
}

// ---------------------------------------------------------------- viewer

export class BrainViewer {
  constructor(canvas, rec, onFrame) {
    this.canvas = canvas;
    this.rec = rec;
    this.onFrame = onFrame;
    this.frame = 0;          // next recorded frame to apply
    this.playing = true;
    this.fps = 30;           // recorded frames shown per second
    this.dimAlpha = 0.30;    // flyviz default; 0 = "active only" (its `s` key)
    this.activity = new Float32Array(rec.n);
    this.acc = 0;
    this.holdUntil = 0;

    const gl = canvas.getContext('webgl2', { antialias: false, premultipliedAlpha: false });
    if (!gl) throw new Error('WebGL 2 is not available in this browser');
    this.gl = gl;
    this.prog = this.#program();
    this.vao = gl.createVertexArray();
    gl.bindVertexArray(this.vao);
    const pos = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, pos);
    gl.bufferData(gl.ARRAY_BUFFER, rec.positions, gl.STATIC_DRAW);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(0);
    this.actBuf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, this.actBuf);
    gl.bufferData(gl.ARRAY_BUFFER, this.activity, gl.DYNAMIC_DRAW);
    gl.vertexAttribPointer(1, 1, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(1);
    this.loc = Object.fromEntries(['uViewProj', 'uEye', 'uPointScale', 'uDimAlpha'].map((u) => [u, gl.getUniformLocation(this.prog, u)]));

    // Frame the whole nervous system, as flyviz does.
    const b = rec.box;
    this.cam = new OrbitCamera();
    this.cam.target = [(b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2];
    this.cam.distance = Math.max(b[3] - b[0], b[4] - b[1], b[5] - b[2]) * 1.3;
    this.#bindInput();
    this.last = performance.now();
    requestAnimationFrame(this.#loop);
  }

  #program() {
    const gl = this.gl;
    const sh = (type, src) => {
      const s = gl.createShader(type);
      gl.shaderSource(s, src);
      gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
      return s;
    };
    const p = gl.createProgram();
    gl.attachShader(p, sh(gl.VERTEX_SHADER, VERT));
    gl.attachShader(p, sh(gl.FRAGMENT_SHADER, FRAG));
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p));
    return p;
  }

  /** Apply one recorded frame, exactly as flyviz updates its trace. */
  #applyFrame(f) {
    const a = this.activity, r = this.rec;
    for (const j of r.spikes[f]) a[j] = 1;
    const d = r.decay;
    for (let i = 0; i < a.length; i++) a[i] *= d;
    const gi = r.gradedIdx[f], gv = r.gradedVal[f];
    for (let k = 0; k < gi.length; k++) a[gi[k]] = Math.max(a[gi[k]], gv[k] / 255);
  }

  /** Jump to a frame by replaying from the start (cheap: ~150 frames). */
  seek(frame) {
    this.activity.fill(0);
    const target = Math.max(0, Math.min(this.rec.frames, frame));
    for (let f = 0; f < target; f++) this.#applyFrame(f);
    this.frame = target;
    this.acc = 0;
    this.#report();
  }

  restart() { this.seek(0); this.playing = true; }

  #report() {
    // Spikes arrive in volleys one synaptic delay (1.8 ms) apart, so a single
    // 0.8 ms frame alternates between a volley and nothing. Counting over the
    // last two frames gives a number that doesn't flicker.
    const count = (f) => (f >= 0 && f < this.rec.frames ? this.rec.spikes[f].length : 0);
    this.onFrame?.({
      frame: this.frame,
      frames: this.rec.frames,
      timeMs: this.frame * this.rec.msPerFrame,
      windowMs: 2 * this.rec.msPerFrame,
      active: count(this.frame - 1) + count(this.frame - 2),
      stimulating: this.frame * this.rec.msPerFrame < this.rec.pulseMs,
      playing: this.playing,
    });
  }

  #loop = (now) => {
    const dt = Math.min(0.1, (now - this.last) / 1000);
    this.last = now;
    if (this.playing && this.visible !== false) {
      if (this.frame >= this.rec.frames) {
        // Hold the last frame a moment, then loop.
        if (!this.holdUntil) this.holdUntil = now + 1500;
        if (now > this.holdUntil) { this.holdUntil = 0; this.seek(0); }
      } else {
        this.acc += dt * this.fps;
        let stepped = false;
        while (this.acc >= 1 && this.frame < this.rec.frames) {
          this.#applyFrame(this.frame++);
          this.acc -= 1;
          stepped = true;
        }
        if (stepped) this.#report();
      }
    }
    this.#draw();
    requestAnimationFrame(this.#loop);
  };

  #draw() {
    const gl = this.gl, c = this.canvas;
    const dpr = Math.min(devicePixelRatio || 1, 2);
    const w = Math.round(c.clientWidth * dpr), h = Math.round(c.clientHeight * dpr);
    if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
    gl.viewport(0, 0, w, h);
    gl.clearColor(0.02, 0.025, 0.04, 1);
    gl.clear(gl.COLOR_BUFFER_BIT);
    gl.disable(gl.DEPTH_TEST);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE);

    gl.bindBuffer(gl.ARRAY_BUFFER, this.actBuf);
    gl.bufferSubData(gl.ARRAY_BUFFER, 0, this.activity);
    gl.useProgram(this.prog);
    gl.uniformMatrix4fv(this.loc.uViewProj, false, this.cam.viewProjection(h > 0 ? w / h : 1));
    gl.uniform3fv(this.loc.uEye, this.cam.eye());
    const pxPerUm = h / (2 * Math.tan(this.cam.fovy / 2));
    gl.uniform1f(this.loc.uPointScale, 3.0 * pxPerUm);
    gl.uniform1f(this.loc.uDimAlpha, this.dimAlpha);
    gl.bindVertexArray(this.vao);
    gl.drawArrays(gl.POINTS, 0, this.rec.n);
  }

  #bindInput() {
    const c = this.canvas;
    const pointers = new Map();
    let pinch = 0;
    c.addEventListener('contextmenu', (e) => e.preventDefault());
    c.addEventListener('pointerdown', (e) => { c.setPointerCapture(e.pointerId); pointers.set(e.pointerId, { x: e.clientX, y: e.clientY, b: e.button }); });
    const end = (e) => { pointers.delete(e.pointerId); pinch = 0; };
    c.addEventListener('pointerup', end);
    c.addEventListener('pointercancel', end);
    c.addEventListener('pointermove', (e) => {
      const p = pointers.get(e.pointerId);
      if (!p) return;
      const dx = e.clientX - p.x, dy = e.clientY - p.y;
      p.x = e.clientX; p.y = e.clientY;
      if (pointers.size === 2) {
        const [a, b] = [...pointers.values()];
        const d = Math.hypot(a.x - b.x, a.y - b.y);
        if (pinch) this.cam.zoom(pinch / d);
        pinch = d;
      } else if (p.b === 2 || e.shiftKey) this.cam.pan(dx, dy);
      else this.cam.orbit(-dx * 0.006, dy * 0.006);
    });
    c.addEventListener('wheel', (e) => { e.preventDefault(); this.cam.zoom(e.deltaY < 0 ? 0.88 : 1.136); }, { passive: false });
  }
}
