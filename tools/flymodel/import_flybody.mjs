// Converts the flybody fruit fly model into FlyMatrix's compact model file.
//
//   node import_flybody.mjs <flybody-assets-dir> <out.bin>
//
// <flybody-assets-dir> holds fruitfly.xml and the .obj meshes from
// https://github.com/TuragaLab/flybody (flybody/fruitfly/assets), licensed
// Apache-2.0. The model is by Google DeepMind and HHMI Janelia Research
// Campus: Vaxenburg, Siwanowicz, Merel et al., "Whole-body physics
// simulation of fruit fly locomotion", Nature 2025.
//
// What this does to it (Apache-2.0 asks that changes be stated):
//  - units converted from centimetres to millimetres;
//  - each mesh baked into its body's frame (geom pos/quat and mesh scale);
//  - meshes welded and simplified with meshoptimizer to a web-sized
//    triangle budget, then quantised (16-bit positions, 8-bit normals);
//  - only the kinematic tree (bodies, hinge joints with axis and range) and
//    the visual meshes are kept; collision geoms, actuators, sites, tendons
//    and physical properties are dropped.
//
// Output format (little-endian), read by src/viz/FlyModel.cpp:
//   "FLYM" u32 version=2
//   u32 nMaterials, then per material: f32 rgba[4]
//   u32 nBodies, then per body: str name, i32 parent, f32 pos[3] (mm),
//       f32 quat[4] (w,x,y,z), u32 nJoints, then per joint: str name,
//       f32 axis[3], f32 lo, f32 hi, f32 ref (MuJoCo springref: the joint's
//       spring rest angle, e.g. the folded wing; 0 if none)
//   u32 nMeshes, then per mesh: i32 body, u32 material, f32 min[3], f32 max[3],
//       u32 nVerts, u32 nIndices, u16 pos[3*nVerts], i8 normal[3*nVerts]
//       (padded to 4 bytes), u16 or u32 indices (u16 when nVerts < 65536)
//   str = u8 length + bytes

import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { join } from 'node:path';
import { MeshoptSimplifier } from 'meshoptimizer';

const [assetsDir, outPath] = process.argv.slice(2);
if (!assetsDir || !outPath) {
  console.error('usage: node import_flybody.mjs <flybody-assets-dir> <out.bin>');
  process.exit(1);
}
const CM_TO_MM = 10;

// ------------------------------------------------------------------ XML helpers
const xml = readFileSync(join(assetsDir, 'fruitfly.xml'), 'utf8');
const attrs = (s) => Object.fromEntries([...s.matchAll(/(\w+)="([^"]*)"/g)].map((m) => [m[1], m[2]]));
const nums = (s, def) => (s === undefined ? def : s.trim().split(/\s+/).map(Number));

// Default classes: nested, each inherits its parent's joint/geom/mesh attributes.
const defaults = {};
{
  // The outermost <default> block: from the first <default> to the first
  // top-level section that follows it.
  const start = xml.indexOf('<default>');
  const ends = ['<asset>', '<worldbody>', '<tendon>', '<actuator>'].map((t) => xml.indexOf(t, start)).filter((i) => i > 0);
  const sec = xml.slice(start, Math.min(...ends));
  const stack = [];
  for (const m of sec.matchAll(/<(\/?)(default|joint|geom|mesh)\b([^>]*?)(\/?)>/g)) {
    const [, close, tag, a, selfClose] = m;
    if (tag === 'default') {
      if (close) { stack.pop(); continue; }
      const cls = attrs(a).class || 'main';
      const parent = stack.length ? defaults[stack[stack.length - 1]] : { joint: {}, geom: {}, mesh: {} };
      defaults[cls] = { joint: { ...parent.joint }, geom: { ...parent.geom }, mesh: { ...parent.mesh } };
      if (!selfClose) stack.push(cls);
    } else if (!close && stack.length) {
      Object.assign(defaults[stack[stack.length - 1]][tag], attrs(a));
    }
  }
}
const meshScaleDefault = nums(defaults.main?.mesh?.scale, [1, 1, 1]);

// Materials and mesh assets.
const materials = [];
const materialIndex = {};
for (const m of xml.matchAll(/<material\b([^>]*)\/>/g)) {
  const a = attrs(m[1]);
  materialIndex[a.name] = materials.length;
  materials.push(nums(a.rgba, [0.5, 0.5, 0.5, 1]));
}
const meshAssets = {};
for (const m of xml.matchAll(/<mesh\b([^>]*)\/>/g)) {
  const a = attrs(m[1]);
  meshAssets[a.name] = { file: a.file, scale: nums(a.scale, meshScaleDefault) };
}

// ------------------------------------------------------------------ body tree
const bodies = [];
const meshGeoms = [];
{
  const wb = xml.slice(xml.indexOf('<worldbody>'), xml.indexOf('</worldbody>'));
  const stack = [];  // indices of open bodies
  const classStack = ['main'];
  for (const m of wb.matchAll(/<(\/?)(body|joint|geom|freejoint)\b([^>]*?)(\/?)>/g)) {
    const [, close, tag, a, selfClose] = m;
    if (tag === 'body') {
      if (close) { stack.pop(); classStack.pop(); continue; }
      const at = attrs(a);
      const cls = at.childclass || classStack[classStack.length - 1];
      bodies.push({
        name: at.name, parent: stack.length ? stack[stack.length - 1] : -1,
        pos: nums(at.pos, [0, 0, 0]).map((v) => v * CM_TO_MM),
        quat: nums(at.quat, [1, 0, 0, 0]), joints: [],
      });
      if (!selfClose) { stack.push(bodies.length - 1); classStack.push(cls); }
      continue;
    }
    if (close || !stack.length) continue;
    const at = attrs(a);
    const cls = at.class || classStack[classStack.length - 1];
    const def = defaults[cls] || defaults.main;
    if (tag === 'joint') {
      const j = { ...def.joint, ...at };
      if ((j.type || 'hinge') !== 'hinge') continue;
      const range = nums(j.range, [-Math.PI, Math.PI]);
      bodies[stack[stack.length - 1]].joints.push({ name: j.name, axis: nums(j.axis, [0, 0, 1]), lo: range[0], hi: range[1],
        ref: j.springref !== undefined ? Number(j.springref) : 0 });
    } else if (tag === 'geom') {
      const g = { ...def.geom, ...at };
      if (!g.mesh || (g.type && g.type !== 'mesh')) continue;
      meshGeoms.push({ body: stack[stack.length - 1], mesh: g.mesh, material: g.material || 'body',
        pos: nums(g.pos, [0, 0, 0]), quat: nums(g.quat, [1, 0, 0, 0]) });
    }
  }
}

// ------------------------------------------------------------------ meshes
const rotate = (q, v) => {
  const [w, x, y, z] = q;
  const n = Math.hypot(w, x, y, z);
  const [qw, qx, qy, qz] = [w / n, x / n, y / n, z / n];
  const ix = qw * v[0] + qy * v[2] - qz * v[1], iy = qw * v[1] + qz * v[0] - qx * v[2];
  const iz = qw * v[2] + qx * v[1] - qy * v[0], iw = -qx * v[0] - qy * v[1] - qz * v[2];
  return [ix * qw + iw * -qx + iy * -qz - iz * -qy, iy * qw + iw * -qy + iz * -qx - ix * -qz,
          iz * qw + iw * -qz + ix * -qy - iy * -qx];
};

function loadObj(file) {
  const text = readFileSync(file, 'utf8');
  const raw = [];
  const tris = [];
  for (const line of text.split('\n')) {
    if (line.startsWith('v ')) {
      const p = line.split(/\s+/);
      raw.push(+p[1], +p[2], +p[3]);
    } else if (line.startsWith('f ')) {
      const idx = line.trim().split(/\s+/).slice(1).map((t) => parseInt(t.split('/')[0], 10) - 1);
      for (let k = 1; k + 1 < idx.length; k++) tris.push(idx[0], idx[k], idx[k + 1]);
    }
  }
  return { raw, tris };
}

// Triangle budget per mesh: a share of the original, within bounds.
const budgetFor = (name, tris) => {
  const big = /^(head|thorax|abdomen)/.test(name) ? 5000 : /wing/.test(name) ? 1200 : 1500;
  return Math.max(Math.min(tris, 200), Math.min(big, Math.round(tris * 0.06)));
};

await MeshoptSimplifier.ready;
const meshes = [];
let totalIn = 0, totalOut = 0;
for (const g of meshGeoms) {
  const asset = meshAssets[g.mesh];
  const file = join(assetsDir, 'obj', asset ? asset.file : g.mesh + '.obj');
  if (!existsSync(file)) { console.warn('missing', file); continue; }
  const { raw, tris } = loadObj(file);
  const scale = asset ? asset.scale : meshScaleDefault;

  // Bake into the body frame, in mm, and weld duplicate vertices.
  const weld = new Map();
  const pos = [];
  const remap = new Int32Array(raw.length / 3);
  for (let i = 0; i < raw.length / 3; i++) {
    const v = [raw[3 * i] * scale[0], raw[3 * i + 1] * scale[1], raw[3 * i + 2] * scale[2]];
    const r = rotate(g.quat, v);
    const p = [(r[0] + g.pos[0]) * CM_TO_MM, (r[1] + g.pos[1]) * CM_TO_MM, (r[2] + g.pos[2]) * CM_TO_MM];
    const key = p.map((c) => Math.round(c * 1e5)).join(',');
    let id = weld.get(key);
    if (id === undefined) { id = pos.length / 3; weld.set(key, id); pos.push(...p); }
    remap[i] = id;
  }
  let indices = Uint32Array.from(tris.map((i) => remap[i]));
  const positions = Float32Array.from(pos);
  const target = budgetFor(g.mesh, tris.length / 3) * 3;
  if (indices.length > target) {
    const [simplified] = MeshoptSimplifier.simplify(indices, positions, 3, target, 0.05, []);
    indices = simplified;
  }
  totalIn += tris.length / 3;
  totalOut += indices.length / 3;

  // Compact the vertex list to what the simplified triangles use.
  const used = new Map();
  const outPos = [];
  const outIdx = new Uint32Array(indices.length);
  for (let k = 0; k < indices.length; k++) {
    let id = used.get(indices[k]);
    if (id === undefined) {
      id = outPos.length / 3;
      used.set(indices[k], id);
      outPos.push(positions[3 * indices[k]], positions[3 * indices[k] + 1], positions[3 * indices[k] + 2]);
    }
    outIdx[k] = id;
  }
  // Area-weighted vertex normals.
  const n = new Float32Array(outPos.length);
  for (let k = 0; k < outIdx.length; k += 3) {
    const [a, b, c] = [outIdx[k], outIdx[k + 1], outIdx[k + 2]];
    const e1 = [0, 1, 2].map((i) => outPos[3 * b + i] - outPos[3 * a + i]);
    const e2 = [0, 1, 2].map((i) => outPos[3 * c + i] - outPos[3 * a + i]);
    const f = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
    for (const v of [a, b, c]) for (let i = 0; i < 3; i++) n[3 * v + i] += f[i];
  }
  meshes.push({ body: g.body, material: materialIndex[g.material] ?? 0, pos: outPos, normals: n, indices: outIdx, name: g.mesh });
}

// ------------------------------------------------------------------ write
const chunks = [];
let size = 0;
const push = (buf) => { chunks.push(buf); size += buf.length; };
const u8 = (v) => push(Buffer.from([v]));
const u32 = (v) => { const b = Buffer.alloc(4); b.writeUInt32LE(v); push(b); };
const i32 = (v) => { const b = Buffer.alloc(4); b.writeInt32LE(v); push(b); };
const f32s = (arr) => { const b = Buffer.alloc(4 * arr.length); arr.forEach((v, i) => b.writeFloatLE(v, 4 * i)); push(b); };
const str = (s) => { const b = Buffer.from(s, 'utf8'); u8(b.length); push(b); };

push(Buffer.from('FLYM'));
u32(2);
u32(materials.length);
for (const m of materials) f32s(m);
u32(bodies.length);
for (const b of bodies) {
  str(b.name);
  i32(b.parent);
  f32s(b.pos);
  f32s(b.quat);
  u32(b.joints.length);
  for (const j of b.joints) { str(j.name); f32s(j.axis); f32s([j.lo, j.hi, j.ref]); }
}
u32(meshes.length);
for (const m of meshes) {
  const nv = m.pos.length / 3;
  const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < nv; i++) for (let k = 0; k < 3; k++) {
    min[k] = Math.min(min[k], m.pos[3 * i + k]);
    max[k] = Math.max(max[k], m.pos[3 * i + k]);
  }
  i32(m.body);
  u32(m.material);
  f32s(min);
  f32s(max);
  u32(nv);
  u32(m.indices.length);
  const q = Buffer.alloc(2 * nv * 3);
  for (let i = 0; i < nv; i++) for (let k = 0; k < 3; k++) {
    const t = max[k] > min[k] ? (m.pos[3 * i + k] - min[k]) / (max[k] - min[k]) : 0;
    q.writeUInt16LE(Math.round(t * 65535), 2 * (3 * i + k));
  }
  push(q);
  const nb = Buffer.alloc(Math.ceil((nv * 3) / 4) * 4);
  for (let i = 0; i < nv; i++) {
    const len = Math.hypot(m.normals[3 * i], m.normals[3 * i + 1], m.normals[3 * i + 2]) || 1;
    for (let k = 0; k < 3; k++) nb.writeInt8(Math.round((m.normals[3 * i + k] / len) * 127), 3 * i + k);
  }
  push(nb);
  if (nv < 65536) {
    const ib = Buffer.alloc(Math.ceil(m.indices.length / 2) * 4);
    m.indices.forEach((v, k) => ib.writeUInt16LE(v, 2 * k));
    push(ib);
  } else {
    const ib = Buffer.alloc(4 * m.indices.length);
    m.indices.forEach((v, k) => ib.writeUInt32LE(v, 4 * k));
    push(ib);
  }
}
writeFileSync(outPath, Buffer.concat(chunks, size));
console.log(`${bodies.length} bodies, ${bodies.reduce((a, b) => a + b.joints.length, 0)} hinge joints, ` +
            `${meshes.length} meshes, ${materials.length} materials`);
console.log(`triangles ${totalIn} -> ${totalOut}, file ${(size / 1e6).toFixed(2)} MB -> ${outPath}`);
