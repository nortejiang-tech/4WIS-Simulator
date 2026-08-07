/**
 * Course furniture for the 3D viewport — traffic cones, slalom poles and
 * painted ground lines.
 *
 * Why this is not just `<coneGeometry>` per marker: from the roof camera the
 * cones are the *instrument*. You judge lane position against them at 60 km/h,
 * 40 m out, so they have to read as real objects — a tapered body, a white
 * reflective band that catches the light, and a base plate that anchors them to
 * the ground plane. A bare orange cone floating at y = 0.3 gives you no depth
 * cue at all.
 *
 * Cost control: a full ISO 3888 course is ~40 cones × 3 parts. Every marker of
 * the same (kind, colour, height) shares one geometry through an InstancedMesh,
 * so a course costs ~3 draw calls per group instead of ~120.
 *
 * Frame: sim world (x fwd, y left, h up) → three (x, h, −y), same as Canvas3D.
 */

import {
  BoxGeometry,
  BufferAttribute,
  BufferGeometry,
  CylinderGeometry,
  Float32BufferAttribute,
  InstancedMesh,
  Matrix4,
  Mesh,
  MeshStandardMaterial,
  Object3D,
} from "three";

import type { PathCone, PathMark } from "@/types/sim";

// ---------------------------------------------------------------------------
// geometry utilities
// ---------------------------------------------------------------------------

/** Concatenate geometries into one (position + normal only, non-indexed). */
export function mergeGeoms(geoms: BufferGeometry[]): BufferGeometry {
  const pos: number[] = [];
  const nrm: number[] = [];
  for (const g of geoms) {
    const src = g.index ? g.toNonIndexed() : g;
    const p = src.getAttribute("position") as BufferAttribute;
    const n = src.getAttribute("normal") as BufferAttribute;
    for (let i = 0; i < p.count; i++) {
      pos.push(p.getX(i), p.getY(i), p.getZ(i));
      nrm.push(n.getX(i), n.getY(i), n.getZ(i));
    }
    if (src !== g) src.dispose();
    g.dispose();
  }
  const out = new BufferGeometry();
  out.setAttribute("position", new Float32BufferAttribute(pos, 3));
  out.setAttribute("normal", new Float32BufferAttribute(nrm, 3));
  return out;
}

interface Part {
  geom: BufferGeometry;
  mat: MeshStandardMaterial;
}

const WHITE = "#f8fafc";
const BASE_DARK = "#1f2937";

function coneParts(color: string, height: number): Part[] {
  const s = height / 0.75;              // 75 cm cone is the reference size
  const rBot = 0.135 * s;
  const rTop = 0.028 * s;
  const plate = 0.035 * s;
  const bodyH = height - plate;
  const r = (f: number) => rBot + (rTop - rBot) * f;   // radius at height fraction

  const body = new CylinderGeometry(rTop, rBot, bodyH, 16, 1, true);
  body.translate(0, plate + bodyH / 2, 0);

  // Reflective collar at mid height, a hair proud of the taper.
  const bandH = 0.14 * s;
  const f0 = 0.46, f1 = f0 + bandH / bodyH;
  const band = new CylinderGeometry(r(f1) * 1.05, r(f0) * 1.05, bandH, 16, 1, true);
  band.translate(0, plate + bodyH * f0 + bandH / 2, 0);

  const base = new BoxGeometry(0.34 * s, plate, 0.34 * s);
  base.translate(0, plate / 2, 0);

  return [
    { geom: body, mat: new MeshStandardMaterial({ color, roughness: 0.55, emissive: color, emissiveIntensity: 0.22 }) },
    { geom: band, mat: new MeshStandardMaterial({ color: WHITE, roughness: 0.3, emissive: WHITE, emissiveIntensity: 0.35 }) },
    { geom: base, mat: new MeshStandardMaterial({ color: BASE_DARK, roughness: 0.9 }) },
  ];
}

function poleParts(color: string, height: number): Part[] {
  // Slightly fatter than a real 40 mm pole: at 60 m a scale-accurate pole is
  // sub-pixel, and the pole is the thing you are aiming at.
  const R = 0.05;
  const SEG = 5;                        // alternating bands, accent first
  const segH = height / SEG;
  const accent: BufferGeometry[] = [];
  const white: BufferGeometry[] = [];
  for (let i = 0; i < SEG; i++) {
    const g = new CylinderGeometry(R, R, segH, 10, 1, true);
    g.translate(0, segH * (i + 0.5), 0);
    (i % 2 === 0 ? accent : white).push(g);
  }
  const foot = new CylinderGeometry(0.14, 0.16, 0.05, 12);
  foot.translate(0, 0.025, 0);

  return [
    { geom: mergeGeoms(accent), mat: new MeshStandardMaterial({ color, roughness: 0.5, emissive: color, emissiveIntensity: 0.25 }) },
    { geom: mergeGeoms(white), mat: new MeshStandardMaterial({ color: WHITE, roughness: 0.4, emissive: WHITE, emissiveIntensity: 0.3 }) },
    { geom: foot, mat: new MeshStandardMaterial({ color: BASE_DARK, roughness: 0.9 }) },
  ];
}

// ---------------------------------------------------------------------------
// cones
// ---------------------------------------------------------------------------

/** One InstancedMesh per (part × kind/colour/height group). */
export function buildCones(cones: PathCone[]): Object3D[] {
  const groups = new Map<string, PathCone[]>();
  for (const c of cones) {
    const h = Math.max(0.15, c.height || 0.75);
    const key = `${c.kind === "pole" ? "pole" : "cone"}|${c.color}|${h.toFixed(3)}`;
    const list = groups.get(key);
    if (list) list.push(c);
    else groups.set(key, [c]);
  }

  const out: Object3D[] = [];
  const m = new Matrix4();
  groups.forEach((list, key) => {
    const [kind, color, hStr] = key.split("|");
    const height = Number(hStr);
    const parts = kind === "pole" ? poleParts(color, height) : coneParts(color, height);
    for (const { geom, mat } of parts) {
      const mesh = new InstancedMesh(geom, mat, list.length);
      list.forEach((c, i) => {
        m.makeTranslation(c.x, 0, -c.y);
        mesh.setMatrixAt(i, m);
      });
      mesh.instanceMatrix.needsUpdate = true;
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      // Instanced bounds are geometry-only; a long cone row would pop out of
      // view near the frustum edge without this.
      mesh.frustumCulled = false;
      out.push(mesh);
    }
  });
  return out;
}

// ---------------------------------------------------------------------------
// painted ground lines
// ---------------------------------------------------------------------------

/** Split a polyline into dash segments of `on` metres every `on + off`. */
export function dashSegments(pts: [number, number][], on: number, off: number): [number, number][][] {
  const out: [number, number][][] = [];
  const period = on + off;
  let s = 0;                     // arc length consumed so far
  let cur: [number, number][] | null = null;
  for (let i = 0; i < pts.length - 1; i++) {
    const [x0, y0] = pts[i];
    const [x1, y1] = pts[i + 1];
    const len = Math.hypot(x1 - x0, y1 - y0);
    if (len < 1e-9) continue;
    let t = 0;
    while (t < len) {
      const phase = (s + t) % period;
      const inDash = phase < on;
      const remain = inDash ? on - phase : period - phase;
      const t1 = Math.min(len, t + remain);
      if (inDash) {
        const a: [number, number] = [x0 + (x1 - x0) * (t / len), y0 + (y1 - y0) * (t / len)];
        const b: [number, number] = [x0 + (x1 - x0) * (t1 / len), y0 + (y1 - y0) * (t1 / len)];
        if (cur) cur.push(b);
        else { cur = [a, b]; out.push(cur); }
      } else {
        cur = null;
      }
      t = t1 + 1e-9;
    }
    s += len;
  }
  return out;
}

/** Flat ribbon along a polyline at height `h`, in three-space. */
export function ribbon(points: [number, number][], width: number, h: number): BufferGeometry {
  const hw = width / 2;
  const pos: number[] = [];
  const nrm: number[] = [];
  for (let i = 0; i < points.length - 1; i++) {
    const [x1, y1] = points[i];
    const [x2, y2] = points[i + 1];
    let dx = x2 - x1, dy = y2 - y1;
    const len = Math.hypot(dx, dy);
    if (len < 1e-9) continue;
    dx /= len; dy /= len;
    const nx = -dy * hw, ny = dx * hw;
    const w = (px: number, py: number) => [px, h, -py];
    const aL = w(x1 + nx, y1 + ny), aR = w(x1 - nx, y1 - ny);
    const bL = w(x2 + nx, y2 + ny), bR = w(x2 - nx, y2 - ny);
    pos.push(...aL, ...aR, ...bL, ...aR, ...bR, ...bL);
    for (let k = 0; k < 6; k++) nrm.push(0, 1, 0);
  }
  const g = new BufferGeometry();
  g.setAttribute("position", new Float32BufferAttribute(pos, 3));
  g.setAttribute("normal", new Float32BufferAttribute(nrm, 3));
  return g;
}

const MARK_HEIGHT = 0.035;   // above scenario paint (0.012) and road (0.01)

/** One mesh per painted mark; dashed marks are merged into a single geometry. */
export function buildMarks(marks: PathMark[]): Object3D[] {
  const out: Object3D[] = [];
  for (const mk of marks) {
    if (!mk.points || mk.points.length < 2) continue;
    const width = Math.max(0.08, mk.width);
    const geom = mk.dash
      ? mergeGeoms(dashSegments(mk.points, 1.6, 1.6).map((seg) => ribbon(seg, width, MARK_HEIGHT)))
      : ribbon(mk.points, width, MARK_HEIGHT);
    if ((geom.getAttribute("position")?.count ?? 0) === 0) { geom.dispose(); continue; }
    const mesh = new Mesh(geom, new MeshStandardMaterial({
      color: mk.color, roughness: 0.85, emissive: mk.color, emissiveIntensity: 0.12,
    }));
    mesh.receiveShadow = true;
    out.push(mesh);
  }
  return out;
}

/** Release the GPU resources held by anything the builders above returned. */
export function disposeCourse(objects: Object3D[]): void {
  for (const o of objects) {
    const mesh = o as Mesh | InstancedMesh;
    mesh.geometry?.dispose?.();
    const mat = mesh.material;
    if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
    else mat?.dispose?.();
  }
}
