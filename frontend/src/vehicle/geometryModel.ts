/**
 * Parameter-driven vehicle geometry math — the single source shared by the
 * three vehicle-page diagrams (whole-vehicle / kingpin-wheel / rack-hardpoints)
 * and the theory-page reference figures. Pure functions, no React.
 *
 * The steering-linkage 4-bar solver is the same one the load page's
 * RackSteerMechanism uses — kept here so there is exactly one implementation
 * (the frontend previously duplicated it; the backend has its own in
 * geometry.py — a known parallel implementation, verified consistent).
 *
 * Frame: hardpoints are local to a wheel's kingpin at straight-ahead, X forward,
 * Y left, metres. The vehicle top view uses body frame (X forward, Y left).
 */

export type Params = Record<string, any> & {
  suspension?: Record<string, number>;
  steering_geometry?: Record<string, number>;
};

export interface Vec2 { x: number; y: number }

export const rad = (d: number) => (d * Math.PI) / 180;
export const deg = (r: number) => (r * 180) / Math.PI;
export const MM = 1000;
export const G = 9.80665;

export function getP(p: Params | null, path: string, fallback = 0): number {
  if (!p) return fallback;
  const v = path.split(".").reduce<any>((acc, k) => acc?.[k], p);
  return Number.isFinite(v) ? v : fallback;
}

// ── vector helpers ───────────────────────────────────────────────────────────
export const vAdd = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x + b.x, y: a.y + b.y });
export const vSub = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x - b.x, y: a.y - b.y });
export const vMul = (a: Vec2, k: number): Vec2 => ({ x: a.x * k, y: a.y * k });
export const vDot = (a: Vec2, b: Vec2) => a.x * b.x + a.y * b.y;
export const vCross = (a: Vec2, b: Vec2) => a.x * b.y - a.y * b.x;
export const vLen = (a: Vec2) => Math.hypot(a.x, a.y);
export const vNorm = (a: Vec2): Vec2 => { const l = Math.max(vLen(a), 1e-12); return { x: a.x / l, y: a.y / l }; };
const wrapPi = (v: number) => { let a = (v + Math.PI) % (2 * Math.PI); if (a < 0) a += 2 * Math.PI; return a - Math.PI; };

// ── core vehicle ─────────────────────────────────────────────────────────────
export interface VehicleGeom {
  L: number; tf: number; tr: number; a: number; b: number; h: number;
  mass: number; iz: number; steerLimit: number; tireR: number; tireW: number;
  frontLoadFrac: number; rearLoadFrac: number;
  minTurnRadius: number;   // ideal-Ackermann centre turning radius [m]
  izRef: number;           // m·a·b engineering estimate (radius of gyration² ≈ a·b)
}

export function vehicleGeom(p: Params | null): VehicleGeom {
  const L = getP(p, "wheelbase", 3.16);
  const tf = getP(p, "track_front", 1.565);
  const tr = getP(p, "track_rear", 1.565);
  const a = getP(p, "cg_to_front", 1.55);
  const b = Math.max(L - a, 1e-6);
  const steerLimit = getP(p, "steer_limit", 0.61);
  const invKappa = (L / 2) / Math.tan(Math.max(steerLimit, 1e-3)) + Math.max(tf, tr) / 2;
  const mass = getP(p, "mass", 2900);
  return {
    L, tf, tr, a, b, h: getP(p, "cg_height", 0.62),
    mass, iz: getP(p, "inertia_z", 7500),
    steerLimit, tireR: getP(p, "tire_radius", 0.395), tireW: getP(p, "tire_width", 0.265),
    frontLoadFrac: b / L, rearLoadFrac: a / L,
    minTurnRadius: invKappa,
    izRef: mass * a * b,
  };
}

// ── kingpin / wheel ──────────────────────────────────────────────────────────
export interface KingpinGeom {
  caster: number; kpi: number; camber: number; scrub: number;
  tireR: number; tireW: number; tPneu: number;
  mechTrail: number;     // R·tanε
  totalTrail: number;    // mechanical + pneumatic
}

export function kingpinGeom(p: Params | null): KingpinGeom {
  const caster = getP(p, "suspension.caster_angle", 0.1047);
  const tireR = getP(p, "tire_radius", 0.395);
  const tPneu = getP(p, "tire_t_pneumatic", 0.03);
  const mechTrail = tireR * Math.tan(caster);
  return {
    caster,
    kpi: getP(p, "suspension.kingpin_inclination", 0.2094),
    camber: getP(p, "suspension.camber", -0.0131),
    scrub: getP(p, "suspension.scrub_radius", 0.015),
    tireR, tireW: getP(p, "tire_width", 0.265), tPneu,
    mechTrail, totalTrail: mechTrail + tPneu,
  };
}

// ── steering linkage 4-bar (per wheel) ───────────────────────────────────────
export interface LinkageBase {
  outer0: Vec2; inner0: Vec2; axis: Vec2;
  armLength: number; tieLength: number; theta0: number; limit: number;
}
export interface LinkageSolution {
  valid: boolean; rackTravel: number; delta: number;
  outer: Vec2; inner: Vec2; armTieAngle: number; tieRackAngle: number; efficiency: number;
}

/** wheelIndex 0=FL 1=FR 2=RL 3=RR. Hardpoints describe the left wheel; right is mirrored. */
export function linkageBase(p: Params | null, wheelIndex: number): LinkageBase {
  const geom = p?.steering_geometry ?? {};
  const isFront = wheelIndex < 2;
  const isLeft = wheelIndex === 0 || wheelIndex === 2;
  const prefix = isFront ? "front" : "rear";
  const mirror = isLeft ? 1 : -1;
  const gv = (k: string, fb: number) => {
    const v = Number((geom as Record<string, number>)[`${prefix}_${k}`]);
    return Number.isFinite(v) ? v : fb;
  };
  const outer0 = { x: gv("outer_x", isFront ? -0.146 : 0.146), y: mirror * gv("outer_y", -0.0118) };
  const inner0 = { x: gv("inner_x", isFront ? -0.176 : 0.176), y: mirror * gv("inner_y", -0.3626) };
  const axisDeg = gv("rack_axis_deg", 90);
  const axis = vNorm({ x: Math.cos(rad(axisDeg)), y: mirror * Math.sin(rad(axisDeg)) });
  return {
    outer0, inner0, axis,
    armLength: Math.max(vLen(outer0), 1e-6),
    tieLength: Math.max(vLen(vSub(inner0, outer0)), 1e-6),
    theta0: Math.atan2(outer0.y, outer0.x),
    limit: Math.max(gv("rack_travel_limit", 0.085), 0.001),
  };
}

export function solveRackToWheel(base: LinkageBase, rackTravel: number, hint = 0): LinkageSolution {
  const inner = vAdd(base.inner0, vMul(base.axis, rackTravel));
  const d = vLen(inner);
  const bad = (delta = hint): LinkageSolution => ({
    valid: false, rackTravel, delta, outer: base.outer0, inner,
    armTieAngle: 0, tieRackAngle: 0, efficiency: 0,
  });
  if (d < 1e-9) return bad();
  const r0 = base.armLength, r1 = base.tieLength;
  const a = (r0 * r0 - r1 * r1 + d * d) / (2 * d);
  const h2 = r0 * r0 - a * a;
  if (h2 < -1e-9) return bad();
  const h = Math.sqrt(Math.max(h2, 0));
  const along = vMul(inner, a / d);
  const perp = { x: -inner.y / d, y: inner.x / d };
  const cands = [vAdd(along, vMul(perp, h)), vSub(along, vMul(perp, h))];
  let outer = cands[0];
  let bestDelta = wrapPi(Math.atan2(outer.y, outer.x) - base.theta0);
  let bestScore = Math.abs(wrapPi(bestDelta - hint));
  for (const c of cands.slice(1)) {
    const dl = wrapPi(Math.atan2(c.y, c.x) - base.theta0);
    const sc = Math.abs(wrapPi(dl - hint));
    if (sc < bestScore) { outer = c; bestDelta = dl; bestScore = sc; }
  }
  const arm = vNorm(outer);
  const tie = vNorm(vSub(inner, outer));
  const armTieAngle = Math.acos(Math.max(-1, Math.min(1, vDot(arm, tie))));
  const rackProj = Math.abs(vDot(tie, base.axis));
  const tieRackAngle = Math.acos(Math.max(0, Math.min(1, rackProj)));
  const efficiency = Math.max(Math.abs(vCross(arm, tie)) * rackProj, 1e-6);
  return { valid: true, rackTravel, delta: bestDelta, outer, inner, armTieAngle, tieRackAngle, efficiency };
}

export function rackTravelProfile(base: LinkageBase, halfSteps = 60): LinkageSolution[] {
  const zero = solveRackToWheel(base, 0, 0);
  const neg: LinkageSolution[] = [], pos: LinkageSolution[] = [];
  let hint = zero.delta;
  for (let i = 1; i <= halfSteps; i++) { const s = solveRackToWheel(base, -base.limit * i / halfSteps, hint); if (s.valid) hint = s.delta; neg.push(s); }
  hint = zero.delta;
  for (let i = 1; i <= halfSteps; i++) { const s = solveRackToWheel(base, base.limit * i / halfSteps, hint); if (s.valid) hint = s.delta; pos.push(s); }
  return [...neg.reverse(), zero, ...pos];
}

// ── front-axle Ackermann from the two front linkages ─────────────────────────
export interface AxleState {
  rackTravel: number; limit: number;
  left: LinkageSolution; right: LinkageSolution;
  deltaLeft: number; deltaRight: number;
  inner: number; outer: number;            // |angles| sorted
  ackermannIdeal: number;                   // ideal inner angle for the current outer
  ackermannErrorDeg: number;                // inner_actual − inner_ideal [deg]
  turnRadius: number;                       // centre turning radius at this state [m]
  minEfficiency: number;
  singular: boolean;
}

/** Solve both front wheels for one rack travel; the rack moves both inner joints
 *  by the same axial amount (shared rack), mirrored in y. */
export function axleState(p: Params | null, rackTravel: number): AxleState {
  const L = getP(p, "wheelbase", 3.16);
  const tf = getP(p, "track_front", 1.565);
  const baseL = linkageBase(p, 0);
  const baseR = linkageBase(p, 1);
  const limit = baseL.limit;
  const rt = Math.max(-limit, Math.min(limit, rackTravel));
  // Shared rigid rack: a world +Δy translation projects onto each wheel's local
  // rack axis. The right axis is mirrored (0,−1), so its local travel is −rt —
  // otherwise the "rack" would stretch instead of translate (both inner joints
  // must shift the same world direction). This is what produces the Ackermann
  // asymmetry between the inner and outer wheel.
  const left = solveRackToWheel(baseL, rt, 0);
  const right = solveRackToWheel(baseR, -rt, 0);
  const dL = left.delta, dR = right.delta;
  const inner = Math.max(Math.abs(dL), Math.abs(dR));
  const outer = Math.min(Math.abs(dL), Math.abs(dR));
  // Ideal Ackermann: cot(δ_outer) − cot(δ_inner) = tf / L.
  let ackIdeal = inner;
  if (outer > 1e-4) {
    const cotInner = 1 / Math.tan(outer) - tf / L;
    ackIdeal = cotInner > 1e-6 ? Math.atan(1 / cotInner) : inner;
  }
  // Outer-wheel Ackermann estimate: rear-axle R = L*cot(outer) − tf/2.
  // Report the trajectory radius of the public body origin (axle midpoint).
  const rearRadius = L / Math.tan(outer) - tf / 2;
  const turnRadius = outer > 1e-4 ? Math.hypot(rearRadius, L / 2) : Infinity;
  return {
    rackTravel: rt, limit, left, right, deltaLeft: dL, deltaRight: dR,
    inner, outer, ackermannIdeal: ackIdeal,
    ackermannErrorDeg: deg(inner - ackIdeal),
    turnRadius,
    minEfficiency: Math.min(left.efficiency, right.efficiency),
    singular: !left.valid || !right.valid || Math.min(left.efficiency, right.efficiency) < 0.08,
  };
}

// ── validity flags (surfaced as red badges on the diagrams) ──────────────────
export type FlagLevel = "warn" | "bad";
export interface Flag { level: FlagLevel; text: string }

export function vehicleFlags(g: VehicleGeom): Flag[] {
  const f: Flag[] = [];
  if (g.a <= 0 || g.a >= g.L) f.push({ level: "bad", text: "质心落在轴距之外（cg_to_front 应在 0…L 内）" });
  if (g.frontLoadFrac < 0.35 || g.frontLoadFrac > 0.65) f.push({ level: "warn", text: `轴荷分配偏极端（前 ${(g.frontLoadFrac * 100).toFixed(0)}%）` });
  if (g.iz > g.izRef * 1.8 || g.iz < g.izRef * 0.5) f.push({ level: "warn", text: `横摆惯量偏离 m·a·b 估算（${g.izRef.toFixed(0)}）较多，核对 inertia_z` });
  if (g.h > 0.9) f.push({ level: "warn", text: "质心偏高，侧倾/载荷转移会很大" });
  return f;
}

export function kingpinFlags(k: KingpinGeom): Flag[] {
  const f: Flag[] = [];
  if (k.scrub < -0.005) f.push({ level: "warn", text: "负 scrub（可能是有意设计，注意制动稳定性方向相反）" });
  if (Math.abs(k.scrub) > 0.06) f.push({ level: "warn", text: `scrub 偏大（${(k.scrub * 1000).toFixed(0)} mm），转向阻力矩敏感` });
  if (k.caster < 0) f.push({ level: "bad", text: "负主销后倾：回正力矩为负，直行不稳" });
  if (deg(k.kpi) > 18) f.push({ level: "warn", text: "主销内倾偏大" });
  return f;
}

export function axleFlags(a: AxleState): Flag[] {
  const f: Flag[] = [];
  if (a.singular) f.push({ level: "bad", text: "连杆不可达或接近奇异，请检查硬点与行程" });
  else if (a.minEfficiency < 0.25) f.push({ level: "warn", text: `连杆几何指标偏低（η=${a.minEfficiency.toFixed(2)}）` });
  if (Math.abs(a.ackermannErrorDeg) > 3) f.push({ level: "warn", text: `阿克曼误差 ${a.ackermannErrorDeg.toFixed(1)}°（内轮偏离理想值）` });
  return f;
}
