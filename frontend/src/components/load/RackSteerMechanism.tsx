import { useEffect, useMemo, useState } from "react";

import { WHEEL_LABELS } from "@/types/sim";
import { deg, fmt, MM, rad, type VehicleParams } from "./types";

type Vec2 = { x: number; y: number };

interface LinkageBase {
  outer0: Vec2;
  inner0: Vec2;
  axis: Vec2;
  armLength: number;
  tieLength: number;
  theta0: number;
  limit: number;
  label: string;
}

interface LinkageSolution {
  valid: boolean;
  rackTravel: number;
  delta: number;
  outer: Vec2;
  inner: Vec2;
  armTieAngle: number;
  tieRackAngle: number;
  efficiency: number;
}

const vAdd = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x + b.x, y: a.y + b.y });
const vSub = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x - b.x, y: a.y - b.y });
const vMul = (a: Vec2, k: number): Vec2 => ({ x: a.x * k, y: a.y * k });
const vDot = (a: Vec2, b: Vec2) => a.x * b.x + a.y * b.y;
const vCross = (a: Vec2, b: Vec2) => a.x * b.y - a.y * b.x;
const vLen = (a: Vec2) => Math.hypot(a.x, a.y);
const vNorm = (a: Vec2): Vec2 => {
  const len = Math.max(vLen(a), 1e-12);
  return { x: a.x / len, y: a.y / len };
};
const wrapPi = (value: number) => {
  let a = (value + Math.PI) % (2 * Math.PI);
  if (a < 0) a += 2 * Math.PI;
  return a - Math.PI;
};

function geometryValue(geom: Record<string, number>, key: string, fallback: number): number {
  const value = Number(geom[key]);
  return Number.isFinite(value) ? value : fallback;
}

function getLinkageBase(params: VehicleParams | null, wheelIndex: number): LinkageBase {
  const geom = params?.steering_geometry ?? {};
  const isFront = wheelIndex < 2;
  const isLeft = wheelIndex === 0 || wheelIndex === 2;
  const prefix = isFront ? "front" : "rear";
  const mirror = isLeft ? 1 : -1;
  const fallbackOuterX = isFront ? -0.145971235 : 0.145971235;
  const fallbackInnerX = isFront ? -0.176324 : 0.176324;
  const outer0 = {
    x: geometryValue(geom, `${prefix}_outer_x`, fallbackOuterX),
    y: mirror * geometryValue(geom, `${prefix}_outer_y`, -0.011844575),
  };
  const inner0 = {
    x: geometryValue(geom, `${prefix}_inner_x`, fallbackInnerX),
    y: mirror * geometryValue(geom, `${prefix}_inner_y`, -0.362586),
  };
  const axisDeg = geometryValue(geom, `${prefix}_rack_axis_deg`, 90);
  const axis = vNorm({ x: Math.cos(rad(axisDeg)), y: mirror * Math.sin(rad(axisDeg)) });
  return {
    outer0,
    inner0,
    axis,
    armLength: Math.max(vLen(outer0), 1e-6),
    tieLength: Math.max(vLen(vSub(inner0, outer0)), 1e-6),
    theta0: Math.atan2(outer0.y, outer0.x),
    limit: Math.max(geometryValue(geom, `${prefix}_rack_travel_limit`, 0.085), 0.001),
    label: WHEEL_LABELS[wheelIndex] ?? "FL",
  };
}

function solveRackToWheel(base: LinkageBase, rackTravel: number, hintDelta = 0): LinkageSolution {
  const inner = vAdd(base.inner0, vMul(base.axis, rackTravel));
  const d = vLen(inner);
  const invalid = (delta = hintDelta): LinkageSolution => ({
    valid: false,
    rackTravel,
    delta,
    outer: base.outer0,
    inner,
    armTieAngle: 0,
    tieRackAngle: 0,
    efficiency: 0,
  });
  if (d < 1e-9) return invalid();

  const r0 = base.armLength;
  const r1 = base.tieLength;
  const a = (r0 * r0 - r1 * r1 + d * d) / (2 * d);
  const h2 = r0 * r0 - a * a;
  if (h2 < -1e-9) return invalid();
  const h = Math.sqrt(Math.max(h2, 0));
  const along = vMul(inner, a / d);
  const perp = { x: -inner.y / d, y: inner.x / d };
  const candidates = [vAdd(along, vMul(perp, h)), vSub(along, vMul(perp, h))];
  let outer = candidates[0];
  let bestDelta = wrapPi(Math.atan2(outer.y, outer.x) - base.theta0);
  let bestScore = Math.abs(wrapPi(bestDelta - hintDelta));
  for (const candidate of candidates.slice(1)) {
    const delta = wrapPi(Math.atan2(candidate.y, candidate.x) - base.theta0);
    const score = Math.abs(wrapPi(delta - hintDelta));
    if (score < bestScore) {
      outer = candidate;
      bestDelta = delta;
      bestScore = score;
    }
  }

  const arm = vNorm(outer);
  const tie = vNorm(vSub(inner, outer));
  const armTieAngle = Math.acos(Math.max(-1, Math.min(1, vDot(arm, tie))));
  const rackProjection = Math.abs(vDot(tie, base.axis));
  const tieRackAngle = Math.acos(Math.max(0, Math.min(1, rackProjection)));
  const efficiency = Math.max(Math.abs(vCross(arm, tie)) * rackProjection, 1e-6);
  return { valid: true, rackTravel, delta: bestDelta, outer, inner, armTieAngle, tieRackAngle, efficiency };
}

function buildRackTravelProfile(base: LinkageBase, halfSteps = 70): LinkageSolution[] {
  const zero = solveRackToWheel(base, 0, 0);
  const neg: LinkageSolution[] = [];
  const pos: LinkageSolution[] = [];
  let hint = zero.delta;
  for (let i = 1; i <= halfSteps; i++) {
    const sol = solveRackToWheel(base, -base.limit * i / halfSteps, hint);
    if (sol.valid) hint = sol.delta;
    neg.push(sol);
  }
  hint = zero.delta;
  for (let i = 1; i <= halfSteps; i++) {
    const sol = solveRackToWheel(base, base.limit * i / halfSteps, hint);
    if (sol.valid) hint = sol.delta;
    pos.push(sol);
  }
  return [...neg.reverse(), zero, ...pos];
}

function makeScaler(points: Vec2[], width: number, height: number, pad: number) {
  const xs = points.map((p) => p.x);
  const ys = points.map((p) => p.y);
  let minX = Math.min(...xs);
  let maxX = Math.max(...xs);
  let minY = Math.min(...ys);
  let maxY = Math.max(...ys);
  if (maxX - minX < 1e-6) {
    minX -= 0.1;
    maxX += 0.1;
  }
  if (maxY - minY < 1e-6) {
    minY -= 0.1;
    maxY += 0.1;
  }
  const scale = Math.min((width - pad * 2) / (maxX - minX), (height - pad * 2) / (maxY - minY));
  return (p: Vec2): Vec2 => ({
    x: pad + (p.x - minX) * scale,
    y: height - pad - (p.y - minY) * scale,
  });
}

function toForwardUpPoint(p: Vec2): Vec2 {
  return { x: p.y, y: p.x };
}

export function RackSteerMechanism({
  params,
  wheelIndex,
}: {
  params: VehicleParams | null;
  wheelIndex: number;
}) {
  const base = useMemo(() => getLinkageBase(params, wheelIndex), [params, wheelIndex]);
  const [rackTravelMm, setRackTravelMm] = useState(0);
  const limitMm = base.limit * MM;

  useEffect(() => {
    setRackTravelMm((value) => Math.max(-limitMm, Math.min(limitMm, value)));
  }, [limitMm]);

  const profile = useMemo(() => buildRackTravelProfile(base), [base]);
  const current = useMemo(() => {
    const nearest = profile.reduce((best, item) => (
      Math.abs(item.rackTravel * MM - rackTravelMm) < Math.abs(best.rackTravel * MM - rackTravelMm)
        ? item
        : best
    ), profile[Math.floor(profile.length / 2)] ?? solveRackToWheel(base, 0, 0));
    return solveRackToWheel(base, rackTravelMm / MM, nearest.delta);
  }, [base, profile, rackTravelMm]);

  const validProfile = profile.filter((p) => p.valid);
  const rackStart = vAdd(base.inner0, vMul(base.axis, -base.limit - 0.04));
  const rackEnd = vAdd(base.inner0, vMul(base.axis, base.limit + 0.04));
  const mechWidth = 380;
  const mechHeight = 220;
  const scalePoint = makeScaler([
    { x: 0, y: 0 },
    base.outer0,
    base.inner0,
    rackStart,
    rackEnd,
    ...validProfile.flatMap((p) => [p.outer, p.inner]),
  ].map(toForwardUpPoint), mechWidth, mechHeight, 24);
  const project = (p: Vec2) => scalePoint(toForwardUpPoint(p));
  const k = project({ x: 0, y: 0 });
  const o0 = project(base.outer0);
  const i0 = project(base.inner0);
  const o = project(current.outer);
  const i = project(current.inner);
  const rs = project(rackStart);
  const re = project(rackEnd);

  const chartWidth = 380;
  const chartHeight = 220;
  const chartPad = { left: 44, right: 14, top: 18, bottom: 34 };
  const angleValues = validProfile.map((p) => deg(p.delta));
  let minAngle = Math.min(...angleValues, deg(current.delta), -1);
  let maxAngle = Math.max(...angleValues, deg(current.delta), 1);
  if (maxAngle - minAngle < 2) {
    minAngle -= 1;
    maxAngle += 1;
  }
  const xScale = (travelMm: number) => {
    const x0 = -limitMm;
    const x1 = limitMm;
    return chartPad.left + ((travelMm - x0) / Math.max(x1 - x0, 1e-9)) * (chartWidth - chartPad.left - chartPad.right);
  };
  const yScale = (angleDeg: number) => chartHeight - chartPad.bottom
    - ((angleDeg - minAngle) / Math.max(maxAngle - minAngle, 1e-9)) * (chartHeight - chartPad.top - chartPad.bottom);
  const curvePath = validProfile
    .map((p, index) => `${index === 0 ? "M" : "L"} ${xScale(p.rackTravel * MM).toFixed(2)} ${yScale(deg(p.delta)).toFixed(2)}`)
    .join(" ");
  const currentX = xScale(rackTravelMm);
  const currentY = yScale(deg(current.delta));

  return (
    <div className="load-linkage-panel">
      <div className="load-linkage-controls">
        <span>{base.label}</span>
        <label>
          齿条位移
          <input
            type="range"
            min={-limitMm}
            max={limitMm}
            step="0.5"
            value={rackTravelMm}
            onChange={(e) => setRackTravelMm(Number(e.target.value))}
          />
        </label>
        <b>{fmt(rackTravelMm, 1)} mm / δ {current.valid ? `${fmt(deg(current.delta), 2)}°` : "不可达"}</b>
      </div>
      <div className="load-linkage-views">
        <svg className="load-linkage-svg" viewBox={`0 0 ${mechWidth} ${mechHeight}`} role="img" aria-label="转向节臂与拉杆硬点示意">
          <line x1="28" y1={mechHeight - 24} x2="28" y2={mechHeight - 58} className="front-axis-line" />
          <path d={`M 22 ${mechHeight - 56} L 28 ${mechHeight - 66} L 34 ${mechHeight - 56}`} className="front-axis-head" />
          <text x="38" y={mechHeight - 57}>Front</text>
          <line x1={rs.x} y1={rs.y} x2={re.x} y2={re.y} className="rack-line" />
          <line x1={i0.x} y1={i0.y} x2={i.x} y2={i.y} className="rack-travel-line" />
          <line x1={k.x} y1={k.y} x2={o0.x} y2={o0.y} className="arm-zero-line" />
          <line x1={k.x} y1={k.y} x2={o.x} y2={o.y} className="arm-line" />
          <line x1={o.x} y1={o.y} x2={i.x} y2={i.y} className="tie-line" />
          <circle cx={k.x} cy={k.y} r="5" className="kingpin-dot" />
          <circle cx={o0.x} cy={o0.y} r="3" className="outer-zero-dot" />
          <circle cx={o.x} cy={o.y} r="4" className="outer-dot" />
          <circle cx={i0.x} cy={i0.y} r="3" className="inner-zero-dot" />
          <circle cx={i.x} cy={i.y} r="4" className="inner-dot" />
          <text x="14" y="22">Arm-tie {fmt(deg(current.armTieAngle), 1)}° · η {fmt(current.efficiency, 3)}</text>
          <text x="14" y="40">Tie-rack {fmt(deg(current.tieRackAngle), 1)}°</text>
        </svg>
        <svg className="load-linkage-svg" viewBox={`0 0 ${chartWidth} ${chartHeight}`} role="img" aria-label="齿条位移和车轮转角关系曲线">
          <line x1={chartPad.left} y1={yScale(0)} x2={chartWidth - chartPad.right} y2={yScale(0)} className="chart-zero-line" />
          <line x1={xScale(0)} y1={chartPad.top} x2={xScale(0)} y2={chartHeight - chartPad.bottom} className="chart-zero-line" />
          <path d={curvePath} className="rack-angle-curve" fill="none" />
          <line x1={currentX} y1={chartPad.top} x2={currentX} y2={chartHeight - chartPad.bottom} className="chart-current-line" />
          <circle cx={currentX} cy={currentY} r="4" className="curve-current-dot" />
          <text x={chartPad.left} y="16">Rack travel → wheel angle</text>
          <text x={chartPad.left} y={chartHeight - 10}>-{fmt(limitMm, 0)} mm</text>
          <text x={chartWidth - chartPad.right - 58} y={chartHeight - 10}>+{fmt(limitMm, 0)} mm</text>
          <text x="6" y={chartPad.top + 4}>{fmt(maxAngle, 1)}°</text>
          <text x="6" y={chartHeight - chartPad.bottom}>{fmt(minAngle, 1)}°</text>
        </svg>
      </div>
    </div>
  );
}
