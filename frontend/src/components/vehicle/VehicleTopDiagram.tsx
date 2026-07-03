/** 核心整车俯视示意图 — 参数驱动 + 可拖拽建模。 */

import { useMemo, useRef } from "react";

import { useVehicleParams } from "@/components/vehicle/VehicleParamsContext";
import { DiagramCard, Flags, Readouts } from "@/components/vehicle/diagramUi";
import { Handle } from "@/components/vehicle/svgHandle";
import { MM, vehicleFlags, vehicleGeom } from "@/vehicle/geometryModel";

const VB_W = 440, VB_H = 380, PAD = 46;
const SIDE_W = 440, SIDE_H = 120;

export default function VehicleTopDiagram() {
  const { value, setValue } = useVehicleParams();
  const svgRef = useRef<SVGSVGElement>(null);
  const sideRef = useRef<SVGSVGElement>(null);

  // Read the live params through the shared editor.
  const p = useMemo(() => ({
    wheelbase: value("wheelbase"),
    track_front: value("track_front"),
    track_rear: value("track_rear"),
    cg_to_front: value("cg_to_front"),
    cg_height: value("cg_height"),
    mass: value("mass"),
    inertia_z: value("inertia_z"),
    steer_limit: value("steer_limit"),
    tire_radius: value("tire_radius"),
    tire_width: value("tire_width"),
  }), [value]);
  const g = vehicleGeom(p);

  // ── top-view scale: fit body + wheels + margin into the viewBox ──
  const halfW = Math.max(g.tf, g.tr) / 2 + g.tireW + 0.15;
  const halfH = g.L / 2 + g.tireR + 0.15;
  const s = Math.min((VB_W - 2 * PAD) / (2 * halfW), (VB_H - 2 * PAD) / (2 * halfH));
  const ox = VB_W / 2, oy = VB_H / 2;
  const P = (x: number, y: number) => ({ X: ox - y * s, Y: oy - x * s }); // +x up, +y left
  const invX = (Yscreen: number) => (oy - Yscreen) / s;                    // screen → body x
  const invY = (Xscreen: number) => (ox - Xscreen) / s;                    // screen → body y

  const wheelPos: [number, number, string][] = [
    [+g.L / 2, +g.tf / 2, "FL"], [+g.L / 2, -g.tf / 2, "FR"],
    [-g.L / 2, +g.tr / 2, "RL"], [-g.L / 2, -g.tr / 2, "RR"],
  ];
  const cgX = g.L / 2 - g.a;                     // body-x of CG
  const cg = P(cgX, 0);
  const wpx = g.tireW * s, hpx = 2 * g.tireR * s;

  // ── turning geometry (ideal Ackermann): ICR on +Y at R ──
  const R = g.minTurnRadius;
  const icrData = { x: 0, y: R };
  const icr = P(icrData.x, icrData.y);
  // clamp the drawn ICR marker to the viewBox edge if the true radius is large
  const icrDrawn = { X: Math.max(8, icr.X), Y: icr.Y };
  const arc = (() => {
    // small path arc of the turn circle passing near the front axle
    const a0 = P(g.L / 2, g.tf / 2), a1 = P(g.L / 2, -g.tf / 2);
    return `M ${a0.X.toFixed(1)} ${a0.Y.toFixed(1)} A ${(R * s).toFixed(1)} ${(R * s).toFixed(1)} 0 0 1 ${a1.X.toFixed(1)} ${a1.Y.toFixed(1)}`;
  })();

  // ── side elevation (CG height) ──
  const sHalfL = g.L / 2 + g.tireR + 0.2;
  const ss = (SIDE_W - 2 * 40) / (2 * sHalfL);
  const sox = SIDE_W / 2, groundY = SIDE_H - 26;
  const SP = (x: number, z: number) => ({ X: sox - x * ss, Y: groundY - z * ss });
  const cgSide = SP(cgX, g.tireR > 0 ? g.h : 0.6);

  const clampA = (a: number) => Math.max(0.05, Math.min(g.L - 0.05, a));

  return (
    <DiagramCard title="核心整车（俯视）" hint="拖动质心/轮/前轴改参数">
      <svg ref={svgRef} className="vg-svg" viewBox={`0 0 ${VB_W} ${VB_H}`} role="img" aria-label="整车俯视参数示意">
        {/* turning circle */}
        <path d={arc} className="vg-turn-arc" fill="none" />
        <line x1={ox} y1={oy} x2={icrDrawn.X} y2={icrDrawn.Y} className="vg-icr-line" />
        <circle cx={icrDrawn.X} cy={icrDrawn.Y} r="4" className="vg-icr-dot" />
        <text x={Math.min(icrDrawn.X + 6, VB_W - 70)} y={icrDrawn.Y + 4} className="vg-t-accent">ICR · R{R < 1e3 ? `=${R.toFixed(1)}m` : ""}</text>

        {/* body outline */}
        {(() => {
          const fl = P(g.L / 2, g.tf / 2 + 0.05), rr = P(-g.L / 2, -g.tf / 2 - 0.05);
          const w = Math.abs(rr.X - fl.X), h = Math.abs(rr.Y - fl.Y);
          return <rect x={Math.min(fl.X, rr.X)} y={Math.min(fl.Y, rr.Y)} width={w} height={h} rx={10} className="vg-body" />;
        })()}
        {/* axle lines */}
        <line {...axle(P(g.L / 2, g.tf / 2), P(g.L / 2, -g.tf / 2))} className="vg-axle" />
        <line {...axle(P(-g.L / 2, g.tr / 2), P(-g.L / 2, -g.tr / 2))} className="vg-axle" />
        {/* centreline */}
        <line {...axle(P(g.L / 2, 0), P(-g.L / 2, 0))} className="vg-centerline" />

        {/* wheels */}
        {wheelPos.map(([x, y, lbl]) => {
          const c = P(x, y);
          return (
            <g key={lbl}>
              <rect x={c.X - wpx / 2} y={c.Y - hpx / 2} width={wpx} height={hpx} rx={4} className="vg-wheel" />
              <text x={c.X} y={c.Y + 3} textAnchor="middle" className="vg-t-wheel">{lbl}</text>
            </g>
          );
        })}

        {/* wheelbase dim (right side) */}
        {dim(P(g.L / 2, -Math.max(g.tf, g.tr) / 2 - 0.1), P(-g.L / 2, -Math.max(g.tf, g.tr) / 2 - 0.1),
          `L ${(g.L * MM).toFixed(0)}`, "v")}
        {/* front track dim */}
        {dim(P(g.L / 2 + 0.12, g.tf / 2), P(g.L / 2 + 0.12, -g.tf / 2), `轮距 ${(g.tf * MM).toFixed(0)}`, "h")}

        {/* CG + load split */}
        <line x1={P(g.L / 2, 0).X} y1={P(g.L / 2, 0).Y} x2={cg.X} y2={cg.Y} className="vg-dim-line" />
        <text x={cg.X + 8} y={(P(g.L / 2, 0).Y + cg.Y) / 2} className="vg-t-muted">a {(g.a * MM).toFixed(0)}</text>
        <circle cx={cg.X} cy={cg.Y} r="7" className="vg-cg" />
        <text x={cg.X} y={cg.Y - 11} textAnchor="middle" className="vg-t-accent">CG</text>
        <text x={P(g.L / 2, 0).X + 10} y={P(g.L / 2 - 0.15, 0).Y} className="vg-t-load">前 {(g.frontLoadFrac * 100).toFixed(0)}%</text>
        <text x={P(-g.L / 2, 0).X + 10} y={P(-g.L / 2 + 0.2, 0).Y} className="vg-t-load">后 {(g.rearLoadFrac * 100).toFixed(0)}%</text>

        {/* draggable handles */}
        <Handle cx={cg.X} cy={cg.Y} svgRef={svgRef} label="拖动改质心前后位置"
          onDrag={(vb) => setValue("cg_to_front", clampA(g.L / 2 - invX(vb.y)))} />
        <Handle cx={P(g.L / 2, g.tf / 2).X} cy={P(g.L / 2, g.tf / 2).Y} svgRef={svgRef} r={5} label="拖动改前轮距"
          onDrag={(vb) => setValue("track_front", Math.max(0.6, Math.min(2.4, 2 * Math.abs(invY(vb.x)))))} />
        <Handle cx={P(-g.L / 2, g.tr / 2).X} cy={P(-g.L / 2, g.tr / 2).Y} svgRef={svgRef} r={5} label="拖动改后轮距"
          onDrag={(vb) => setValue("track_rear", Math.max(0.6, Math.min(2.4, 2 * Math.abs(invY(vb.x)))))} />
        <Handle cx={P(g.L / 2, 0).X} cy={P(g.L / 2, 0).Y} svgRef={svgRef} r={5} label="拖动改轴距"
          onDrag={(vb) => setValue("wheelbase", Math.max(1.8, Math.min(4.2, 2 * Math.max(invX(vb.y), 0.9))))} />
      </svg>

      {/* side elevation for CG height */}
      <svg ref={sideRef} className="vg-svg vg-svg-side" viewBox={`0 0 ${SIDE_W} ${SIDE_H}`} role="img" aria-label="整车侧视 质心高">
        <line x1={20} y1={groundY} x2={SIDE_W - 20} y2={groundY} className="vg-ground" />
        <text x={22} y={groundY + 16} className="vg-t-muted">地面（侧视）</text>
        {[[+1, "前"], [-1, "后"]].map(([sgn, lbl]) => {
          const c = SP((sgn as number) * g.L / 2, g.tireR);
          return <g key={lbl as string}>
            <circle cx={c.X} cy={c.Y} r={g.tireR * ss} className="vg-wheel-side" />
            <text x={c.X} y={groundY + 16} textAnchor="middle" className="vg-t-muted">{lbl}</text>
          </g>;
        })}
        <line x1={cgSide.X} y1={groundY} x2={cgSide.X} y2={cgSide.Y} className="vg-dim-line" />
        <text x={cgSide.X + 6} y={(groundY + cgSide.Y) / 2} className="vg-t-muted">h {(g.h * MM).toFixed(0)}</text>
        <circle cx={cgSide.X} cy={cgSide.Y} r="7" className="vg-cg" />
        <text x={cgSide.X} y={cgSide.Y - 11} textAnchor="middle" className="vg-t-accent">CG</text>
        <Handle cx={cgSide.X} cy={cgSide.Y} svgRef={sideRef} label="拖动改质心高度"
          onDrag={(vb) => setValue("cg_height", Math.max(0.2, Math.min(1.1, (groundY - vb.y) / ss)))} />
      </svg>

      <Readouts items={[
        ["轴距 L", `${(g.L * MM).toFixed(0)} mm`],
        ["前/后轴荷", `${(g.frontLoadFrac * 100).toFixed(0)} / ${(g.rearLoadFrac * 100).toFixed(0)} %`],
        ["质心高", `${(g.h * MM).toFixed(0)} mm`],
        ["阿克曼最小转弯半径", R < 1e3 ? `${R.toFixed(2)} m` : "—"],
        ["整备质量", `${g.mass.toFixed(0)} kg`],
        ["横摆惯量", `${g.iz.toFixed(0)} kg·m²`],
      ]} />
      <Flags flags={vehicleFlags(g)} />
    </DiagramCard>
  );
}

function axle(a: { X: number; Y: number }, b: { X: number; Y: number }) {
  return { x1: a.X, y1: a.Y, x2: b.X, y2: b.Y };
}
function dim(a: { X: number; Y: number }, b: { X: number; Y: number }, label: string, orient: "h" | "v") {
  const mid = { X: (a.X + b.X) / 2, Y: (a.Y + b.Y) / 2 };
  return (
    <g>
      <line x1={a.X} y1={a.Y} x2={b.X} y2={b.Y} className="vg-dim-line" />
      <text x={orient === "v" ? mid.X + 6 : mid.X} y={orient === "v" ? mid.Y : mid.Y - 5}
        textAnchor={orient === "v" ? "start" : "middle"} className="vg-t-dim">{label}</text>
    </g>
  );
}
