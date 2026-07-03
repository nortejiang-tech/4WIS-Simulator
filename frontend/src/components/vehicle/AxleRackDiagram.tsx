/** 齿条硬点（前桥俯视）— 4-bar 连杆求解 + 阿克曼 + 效率 + 奇异，可拖硬点。 */

import { useMemo, useRef, useState } from "react";

import { useVehicleParams } from "@/components/vehicle/VehicleParamsContext";
import { DiagramCard, Flags, Readouts } from "@/components/vehicle/diagramUi";
import { Handle } from "@/components/vehicle/svgHandle";
import {
  MM, Vec2, axleFlags, axleState, deg, getP, linkageBase, vAdd,
} from "@/vehicle/geometryModel";

const VB_W = 460, VB_H = 360, PAD = 40;

export default function AxleRackDiagram() {
  const { value, setValue } = useVehicleParams();
  const svgRef = useRef<SVGSVGElement>(null);
  const [rackMm, setRackMm] = useState(0);

  // Live params (any relevant edit re-renders).
  const p = useMemo(() => ({
    wheelbase: value("wheelbase"),
    track_front: value("track_front"),
    steering_geometry: {
      front_outer_x: value("steering_geometry.front_outer_x"),
      front_outer_y: value("steering_geometry.front_outer_y"),
      front_inner_x: value("steering_geometry.front_inner_x"),
      front_inner_y: value("steering_geometry.front_inner_y"),
      front_rack_axis_deg: value("steering_geometry.front_rack_axis_deg"),
      front_rack_travel_limit: value("steering_geometry.front_rack_travel_limit"),
    },
  }), [value]);

  const tf = getP(p, "track_front", 1.565);
  const tireR = value("tire_radius"), tireW = value("tire_width");
  const baseL = linkageBase(p, 0), baseR = linkageBase(p, 1);
  const limitMm = baseL.limit * MM;
  const rt = Math.max(-limitMm, Math.min(limitMm, rackMm)) / MM;
  const st = axleState(p, rt);

  const kpL: Vec2 = { x: 0, y: tf / 2 }, kpR: Vec2 = { x: 0, y: -tf / 2 };
  const gOuterL = vAdd(kpL, baseL.outer0), gInnerL = vAdd(kpL, baseL.inner0);
  const gOuterR = vAdd(kpR, baseR.outer0), gInnerR = vAdd(kpR, baseR.inner0);
  const lOuterL = vAdd(kpL, st.left.outer), lInnerL = vAdd(kpL, st.left.inner);
  const lOuterR = vAdd(kpR, st.right.outer), lInnerR = vAdd(kpR, st.right.inner);

  // ── fit scale ──
  const pts = [kpL, kpR, gOuterL, gInnerL, gOuterR, gInnerR, lOuterL, lInnerL, lOuterR, lInnerR,
    { x: 0.2, y: 0 }, { x: -0.4, y: 0 }];
  const xs = pts.map((q) => q.x), ys = pts.map((q) => q.y);
  const minX = Math.min(...xs) - 0.05, maxX = Math.max(...xs) + 0.05;
  const minY = Math.min(...ys) - 0.05, maxY = Math.max(...ys) + 0.05;
  const s = Math.min((VB_W - 2 * PAD) / (maxY - minY), (VB_H - 2 * PAD) / (maxX - minX));
  const ox = VB_W / 2 + ((minY + maxY) / 2) * s;      // y left → screen x = ox - y*s
  const oy = VB_H / 2 + ((minX + maxX) / 2) * s;      // x forward → screen y = oy - x*s
  const P = (q: Vec2) => ({ X: ox - q.y * s, Y: oy - q.x * s });
  const invY = (X: number) => (ox - X) / s;
  const invX = (Y: number) => (oy - Y) / s;

  const wheelRect = (kp: Vec2, delta: number, lbl: string) => {
    const c = P(kp);
    const w = tireW * s, h = 2 * tireR * s;
    return (
      <g key={lbl} transform={`rotate(${(-deg(delta)).toFixed(2)} ${c.X.toFixed(1)} ${c.Y.toFixed(1)})`}>
        <rect x={c.X - w / 2} y={c.Y - h / 2} width={w} height={h} rx={4} className="vg-wheel" />
        <text x={c.X} y={c.Y + 3} textAnchor="middle" className="vg-t-wheel">{lbl}</text>
      </g>
    );
  };
  const seg = (a: Vec2, b: Vec2, cls: string) => {
    const A = P(a), B = P(b);
    return <line x1={A.X} y1={A.Y} x2={B.X} y2={B.Y} className={cls} />;
  };

  return (
    <DiagramCard title="齿条硬点（前桥俯视）" hint="拖球头改硬点；滑杆推齿条看阿克曼">
      <svg ref={svgRef} className="vg-svg" viewBox={`0 0 ${VB_W} ${VB_H}`} role="img" aria-label="前桥齿条硬点连杆">
        {/* front arrow */}
        <line x1={26} y1={VB_H - 22} x2={26} y2={VB_H - 54} className="vg-axis" markerEnd="url(#vg-arrow)" />
        <text x={32} y={VB_H - 40} className="vg-t-muted">Front</text>
        <defs><marker id="vg-arrow" markerWidth="8" markerHeight="8" refX="4" refY="4" orient="auto">
          <path d="M0,0 L8,4 L0,8 z" className="vg-arrowhead" /></marker></defs>

        {/* centreline + axle */}
        {seg({ x: 0.15, y: 0 }, { x: -0.4, y: 0 }, "vg-centerline")}
        {seg(kpL, kpR, "vg-axle")}

        {/* rack bar (straight-ahead inner joints, extended along axis) */}
        {seg(vAdd(gInnerL, { x: baseL.axis.x * -(baseL.limit + 0.03), y: baseL.axis.y * -(baseL.limit + 0.03) }),
          vAdd(gInnerR, { x: baseR.axis.x * -(baseR.limit + 0.03), y: baseR.axis.y * -(baseR.limit + 0.03) }),
          "vg-rack")}

        {/* ghost (straight-ahead) linkage — editable definition */}
        {seg(kpL, gOuterL, "vg-arm-ghost")}{seg(gOuterL, gInnerL, "vg-tie-ghost")}
        {seg(kpR, gOuterR, "vg-arm-ghost")}{seg(gOuterR, gInnerR, "vg-tie-ghost")}

        {/* live (steered) linkage */}
        {seg(kpL, lOuterL, st.singular ? "vg-arm-bad" : "vg-arm")}{seg(lOuterL, lInnerL, "vg-tie")}
        {seg(kpR, lOuterR, st.singular ? "vg-arm-bad" : "vg-arm")}{seg(lOuterR, lInnerR, "vg-tie")}

        {wheelRect(kpL, st.deltaLeft, "FL")}
        {wheelRect(kpR, st.deltaRight, "FR")}

        {/* kingpin dots */}
        <circle {...dot(P(kpL))} className="vg-kingpin-dot" /><circle {...dot(P(kpR))} className="vg-kingpin-dot" />
        {/* live joint dots */}
        <circle {...dot(P(lInnerL))} className="vg-joint" /><circle {...dot(P(lInnerR))} className="vg-joint" />

        {/* editable ghost handles (left side writes the shared front_* hardpoints) */}
        <Handle {...dot(P(gOuterL))} svgRef={svgRef} r={5} label="拖动改外球头（转向臂）"
          onDrag={(vb) => {
            setValue("steering_geometry.front_outer_x", clamp(invX(vb.y) - kpL.x, -0.4, 0.1));
            setValue("steering_geometry.front_outer_y", (invY(vb.x) - kpL.y));
          }} />
        <Handle {...dot(P(gInnerL))} svgRef={svgRef} r={5} label="拖动改内球头（齿条端）"
          onDrag={(vb) => {
            setValue("steering_geometry.front_inner_x", clamp(invX(vb.y) - kpL.x, -0.5, 0.1));
            setValue("steering_geometry.front_inner_y", (invY(vb.x) - kpL.y));
          }} />

        <text x={12} y={20} className={st.singular ? "vg-t-bad" : "vg-t-accent"}>
          δL {deg(st.deltaLeft).toFixed(1)}° · δR {deg(st.deltaRight).toFixed(1)}° · η {st.minEfficiency.toFixed(2)}
        </text>
      </svg>

      <div className="vg-slider">
        <span>齿条位移</span>
        <input type="range" min={-limitMm} max={limitMm} step={0.5} value={rackMm}
          onChange={(e) => setRackMm(Number(e.target.value))} />
        <b>{rt >= 0 ? "+" : ""}{(rt * MM).toFixed(1)} mm</b>
      </div>

      <Readouts items={[
        ["内轮 / 外轮转角", `${deg(st.inner).toFixed(1)} / ${deg(st.outer).toFixed(1)} °`],
        ["理想阿克曼内轮角", `${deg(st.ackermannIdeal).toFixed(1)} °`],
        ["阿克曼误差", `${st.ackermannErrorDeg.toFixed(2)} °`],
        ["转弯半径", Number.isFinite(st.turnRadius) ? `${st.turnRadius.toFixed(2)} m` : "直行"],
        ["连杆效率 η", st.minEfficiency.toFixed(3)],
        ["前齿条半行程", `${limitMm.toFixed(0)} mm`],
      ]} />
      <Flags flags={axleFlags(st)} />
    </DiagramCard>
  );
}

function dot(p: { X: number; Y: number }) { return { cx: p.X, cy: p.Y, r: 4 }; }
function clamp(v: number, lo: number, hi: number) { return Math.max(lo, Math.min(hi, v)); }
