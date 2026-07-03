/** 核心主销 / 车轮示意图 — 正视(内倾·外倾·scrub) + 侧视(后倾·拖距)，可拖建模。 */

import { useMemo, useRef } from "react";

import { useVehicleParams } from "@/components/vehicle/VehicleParamsContext";
import { DiagramCard, Flags, Readouts } from "@/components/vehicle/diagramUi";
import { Handle } from "@/components/vehicle/svgHandle";
import { MM, deg, kingpinFlags, kingpinGeom, rad } from "@/vehicle/geometryModel";

const W = 320, H = 300;

export default function WheelKingpinDiagram() {
  const { value, setValue } = useVehicleParams();
  const frontRef = useRef<SVGSVGElement>(null);
  const sideRef = useRef<SVGSVGElement>(null);

  const p = useMemo(() => ({
    "suspension.caster_angle": value("suspension.caster_angle"),
    "suspension.kingpin_inclination": value("suspension.kingpin_inclination"),
    "suspension.camber": value("suspension.camber"),
    "suspension.scrub_radius": value("suspension.scrub_radius"),
    tire_radius: value("tire_radius"),
    tire_width: value("tire_width"),
    tire_t_pneumatic: value("tire_t_pneumatic"),
  }), [value]);
  const k = kingpinGeom({
    suspension: {
      caster_angle: p["suspension.caster_angle"],
      kingpin_inclination: p["suspension.kingpin_inclination"],
      camber: p["suspension.camber"],
      scrub_radius: p["suspension.scrub_radius"],
    },
    tire_radius: p.tire_radius, tire_width: p.tire_width, tire_t_pneumatic: p.tire_t_pneumatic,
  });

  const R = k.tireR, wW = k.tireW;
  const sc = 175 / (2 * R + 0.14);          // px per metre, shared by both views
  const cx = W / 2, groundY = H - 46;
  const rot = (x: number, y: number, ang: number) =>
    ({ x: x * Math.cos(ang) - y * Math.sin(ang), y: x * Math.sin(ang) + y * Math.cos(ang) });

  // ── FRONT VIEW (looking along +X): +x = outboard(right), y = up ──
  const F = (x: number, y: number) => ({ X: cx + x * sc, Y: groundY - y * sc });
  const Hk = 2 * R * 1.12;                    // kingpin axis draw length
  // wheel cross-section corners tilted by camber about the contact (0,0)
  const camber = k.camber;                    // +γ tilts top outboard
  const wc = [[-wW / 2, 0], [wW / 2, 0], [wW / 2, 2 * R], [-wW / 2, 2 * R]]
    .map(([x, y]) => rot(x, y, camber)).map(({ x, y }) => F(x, y));
  const wheelTop = rot(0, 2 * R, camber);     // camber handle
  const kpGround = { x: k.scrub, y: 0 };      // kingpin axis ground point
  const kpTop = { x: k.scrub + Hk * Math.sin(k.kpi), y: Hk * Math.cos(k.kpi) };
  const fContact = F(0, 0), fKpG = F(kpGround.x, kpGround.y), fKpT = F(kpTop.x, kpTop.y), fWT = F(wheelTop.x, wheelTop.y);

  // ── SIDE VIEW (looking along +Y): +x = forward(right), y = up ──

  const S = (x: number, y: number) => ({ X: cx - x * sc, Y: groundY - y * sc }); // +x forward → left
  const caster = k.caster;                    // +ε leans axis back → ground point ahead
  const tMech = k.mechTrail;                  // R·tanε ahead of contact
  const sKpG = { x: tMech, y: 0 };
  const sKpT = { x: tMech - Hk * Math.sin(caster), y: Hk * Math.cos(caster) };
  const sContact = S(0, 0), sKpGp = S(sKpG.x, sKpG.y), sKpTp = S(sKpT.x, sKpT.y);
  const tPneuP = S(-k.tPneu, 0);              // pneumatic trail behind contact

  const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

  return (
    <DiagramCard title="核心主销 / 车轮" hint="拖主销轴顶改内倾·后倾，拖轮顶改外倾">
      <div className="vg-two">
        {/* FRONT VIEW */}
        <svg ref={frontRef} className="vg-svg" viewBox={`0 0 ${W} ${H}`} role="img" aria-label="主销正视 内倾外倾scrub">
          <text x={10} y={18} className="vg-t-muted">正视 · 车内 ← → 车外</text>
          <line x1={16} y1={groundY} x2={W - 16} y2={groundY} className="vg-ground" />
          {/* vertical reference */}
          <line x1={fKpG.X} y1={groundY} x2={fKpG.X} y2={groundY - Hk * sc} className="vg-axis-dash" />
          {/* wheel */}
          <polygon points={wc.map((c) => `${c.X.toFixed(1)},${c.Y.toFixed(1)}`).join(" ")} className="vg-wheel-x" />
          {/* kingpin axis */}
          <line x1={fKpG.X} y1={fKpG.Y} x2={fKpT.X} y2={fKpT.Y} className="vg-kingpin" />
          {/* scrub */}
          <line x1={fContact.X} y1={groundY + 10} x2={fKpG.X} y2={groundY + 10} className="vg-dim-line" />
          <circle cx={fContact.X} cy={fContact.Y} r="3.5" className="vg-dot" />
          <text x={(fContact.X + fKpG.X) / 2} y={groundY + 26} textAnchor="middle" className="vg-t-dim">
            scrub {(k.scrub * MM).toFixed(0)}
          </text>
          <text x={fKpT.X + 4} y={fKpT.Y - 4} className="vg-t-accent">内倾 {deg(k.kpi).toFixed(1)}°</text>
          <text x={fWT.X + 6} y={fWT.Y + 4} className="vg-t-accent">外倾 {deg(k.camber).toFixed(2)}°</text>

          <Handle cx={fKpT.X} cy={fKpT.Y} svgRef={frontRef} label="拖动改主销内倾"
            onDrag={(vb) => {
              const dx = (vb.x - fKpG.X), dy = (groundY - vb.y);
              setValue("suspension.kingpin_inclination", clamp(Math.atan2(dx, Math.max(dy, 1)), 0, rad(25)));
            }} />
          <Handle cx={fWT.X} cy={fWT.Y} svgRef={frontRef} r={5} label="拖动改车轮外倾"
            onDrag={(vb) => {
              const dx = (vb.x - fContact.X), dy = (groundY - vb.y);
              setValue("suspension.camber", clamp(Math.atan2(dx, Math.max(dy, 1)), rad(-6), rad(6)));
            }} />
          <Handle cx={fKpG.X} cy={groundY} svgRef={frontRef} r={5} label="拖动改主销偏置 scrub"
            onDrag={(vb) => setValue("suspension.scrub_radius", clamp((vb.x - fContact.X) / sc, -0.03, 0.09))} />
        </svg>

        {/* SIDE VIEW */}
        <svg ref={sideRef} className="vg-svg" viewBox={`0 0 ${W} ${H}`} role="img" aria-label="主销侧视 后倾拖距">
          <text x={10} y={18} className="vg-t-muted">侧视 · 车后 ← → 车前</text>
          <line x1={16} y1={groundY} x2={W - 16} y2={groundY} className="vg-ground" />
          <line x1={sKpGp.X} y1={groundY} x2={sKpGp.X} y2={groundY - Hk * sc} className="vg-axis-dash" />
          <circle cx={sContact.X} cy={sContact.Y - k.tireR * sc} r={k.tireR * sc} className="vg-wheel-side" />
          <line x1={sKpGp.X} y1={sKpGp.Y} x2={sKpTp.X} y2={sKpTp.Y} className="vg-kingpin" />
          {/* mechanical trail */}
          <line x1={sContact.X} y1={groundY + 10} x2={sKpGp.X} y2={groundY + 10} className="vg-dim-line" />
          <text x={(sContact.X + sKpGp.X) / 2} y={groundY + 26} textAnchor="middle" className="vg-t-dim">
            t_m {(k.mechTrail * MM).toFixed(0)}
          </text>
          {/* pneumatic trail */}
          <line x1={sContact.X} y1={groundY + 24} x2={tPneuP.X} y2={groundY + 24} className="vg-dim-line2" />
          <text x={(sContact.X + tPneuP.X) / 2} y={groundY + 40} textAnchor="middle" className="vg-t-muted">
            t_p {(k.tPneu * MM).toFixed(0)}
          </text>
          <circle cx={sContact.X} cy={sContact.Y} r="3.5" className="vg-dot" />
          <text x={sKpTp.X - 8} y={sKpTp.Y - 4} textAnchor="end" className="vg-t-accent">后倾 {deg(k.caster).toFixed(1)}°</text>

          <Handle cx={sKpTp.X} cy={sKpTp.Y} svgRef={sideRef} label="拖动改主销后倾"
            onDrag={(vb) => {
              const dx = (sKpGp.X - vb.x), dy = (groundY - vb.y);   // forward is −screenX
              setValue("suspension.caster_angle", clamp(Math.atan2(dx, Math.max(dy, 1)), rad(-3), rad(14)));
            }} />
        </svg>
      </div>

      <Readouts items={[
        ["主销后倾 ε", `${deg(k.caster).toFixed(2)}°`],
        ["主销内倾", `${deg(k.kpi).toFixed(2)}°`],
        ["外倾 γ", `${deg(k.camber).toFixed(2)}°`],
        ["主销偏置 scrub", `${(k.scrub * MM).toFixed(1)} mm`],
        ["机械拖距 R·tanε", `${(k.mechTrail * MM).toFixed(1)} mm`],
        ["气胎拖距 t_p", `${(k.tPneu * MM).toFixed(1)} mm`],
        ["总拖距", `${(k.totalTrail * MM).toFixed(1)} mm`],
        ["轮径 R / 胎宽", `${(k.tireR * MM).toFixed(0)} / ${(k.tireW * MM).toFixed(0)} mm`],
      ]} />
      <Flags flags={kingpinFlags(k)} />
    </DiagramCard>
  );
}
