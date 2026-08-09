/**
 * GripPanel — per-wheel friction budget and the vehicle g-g envelope.
 *
 * Four friction circles laid out in wheel-plan order (FL FR / RL RR), each with
 * the current operating point and a short trail, plus an a_x–a_y plot of the
 * whole vehicle against its μ·g envelope.
 *
 * Two deliberate choices, both of which carry information a plain circle does
 * not:
 *
 *   1. **The circle radius is scaled by μ·Fz, not normalised.** A normalised
 *      circle always looks the same size, which hides the thing that actually
 *      decides whether a wheel lets go — load transfer. Here the inner wheels
 *      visibly shrink as you turn in and the front pair swell under braking,
 *      so the Z axis of "what is this wheel doing in X, Y and Z" is legible
 *      without a separate readout.
 *
 *   2. **Utilisation is always paired with a beyond-peak marker.** Two wheels
 *      can both read 95% utilisation with opposite meanings: one climbing the
 *      tyre curve and controllable, one over the peak and departing. The dot
 *      turns into a ring once the slip is past the peak, so the difference is
 *      visible at a glance rather than inferred from a number.
 *
 * Everything is derived from the state frame — no local physics.
 */

import { useEffect, useRef, useState } from "react";
import Panel from "@/components/Panel";
import { useSimStore } from "@/store/sim";
import type { SimStateMessage, WheelState } from "@/types/sim";

const WHEEL_LABELS = ["FL", "FR", "RL", "RR"] as const;
const TRAIL_LEN = 40;          // ~0.7 s of history at the 60 Hz push rate
const CIRCLE_PX = 104;         // drawing box per wheel

/** Colour ramp for remaining grip: comfortable → working → out. */
function marginColor(util: number, beyond: boolean): string {
  if (beyond) return "#f87171";
  if (util >= 0.9) return "#fb923c";
  if (util >= 0.7) return "#fbbf24";
  return "#4ade80";
}

/** The largest capacity across the four wheels — the common scale for all
 *  four circles, so their relative sizes mean something. */
function refCapacity(wheels: WheelState[]): number {
  let max = 0;
  for (const w of wheels) max = Math.max(max, w.grip_capacity ?? 0);
  return max > 1 ? max : 1;
}

function FrictionCircle({
  wheel, label, ref_cap, trail,
}: {
  wheel: WheelState;
  label: string;
  ref_cap: number;
  trail: Array<[number, number]>;
}) {
  const cap = wheel.grip_capacity ?? 0;
  const fx = wheel.tire_fx ?? 0;
  const fy = wheel.tire_fy ?? 0;
  const util = wheel.grip_util ?? 0;
  const beyond = !!(wheel.grip_beyond_peak_lat || wheel.grip_beyond_peak_long);

  const R = CIRCLE_PX / 2 - 8;          // px radius for the LARGEST capacity
  const r = R * (ref_cap > 0 ? cap / ref_cap : 0);   // this wheel's circle
  const c = CIRCLE_PX / 2;
  // Force → px, using the same scale the circle was drawn with, so the dot
  // sits on the rim exactly when the tyre saturates.
  const k = cap > 1 ? r / cap : 0;
  // Screen axes: +Fy (wheel's left) draws left, +Fx (forward) draws up.
  const px = c - fy * k;
  const py = c - fx * k;
  const col = marginColor(util, beyond);

  return (
    <div style={{ display: "flex", flexDirection: "column", alignItems: "center" }}>
      <svg width={CIRCLE_PX} height={CIRCLE_PX} role="img"
           aria-label={`${label} 附着圆 利用率 ${(util * 100).toFixed(0)}%`}>
        {/* full-capacity reference (the biggest loaded wheel right now) */}
        <circle cx={c} cy={c} r={R} fill="none" stroke="var(--border)"
                strokeDasharray="2 3" strokeWidth={1} opacity={0.5} />
        {/* this wheel's actual capacity — shrinks/grows with load transfer */}
        <circle cx={c} cy={c} r={r} fill={col} fillOpacity={0.07}
                stroke={col} strokeWidth={1.4} />
        <line x1={c} y1={c - r} x2={c} y2={c + r} stroke="var(--border)" strokeWidth={0.6} />
        <line x1={c - r} y1={c} x2={c + r} y2={c} stroke="var(--border)" strokeWidth={0.6} />
        {trail.length > 1 && (
          <polyline
            points={trail.map(([tx, ty]) => `${c - ty * k},${c - tx * k}`).join(" ")}
            fill="none" stroke={col} strokeWidth={1} opacity={0.35} />
        )}
        {beyond ? (
          <circle cx={px} cy={py} r={5} fill="none" stroke={col} strokeWidth={2.2} />
        ) : (
          <circle cx={px} cy={py} r={4} fill={col} />
        )}
      </svg>
      <div className="panel-small" style={{ display: "flex", gap: 6, alignItems: "baseline" }}>
        <span style={{ color: "var(--muted)" }}>{label}</span>
        <span className="hud-mono" style={{ color: col, fontWeight: 600 }}>
          {(util * 100).toFixed(0)}%
        </span>
        {beyond && <span className="hud-mono" style={{ color: "#f87171" }}>过峰</span>}
      </div>
      <div className="panel-small hud-mono" style={{ color: "var(--muted)" }}>
        余 {Math.round(wheel.grip_margin_lat ?? 0)}N 侧 · {Math.round(wheel.grip_margin_long ?? 0)}N 纵
      </div>
      <div className="panel-small hud-mono" style={{ color: "var(--muted)" }}>
        Fz {Math.round(wheel.fz ?? 0)}N
      </div>
    </div>
  );
}

function GgDiagram({ state, trail }: { state: SimStateMessage; trail: Array<[number, number]> }) {
  const size = 150;
  const c = size / 2;
  const R = c - 10;
  const env = state.accel?.envelope ?? 0;
  const ax = state.accel?.ax ?? 0;
  const ay = state.accel?.ay ?? 0;
  const k = env > 0.01 ? R / env : 0;
  const mag = Math.hypot(ax, ay);
  const frac = env > 0.01 ? mag / env : 0;
  const col = marginColor(frac, false);

  return (
    <div style={{ display: "flex", flexDirection: "column", alignItems: "center" }}>
      <svg width={size} height={size} role="img" aria-label="g-g 图">
        <circle cx={c} cy={c} r={R} fill="none" stroke={col} strokeOpacity={0.5}
                strokeWidth={1.4} />
        <circle cx={c} cy={c} r={R * 0.5} fill="none" stroke="var(--border)"
                strokeDasharray="2 3" strokeWidth={0.8} opacity={0.6} />
        <line x1={c} y1={c - R} x2={c} y2={c + R} stroke="var(--border)" strokeWidth={0.6} />
        <line x1={c - R} y1={c} x2={c + R} y2={c} stroke="var(--border)" strokeWidth={0.6} />
        {trail.length > 1 && (
          <polyline points={trail.map(([tx, ty]) => `${c - ty * k},${c - tx * k}`).join(" ")}
                    fill="none" stroke={col} strokeWidth={1} opacity={0.4} />
        )}
        <circle cx={c - ay * k} cy={c - ax * k} r={4} fill={col} />
      </svg>
      <div className="panel-small hud-mono" style={{ color: "var(--muted)" }}>
        aₓ {ax.toFixed(2)} · a_y {ay.toFixed(2)} m/s²
      </div>
      <div className="panel-small hud-mono" style={{ color: "var(--muted)" }}>
        包络 μg = {env.toFixed(2)} · 用了 {(frac * 100).toFixed(0)}%
      </div>
    </div>
  );
}

export default function GripPanel() {
  const state = useSimStore((s) => s.state);
  // Trails are refs, not state: they update at the push rate and must not
  // re-render the whole tree on every frame.
  const wheelTrails = useRef<Array<Array<[number, number]>>>([[], [], [], []]);
  const ggTrail = useRef<Array<[number, number]>>([]);
  const [, bump] = useState(0);

  useEffect(() => {
    const id = window.setInterval(() => {
      const st = useSimStore.getState().state;
      if (st) {
        st.wheels.forEach((w, i) => {
          const t = wheelTrails.current[i];
          t.push([w.tire_fx ?? 0, w.tire_fy ?? 0]);
          if (t.length > TRAIL_LEN) t.shift();
        });
        if (st.accel) {
          ggTrail.current.push([st.accel.ax, st.accel.ay]);
          if (ggTrail.current.length > TRAIL_LEN) ggTrail.current.shift();
        }
      }
      bump((n) => (n + 1) % 1_000_000);
    }, 66);   // ~15 Hz redraw — plenty for reading, cheap to render
    return () => window.clearInterval(id);
  }, []);

  if (!state) return null;
  const valid = state.accel?.valid ?? false;
  const ref_cap = refCapacity(state.wheels);

  return (
    <Panel title="附着状态与余量">
      {!valid && (
        <div className="panel-small" style={{ color: "var(--muted)", marginBottom: 6 }}>
          当前模型无轮胎力（运动学模型），附着圆不适用。切到 simplified_dynamic 或 multibody 查看。
        </div>
      )}
      <div style={{
        display: "grid", gridTemplateColumns: "1fr 1fr", gap: 4,
        justifyItems: "center", opacity: valid ? 1 : 0.35,
      }}>
        {[0, 1, 2, 3].map((i) => (
          <FrictionCircle key={i} wheel={state.wheels[i]} label={WHEEL_LABELS[i]}
                          ref_cap={ref_cap} trail={wheelTrails.current[i]} />
        ))}
      </div>
      <div style={{ marginTop: 10, display: "flex", justifyContent: "center",
                    opacity: valid ? 1 : 0.35 }}>
        <GgDiagram state={state} trail={ggTrail.current} />
      </div>
      <div className="panel-small" style={{ color: "var(--muted)", marginTop: 8, lineHeight: 1.5 }}>
        圆的半径 = μ·Fz，所以载荷转移看得见：转弯时内侧圆缩小、制动时前轮圆变大。
        圆点变成空心环 = 滑移已过轮胎特性峰值——此时利用率再高也只会越滑越少力，
        与"还在爬坡的 95%"性质相反。
      </div>
    </Panel>
  );
}
