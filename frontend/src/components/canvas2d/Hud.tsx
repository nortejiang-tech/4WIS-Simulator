/**
 * Hud — DOM overlay on the 2D canvas: strategy, speeds, pose, per-wheel
 * δ / μ / ICR deviation, and zoom controls.
 */

import { useSimStore } from "@/store/sim";
import type { SimStateMessage } from "@/types/sim";
import { fmtKmh } from "@/ui/units";
import { muTextColor } from "./colors";
import GripBudget from "./GripBudget";
import Speedometer from "./Speedometer";
import "@/components/CanvasHud.css";

export default function Hud({ state }: { state: SimStateMessage }) {
  const pxm = useSimStore((s) => s.view2dPxm);
  const setPxm = useSimStore((s) => s.setView2dPxm);
  const cruiseOn = useSimStore((s) => s.cruiseOn);
  const cruiseSpeed = useSimStore((s) => s.cruiseSpeed);
  const holdSpeed = useSimStore((s) => s.holdSpeed);

  const yaw_rate_deg = (state.velocity.yaw_rate * 180) / Math.PI;
  const psi_deg = (state.pose.psi * 180) / Math.PI;
  const devs = state.wheels.map((w) => w.icr_dev);
  const anyDev = devs.some((d) => d != null);
  const drv = state.driver;
  const gear = drv?.gear ?? 1;
  const gearLabel = gear < 0 ? "R" : gear === 0 ? "N" : "D";
  const anyLocked = state.wheels.some((w) => w.locked);
  // Steering-feel readouts. The ratio and the effective front angle come from
  // the backend rather than being re-derived here: a front-end copy of the
  // curve drifts the moment a parameter changes (it hard-coded v_ref = 22), and
  // it cannot show the grip soft limit at all — so it reported a ratio implying
  // a front angle the vehicle never received. δ_eff is the honest number.
  const swRange = state.params?.steer_wheel_range ?? 540;
  const thetaSw = (drv?.steering ?? 0) * (swRange / 2);
  const ratio = drv?.steer_ratio ?? 0;
  const deltaEff = drv?.steer_delta_eff_deg ?? 0;

  // Speed target, if an assist is holding one — drawn as a tick on the gauge
  // so closing on it needs no mental arithmetic.
  const vMaxKmh = (state.params?.v_max ?? 20) * 3.6;
  const targetKmh = cruiseOn ? cruiseSpeed * 3.6
    : holdSpeed ? (drv?.throttle ?? 0) * vMaxKmh : null;
  const targetLabel = cruiseOn ? `巡航 ${Math.round(cruiseSpeed * 3.6)}`
    : holdSpeed ? `保持 ${Math.round((targetKmh ?? 0))}` : null;

  return (
    <div className="canvas-hud">
      <Speedometer speedKmh={state.velocity.vx * 3.6} maxKmh={vMaxKmh}
                   targetKmh={targetKmh} label={targetLabel} />
      <div className="hud-row">
        <span className="hud-label">策略</span>
        <span className="hud-value strong">{state.strategy}</span>
      </div>
      <div className="hud-row">
        <span className="hud-label">档位</span>
        <span className="hud-value hud-mono strong" style={{ color: gear < 0 ? "#fbbf24" : undefined }}>
          {gearLabel}
        </span>
        {drv && (drv.brake ?? 0) > 0.02 && (
          <span className="hud-value hud-mono" style={{ color: "#f87171", marginLeft: 8 }}>
            刹车 {(drv.brake! * 100).toFixed(0)}%
          </span>
        )}
        {drv && drv.handbrake ? (
          <span className="hud-value hud-mono" style={{ color: "#fbbf24", marginLeft: 8 }}>手刹</span>
        ) : null}
        {anyLocked && (
          <span className="hud-value hud-mono" style={{ color: "#f87171", marginLeft: 8 }}>⚠ 抱死</span>
        )}
      </div>
      <div className="hud-row">
        <span className="hud-label">vₓ</span>
        <span className="hud-value hud-mono">{fmtKmh(state.velocity.vx)} km/h</span>
      </div>
      <div className="hud-row">
        <span className="hud-label">ψ̇</span>
        <span className="hud-value hud-mono">{yaw_rate_deg.toFixed(1)} °/s</span>
      </div>
      <div className="hud-row">
        <span className="hud-label">pose</span>
        <span className="hud-value hud-mono">
          ({state.pose.x.toFixed(1)}, {state.pose.y.toFixed(1)}) {psi_deg.toFixed(0)}°
        </span>
      </div>
      <div className="hud-row">
        <span className="hud-label">δ (°)</span>
        <span className="hud-value hud-mono hud-small">
          {state.wheels.map((w) => ((w.delta * 180) / Math.PI).toFixed(1)).join(" / ")}
        </span>
      </div>
      <div className="hud-row">
        <span className="hud-label">方向盘</span>
        <span className="hud-value hud-mono hud-small">
          θ_sw {thetaSw.toFixed(0)}° · i {ratio.toFixed(1)}:1 · δ {deltaEff.toFixed(1)}°
        </span>
      </div>
      <div className="hud-row">
        <span className="hud-label">μ</span>
        <span className="hud-value hud-mono hud-small">
          {state.wheels.map((w, i) => (
            <span key={i} style={{ color: muTextColor(w.mu) }}>
              {(w.mu ?? 1).toFixed(2)}{i < 3 ? " / " : ""}
            </span>
          ))}
        </span>
      </div>
      {(state.accel?.valid ?? false) && <GripBudget wheels={state.wheels} />}
      <div className="hud-row">
        <span className="hud-label">ICR偏差</span>
        <span className="hud-value hud-mono hud-small">
          {anyDev
            ? devs.map((d, i) => (
                <span key={i} style={{ color: d != null && Math.abs(d) > 0.3 ? "#f87171" : undefined }}>
                  {d == null ? "—" : d.toFixed(2)}{i < 3 ? " / " : ""}
                </span>
              ))
            : "—（直行）"}
        </span>
      </div>
      <div className="hud-zoom">
        <button onClick={() => setPxm(pxm * 1.25)}>+</button>
        <span className="hud-mono hud-small">{pxm.toFixed(0)} px/m</span>
        <button onClick={() => setPxm(pxm / 1.25)}>−</button>
      </div>
    </div>
  );
}
