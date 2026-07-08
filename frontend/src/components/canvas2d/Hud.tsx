/**
 * Hud — DOM overlay on the 2D canvas: strategy, speeds, pose, per-wheel
 * δ / μ / ICR deviation, and zoom controls.
 */

import { useSimStore } from "@/store/sim";
import type { SimStateMessage } from "@/types/sim";
import { fmtKmh } from "@/ui/units";
import { muTextColor } from "./colors";
import "@/components/CanvasHud.css";

export default function Hud({ state }: { state: SimStateMessage }) {
  const pxm = useSimStore((s) => s.view2dPxm);
  const setPxm = useSimStore((s) => s.setView2dPxm);

  const yaw_rate_deg = (state.velocity.yaw_rate * 180) / Math.PI;
  const psi_deg = (state.pose.psi * 180) / Math.PI;
  const devs = state.wheels.map((w) => w.icr_dev);
  const anyDev = devs.some((d) => d != null);

  return (
    <div className="canvas-hud">
      <div className="hud-row">
        <span className="hud-label">策略</span>
        <span className="hud-value strong">{state.strategy}</span>
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
        <span className="hud-label">μ</span>
        <span className="hud-value hud-mono hud-small">
          {state.wheels.map((w, i) => (
            <span key={i} style={{ color: muTextColor(w.mu) }}>
              {(w.mu ?? 1).toFixed(2)}{i < 3 ? " / " : ""}
            </span>
          ))}
        </span>
      </div>
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
