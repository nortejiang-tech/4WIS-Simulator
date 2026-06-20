import { WHEEL_LABELS } from "@/types/sim";
import { numInput, selectStyle } from "@/ui/styles";
import {
  BODY_COUPLING_HINTS,
  BODY_COUPLING_LABELS,
  displayNumber,
  MODE_OPTIONS,
  type BodyCoupling,
} from "./types";

interface Props {
  wheelIndex: number;
  setWheelIndex: (i: number) => void;
  mode: string;
  setMode: (m: string) => void;
  mu: number;
  setMu: (v: number) => void;
  speedMaxKmh: number;
  setSpeedMaxKmh: (v: number) => void;
  speedSteps: number;
  setSpeedSteps: (v: number) => void;
  profileSpeedKmh: number;
  setProfileSpeedKmh: (v: number) => void;
  angleMaxDeg: number;
  setAngleMaxDeg: (v: number) => void;
  angleSteps: number;
  setAngleSteps: (v: number) => void;
  bodyCoupling: BodyCoupling;
  setBodyCoupling: (v: BodyCoupling) => void;
  onRun: () => void;
  busy: boolean;
  paramsReady: boolean;
}

export function LoadControls(p: Props) {
  const effectiveProfileSpeedKmh = Math.max(0, Math.min(p.profileSpeedKmh, p.speedMaxKmh));
  return (
    <div className="load-controls">
      <span className="load-section-label">计算设置</span>
      <label>
        车轮
        <select value={p.wheelIndex} onChange={(e) => p.setWheelIndex(Number(e.target.value))} style={selectStyle}>
          {WHEEL_LABELS.map((w, i) => <option key={w} value={i}>{w}</option>)}
        </select>
      </label>
      <label>
        模式
        <select value={p.mode} onChange={(e) => p.setMode(e.target.value)} style={selectStyle}>
          {MODE_OPTIONS.map(([value, label]) => <option key={value} value={value}>{label}</option>)}
        </select>
      </label>
      <label>μ <input type="number" step="0.05" value={p.mu} onChange={(e) => p.setMu(Number(e.target.value))} style={numInput} /></label>
      <label>最高车速 km/h <input type="number" step="5" value={p.speedMaxKmh} onChange={(e) => p.setSpeedMaxKmh(Number(e.target.value))} style={numInput} /></label>
      <label>速度点 <input type="number" step="1" value={p.speedSteps} onChange={(e) => p.setSpeedSteps(Number(e.target.value))} style={numInput} /></label>
      <label className="load-profile-speed">
        剖面车速 km/h
        <input
          type="number"
          min={0}
          max={p.speedMaxKmh}
          step="1"
          value={displayNumber(effectiveProfileSpeedKmh)}
          onChange={(e) => p.setProfileSpeedKmh(Number(e.target.value))}
          style={numInput}
        />
        <input
          type="range"
          min={0}
          max={Math.max(p.speedMaxKmh, 1)}
          step="1"
          value={effectiveProfileSpeedKmh}
          onChange={(e) => p.setProfileSpeedKmh(Number(e.target.value))}
          aria-label="剖面车速"
        />
      </label>
      <label>最大转角 ° <input type="number" step="1" value={p.angleMaxDeg} onChange={(e) => p.setAngleMaxDeg(Number(e.target.value))} style={numInput} /></label>
      <label>转角点 <input type="number" step="2" value={p.angleSteps} onChange={(e) => p.setAngleSteps(Number(e.target.value))} style={numInput} /></label>
      <div className="load-bcoup" title={BODY_COUPLING_HINTS[p.bodyCoupling]}>
        <span className="load-bcoup-label">受力口径</span>
        <div className="load-bcoup-seg" role="group" aria-label="受力分析口径">
          {(["vehicle", "isolated"] as BodyCoupling[]).map((id) => (
            <button
              key={id}
              type="button"
              className={`load-bcoup-btn${p.bodyCoupling === id ? " active" : ""}`}
              onClick={() => p.setBodyCoupling(id)}
              title={BODY_COUPLING_HINTS[id]}
            >
              {BODY_COUPLING_LABELS[id]}
            </button>
          ))}
        </div>
      </div>
      <button onClick={p.onRun} disabled={p.busy || !p.paramsReady}>{p.busy ? "计算中..." : "重新计算"}</button>
    </div>
  );
}
