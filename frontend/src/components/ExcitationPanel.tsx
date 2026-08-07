/**
 * ExcitationPanel — standardized open-loop test maneuvers.
 *
 * Drives the driver `steering` input along a predefined time profile (the
 * active strategy still maps that input to wheel angles), while the throttle
 * commands a fixed target cruise speed (throttle = target/v_max, since the
 * models treat throttle as a target-speed fraction). This turns "design a
 * strategy → run a repeatable maneuver → read the score" into a one-click loop.
 *
 * Maneuvers:
 *   step   — 角阶跃: instant step to amplitude after a 0.5 s lead-in
 *   sine   — 单频正弦: steady A·sin(2πft)
 *   sweep  — 正弦扫频: linear chirp f0→f (frequency response / 操稳带宽)
 *   dlc    — 双移线: ISO-3888-style left-then-right double bump
 *
 * Pure frontend — no backend changes. Emits driver messages on a fixed-rate
 * timer (~60 Hz) so the maneuver runs identically regardless of tab focus
 * (requestAnimationFrame pauses in background tabs).
 */

import { useEffect, useRef, useState } from "react";
import { setDriver } from "@/api/ws";
import { useSimStore } from "@/store/sim";
import { fromKmh } from "@/ui/units";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

type Maneuver = "step" | "sine" | "sweep" | "dlc";

const MANEUVERS: { key: Maneuver; label: string; hint: string }[] = [
  { key: "step",  label: "角阶跃",   hint: "瞬态响应 / 超调" },
  { key: "sine",  label: "单频正弦", hint: "稳态往复" },
  { key: "sweep", label: "正弦扫频", hint: "频率响应 / 带宽" },
  { key: "dlc",   label: "双移线",   hint: "ISO-3888 换道" },
];

const LEAD_IN = 0.5;        // seconds of zero steer before step

// raised-sine window centred in [lo, hi], returns 0..1
function bump(u: number, lo: number, hi: number): number {
  if (u <= lo || u >= hi) return 0;
  const x = (u - lo) / (hi - lo);
  return Math.sin(Math.PI * x) ** 2;
}

// steering ∈ [-1,1] as a function of elapsed time t [s]
function steerProfile(m: Maneuver, t: number, amp: number, freq: number, dur: number): number {
  switch (m) {
    case "step":
      return t < LEAD_IN ? 0 : amp;
    case "sine":
      return amp * Math.sin(2 * Math.PI * freq * t);
    case "sweep": {
      const f0 = 0.1;
      // linear chirp: instantaneous phase ∫2π f(τ)dτ, f(τ)=f0+(freq-f0)τ/dur
      const phase = 2 * Math.PI * (f0 * t + ((freq - f0) / (2 * dur)) * t * t);
      return amp * Math.sin(phase);
    }
    case "dlc": {
      const u = t / dur;
      return amp * (bump(u, 0.12, 0.42) - bump(u, 0.5, 0.8));
    }
  }
}

export default function ExcitationPanel() {
  const [maneuver, setManeuver] = useState<Maneuver>("step");
  const [amp, setAmp] = useState(0.6);       // normalised steering amplitude
  const [freq, setFreq] = useState(0.5);     // Hz
  const [dur, setDur] = useState(8);         // seconds
  const [speedKmh, setSpeedKmh] = useState(20);  // target cruise speed [km/h]
  const [clearFirst, setClearFirst] = useState(true);
  const [running, setRunning] = useState(false);
  const [progress, setProgress] = useState(0);

  const timerRef = useRef<number | null>(null);
  const startRef = useRef(0);
  // mirror state into refs so the timer loop reads live values
  const cfg = useRef({ maneuver, amp, freq, dur, speedKmh });
  cfg.current = { maneuver, amp, freq, dur, speedKmh };

  const stop = (zero = true) => {
    if (timerRef.current != null) window.clearInterval(timerRef.current);
    timerRef.current = null;
    setRunning(false);
    setProgress(0);
    if (zero) setDriver(0, 0);
  };

  // cleanup on unmount
  useEffect(() => () => { if (timerRef.current != null) window.clearInterval(timerRef.current); }, []);

  const start = () => {
    if (running) { stop(); return; }
    if (clearFirst) useSimStore.getState().clearHistory();
    setRunning(true);
    startRef.current = Date.now();

    timerRef.current = window.setInterval(() => {
      const { maneuver: m, amp: a, freq: f, dur: d, speedKmh: tgtKmh } = cfg.current;
      const t = (Date.now() - startRef.current) / 1000;
      if (t >= d) { stop(); return; }
      setProgress(t / d);

      const steering = Math.max(-1, Math.min(1, steerProfile(m, t, a, f, d)));
      // Throttle is a target-speed fraction (commanded speed = throttle·v_max),
      // NOT an acceleration pedal — so command the target speed directly.
      const vmax = useSimStore.getState().state?.params.v_max ?? 15;
      const throttle = Math.max(-1, Math.min(1, fromKmh(tgtKmh) / vmax));
      // steer_bypass_feel: an open-loop steer test measures the *vehicle*, so
      // the amplitude must not be reshaped by the driver-input feel layer
      // (variable gear ratio + grip soft limit) — same rule the front_deg
      // validation experiments follow. Without it the amplitude knob went dead
      // above ~0.14 at 60 km/h and the sweep amplitude varied with speed.
      setDriver(throttle, steering, { steer_bypass_feel: true });
    }, 1000 / 60);
  };

  const showFreq = maneuver === "sine" || maneuver === "sweep";

  return (
    <Panel
      title="开环激励测试"
      help={HELP.excitation}
      badge={running && (
        <span style={{ fontSize: 10, color: "#facc15", marginLeft: "auto" }}>
          ▶ {(progress * 100).toFixed(0)}%
        </span>
      )}
    >
      {/* Maneuver picker */}
      <div style={{ display: "flex", gap: 3, flexWrap: "wrap", marginBottom: 8 }}>
        {MANEUVERS.map((mv) => (
          <button key={mv.key}
            title={mv.hint}
            style={{
              fontSize: 11, padding: "2px 9px",
              background: maneuver === mv.key ? "rgba(250,204,21,0.12)" : "var(--surface)",
              border: `1px solid ${maneuver === mv.key ? "#facc15" : "var(--border)"}`,
              color: maneuver === mv.key ? "#facc15" : "var(--muted)",
              borderRadius: 4, cursor: "pointer",
            }}
            onClick={() => !running && setManeuver(mv.key)}
          >
            {mv.label}
          </button>
        ))}
      </div>

      {/* Sliders */}
      <Slider label="转向幅值" value={amp} min={0.05} max={1} step={0.05}
        fmt={(v) => v.toFixed(2)} onChange={setAmp} disabled={running} />
      {showFreq && (
        <Slider label={maneuver === "sweep" ? "终止频率" : "频率"} value={freq}
          min={0.1} max={3} step={0.1} fmt={(v) => v.toFixed(1) + " Hz"}
          onChange={setFreq} disabled={running} />
      )}
      <Slider label="持续时间" value={dur} min={2} max={30} step={1}
        fmt={(v) => v.toFixed(0) + " s"} onChange={setDur} disabled={running} />

      {/* Target speed — slider + manual km/h entry */}
      <div style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 4 }}>
        <span style={{ fontSize: 10, color: "var(--muted)", width: 56, flexShrink: 0 }}>目标车速</span>
        <input type="range" min={0} max={120} step={1} value={speedKmh} disabled={running}
          style={{ flex: 1, accentColor: "#facc15" }}
          onChange={(e) => setSpeedKmh(Number(e.target.value))} />
        <input type="number" min={0} max={300} step={1} value={speedKmh} disabled={running}
          style={{
            width: 48, background: "var(--surface)", border: "1px solid var(--border)",
            color: "var(--text)", borderRadius: 4, padding: "2px 4px", fontSize: 11,
          }}
          onChange={(e) => setSpeedKmh(Math.max(0, Number(e.target.value)))} />
        <span style={{ fontSize: 10, color: "var(--muted)", width: 30 }}>km/h</span>
      </div>

      <label style={{ display: "flex", alignItems: "center", gap: 5, fontSize: 11,
                      color: "var(--muted)", marginTop: 4, cursor: "pointer" }}>
        <input type="checkbox" checked={clearFirst}
          onChange={(e) => setClearFirst(e.target.checked)} disabled={running} />
        开始时清空历史（便于评分）
      </label>

      <button
        style={{
          width: "100%", marginTop: 8, padding: "6px 0", fontSize: 13,
          background: running ? "#ef4444" : "#facc15",
          color: running ? "#fff" : "#1a1208",
          border: "none", borderRadius: 5, cursor: "pointer", fontWeight: 600,
        }}
        onClick={start}
      >
        {running ? "■ 停止" : "▶ 开始激励"}
      </button>

      <div style={{ fontSize: 10, color: "var(--muted)", marginTop: 6, lineHeight: 1.5 }}>
        激励驱动驾驶员转向输入（由当前策略映射到四轮），油门用比例控制保持目标车速。运行时请勿用键盘干预。
      </div>
    </Panel>
  );
}

// ─── Small labelled slider ────────────────────────────────────────────────────

function Slider({ label, value, min, max, step, fmt, onChange, disabled }: {
  label: string; value: number; min: number; max: number; step: number;
  fmt: (v: number) => string; onChange: (v: number) => void; disabled?: boolean;
}) {
  return (
    <div style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 4 }}>
      <span style={{ fontSize: 10, color: "var(--muted)", width: 56, flexShrink: 0 }}>{label}</span>
      <input type="range" min={min} max={max} step={step} value={value} disabled={disabled}
        style={{ flex: 1, accentColor: "#facc15" }}
        onChange={(e) => onChange(Number(e.target.value))} />
      <span style={{ fontFamily: "monospace", fontSize: 11, width: 56,
                     textAlign: "right", color: "var(--text)" }}>{fmt(value)}</span>
    </div>
  );
}
