import { useState } from "react";

import { resetSim, setModel, setSceneMu, setStrategy, setDriverMode } from "@/api/ws";
import { useSimStore } from "@/store/sim";
import { fmtKmh, toKmh, fromKmh } from "@/ui/units";
import Panel from "@/components/Panel";
import GamepadConfigPanel from "@/components/GamepadConfigPanel";
import { HELP } from "@/ui/help";

const MODEL_LABELS: { id: string; label: string; layer: "primary" | "research" }[] = [
  { id: "kinematic", label: "运动学", layer: "primary" },
  { id: "simplified_dynamic", label: "动力学", layer: "primary" },
  { id: "multibody", label: "多体(14DOF)", layer: "research" },
];

// Road-surface friction presets (base μ). User can fine-tune after picking.
const SURFACE_PRESETS: { label: string; mu: number }[] = [
  { label: "水泥路", mu: 0.85 },
  { label: "干沥青", mu: 0.90 },
  { label: "雨天湿路", mu: 0.55 },
  { label: "雪面", mu: 0.30 },
  { label: "冰面", mu: 0.12 },
];

const STRATEGY_LABELS: Record<string, string> = {
  ackermann: "传统阿克曼",
  ideal_ackermann: "理想阿克曼",
  rear_wheel_steer: "后轮转向",
  crab: "蟹行",
  zero_radius: "零半径",
  follow_trajectory: "轨迹跟踪",
  fault_reconfig: "容错重构",
  manual_wheel: "手柄直控",
  manual_body: "手柄全向",
  user_python: "Python 策略",
  user_js: "JS 策略",
};

// Sub-modes of the unified 后轮转向 (rear_wheel_steer) strategy.
const RWS_MODES: { id: string; label: string; hint: string }[] = [
  { id: "fixed_ratio", label: "定比", hint: "δ后 = 固定比例 × δ前（任意车速同一比例，基准对照）" },
  { id: "speed_schedule", label: "车速调度", hint: "δ后 = k(vx)·δ前；低速反相、高速同相（曲线可在「设计」页编辑）" },
  { id: "yaw_feedback", label: "横摆反馈", hint: "δ后 = g1·δ前 + g2·横摆角速度（闭环抑制甩动）" },
  { id: "transient", label: "稳态+瞬态", hint: "零侧偏稳态 + 快打方向时瞬态先反相（转入更利落）" },
  { id: "model_following", label: "模型跟踪", hint: "零侧偏前馈 + 跟踪参考横摆" },
];
const DEFAULT_RWS_MODE = "speed_schedule";
const fmtMm = (m: number) => (m * 1000).toFixed(0);

export default function ControlPanel() {
  const strategies = useSimStore((s) => s.strategies);
  const current = useSimStore((s) => s.state?.strategy);
  const driver = useSimStore((s) => s.state?.driver);
  const params = useSimStore((s) => s.state?.params);
  const holdSpeed = useSimStore((s) => s.holdSpeed);
  const setHoldSpeed = useSimStore((s) => s.setHoldSpeed);
  const cruiseOn = useSimStore((s) => s.cruiseOn);
  const cruiseSpeed = useSimStore((s) => s.cruiseSpeed);
  const pedalExpo = useSimStore((s) => s.pedalExpo);
  const setPedalExpo = useSimStore((s) => s.setPedalExpo);
  const setCruiseOn = useSimStore((s) => s.setCruiseOn);
  const setCruiseSpeed = useSimStore((s) => s.setCruiseSpeed);
  const steerReturn = useSimStore((s) => s.steerReturn);
  const setSteerReturn = useSimStore((s) => s.setSteerReturn);
  const requestZero = useSimStore((s) => s.requestZero);
  const pushToast = useSimStore((s) => s.pushToast);
  const modelType = useSimStore((s) => s.state?.model_type);
  const baseMu = useSimStore((s) => s.state?.scene?.base_mu ?? 0.85);
  const vMax = params?.v_max ?? 20;
  const [rwsMode, setRwsMode] = useState(DEFAULT_RWS_MODE);

  const applyModel = (modelId: string) => {
    setModel(modelId).catch((e) => pushToast("error", `模型切换失败：${e?.message ?? e}`));
  };
  const applySceneMu = (mu: number) => {
    setSceneMu(mu).catch((e) => pushToast("error", `路面摩擦设置失败：${e?.message ?? e}`));
  };

  // Pick a strategy; for 后轮转向 also push the current sub-mode so the backend
  // dispatches the right control law immediately.
  const pickStrategy = (s: string) => {
    setStrategy(s);
    if (s === "rear_wheel_steer") setDriverMode({ rws_mode: rwsMode });
  };
  const pickRwsMode = (m: string) => {
    setRwsMode(m);
    setDriverMode({ rws_mode: m });
  };

  return (
    <>
      <Panel title="动力学模型" help={HELP.model}>
        <div className="strategy-buttons">
          {MODEL_LABELS.filter((m) => m.layer === "primary").map((m) => (
            <button
              key={m.id}
              className={m.id === modelType ? "active" : ""}
              onClick={() => applyModel(m.id)}
            >
              {m.label}
            </button>
          ))}
        </div>
        <div style={{ marginTop: 8 }}>
          <div style={{ fontSize: 10, color: "var(--muted)", marginBottom: 4 }}>
            研究模型
          </div>
          <div className="strategy-buttons">
            {MODEL_LABELS.filter((m) => m.layer === "research").map((m) => (
              <button
                key={m.id}
                className={m.id === modelType ? "active" : ""}
                onClick={() => applyModel(m.id)}
              >
                {m.label}
              </button>
            ))}
          </div>
          <div style={{ fontSize: 10, color: "var(--muted)", lineHeight: 1.45, marginTop: 4 }}>
            动力学用于轮胎力与平面响应；多体增加垂向悬架自由度，仍需参数标定。
          </div>
        </div>
      </Panel>

      <Panel title="控制策略" help={HELP.strategy}>
        <div className="strategy-buttons">
          {strategies.map((s) => (
            <button
              key={s}
              className={s === current ? "active" : ""}
              onClick={() => pickStrategy(s)}
            >
              {STRATEGY_LABELS[s] ?? s}
            </button>
          ))}
        </div>
        {current === "zero_radius" && (
          <div className="panel-small" style={{ marginTop: 8, lineHeight: 1.5 }}>
            <label>满输入旋转速率（°/s）
              <input aria-label="满输入旋转速率" type="number" min={0} max={180} step={5}
                value={Number(driver?.mode_params?.spin_rate_max_dps ?? 30)}
                onChange={(e) => setDriverMode({ spin_rate_max_dps: Number(e.target.value) })}
                style={{ width: 70, marginLeft: 6 }} />
            </label>
            <div>这是指令速率上限；实际角速度由模型与轮胎决定。</div>
            {params && params.steer_limit < Math.atan2(params.wheelbase, Math.min(params.track_front, params.track_rear)) && (
              <div role="status" style={{ color: "#fbbf24" }}>
                纯滚动原地转向需约 {(Math.atan2(params.wheelbase, Math.min(params.track_front, params.track_rear)) * 180 / Math.PI).toFixed(1)}°；
                当前轮角上限 {(params.steer_limit * 180 / Math.PI).toFixed(1)}°，会出现轮胎擦滑。
              </div>
            )}
          </div>
        )}
        {current === "rear_wheel_steer" && (
          <div style={{ marginTop: 8, display: "flex", flexDirection: "column", gap: 4 }}>
            <div style={{ display: "flex", alignItems: "center", gap: 6 }}>
              <span style={{ fontSize: 11, color: "var(--muted)", flexShrink: 0 }}>后轮控制方法</span>
              <select
                value={rwsMode}
                onChange={(e) => pickRwsMode(e.target.value)}
                style={{ flex: 1, background: "var(--bg-2)", border: "1px solid var(--border)",
                         color: "var(--text)", borderRadius: 6, padding: "5px 8px", fontSize: 12 }}
              >
                {RWS_MODES.map((m) => <option key={m.id} value={m.id}>{m.label}</option>)}
              </select>
            </div>
            <div style={{ fontSize: 10, color: "var(--muted)", lineHeight: 1.5 }}>
              {RWS_MODES.find((m) => m.id === rwsMode)?.hint}
              {rwsMode === "speed_schedule" && <>　·　曲线在「设计」页 k(vx) 调度模式编辑</>}
            </div>
          </div>
        )}
      </Panel>

      <Panel title="驾驶输入（实时回读）" help={HELP.driver}>
        <div className="driver-readout">
          <span>throttle</span>
          <span className="value">{driver?.throttle.toFixed(2) ?? "—"}</span>
          <span>steering</span>
          <span className="value">{driver?.steering.toFixed(2) ?? "—"}</span>
        </div>
        <DriverBars driver={driver} />

        <div style={{ display: "flex", alignItems: "center", gap: 8, marginTop: 8 }}>
          <label style={{ display: "flex", alignItems: "center", gap: 6, cursor: "pointer", fontSize: 12 }}>
            <input type="checkbox" checked={holdSpeed} onChange={(e) => setHoldSpeed(e.target.checked)} />
            保持车速（松手不减速）
          </label>
          {holdSpeed && (
            <button style={{ marginLeft: "auto" }} onClick={() => requestZero()}>急停归零</button>
          )}
        </div>
        {holdSpeed && driver && (
          <div className="panel-small" style={{ color: "var(--muted)", marginTop: 4 }}>
            目标车速 ≈ {fmtKmh(driver.throttle * vMax)} km/h（W/S 增减，松手保持；Space 急停）
          </div>
        )}

        {/* Fixed-speed cruise: type an exact speed in km/h and hold it. */}
        <div style={{ display: "flex", alignItems: "center", gap: 8, marginTop: 8 }}>
          <label style={{ display: "flex", alignItems: "center", gap: 6, cursor: "pointer", fontSize: 12 }}>
            <input type="checkbox" checked={cruiseOn} onChange={(e) => setCruiseOn(e.target.checked)} />
            定速巡航
          </label>
          <input
            type="number" min={0} max={Math.round(toKmh(vMax))} step={1}
            value={Math.round(toKmh(cruiseSpeed))}
            onChange={(e) => setCruiseSpeed(fromKmh(Number(e.target.value)))}
            style={{
              width: 64, marginLeft: "auto", background: "var(--bg-2)",
              border: "1px solid var(--border)", color: "var(--text)",
              borderRadius: 6, padding: "4px 6px", fontSize: 12,
            }}
          />
          <span className="panel-small" style={{ color: "var(--muted)" }}>km/h</span>
        </div>
        {cruiseOn && (
          <div className="panel-small" style={{ color: "var(--muted)", marginTop: 4 }}>
            定速保持 {Math.round(toKmh(cruiseSpeed))} km/h（覆盖 W/S；转向仍可手动/策略控制）
          </div>
        )}

        {/* Pedal curve. The axis spans 0…v_max (200 km/h by default), so a
            fifth of pedal travel is already 40 km/h — expo buys back
            resolution where the driving actually happens. The speed limit,
            which rescales the range instead of shaping it, is the other half
            of this and lives in the vehicle params. */}
        <div style={{ display: "flex", alignItems: "center", gap: 8, marginTop: 8 }}>
          <span className="panel-small" style={{ minWidth: 72 }}>油门曲线</span>
          <input
            type="range" min={0} max={2} step={0.1} value={pedalExpo}
            onChange={(e) => setPedalExpo(Number(e.target.value))}
            style={{ flex: 1 }}
          />
          <span className="panel-small hud-mono" style={{ width: 30, textAlign: "right" }}>
            {pedalExpo.toFixed(1)}
          </span>
        </div>
        <div className="panel-small" style={{ color: "var(--muted)", marginTop: 2 }}>
          {pedalExpo === 0
            ? "线性：半油门 = 半速。低速段不好控就往右调。"
            : `半油门 ≈ ${Math.round(toKmh(Math.pow(0.5, 1 + pedalExpo) * vMax))} km/h`
              + `（线性时是 ${Math.round(toKmh(0.5 * vMax))}）`}
        </div>

        <div style={{ marginTop: 10 }}>
          <div style={{ display: "flex", justifyContent: "space-between", fontSize: 12, color: "var(--muted)" }}>
            <span>转向回正速度</span>
            <span className="panel-mono">{steerReturn === 0 ? "保持（不回正）" : `${(steerReturn * 1.5).toFixed(2)}×`}</span>
          </div>
          <input
            type="range" min={0} max={1} step={0.01}
            value={steerReturn}
            onChange={(e) => setSteerReturn(Number(e.target.value))}
            style={{ width: "100%" }}
          />
          <div style={{ display: "flex", justifyContent: "space-between", fontSize: 10, color: "var(--muted)" }}>
            <span>0 保持</span><span>默认</span><span>1.5× 最快</span>
          </div>
        </div>

        <div className="keyhint" style={{ marginTop: 8 }}>
          <kbd>W</kbd><span>{holdSpeed ? "加速（保持）" : "前进 / 倒车时刹车"}</span>
          <kbd>S</kbd><span>{holdSpeed ? "减速（保持）" : "刹车 / 停稳后再按入倒挡"}</span>
          <kbd>A</kbd><span>左转向</span>
          <kbd>D</kbd><span>右转向</span>
          <kbd>Space</kbd><span>手刹（驻车制动）</span>
          <kbd>R</kbd><span>重置位姿与轨迹</span>
          <kbd>1‒5</kbd><span>切换策略</span>
        </div>
      </Panel>

      <GamepadConfigPanel />

      <Panel title="路面摩擦" help={HELP.friction}>
        <div style={{ display: "flex", gap: 6, alignItems: "center" }}>
          <select
            value={SURFACE_PRESETS.find((s) => Math.abs(s.mu - baseMu) < 1e-6)?.label ?? ""}
            onChange={(e) => {
              const preset = SURFACE_PRESETS.find((s) => s.label === e.target.value);
              if (preset) applySceneMu(preset.mu);
            }}
            style={{ flex: 1, background: "var(--bg-2)", border: "1px solid var(--border)", color: "var(--text)", borderRadius: 6, padding: "6px 8px", fontSize: 12 }}
          >
            <option value="" disabled>自定义 μ</option>
            {SURFACE_PRESETS.map((s) => (
              <option key={s.label} value={s.label}>{s.label}（μ={s.mu}）</option>
            ))}
          </select>
          <span className="panel-mono" style={{ fontSize: 13, minWidth: 42, textAlign: "right" }}>{baseMu.toFixed(2)}</span>
        </div>
        <input
          type="range" min={0.05} max={1.0} step={0.01}
          value={baseMu}
          onChange={(e) => applySceneMu(Number(e.target.value))}
          style={{ width: "100%", marginTop: 6 }}
        />
        <div className="panel-small" style={{ color: "var(--muted)" }}>
          基础轮胎-地面摩擦系数（μ）。选预设后仍可拖动微调。
        </div>
      </Panel>

      <Panel title="车辆参数" help={HELP.vehicleQuick}>
        {params ? (
          <div className="params-list">
            <span>轴距 L</span><span className="value">{fmtMm(params.wheelbase)} mm</span>
            <span>前主销距/轮距</span><span className="value">{fmtMm(params.track_front)} mm</span>
            <span>后主销距/轮距</span><span className="value">{fmtMm(params.track_rear)} mm</span>
            <span>轮胎半径</span><span className="value">{fmtMm(params.tire_radius)} mm</span>
            <span>最大转角</span><span className="value">{((params.steer_limit*180)/Math.PI).toFixed(1)}°</span>
            <span>最大车速</span><span className="value">{fmtKmh(params.v_max)} km/h</span>
          </div>
        ) : (
          <div className="driver-readout"><span>—</span></div>
        )}
      </Panel>

      <Panel title="仿真控制" help={HELP.simControl}>
        <div className="btn-row">
          <button className="warn" onClick={() => resetSim()}>重置位姿</button>
        </div>
      </Panel>
    </>
  );
}

function DriverBars({ driver }: { driver: { throttle: number; steering: number } | undefined }) {
  const throttle = driver?.throttle ?? 0;
  const steering = driver?.steering ?? 0;
  return (
    <div style={{ display: "flex", flexDirection: "column", gap: 4 }}>
      <SignedBar value={throttle} />
      <SignedBar value={steering} />
    </div>
  );
}

function SignedBar({ value }: { value: number }) {
  const v = Math.max(-1, Math.min(1, value));
  // Bar splits at the centre; positive expands right, negative expands left.
  const pct = Math.abs(v) * 50;
  const start = v >= 0 ? 50 : 50 - pct;
  return (
    <div className="bar">
      <div className="fill" style={{ left: `${start}%`, width: `${pct}%` }} />
      <div style={{ position: "absolute", left: "50%", top: 0, bottom: 0, width: 1, background: "#475569" }} />
    </div>
  );
}
