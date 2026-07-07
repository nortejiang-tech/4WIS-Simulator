/**
 * GamepadConfigPanel — mapping presets + live calibration for the gamepad.
 *
 * The panel polls the first connected gamepad in its own rAF (only while
 * mounted) to (a) drive the live axis/button monitor and (b) implement
 * click-to-bind ("learn"): arm a channel, then wiggle any axis and the
 * largest-moved axis is assigned to it. This makes the mapping device-agnostic
 * — wheels / HOTAS with non-standard axis order just get rebound by feel.
 */

import { useEffect, useRef, useState } from "react";

import { setStrategy } from "@/api/ws";
import {
  GamepadConfig,
  activeChannels,
  defaultGamepadConfig,
  firstGamepad,
  readAxis,
  requiredStrategy,
} from "@/input/gamepadConfig";
import { useSimStore } from "@/store/sim";
import Panel from "@/components/Panel";
import "./GamepadConfigPanel.css";

interface Preset {
  key: string;
  label: string;
  mode: GamepadConfig["mode"];
  grouping?: GamepadConfig["grouping"];
  hint: string;
}

const PRESETS: Preset[] = [
  { key: "assisted", label: "经典", mode: "assisted",
    hint: "左摇杆转向 · 扳机油门 → 交给当前控制策略（阿克曼/后轮转向/蟹行…）" },
  { key: "front_rear", label: "前后轴独立", mode: "direct", grouping: "front_rear",
    hint: "左摇杆X=前轴角，右摇杆X=后轴角。反相打小半径 / 同相蟹行都靠手给" },
  { key: "left_right", label: "左右侧独立", mode: "direct", grouping: "left_right",
    hint: "左摇杆X=左侧两轮(FL+RL)，右摇杆X=右侧两轮(FR+RR)" },
  { key: "per_wheel", label: "逐轮直控", mode: "direct", grouping: "per_wheel",
    hint: "左摇杆=左边两轮(X前/Y后)，右摇杆=右边两轮。四轮完全独立" },
  { key: "crab", label: "蟹行", mode: "direct", grouping: "crab",
    hint: "左摇杆X=四轮同角，纯横移" },
  { key: "holonomic", label: "全向车身", mode: "holonomic",
    hint: "左摇杆=平移(前进+横移)，右摇杆X=自转。斜开+自转同时做" },
];

export default function GamepadConfigPanel() {
  const cfg = useSimStore((s) => s.gamepadConfig);
  const setCfg = useSimStore((s) => s.setGamepadConfig);
  const gamepadEnabled = useSimStore((s) => s.gamepadEnabled);
  const setGamepadEnabled = useSimStore((s) => s.setGamepadEnabled);
  const gamepadId = useSimStore((s) => s.gamepadId);

  const [live, setLive] = useState<{ axes: number[]; buttons: number[] }>({ axes: [], buttons: [] });
  const [learning, setLearning] = useState<string | null>(null);
  const learnRef = useRef<{ channel: string | null; baseline: number[] }>({ channel: null, baseline: [] });

  // Poll gamepad while mounted (monitor + learn). Reads/writes store via
  // getState so the loop never needs to re-subscribe on config changes.
  useEffect(() => {
    let raf = 0;
    const tick = () => {
      const gp = firstGamepad();
      if (gp) {
        setLive({ axes: Array.from(gp.axes), buttons: gp.buttons.map((b) => b.value) });
        const L = learnRef.current;
        if (L.channel) {
          if (L.baseline.length === 0) {
            L.baseline = Array.from(gp.axes);
          } else {
            let best = -1, bestD = 0.5;
            for (let i = 0; i < gp.axes.length; i++) {
              const d = Math.abs((gp.axes[i] ?? 0) - (L.baseline[i] ?? 0));
              if (d > bestD) { bestD = d; best = i; }
            }
            if (best >= 0) {
              const c = useSimStore.getState().gamepadConfig;
              setCfg({ ...c, channels: { ...c.channels, [L.channel]: { ...c.channels[L.channel], axis: best } } });
              L.channel = null; L.baseline = [];
              setLearning(null);
            }
          }
        }
      }
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [setCfg]);

  const pickPreset = (p: Preset) => {
    setCfg({ ...cfg, mode: p.mode, grouping: p.grouping ?? cfg.grouping });
    const need = requiredStrategy(p.mode);
    if (need) {
      setStrategy(need);
    } else {
      // Assisted: if we're leaving a manual strategy, restore a normal driving
      // one so (throttle, steering) is interpreted again.
      const cur = useSimStore.getState().state?.strategy;
      if (cur === "manual_wheel" || cur === "manual_body") setStrategy("ideal_ackermann");
    }
  };

  const armLearn = (channel: string) => {
    learnRef.current = { channel, baseline: [] };
    setLearning(channel);
  };

  const setChannel = (key: string, patch: Partial<GamepadConfig["channels"][string]>) =>
    setCfg({ ...cfg, channels: { ...cfg.channels, [key]: { ...cfg.channels[key], ...patch } } });

  const activePreset = PRESETS.find(
    (p) => p.mode === cfg.mode && (p.mode !== "direct" || p.grouping === cfg.grouping),
  );
  const gp = firstGamepad();

  return (
    <Panel
      title="🎮 手柄映射与校准"
      help={
        <div style={{ lineHeight: 1.6 }}>
          选一个映射模式，下方按需为每个通道「绑定」物理轴（点绑定后拨动要用的摇杆/踏板即可自动识别），
          可调死区、灵敏度(expo)、反向。配置自动保存在本机。
        </div>
      }
      badge={
        <span className="small" style={{ marginLeft: "auto",
          color: gamepadId ? "var(--good)" : "var(--muted)", maxWidth: 130,
          overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}
          title={gamepadId ?? "未检测到设备"}>
          {gamepadId ? "● 已连接" : "○ 未检测"}
        </span>
      }
    >
      <label style={{ display: "flex", alignItems: "center", gap: 6, cursor: "pointer", fontSize: 12, marginBottom: 8 }}>
        <input type="checkbox" checked={gamepadEnabled} onChange={(e) => setGamepadEnabled(e.target.checked)} />
        启用手柄输入
      </label>

      {/* preset selector */}
      <div className="gp-presets">
        {PRESETS.map((p) => (
          <button key={p.key} className={`gp-preset ${activePreset?.key === p.key ? "on" : ""}`}
                  data-testid={`gp-preset-${p.key}`}
                  title={p.hint} onClick={() => pickPreset(p)}>
            {p.label}
          </button>
        ))}
      </div>
      {activePreset && <div className="small" style={{ color: "var(--muted)", margin: "4px 0 8px", lineHeight: 1.5 }}>
        {activePreset.hint}
        {cfg.mode !== "assisted" && <> · <b>已接管策略</b>：{requiredStrategy(cfg.mode)}</>}
      </div>}

      {/* channel bindings */}
      <div className="gp-chan-head">通道绑定</div>
      {activeChannels(cfg).map((ch) => {
        const b = cfg.channels[ch.key];
        const val = gp ? readAxis(gp, b, cfg.deadzone, cfg.sensitivity) : 0;
        const armed = learning === ch.key;
        return (
          <div key={ch.key} className="gp-chan" data-testid={`gp-channel-${ch.key}`}>
            <div className="gp-chan-top">
              <span className="gp-chan-label">{ch.label}</span>
              <span className="gp-chan-axis">{b.axis < 0 ? "未绑定" : `轴 ${b.axis}`}</span>
              <button className={`gp-learn ${armed ? "arm" : ""}`} onClick={() => armLearn(ch.key)}>
                {armed ? "拨动轴…" : "绑定"}
              </button>
            </div>
            <div className="gp-bar"><div className="gp-bar-fill"
              style={{ left: `${50 + Math.min(0, val) * 50}%`, width: `${Math.abs(val) * 50}%` }} /></div>
            <div className="gp-chan-ctl">
              <label><input type="checkbox" checked={b.invert}
                onChange={(e) => setChannel(ch.key, { invert: e.target.checked })} /> 反向</label>
              <span style={{ display: "flex", alignItems: "center", gap: 4, flex: 1 }}>
                <span style={{ color: "var(--muted)" }}>expo</span>
                <input type="range" min={0} max={0.8} step={0.05} value={b.expo} style={{ flex: 1 }}
                  onChange={(e) => setChannel(ch.key, { expo: Number(e.target.value) })} />
              </span>
            </div>
          </div>
        );
      })}

      {/* throttle */}
      <div className="gp-chan-head">油门</div>
      <div className="gp-chan">
        <div className="gp-chan-top">
          <span className="gp-chan-label">来源</span>
          <select className="gp-sel" aria-label="油门来源" value={cfg.throttle.source}
            onChange={(e) => setCfg({ ...cfg, throttle: { ...cfg.throttle, source: e.target.value as "triggers" | "axis" } })}>
            <option value="triggers">扳机 RT/LT</option>
            <option value="axis">摇杆轴</option>
          </select>
        </div>
        {cfg.throttle.source === "axis" && (
          <div className="gp-chan-ctl">
            <button className={`gp-learn ${learning === "__thr" ? "arm" : ""}`}
              onClick={() => {
                // reuse learn machinery into throttle.axis
                learnRef.current = { channel: null, baseline: [] };
                setLearning("__thr");
                const arm = () => {
                  const g = firstGamepad(); if (!g) return;
                  let base = Array.from(g.axes);
                  const scan = () => {
                    const gg = firstGamepad(); if (!gg) return;
                    let best = -1, bestD = 0.5;
                    for (let i = 0; i < gg.axes.length; i++) {
                      const d = Math.abs((gg.axes[i] ?? 0) - (base[i] ?? 0));
                      if (d > bestD) { bestD = d; best = i; }
                    }
                    if (best >= 0) {
                      const c = useSimStore.getState().gamepadConfig;
                      setCfg({ ...c, throttle: { ...c.throttle, axis: { ...c.throttle.axis, axis: best } } });
                      setLearning(null); return;
                    }
                    requestAnimationFrame(scan);
                  };
                  requestAnimationFrame(scan);
                };
                requestAnimationFrame(arm);
              }}>
              {learning === "__thr" ? "拨动轴…" : `绑定（当前轴 ${cfg.throttle.axis.axis}）`}
            </button>
            <label><input type="checkbox" checked={cfg.throttle.axis.invert}
              onChange={(e) => setCfg({ ...cfg, throttle: { ...cfg.throttle, axis: { ...cfg.throttle.axis, invert: e.target.checked } } })} /> 反向</label>
          </div>
        )}
      </div>

      {/* global tuning */}
      <div className="gp-chan-head">全局</div>
      <div className="gp-glob">
        <span>死区 {cfg.deadzone.toFixed(2)}</span>
        <input type="range" aria-label="手柄死区" min={0} max={0.35} step={0.01} value={cfg.deadzone}
          onChange={(e) => setCfg({ ...cfg, deadzone: Number(e.target.value) })} />
      </div>
      <div className="gp-glob">
        <span>转向灵敏度 {cfg.sensitivity.toFixed(2)}×</span>
        <input type="range" aria-label="手柄转向灵敏度" min={0.2} max={1.5} step={0.05} value={cfg.sensitivity}
          onChange={(e) => setCfg({ ...cfg, sensitivity: Number(e.target.value) })} />
      </div>

      {/* live monitor */}
      <div className="gp-chan-head">实时监视{!gp && <span className="small" style={{ color: "var(--muted)", fontWeight: 400 }}>（连接手柄后显示）</span>}</div>
      {gp && (
        <div className="gp-mon">
          {live.axes.map((v, i) => (
            <div key={i} className="gp-mon-axis" title={`轴 ${i} = ${v.toFixed(2)}`}>
              <span className="gp-mon-lab">{i}</span>
              <div className="gp-bar"><div className="gp-bar-fill"
                style={{ left: `${50 + Math.min(0, v) * 50}%`, width: `${Math.abs(v) * 50}%` }} /></div>
            </div>
          ))}
          <div className="gp-mon-btns">
            {live.buttons.map((v, i) => v > 0.15 && (
              <span key={i} className="gp-btn-dot" title={`键 ${i} = ${v.toFixed(2)}`}>{i}</span>
            ))}
          </div>
        </div>
      )}

      <button className="gp-reset" data-testid="gp-reset" onClick={() => setCfg({ ...defaultGamepadConfig(), mode: cfg.mode, grouping: cfg.grouping })}>
        恢复默认绑定
      </button>
    </Panel>
  );
}
