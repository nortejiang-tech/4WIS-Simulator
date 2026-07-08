/**
 * JsStrategyPanel — browser-side JS strategy sandbox.
 *
 * When "user_js" is the active strategy, the user's function is evaluated on
 * every incoming state update. Results are sent back via WebSocket as a
 * {type: "steer_cmd", fl, fr, rl, rr} message which the backend's
 * UserJsStrategy passes directly to the simulation loop.
 *
 * The JS function signature:
 *   function compute(driver, state) { return { fl, fr, rl, rr } }
 *   // driver: {throttle, steering, handbrake}
 *   // state:  {t, x, y, psi, vx, vy, yaw_rate,
 *   //          delta:[fl,fr,rl,rr], fz:[…], torque_steer:[…],
 *   //          steer_limit, wheelbase, track_front, track_rear}
 *   // return: {fl, fr, rl, rr} angles in radians
 */

import { useEffect, useRef, useState } from "react";
import { useSimStore } from "@/store/sim";
import { sendMessage } from "@/api/ws";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

const EXAMPLES: { label: string; code: string }[] = [
  {
    label: "全轮同步（crab）",
    code: `function compute(driver, state) {
  const angle = driver.steering * state.steer_limit;
  return { fl: angle, fr: angle, rl: angle, rr: angle };
}`,
  },
  {
    label: "理想阿克曼",
    code: `function compute(driver, state) {
  const s = driver.steering;
  if (Math.abs(s) < 1e-4) return { fl: 0, fr: 0, rl: 0, rr: 0 };
  const L  = state.wheelbase;
  const tf = state.track_front;
  const tr = state.track_rear;
  const icr_y = L / Math.tan(s * state.steer_limit);
  function wd(wx, wy) { return Math.atan2(wx, icr_y - wy); }
  return {
    fl: wd(+L/2, +tf/2), fr: wd(+L/2, -tf/2),
    rl: wd(-L/2, +tr/2), rr: wd(-L/2, -tr/2),
  };
}`,
  },
  {
    label: "后轮反向（灵活性+）",
    code: `function compute(driver, state) {
  const angle = driver.steering * state.steer_limit;
  return { fl: angle, fr: angle, rl: -angle * 0.5, rr: -angle * 0.5 };
}`,
  },
  {
    label: "仅前轮转向（传统）",
    code: `function compute(driver, state) {
  const angle = driver.steering * state.steer_limit;
  return { fl: angle, fr: angle, rl: 0, rr: 0 };
}`,
  },
];

const PANEL_CODE_KEY = "sim4wis_js_strategy_code";

export default function JsStrategyPanel() {
  const activeStrategy = useSimStore((s) => s.state?.strategy);
  const isActive = activeStrategy === "user_js";

  const [code, setCode] = useState<string>(
    () => localStorage.getItem(PANEL_CODE_KEY) ?? EXAMPLES[1].code
  );
  const [error, setError] = useState<string | null>(null);
  const [execCount, setExecCount] = useState(0);
  const fnRef = useRef<Function | null>(null);

  // Recompile whenever code changes
  useEffect(() => {
    try {
      // eslint-disable-next-line no-new-func
      fnRef.current = new Function("driver", "state", `"use strict";\n${code}\nreturn compute(driver, state);`);
      setError(null);
    } catch (e: any) {
      fnRef.current = null;
      setError(String(e));
    }
    localStorage.setItem(PANEL_CODE_KEY, code);
  }, [code]);

  // Run the JS function on every state update when user_js is active
  useEffect(() => {
    const unsub = useSimStore.subscribe((s) => {
      const msg = s.state;
      if (!isActive || !msg || !fnRef.current) return;
      const driver = msg.driver;
      const w = msg.wheels;
      const state = {
        t: msg.t,
        x: msg.pose.x, y: msg.pose.y, psi: msg.pose.psi,
        vx: msg.velocity.vx, vy: msg.velocity.vy, yaw_rate: msg.velocity.yaw_rate,
        delta:         w.map((wh) => wh.delta),
        fz:            w.map((wh) => wh.fz),
        torque_steer:  w.map((wh) => wh.torque_steer),
        steer_limit:   msg.params.steer_limit,
        wheelbase:     msg.params.wheelbase,
        track_front:   msg.params.track_front,
        track_rear:    msg.params.track_rear,
      };
      try {
        const result = fnRef.current!(driver, state);
        if (result && typeof result === "object") {
          sendMessage({ type: "steer_cmd" as any, fl: result.fl ?? 0, fr: result.fr ?? 0, rl: result.rl ?? 0, rr: result.rr ?? 0 } as any);
          setExecCount((n) => n + 1);
        }
      } catch (e: any) {
        setError(String(e));
      }
    });
    return unsub;
  }, [isActive]);

  return (
    <Panel
      title="JS 策略沙箱"
      help={HELP.jsStrategy}
      badge={
        <span style={{ fontSize: 10, marginLeft: "auto",
                       color: isActive ? "#22d3ee" : "var(--muted)" }}>
          {isActive ? `▶ 激活 · ${execCount} 帧` : "（切到 user_js 策略后生效）"}
        </span>
      }
    >
      {/* Example picker */}
      <div style={{ display: "flex", gap: 4, flexWrap: "wrap", marginBottom: 6 }}>
        {EXAMPLES.map((ex) => (
          <button
            key={ex.label}
            style={{ fontSize: 10 }}
            onClick={() => setCode(ex.code)}
          >
            {ex.label}
          </button>
        ))}
      </div>

      {/* Code editor */}
      <textarea
        value={code}
        onChange={(e) => setCode(e.target.value)}
        spellCheck={false}
        style={{
          width: "100%", height: 200, resize: "vertical",
          fontFamily: "monospace", fontSize: 11,
          background: "var(--surface)", color: "var(--text)",
          border: `1px solid ${error ? "#ef4444" : "var(--border)"}`,
          borderRadius: 4, padding: 6, boxSizing: "border-box",
        }}
      />

      {error && (
        <div style={{
          fontSize: 11, color: "#ef4444", background: "rgba(239,68,68,0.1)",
          borderRadius: 4, padding: "4px 6px", marginTop: 4,
          fontFamily: "monospace", whiteSpace: "pre-wrap", wordBreak: "break-all",
        }}>
          {error}
        </div>
      )}

      <div className="panel-small" style={{ color: "var(--muted)", marginTop: 6, lineHeight: 1.4 }}>
        函数在浏览器中执行，结果通过 WebSocket 发回仿真器。<br />
        签名：<code>function compute(driver, state) {"{"} return {"{"}fl, fr, rl, rr{"}"} {"}"}</code>（rad）
      </div>
    </Panel>
  );
}
