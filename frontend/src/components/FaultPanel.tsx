/**
 * FaultPanel — ISO 26262 / functional safety fault injection UI.
 *
 * Lets users add/remove actuator and sensor faults per wheel to validate
 * control strategy robustness and on-board diagnostic responses.
 */

import { useEffect, useState } from "react";
import { fetchJSON, postJSON, deleteJSON, patchJSON } from "@/api/http";
import { useSimStore } from "@/store/sim";
import { selectStyle, numInput } from "@/ui/styles";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

type WheelName = "fl" | "fr" | "rl" | "rr";
type FaultType =
  | "motor_stuck_zero"
  | "motor_stuck_angle"
  | "motor_limited_range"
  | "sensor_bias"
  | "sensor_noise"
  | "sensor_dropout";

interface FaultEntry {
  id: string;
  fault_type: FaultType;
  wheel: WheelName;
  value: number;
  active: boolean;
}

const FAULT_LABELS: Record<FaultType, string> = {
  motor_stuck_zero:    "执行器卡死（0°）",
  motor_stuck_angle:   "执行器卡死（固定角度）",
  motor_limited_range: "执行器角度限幅",
  sensor_bias:         "传感器偏差",
  sensor_noise:        "传感器噪声",
  sensor_dropout:      "传感器掉线（保持最后值）",
};

const WHEEL_LABELS: Record<WheelName, string> = { fl: "FL", fr: "FR", rl: "RL", rr: "RR" };

const VALUE_HINT: Record<FaultType, string | null> = {
  motor_stuck_zero:    null,
  motor_stuck_angle:   "目标角 (rad)",
  motor_limited_range: "最大角 (rad)",
  sensor_bias:         "偏差 (rad)",
  sensor_noise:        "噪声标准差 (rad)",
  sensor_dropout:      null,
};

const FAULT_TYPES = Object.keys(FAULT_LABELS) as FaultType[];
const WHEEL_NAMES: WheelName[] = ["fl", "fr", "rl", "rr"];

function formatError(e: unknown): string {
  return e instanceof Error ? e.message : String(e);
}

export default function FaultPanel() {
  const pushToast = useSimStore((s) => s.pushToast);
  const faultActive = useSimStore((s) => s.state?.fault_active ?? false);

  const [faults, setFaults] = useState<FaultEntry[]>([]);
  const [newType, setNewType] = useState<FaultType>("motor_stuck_zero");
  const [newWheel, setNewWheel] = useState<WheelName>("fl");
  const [newValue, setNewValue] = useState<number>(0.0);
  const [busy, setBusy] = useState(false);
  const [listError, setListError] = useState<string | null>(null);

  const refresh = () =>
    fetchJSON<{ faults: FaultEntry[] }>("/api/faults")
      .then((r) => {
        setFaults(r.faults);
        setListError(null);
      })
      .catch((e) => {
        const message = `读取故障失败：${formatError(e)}`;
        setFaults([]);
        setListError(message);
        pushToast("error", message);
      });

  useEffect(() => { refresh(); }, []); // eslint-disable-line react-hooks/exhaustive-deps

  const add = async () => {
    setBusy(true);
    try {
      const f = await postJSON<FaultEntry>("/api/faults", {
        fault_type: newType,
        wheel: newWheel,
        value: newValue,
        active: true,
      });
      setFaults((prev) => [...prev, f]);
      setListError(null);
    } catch (e) {
      pushToast("error", `添加故障失败：${formatError(e)}`);
    } finally {
      setBusy(false);
    }
  };

  const toggle = async (id: string, active: boolean) => {
    try {
      const updated = await patchJSON<FaultEntry>(`/api/faults/${id}`, { active });
      setFaults((prev) => prev.map((f) => (f.id === id ? updated : f)));
    } catch (e) {
      pushToast("error", `切换状态失败：${formatError(e)}`);
    }
  };

  const remove = async (id: string) => {
    try {
      await deleteJSON(`/api/faults/${id}`);
      setFaults((prev) => prev.filter((f) => f.id !== id));
    } catch (e) {
      pushToast("error", `删除故障失败：${formatError(e)}`);
    }
  };

  const clearAll = async () => {
    try {
      await deleteJSON("/api/faults");
      setFaults([]);
      setListError(null);
    } catch (e) {
      pushToast("error", `清除失败：${formatError(e)}`);
    }
  };

  const needsValue = VALUE_HINT[newType] !== null;

  return (
    <Panel
      title="故障注入"
      help={HELP.fault}
      badge={faultActive && (
        <span style={{
          fontSize: 10, padding: "1px 5px", borderRadius: 4,
          background: "#ef4444", color: "#fff", marginLeft: "auto",
        }}>
          ● 激活
        </span>
      )}
    >

      {/* Add fault row */}
      <div style={{ display: "flex", flexWrap: "wrap", gap: 4, marginBottom: 6 }}>
        <select value={newType} onChange={(e) => setNewType(e.target.value as FaultType)} style={selectStyle}>
          {FAULT_TYPES.map((t) => (
            <option key={t} value={t}>{FAULT_LABELS[t]}</option>
          ))}
        </select>
        <select value={newWheel} onChange={(e) => setNewWheel(e.target.value as WheelName)} style={{ ...selectStyle, width: 52 }}>
          {WHEEL_NAMES.map((w) => (
            <option key={w} value={w}>{WHEEL_LABELS[w]}</option>
          ))}
        </select>
        {needsValue && (
          <input
            type="number"
            step={0.01}
            value={newValue}
            onChange={(e) => setNewValue(Number(e.target.value))}
            placeholder={VALUE_HINT[newType] ?? ""}
            title={VALUE_HINT[newType] ?? ""}
            style={{ ...numInput, width: 70 }}
          />
        )}
        <button onClick={add} disabled={busy}>+ 添加</button>
        <button onClick={refresh} disabled={busy}>刷新故障</button>
      </div>

      {/* Fault list */}
      {listError ? (
        <div className="panel-small" role="alert" style={{ color: "var(--bad)" }}>{listError}</div>
      ) : faults.length === 0 ? (
        <div className="panel-small" style={{ color: "var(--muted)" }}>暂无故障配置</div>
      ) : (
        <>
          {faults.map((f) => (
            <div key={f.id} style={{
              display: "flex", alignItems: "center", gap: 4,
              marginBottom: 3, opacity: f.active ? 1 : 0.45,
            }}>
              <span style={{
                width: 8, height: 8, borderRadius: "50%",
                background: f.active ? "#ef4444" : "var(--muted)",
                flexShrink: 0,
              }} />
              <span className="panel-small" style={{ flex: 1, color: "var(--text)" }}>
                {WHEEL_LABELS[f.wheel]} · {FAULT_LABELS[f.fault_type]}
                {VALUE_HINT[f.fault_type] !== null && ` (${f.value.toFixed(3)})`}
              </span>
              <button
                style={{ fontSize: 10, padding: "1px 5px" }}
                onClick={() => toggle(f.id, !f.active)}
              >
                {f.active ? "停用" : "启用"}
              </button>
              <button
                style={{ fontSize: 10, padding: "1px 5px" }}
                onClick={() => remove(f.id)}
              >
                ×
              </button>
            </div>
          ))}
          <button
            style={{ marginTop: 4, fontSize: 11 }}
            onClick={clearAll}
          >
            清空全部故障
          </button>
        </>
      )}

      <div className="panel-small" style={{ color: "var(--muted)", marginTop: 6, lineHeight: 1.4 }}>
        执行器故障影响实际转角；传感器故障影响上报值（不影响物理）。
      </div>
    </Panel>
  );
}
