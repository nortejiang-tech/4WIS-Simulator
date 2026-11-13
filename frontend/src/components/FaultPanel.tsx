/**
 * FaultPanel — multi-point fault timeline editor.
 *
 * Builds a local draft timeline (one entry per checked wheel, each addition may
 * use a different fault type / relative delay) and applies it atomically via
 * POST /api/faults/schedule, which REPLACES the whole configured schedule.
 * The applied schedule is polled from GET /api/faults/schedule once per second
 * and is kept strictly separate from the editable draft list.
 */

import { useEffect, useRef, useState, type CSSProperties } from "react";
import { fetchJSON, postJSON, deleteJSON, patchJSON } from "@/api/http";
import { useSimStore } from "@/store/sim";
import { selectStyle, numInput } from "@/ui/styles";
import Panel from "@/components/Panel";

type WheelName = "fl" | "fr" | "rl" | "rr";
type FaultType =
  | "motor_stuck_zero"
  | "motor_stuck_hold"
  | "motor_stuck_angle"
  | "motor_free_caster"
  | "motor_limited_range"
  | "sensor_bias"
  | "sensor_noise"
  | "sensor_dropout";

type Phase = "scheduled" | "active" | "completed" | "disabled";

/** One configured schedule entry as returned by GET /api/faults/schedule. */
interface FaultEntry {
  id: string;
  fault_type: FaultType;
  wheel: WheelName;
  value: number;
  active: boolean;
  t_start: number;
  t_end: number | null;
  effective_active: boolean;
  phase: Phase;
  eta_rev?: number;
  j_steer?: number;
  c_damp?: number;
  tau_coulomb?: number;
}

interface ScheduleEvent {
  id: string;
  event: string;
  time: number;
  wheel?: number | null;
  fault_type?: string | null;
}

interface ScheduleResponse {
  clock: string;
  time: number;
  seed: number;
  faults: FaultEntry[];
  events: ScheduleEvent[];
}

/** Editable draft row; delays/durations are relative seconds, value may be blank. */
interface Draft {
  key: number;
  fault_type: FaultType;
  wheel: WheelName;
  value: string;        // "" = omit / default
  delay: string;        // relative start delay, >= 0
  duration: string;     // "" = persistent (t_end null), else > 0
  j_steer: string;
  c_damp: string;
  tau_coulomb: string;
  eta_rev: string;
}

const FAULT_LABELS: Record<FaultType, string> = {
  motor_stuck_zero:    "故障目标归零",
  motor_stuck_angle:   "故障固定目标角",
  motor_stuck_hold:    "机械卡滞（当前位置）",
  motor_free_caster:   "失电自由转向",
  motor_limited_range: "执行器角度限幅",
  sensor_bias:         "传感器偏差",
  sensor_noise:        "传感器噪声",
  sensor_dropout:      "传感器掉线（保持最后值）",
};

const WHEEL_LABELS: Record<WheelName, string> = { fl: "FL", fr: "FR", rl: "RL", rr: "RR" };

const PHASE_LABELS: Record<Phase, string> = {
  scheduled: "已排程",
  active:    "进行中",
  completed: "已完成",
  disabled:  "已停用",
};

/** Fault types whose value field is meaningful; value is optional everywhere. */
const VALUE_HINT: Partial<Record<FaultType, string>> = {
  motor_stuck_angle:   "目标角 (rad)",
  motor_stuck_zero:    "可留空",
  motor_limited_range: "最大角 (rad)",
  sensor_bias:         "偏差 (rad)",
  sensor_noise:        "噪声σ (rad)",
  sensor_dropout:      "可留空",
};

const FREE_CASTER = "motor_free_caster";

/** Free-caster passive parameters: label, unit, default. */
const PASSIVE_PARAMS = [
  { key: "j_steer",     label: "转向惯量", unit: "kg·m²", def: "3" },
  { key: "c_damp",      label: "转向阻尼", unit: "N·m·s/rad", def: "80" },
  { key: "tau_coulomb", label: "库仑摩擦", unit: "N·m",   def: "5" },
  { key: "eta_rev",     label: "反驱效率", unit: "-",     def: "0.6" },
] as const;

type PassiveKey = (typeof PASSIVE_PARAMS)[number]["key"];

const FAULT_TYPES = Object.keys(FAULT_LABELS) as FaultType[];
const WHEEL_NAMES: WheelName[] = ["fl", "fr", "rl", "rr"];

const SCOPE_NOTE =
  "机械卡滞/自由转向需动力学与独立转向；共享齿条不支持；传感器只改观测不进反馈；仿真暂停时计时暂停。";

function formatError(e: unknown): string {
  const raw = e instanceof Error ? e.message : String(e);
  if (raw.includes("overlapping")) return "同一车轮的故障时间窗口冲突，请调整起止时间或移除重复故障。";
  if (raw.includes("coupled racks")) return "当前转向结构使用共享齿条，请选择独立 4WIS 转向配置。";
  if (raw.includes("requires a force-based")) return "机械卡滞和自由转向需要简化动力学或多体动力学模型。";
  if (raw.includes("steer_limit")) return "故障角度超出车辆转向机械限位，请减小角度。";
  try {
    const detail: unknown = JSON.parse(raw);
    if (Array.isArray(detail)) {
      return detail.map((item: {msg?: string}) => item.msg ?? "参数无效").join("；");
    }
  } catch { /* Plain-text server/network errors remain readable. */ }
  return raw;
}

function fmt(x: number, digits = 1): string {
  return Number.isFinite(x) ? x.toFixed(digits) : "—";
}

function fmtOpt(x: number | null | undefined, digits = 1): string {
  return x != null && Number.isFinite(x) ? x.toFixed(digits) : "持续";
}

/** Parse a numeric text field; returns null when blank/NaN/non-finite. */
function parseNum(text: string): number | null {
  const t = text.trim();
  if (t === "") return null;
  const n = Number(t);
  return Number.isFinite(n) ? n : null;
}

function makeDraft(fault_type: FaultType, wheel: WheelName, delay: string, duration: string): Draft {
  return {
    key: Math.random(),
    fault_type,
    wheel,
    value: "",
    delay,
    duration,
    j_steer: "3",
    c_damp: "80",
    tau_coulomb: "5",
    eta_rev: "0.6",
  };
}

const labelStyle: CSSProperties = { color: "var(--muted)", fontSize: 10 };
const rowStyle: CSSProperties = { display: "flex", flexWrap: "wrap", alignItems: "center", gap: 4 };
const smallBtn: CSSProperties = { fontSize: 10, padding: "1px 5px" };

export default function FaultPanel() {
  const pushToast = useSimStore((s) => s.pushToast);
  const simTime = useSimStore((s) => s.state?.t ?? 0);
  const faultActive = useSimStore((s) => s.state?.fault_active ?? false);

  // ---- applied schedule (backend authority, polled) ----
  const [schedule, setSchedule] = useState<ScheduleResponse | null>(null);
  const [pollError, setPollError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [applyError, setApplyError] = useState<string | null>(null);

  // ---- local draft builder ----
  const [drafts, setDrafts] = useState<Draft[]>([]);
  const [draftType, setDraftType] = useState<FaultType>("motor_stuck_hold");
  const [draftValue, setDraftValue] = useState<string>("");
  const [draftDelay, setDraftDelay] = useState<string>("2");
  const [draftDuration, setDraftDuration] = useState<string>("3");
  const [draftPassive, setDraftPassive] = useState<Record<PassiveKey, string>>({
    j_steer: "3", c_damp: "80", tau_coulomb: "5", eta_rev: "0.6",
  });
  const [checkedWheels, setCheckedWheels] = useState<Record<WheelName, boolean>>({
    fl: true, fr: true, rl: false, rr: false,
  });
  const [seedText, setSeedText] = useState<string>("0");
  const [draftError, setDraftError] = useState<string | null>(null);

  const busyRef = useRef(busy);
  busyRef.current = busy;

  const loadSchedule = (opts: { silent?: boolean } = {}) =>
    fetchJSON<ScheduleResponse>("/api/faults/schedule")
      .then((r) => {
        setSchedule(r);
        setPollError(null);
        return r;
      })
      .catch((e) => {
        const message = `读取时间线失败：${formatError(e)}`;
        if (!opts.silent) {
          setPollError(message);
          pushToast("error", message);
        } else {
          setPollError(message);
        }
        return null;
      });

  // Initial load + 1 s polling with cleanup. Poll failures set one error state
  // and never toast (no spam); drafts are never touched by polling.
  useEffect(() => {
    let alive = true;
    void loadSchedule();
    const timer = window.setInterval(() => {
      if (!alive || busyRef.current) return;
      fetchJSON<ScheduleResponse>("/api/faults/schedule")
        .then((r) => {
          if (!alive) return;
          setSchedule(r);
          setPollError(null);
        })
        .catch((e) => {
          if (alive) setPollError(`读取时间线失败：${formatError(e)}`);
        });
    }, 1000);
    return () => {
      alive = false;
      window.clearInterval(timer);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // ---- draft builder actions ----
  const addToTimeline = () => {
    const wheels = WHEEL_NAMES.filter((w) => checkedWheels[w]);
    if (wheels.length === 0) {
      setDraftError("请至少勾选一个车轮。");
      return;
    }
    setDrafts((prev) => [
      ...prev,
      ...wheels.map((w) =>
        ({ ...makeDraft(draftType, w, draftDelay, draftDuration), value: draftValue, ...draftPassive }),
      ),
    ]);
    setDraftError(null);
    setApplyError(null);
  };

  const applyPreset = (which: "both_front" | "staggered") => {
    if (which === "both_front") {
      setDrafts([
        makeDraft("motor_stuck_hold", "fl", "2", "3"),
        makeDraft("motor_stuck_hold", "fr", "2", "3"),
      ]);
    } else {
      setDrafts([
        makeDraft("motor_stuck_hold", "fl", "2", "4"),
        makeDraft("motor_free_caster", "rr", "4", "3"),
      ]);
    }
    setDraftError(null);
    setApplyError(null);
  };

  const updateDraft = (key: number, patch: Partial<Draft>) =>
    setDrafts((prev) => prev.map((d) => (d.key === key ? { ...d, ...patch } : d)));

  const removeDraft = (key: number) =>
    setDrafts((prev) => prev.filter((d) => d.key !== key));

  // ---- apply draft as the whole schedule (atomic replace) ----
  const applyTimeline = async () => {
    setDraftError(null);
    setApplyError(null);

    if (drafts.length === 0) {
      setDraftError("时间线为空：请先“+ 加入时间线”添加至少一条故障。");
      return;
    }
    const seed = parseNum(seedText);
    if (seed === null || !Number.isInteger(seed) || seed < 0 || seed > 4294967295) {
      setDraftError("随机种子必须是 0 到 4294967295 的整数。");
      return;
    }

    // Validate every draft field up-front; never clamp silently.
    const rows: { d: Draft; value: number | null; delay: number; duration: number | null; passive: Record<PassiveKey, number> }[] = [];
    for (let i = 0; i < drafts.length; i++) {
      const d = drafts[i];
      const tag = `第 ${i + 1} 条（${WHEEL_LABELS[d.wheel]}）`;
      const value = parseNum(d.value);
      const delay = parseNum(d.delay);
      const duration = d.duration.trim() === "" ? null : parseNum(d.duration);

      if (delay === null || delay < 0) {
        setDraftError(`${tag}：起始延时必须是不小于 0 的数值。`);
        return;
      }
      if (d.duration.trim() !== "" && (duration === null || duration <= 0)) {
        setDraftError(`${tag}：持续秒数必须大于 0（留空表示持续生效）。`);
        return;
      }
      if (d.value.trim() !== "" && value === null) {
        setDraftError(`${tag}：故障数值必须是有限数字。`); return;
      }
      if (["motor_limited_range", "sensor_noise"].includes(d.fault_type) && value !== null && value < 0) {
        setDraftError(`${tag}：限幅和噪声标准差不能为负。`); return;
      }
      const passive = {} as Record<PassiveKey, number>;
      if (d.fault_type === FREE_CASTER) {
        for (const p of PASSIVE_PARAMS) {
          const v = parseNum(d[p.key]);
          if (v === null || v < 0) {
            setDraftError(`${tag}：${p.label}必须是不小于 0 的数值。`);
            return;
          }
          if ((p.key === "j_steer" && v <= 0) || (p.key === "eta_rev" && v > 1)) {
            setDraftError(`${tag}：转向惯量必须大于 0，反驱效率须在 0 到 1 之间。`); return;
          }
          passive[p.key] = v;
        }
      }
      rows.push({ d, value, delay, duration, passive });
    }

    setBusy(true);
    try {
      const now = await loadSchedule({ silent: true });
      if (!now || !Number.isFinite(now.time)) {
        setApplyError("无法读取当前仿真时间，未提交时间线。");
        return;
      }
      const base = now.time;
      const faults = rows.map(({ d, value, delay, duration, passive }) => {
        const t_start = base + delay;
        const entry: Record<string, unknown> = {
          fault_type: d.fault_type,
          wheel: d.wheel,
          value: value ?? 0,
          active: true,
          t_start,
          t_end: duration === null ? null : t_start + duration,
        };
        if (d.fault_type === FREE_CASTER) {
          entry.j_steer = passive.j_steer;
          entry.c_damp = passive.c_damp;
          entry.tau_coulomb = passive.tau_coulomb;
          entry.eta_rev = passive.eta_rev;
        }
        return entry;
      });

      await postJSON("/api/faults/schedule", { seed, faults });
      const fresh = await loadSchedule({ silent: true });
      if (!fresh) {
        // Submission succeeded but the refresh failed: say so honestly.
        setApplyError("时间线已提交，但刷新当前时间线失败，请点“刷新故障”。");
        pushToast("info", "时间线已提交（刷新失败）");
        return;
      }
      pushToast("info", `已应用 ${faults.length} 条故障时间线（替换原计划）`);
      // Success: drafts persist untouched for further editing.
    } catch (e) {
      setApplyError(`应用时间线失败：${formatError(e)}`);
      pushToast("error", `应用时间线失败：${formatError(e)}`);
    } finally {
      setBusy(false);
    }
  };

  // ---- applied-schedule actions (legacy endpoints) ----
  const toggleFault = async (id: string, active: boolean) => {
    try {
      const updated = await patchJSON<FaultEntry>(`/api/faults/${id}`, { active });
      setSchedule((prev) =>
        prev ? { ...prev, faults: prev.faults.map((f) => (f.id === id ? { ...f, ...updated } : f)) } : prev,
      );
    } catch (e) {
      pushToast("error", `切换状态失败：${formatError(e)}`);
    }
  };

  const removeFault = async (id: string) => {
    try {
      await deleteJSON(`/api/faults/${id}`);
      setSchedule((prev) => (prev ? { ...prev, faults: prev.faults.filter((f) => f.id !== id) } : prev));
    } catch (e) {
      pushToast("error", `删除故障失败：${formatError(e)}`);
    }
  };

  const clearAll = async () => {
    try {
      await deleteJSON("/api/faults");
      await loadSchedule({ silent: true });
      pushToast("info", "已清除全部故障计划");
    } catch (e) {
      pushToast("error", `清除失败：${formatError(e)}`);
    }
  };

  const refreshNow = () => {
    void loadSchedule();
  };

  const faults = schedule?.faults ?? [];
  const events = (schedule?.events ?? []).slice(-8).reverse();
  const errText = [pollError, applyError].filter(Boolean).join("；");
  const needsValue = VALUE_HINT[draftType] !== undefined;

  return (
    <Panel
      title="故障注入"
      help={
        <div style={{ fontSize: 11, lineHeight: 1.5 }}>
          先本地编排“草稿时间线”，再“应用时间线”一次性提交。应用会<b>替换</b>当前故障计划；
          相对延时以提交时刻的仿真时间为基准。范围说明：{SCOPE_NOTE}
        </div>
      }
      badge={faultActive && (
        <span style={{
          fontSize: 10, padding: "1px 5px", borderRadius: 4,
          background: "#ef4444", color: "#fff", marginLeft: "auto",
        }}>
          ● 激活
        </span>
      )}
    >

      {/* Simulation clock + true active state */}
      <div style={{ ...rowStyle, marginBottom: 6 }}>
        <span className="panel-small" style={{ color: "var(--muted)" }}>
          仿真时间 <b style={{ color: "var(--text)" }}>{fmt(simTime, 1)} s</b>
        </span>
        <span className="panel-small" style={{ color: faultActive ? "var(--bad)" : "var(--muted)" }}>
          {faultActive ? "故障生效中" : "无激活故障"}
        </span>
        <button style={{ ...smallBtn, marginLeft: "auto" }} onClick={refreshNow} disabled={busy}>
          刷新故障
        </button>
      </div>

      {/* ---------- Draft timeline builder ---------- */}
      <div style={{ borderTop: "1px solid var(--border)", paddingTop: 6 }}>
        <div className="panel-small" style={{ color: "var(--muted)", marginBottom: 4 }}>
          草稿时间线（本地，未提交）
        </div>

        <div style={{ ...rowStyle, marginBottom: 4 }}>
          <label style={labelStyle}>
            故障类型
            <select
              aria-label="草稿故障类型"
              value={draftType}
              onChange={(e) => setDraftType(e.target.value as FaultType)}
              style={selectStyle}
            >
              {FAULT_TYPES.map((t) => (
                <option key={t} value={t}>{FAULT_LABELS[t]}</option>
              ))}
            </select>
          </label>
        </div>

        <div style={{ ...rowStyle, marginBottom: 4 }}>
          <div style={labelStyle}>
            车轮
            <span style={{ display: "flex", gap: 6, alignItems: "center" }}>
              {WHEEL_NAMES.map((w) => (
                <label key={w} style={{ ...labelStyle, display: "flex", alignItems: "center", gap: 2 }}>
                  <input
                    type="checkbox"
                    aria-label={`草稿车轮 ${WHEEL_LABELS[w]}`}
                    checked={checkedWheels[w]}
                    onChange={(e) => setCheckedWheels((p) => ({ ...p, [w]: e.target.checked }))}
                  />
                  {WHEEL_LABELS[w]}
                </label>
              ))}
            </span>
          </div>
        </div>

        <div style={{ ...rowStyle, marginBottom: 4 }}>
          <label style={labelStyle}>
            数值
            <input
              type="number"
              step={0.01}
              aria-label={`故障数值 rad，${VALUE_HINT[draftType] ?? "可选"}`}
              value={draftValue}
              onChange={(e) => setDraftValue(e.target.value)}
              placeholder={needsValue ? (VALUE_HINT[draftType] ?? "") : "可选"}
              style={{ ...numInput, width: 78 }}
            />
          </label>
          <label style={labelStyle}>
            延时 s
            <input
              type="number"
              step={0.1}
              min={0}
              aria-label="相对起始延时（秒，不小于 0）"
              value={draftDelay}
              onChange={(e) => setDraftDelay(e.target.value)}
              style={{ ...numInput, width: 62 }}
            />
          </label>
          <label style={labelStyle}>
            持续 s
            <input
              type="number"
              step={0.1}
              min={0}
              aria-label="持续秒数（大于 0，留空表示持续生效）"
              value={draftDuration}
              onChange={(e) => setDraftDuration(e.target.value)}
              placeholder="持续"
              style={{ ...numInput, width: 62 }}
            />
          </label>
        </div>

        {draftType === FREE_CASTER && (
          <div style={{ ...rowStyle, marginBottom: 4 }}>
            {PASSIVE_PARAMS.map((p) => (
              <label key={p.key} style={labelStyle}>
                {p.label} ({p.unit})
                <input
                  type="number"
                  step={0.1}
                  min={0}
                  aria-label={`自由转向参数 ${p.label}，单位 ${p.unit}，默认 ${p.def}`}
                  value={draftPassive[p.key]}
                  onChange={(e) => setDraftPassive((prev) => ({ ...prev, [p.key]: e.target.value }))}
                  style={{ ...numInput, width: 60, marginLeft: 2 }}
                />
              </label>
            ))}
          </div>
        )}

        <div style={{ ...rowStyle, marginBottom: 4 }}>
          <button onClick={addToTimeline} disabled={busy}>+ 加入时间线</button>
          <button style={smallBtn} onClick={() => applyPreset("both_front")} disabled={busy}>
            双前轮同时
          </button>
          <button style={smallBtn} onClick={() => applyPreset("staggered")} disabled={busy}>
            前后轮错时
          </button>
        </div>

        {draftError && (
          <div className="panel-small" role="alert" style={{ color: "var(--bad)" }}>{draftError}</div>
        )}

        {drafts.length === 0 ? (
          <div className="panel-small" style={{ color: "var(--muted)" }}>
            暂无草稿条目
          </div>
        ) : (
          <div style={{ marginBottom: 4 }}>
            {drafts.map((d, i) => (
              <div key={d.key} style={{ ...rowStyle, marginBottom: 2, fontSize: 11 }}>
                <span style={{ color: "var(--muted)", width: 16 }}>{i + 1}.</span>
                <label style={labelStyle}>
                  类型
                  <select
                    aria-label={`第 ${i + 1} 条故障类型`}
                    value={d.fault_type}
                    onChange={(e) => updateDraft(d.key, { fault_type: e.target.value as FaultType })}
                    style={{ ...selectStyle, maxWidth: 150 }}
                  >
                    {FAULT_TYPES.map((t) => (
                      <option key={t} value={t}>{FAULT_LABELS[t]}</option>
                    ))}
                  </select>
                </label>
                <label style={labelStyle}>
                  轮
                  <select
                    aria-label={`第 ${i + 1} 条故障车轮`}
                    value={d.wheel}
                    onChange={(e) => updateDraft(d.key, { wheel: e.target.value as WheelName })}
                    style={{ ...selectStyle, width: 60 }}
                  >
                    {WHEEL_NAMES.map((w) => (
                      <option key={w} value={w}>{WHEEL_LABELS[w]}</option>
                    ))}
                  </select>
                </label>
                <label style={labelStyle}>
                  值
                  <input
                    type="number"
                    step={0.01}
                    aria-label={`第 ${i + 1} 条故障数值 rad，可留空`}
                    value={d.value}
                    onChange={(e) => updateDraft(d.key, { value: e.target.value })}
                    placeholder="—"
                    style={{ ...numInput, width: 54 }}
                  />
                </label>
                <label style={labelStyle}>
                  延时
                  <input
                    type="number"
                    step={0.1}
                    min={0}
                    aria-label={`第 ${i + 1} 条起始延时秒`}
                    value={d.delay}
                    onChange={(e) => updateDraft(d.key, { delay: e.target.value })}
                    style={{ ...numInput, width: 52 }}
                  />
                </label>
                <label style={labelStyle}>
                  持续
                  <input
                    type="number"
                    step={0.1}
                    min={0}
                    aria-label={`第 ${i + 1} 条持续秒数，留空为持续`}
                    value={d.duration}
                    onChange={(e) => updateDraft(d.key, { duration: e.target.value })}
                    placeholder="持续"
                    style={{ ...numInput, width: 52 }}
                  />
                </label>
                {d.fault_type === FREE_CASTER && <span className="panel-small" style={{color: "var(--muted)", flexBasis: "100%"}}>
                  J={d.j_steer} kg·m² · c={d.c_damp} N·m·s/rad · τc={d.tau_coulomb} N·m · η={d.eta_rev}
                </span>}
                <button
                  style={smallBtn}
                  aria-label={`移除草稿第 ${i + 1} 条`}
                  title="移除该草稿"
                  onClick={() => removeDraft(d.key)}
                >
                  ×
                </button>
              </div>
            ))}
          </div>
        )}

        <div style={{ ...rowStyle, marginBottom: 4 }}>
          <label style={labelStyle}>
            随机种子
            <input
              type="number"
              step={1}
              min={0}
              aria-label="随机种子（不小于 0 的整数）"
              value={seedText}
              onChange={(e) => setSeedText(e.target.value)}
              style={{ ...numInput, width: 70, marginLeft: 2 }}
            />
          </label>
          <button onClick={applyTimeline} disabled={busy} style={{ marginLeft: "auto" }}>
            {busy ? "提交中…" : "应用时间线"}
          </button>
        </div>

        <div className="panel-small" style={{ color: "var(--muted)", marginBottom: 6 }}>
          应用会<b>替换</b>当前故障计划；相对延时以提交时刻仿真时间为基准，相同延时即同时发生。
        </div>
      </div>

      {/* ---------- Applied schedule (backend) ---------- */}
      <div style={{ borderTop: "1px solid var(--border)", paddingTop: 6 }}>
        <div className="panel-small" style={{ color: "var(--muted)", marginBottom: 4 }}>
          当前故障计划（仿真时刻 t={fmt(schedule?.time ?? NaN, 1)} s）
        </div>

        {errText && (
          <div className="panel-small" role="alert" style={{ color: "var(--bad)", marginBottom: 4 }}>
            {errText}
          </div>
        )}

        {pollError && !schedule ? null : faults.length === 0 ? (
          <div className="panel-small" style={{ color: "var(--muted)" }}>暂无故障配置</div>
        ) : (
          <>
            {faults.map((f) => (
              <div key={f.id} style={{
                display: "flex", flexWrap: "wrap", alignItems: "center", gap: 4,
                marginBottom: 3, opacity: f.active ? 1 : 0.45,
              }}>
                <span style={{
                  width: 8, height: 8, borderRadius: "50%",
                  background: f.effective_active ? "#ef4444" : "var(--muted)",
                  flexShrink: 0,
                }} />
                <span className="panel-small" style={{ flex: "1 1 100%", color: "var(--text)" }}>
                  {WHEEL_LABELS[f.wheel]} · {FAULT_LABELS[f.fault_type] ?? f.fault_type}
                  {VALUE_HINT[f.fault_type] !== undefined && ` (${fmt(f.value, 3)})`}
                </span>
                <span className="panel-small" style={{ color: "var(--muted)" }}>
                  {PHASE_LABELS[f.phase] ?? f.phase} · {fmt(f.t_start, 3)}→{fmtOpt(f.t_end, 3)}
                </span>
                <button style={smallBtn} onClick={() => toggleFault(f.id, !f.active)}>
                  {f.active ? "停用" : "启用"}
                </button>
                <button style={smallBtn} aria-label={`删除故障 ${WHEEL_LABELS[f.wheel]}`} onClick={() => removeFault(f.id)}>
                  ×
                </button>
              </div>
            ))}
            <button style={{ marginTop: 4, fontSize: 11 }} onClick={clearAll}>
              清空全部故障
            </button>
          </>
        )}
      </div>

      {/* ---------- Recent events ---------- */}
      <div style={{ borderTop: "1px solid var(--border)", paddingTop: 6, marginTop: 6 }}>
        <div className="panel-small" style={{ color: "var(--muted)", marginBottom: 4 }}>
          最近事件（最多 8 条）
        </div>
        {events.length === 0 ? (
          <div className="panel-small" style={{ color: "var(--muted)" }}>暂无事件</div>
        ) : (
          events.map((ev, index) => (
            <div key={`${ev.id}-${ev.event}-${ev.time}-${index}`} className="panel-small" style={{ color: "var(--muted)" }}>
              t={fmt(ev.time, 3)}s · {({activated: "触发", deactivated: "结束", removed: "移除"} as Record<string, string>)[ev.event] ?? ev.event}
              {ev.wheel != null ? ` · ${WHEEL_NAMES[ev.wheel]?.toUpperCase()}` : ""}
              {ev.fault_type ? ` · ${FAULT_LABELS[ev.fault_type as FaultType] ?? ev.fault_type}` : ""}
            </div>
          ))
        )}
      </div>

      <div className="panel-small" style={{ color: "var(--muted)", marginTop: 6, lineHeight: 1.4 }}>
        范围：{SCOPE_NOTE}
      </div>
    </Panel>
  );
}
