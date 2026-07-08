/**
 * ExperimentPage — the "试验" workflow stage (platform refactor Phase B).
 *
 * Compose a reproducible experiment (maneuver segments × speed targets ×
 * strategy/model/path), optionally expand it into a variant matrix
 * (strategies × speeds), launch a headless batch on the backend, watch
 * progress, and jump to the analysis page with the produced runs selected.
 *
 * The editor edits the full backend Experiment schema; "运行" always sends the
 * *current editor state* inline (unsaved edits run as-is — saving to the
 * library is an explicit separate act, like CarMaker TestRuns).
 */

import { useEffect, useRef, useState } from "react";

import {
  BatchStatus,
  ExperimentListItem,
  ExperimentT,
  ManeuverStepT,
  SteerKind,
  VariantT,
  defaultExperiment,
  defaultStep,
  deleteExperiment,
  getBatch,
  getExperiment,
  listExperiments,
  maneuverTemplates,
  saveExperiment,
  startBatch,
} from "@/api/experiments";
import { useSimStore } from "@/store/sim";
import "./WorkflowPage.css";

const MODEL_OPTIONS = [
  { id: "kinematic", label: "运动学" },
  { id: "simplified_dynamic", label: "动力学" },
  { id: "multibody", label: "多体(14DOF)" },
];

const STEER_KINDS: { id: SteerKind; label: string }[] = [
  { id: "constant", label: "恒值" },
  { id: "step", label: "角阶跃" },
  { id: "ramp", label: "斜坡" },
  { id: "sine", label: "单频正弦" },
  { id: "sweep", label: "扫频" },
  { id: "dlc", label: "双移线" },
];

const KPI_COLUMNS: { key: string; label: string; digits: number }[] = [
  { key: "yaw_rate_peak_dps", label: "横摆峰值°/s", digits: 1 },
  { key: "rack_force_peak_n", label: "齿条峰值N", digits: 0 },
  { key: "icr_dev_rms_m", label: "瞬心RMS m", digits: 2 },
  { key: "steer_energy_nms", label: "能耗N·m·s", digits: 0 },
];

export default function ExperimentPage() {
  const strategies = useSimStore((s) => s.strategies);
  const pushToast = useSimStore((s) => s.pushToast);
  const setPage = useSimStore((s) => s.setPage);
  const setAnalysisPreselect = useSimStore((s) => s.setAnalysisPreselect);

  const [exps, setExps] = useState<ExperimentListItem[]>([]);
  const [expListError, setExpListError] = useState<string | null>(null);
  const [exp, setExp] = useState<ExperimentT>(() => defaultExperiment());
  const [pathTemplates, setPathTemplates] = useState<string[]>([]);
  const [pathTemplateError, setPathTemplateError] = useState<string | null>(null);
  const [stratSel, setStratSel] = useState<string[]>([]);
  const [speedsText, setSpeedsText] = useState("");
  const [job, setJob] = useState<BatchStatus | null>(null);
  const pollRef = useRef<number | null>(null);

  const refreshList = () => listExperiments()
    .then((nextExps) => {
      setExps(nextExps);
      setExpListError(null);
    })
    .catch((err) => {
      setExps([]);
      const message = `读取实验库失败：${(err as Error).message}`;
      setExpListError(message);
      pushToast("error", message);
    });

  useEffect(() => {
    refreshList();
    maneuverTemplates()
      .then((d) => {
        setPathTemplates(d.path_templates);
        setPathTemplateError(null);
      })
      .catch((err) => {
        setPathTemplates([]);
        const message = `读取机动模板失败：${(err as Error).message}`;
        setPathTemplateError(message);
        pushToast("error", message);
      });
    return () => { if (pollRef.current != null) window.clearInterval(pollRef.current); };
  }, []);

  const patch = (p: Partial<ExperimentT>) => setExp((e) => ({ ...e, ...p }));
  const patchStep = (i: number, p: Partial<ManeuverStepT>) =>
    setExp((e) => {
      const steps = e.maneuver.steps.map((s, k) => (k === i ? { ...s, ...p } : s));
      return { ...e, maneuver: { ...e.maneuver, steps } };
    });
  const patchSteer = (i: number, p: Partial<ManeuverStepT["steer"]>) =>
    setExp((e) => {
      const steps = e.maneuver.steps.map((s, k) => (k === i ? { ...s, steer: { ...s.steer, ...p } } : s));
      return { ...e, maneuver: { ...e.maneuver, steps } };
    });

  const load = (name: string) =>
    getExperiment(name).then(setExp).catch((err) => pushToast("error", `读取失败：${err.message}`));

  const save = () =>
    saveExperiment(exp)
      .then(() => { pushToast("info", `已保存：${exp.name}`); refreshList(); })
      .catch((err) => pushToast("error", `保存失败：${err.message}`));

  const remove = (name: string) =>
    deleteExperiment(name)
      .then(() => { pushToast("info", `已删除：${name}`); refreshList(); })
      .catch((err) => pushToast("error", `删除失败：${err.message}`));

  // ---- variant matrix ------------------------------------------------------

  const speeds = speedsText
    .split(/[,，\s]+/)
    .map((s) => Number(s))
    .filter((v) => Number.isFinite(v) && v > 0);

  const variants: VariantT[] = [];
  const stratAxis = stratSel.length > 0 ? stratSel : [null];
  const speedAxis = speeds.length > 0 ? speeds : [null];
  for (const s of stratAxis) {
    for (const v of speedAxis) {
      if (s == null && v == null) continue;
      variants.push({
        label: [s, v != null ? `${v}km/h` : null].filter(Boolean).join(" @ "),
        overrides: {
          ...(s != null ? { strategy: s } : {}),
          ...(v != null ? { "maneuver.steps.0.speed_kmh": v } : {}),
        },
      });
    }
  }

  const run = async () => {
    try {
      const jobId = await startBatch(exp, variants);
      setJob({ job_id: jobId, status: "running", total: Math.max(1, variants.length),
               done: 0, current_label: "", error: "", elapsed_s: 0, runs: [] });
      if (pollRef.current != null) window.clearInterval(pollRef.current);
      pollRef.current = window.setInterval(async () => {
        try {
          const st = await getBatch(jobId);
          setJob(st);
          if (st.status !== "running" && pollRef.current != null) {
            window.clearInterval(pollRef.current);
            pollRef.current = null;
          }
        } catch { /* poll again */ }
      }, 400);
    } catch (err) {
      pushToast("error", `启动失败：${(err as Error).message}`);
    }
  };

  const goAnalysis = () => {
    if (job) setAnalysisPreselect(job.runs.map((r) => r.run_id));
    setPage("analysis");
  };

  const running = job?.status === "running";

  return (
    <div className="wf-page wf-3col">
      {/* ── left: experiment library ─────────────────────────────── */}
      <aside className="wf-col">
        <div className="wf-col-head">
          <span>实验库</span>
          <span style={{ display: "flex", gap: 6 }}>
            <button className="wf-btn" onClick={refreshList}>刷新</button>
            <button className="wf-btn" onClick={() => setExp(defaultExperiment())}>＋ 新建</button>
          </span>
        </div>
        <div className="wf-list">
          {exps.map((e) => (
            <div key={e.name}
                 className={`wf-list-item ${e.name === exp.name ? "active" : ""}`}
                 onClick={() => load(e.name)}>
              <div className="wf-list-title">{e.name}</div>
              <div className="wf-list-sub">{e.strategy} · {e.model_type}</div>
              {e.description && <div className="wf-list-desc">{e.description}</div>}
              <button className="wf-x" title="删除"
                      onClick={(ev) => { ev.stopPropagation(); remove(e.name); }}>✕</button>
            </div>
          ))}
          {expListError ? (
            <div className="wf-empty" role="alert">{expListError}</div>
          ) : exps.length === 0 && <div className="wf-empty">暂无已保存实验</div>}
        </div>
      </aside>

      {/* ── center: editor ───────────────────────────────────────── */}
      <section className="wf-col wf-main">
        <div className="wf-col-head">
          <span>实验定义</span>
          <button className="wf-btn primary" onClick={save}>保存到实验库</button>
        </div>

        <div className="wf-form">
          <label className="wf-field">
            <span>名称</span>
            <input className="wf-input" value={exp.name}
                   onChange={(e) => patch({ name: e.target.value.replace(/[^\w\-一-鿿]/g, "_") })} />
          </label>
          <label className="wf-field wf-grow">
            <span>说明</span>
            <input className="wf-input" value={exp.description}
                   onChange={(e) => patch({ description: e.target.value })} />
          </label>
        </div>

        <div className="wf-form">
          <label className="wf-field">
            <span>控制策略</span>
            <select className="wf-input" value={exp.strategy}
                    onChange={(e) => patch({ strategy: e.target.value })}>
              {(strategies.length ? strategies : [exp.strategy]).map((s) => (
                <option key={s} value={s}>{s}</option>
              ))}
            </select>
          </label>
          <label className="wf-field">
            <span>动力学模型</span>
            <select className="wf-input" value={exp.model_type}
                    onChange={(e) => patch({ model_type: e.target.value })}>
              {MODEL_OPTIONS.map((m) => <option key={m.id} value={m.id}>{m.label}</option>)}
            </select>
          </label>
          <label className="wf-field">
            <span>参考路径</span>
            <select className="wf-input" value={exp.path?.template ?? ""}
                    onChange={(e) => patch({
                      path: e.target.value
                        ? { template: e.target.value, params: {}, waypoints: null, closed: false }
                        : null,
                    })}>
              <option value="">（无）</option>
              {pathTemplates.map((t) => <option key={t} value={t}>{t}</option>)}
            </select>
            {pathTemplateError && (
              <span className="wf-small" role="alert" style={{ color: "var(--bad)", marginTop: 4 }}>
                {pathTemplateError}
              </span>
            )}
          </label>
          <label className="wf-field">
            <span>记录频率 Hz</span>
            <input className="wf-input num" type="number" min={1} max={200} value={exp.record_hz}
                   onChange={(e) => patch({ record_hz: Number(e.target.value) || 50 })} />
          </label>
        </div>

        <div className="wf-subhead">
          机动分段（按仿真时间顺序执行）
          <button className="wf-btn" onClick={() =>
            setExp((e) => ({ ...e, maneuver: { ...e.maneuver, steps: [...e.maneuver.steps, defaultStep()] } }))
          }>＋ 加一段</button>
        </div>

        {exp.maneuver.steps.map((st, i) => (
          <div className="wf-step" key={i}>
            <div className="wf-step-head">
              <span className="wf-step-no">段 {i + 1}</span>
              <input className="wf-input" style={{ width: 120 }} placeholder="段名"
                     value={st.name} onChange={(e) => patchStep(i, { name: e.target.value })} />
              <button className="wf-x" title="删除此段" onClick={() =>
                setExp((e) => ({ ...e, maneuver: { ...e.maneuver, steps: e.maneuver.steps.filter((_, k) => k !== i) } }))
              }>✕</button>
            </div>
            <div className="wf-form">
              <label className="wf-field"><span>时长 s</span>
                <input className="wf-input num" type="number" min={0.1} step={0.5} value={st.duration}
                       onChange={(e) => patchStep(i, { duration: Number(e.target.value) || 1 })} /></label>
              <label className="wf-field"><span>目标车速 km/h</span>
                <input className="wf-input num" type="number" min={0} placeholder="保持"
                       value={st.speed_kmh ?? ""}
                       onChange={(e) => patchStep(i, { speed_kmh: e.target.value === "" ? null : Number(e.target.value) })} /></label>
              <label className="wf-field"><span>车速斜坡 s</span>
                <input className="wf-input num" type="number" min={0} step={0.5} value={st.speed_ramp_s}
                       onChange={(e) => patchStep(i, { speed_ramp_s: Number(e.target.value) || 0 })} /></label>
              <label className="wf-field"><span>转向剖面</span>
                <select className="wf-input" value={st.steer.kind}
                        onChange={(e) => patchSteer(i, { kind: e.target.value as SteerKind })}>
                  {STEER_KINDS.map((k) => <option key={k.id} value={k.id}>{k.label}</option>)}
                </select></label>
              <label className="wf-field"><span>幅值 (-1..1)</span>
                <input className="wf-input num" type="number" min={-1} max={1} step={0.01} value={st.steer.amplitude}
                       onChange={(e) => patchSteer(i, { amplitude: Number(e.target.value) || 0 })} /></label>
              {st.steer.kind === "sine" && (
                <label className="wf-field"><span>频率 Hz</span>
                  <input className="wf-input num" type="number" min={0.05} step={0.1} value={st.steer.freq_hz}
                         onChange={(e) => patchSteer(i, { freq_hz: Number(e.target.value) || 0.5 })} /></label>
              )}
              {st.steer.kind === "sweep" && (<>
                <label className="wf-field"><span>起始 Hz</span>
                  <input className="wf-input num" type="number" min={0.05} step={0.05} value={st.steer.f0_hz}
                         onChange={(e) => patchSteer(i, { f0_hz: Number(e.target.value) || 0.1 })} /></label>
                <label className="wf-field"><span>终止 Hz</span>
                  <input className="wf-input num" type="number" min={0.1} step={0.1} value={st.steer.f1_hz}
                         onChange={(e) => patchSteer(i, { f1_hz: Number(e.target.value) || 2 })} /></label>
              </>)}
              {st.steer.kind === "step" && (
                <label className="wf-field"><span>阶跃延时 s</span>
                  <input className="wf-input num" type="number" min={0} step={0.1} value={st.steer.t_step}
                         onChange={(e) => patchSteer(i, { t_step: Number(e.target.value) || 0 })} /></label>
              )}
              {st.steer.kind === "ramp" && (
                <label className="wf-field"><span>起始值</span>
                  <input className="wf-input num" type="number" min={-1} max={1} step={0.01} value={st.steer.start}
                         onChange={(e) => patchSteer(i, { start: Number(e.target.value) || 0 })} /></label>
              )}
            </div>
          </div>
        ))}

        <div className="wf-small" style={{ color: "var(--muted)", marginTop: 6, lineHeight: 1.6 }}>
          转向幅值是归一化驾驶员输入（理想阿克曼下为曲率分数）：60 km/h 时 0.05 ≈ 4.6 m/s²
          侧向加速度；&gt;0.06 将超出附着极限变成甩尾工况。目标车速建议始终配 2–4 s 斜坡。
        </div>
      </section>

      {/* ── right: variant matrix + run ──────────────────────────── */}
      <aside className="wf-col">
        <div className="wf-col-head"><span>运行矩阵</span></div>

        <div className="wf-subhead">按策略扫描（可多选）</div>
        <div className="wf-chiprow">
          {strategies.map((s) => (
            <button key={s}
                    className={`wf-chip ${stratSel.includes(s) ? "on" : ""}`}
                    onClick={() => setStratSel((cur) =>
                      cur.includes(s) ? cur.filter((x) => x !== s) : [...cur, s])}>
              {s}
            </button>
          ))}
        </div>

        <div className="wf-subhead">按车速扫描（km/h，逗号分隔，作用于第 1 段）</div>
        <input className="wf-input" placeholder="如：40, 60, 80" value={speedsText}
               onChange={(e) => setSpeedsText(e.target.value)} />

        <div className="wf-small" style={{ color: "var(--muted)", margin: "8px 0" }}>
          {variants.length > 0
            ? `将展开为 ${variants.length} 个变体运行`
            : "无变体 → 按当前定义运行 1 次"}
        </div>

        <button className="wf-btn primary big" disabled={running} onClick={run}>
          {running ? `运行中 ${job?.done}/${job?.total} …` : `▶ 运行（${Math.max(1, variants.length)} runs）`}
        </button>

        {job && (
          <div className="wf-job">
            <div className="wf-progress">
              <div className="wf-progress-fill"
                   style={{ width: `${(job.done / Math.max(1, job.total)) * 100}%`,
                            background: job.status === "error" ? "var(--bad)" : "var(--accent)" }} />
            </div>
            <div className="wf-small" style={{ color: "var(--muted)", marginTop: 4 }}>
              {job.status === "running" && `正在跑：${job.current_label || "…"}`}
              {job.status === "done" && `✓ 完成 ${job.done} runs · ${job.elapsed_s.toFixed(1)} s`}
              {job.status === "error" && `✗ 失败：${job.error}`}
              {job.status === "cancelled" && "已取消"}
            </div>

            {job.runs.length > 0 && (
              <table className="wf-kpi-mini">
                <thead>
                  <tr><th>变体</th>{KPI_COLUMNS.map((c) => <th key={c.key}>{c.label}</th>)}</tr>
                </thead>
                <tbody>
                  {job.runs.map((r) => (
                    <tr key={r.run_id}>
                      <td>{r.label}</td>
                      {KPI_COLUMNS.map((c) => (
                        <td key={c.key}>{r.kpis[c.key] != null ? r.kpis[c.key].toFixed(c.digits) : "—"}</td>
                      ))}
                    </tr>
                  ))}
                </tbody>
              </table>
            )}

            {job.status === "done" && (
              <button className="wf-btn primary big" style={{ marginTop: 8 }} onClick={goAnalysis}>
                → 去分析页对比这些 runs
              </button>
            )}
          </div>
        )}
      </aside>
    </div>
  );
}
