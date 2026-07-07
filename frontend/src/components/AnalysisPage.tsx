/**
 * AnalysisPage — the "分析" workflow stage (platform refactor Phase B).
 *
 * Run browser (left) → select up to 6 runs (palette-coloured) → KPI comparison
 * table + overlay charts (uPlot, one channel across all selected runs) + a
 * top-down trajectory overlay. Data comes from the Phase-A run artifacts via
 * /api/runs; nothing here touches the realtime simulator.
 *
 * Alignment note: overlay charts use the longest run's time base and align
 * the rest by sample index — exact for runs from one batch (same record_hz,
 * t₀ = 0), approximate if you mix experiments with different record rates.
 */

import { useEffect, useRef, useState } from "react";

import {
  RunListItem,
  deleteRun,
  getRunData,
  listRuns,
} from "@/api/experiments";
import { exportPNG, useLiveChart } from "@/charts/uplotFactory";
import ReplayPanel from "@/components/ReplayPanel";
import { useSimStore } from "@/store/sim";
import "./WorkflowPage.css";

const PALETTE = ["#60a5fa", "#f59e0b", "#34d399", "#f87171", "#a78bfa", "#22d3ee"];
const MAX_SELECT = PALETTE.length;
const TARGET_POINTS = 2000;

const CHANNEL_LABELS: Record<string, string> = {
  vx: "纵向车速 vx [m/s]",
  vy: "侧向速度 vy [m/s]",
  yaw_rate: "横摆角速度 [rad/s]",
  driver_steering: "驾驶员转向输入 [-1..1]",
  delta_fl: "FL 轮转角 [rad]",
  delta_fr: "FR 轮转角 [rad]",
  rack_force_fl: "FL 齿条力 [N]",
  motor_torque_fl: "FL 转向电机力矩 [N·m]",
  slip_alpha_fl: "FL 侧偏角 [rad]",
  slip_kappa_fl: "FL 滑移率 [-]",
  torque_steer_fl: "FL 主销力矩 [N·m]",
  fz_fl: "FL 垂向载荷 [N]",
  icr_dev_fl: "FL 瞬心偏差 [m]",
};

const PRESET_CHIPS = ["vx", "yaw_rate", "driver_steering", "delta_fl", "rack_force_fl", "slip_alpha_fl"];

// Channels the ghost-vehicle replay needs on top of whatever is charted.
const REPLAY_CHANNELS = ["pose_x", "pose_y", "pose_psi", "delta_fl", "delta_fr", "delta_rl", "delta_rr"];

const KPI_ROWS: { key: string; label: string; digits: number }[] = [
  { key: "yaw_rate_peak_dps", label: "横摆角速度峰值 °/s", digits: 2 },
  { key: "yaw_gain_dps", label: "横摆增益 °/s / 单位输入", digits: 1 },
  { key: "yaw_rise_time_s", label: "横摆上升时间 10-90% s", digits: 3 },
  { key: "yaw_overshoot_pct", label: "横摆超调 %", digits: 1 },
  { key: "yaw_settling_time_s", label: "横摆稳定时间 5% s", digits: 2 },
  { key: "vy_peak_kmh", label: "侧向速度峰值 km/h", digits: 2 },
  { key: "icr_dev_peak_m", label: "瞬心偏差峰值 m", digits: 3 },
  { key: "icr_dev_rms_m", label: "瞬心偏差 RMS m", digits: 3 },
  { key: "rack_force_peak_n", label: "齿条力峰值 N", digits: 0 },
  { key: "steer_energy_nms", label: "转向能耗 N·m·s", digits: 1 },
  { key: "slip_alpha_peak_deg", label: "侧偏角峰值 °", digits: 2 },
  { key: "speed_error_rms_kmh", label: "车速跟踪 RMS km/h", digits: 2 },
];

interface RunCache {
  t: number[];
  channels: Record<string, (number | null)[]>;
}

export default function AnalysisPage() {
  const pushToast = useSimStore((s) => s.pushToast);
  const preselect = useSimStore((s) => s.analysisPreselect);
  const setPreselect = useSimStore((s) => s.setAnalysisPreselect);

  const [runs, setRuns] = useState<RunListItem[]>([]);
  const [selected, setSelected] = useState<string[]>([]);   // run ids, palette order
  const [charts, setCharts] = useState<string[]>(["vx", "yaw_rate"]);
  const [pickerCh, setPickerCh] = useState("rack_force_fl");
  const [dataVersion, setDataVersion] = useState(0);
  const [replayOpen, setReplayOpen] = useState(false);
  const [replayT, setReplayT] = useState<number | null>(null);
  const cache = useRef<Map<string, RunCache>>(new Map());

  const refresh = () => listRuns().then(setRuns).catch(() => setRuns([]));
  useEffect(() => { refresh(); }, []);

  // Apply preselection handed over from the experiment page (once).
  useEffect(() => {
    if (preselect.length > 0) {
      setSelected(preselect.slice(0, MAX_SELECT));
      setPreselect([]);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [preselect]);

  const colorOf = (runId: string): string | null => {
    const i = selected.indexOf(runId);
    return i >= 0 ? PALETTE[i] : null;
  };

  const toggle = (runId: string) =>
    setSelected((cur) => {
      if (cur.includes(runId)) return cur.filter((x) => x !== runId);
      if (cur.length >= MAX_SELECT) return cur;
      return [...cur, runId];
    });

  const remove = (runId: string) =>
    deleteRun(runId)
      .then(() => {
        cache.current.delete(runId);
        setSelected((cur) => cur.filter((x) => x !== runId));
        refresh();
      })
      .catch((err) => pushToast("error", `删除失败：${err.message}`));

  // ---- data loading ----------------------------------------------------------

  const neededChannels = [...new Set([
    ...charts, "pose_x", "pose_y",
    ...(replayOpen ? REPLAY_CHANNELS : []),
  ])];

  useEffect(() => {
    let cancelled = false;
    (async () => {
      let loadedAny = false;
      for (const runId of selected) {
        const meta = runs.find((r) => r.run_id === runId);
        const dec = meta?.n_samples ? Math.max(1, Math.ceil(meta.n_samples / TARGET_POINTS)) : 1;
        const have = cache.current.get(runId);
        const missing = neededChannels.filter((c) => !have?.channels[c]);
        if (missing.length === 0) continue;
        try {
          const d = await getRunData(runId, missing, dec);
          if (cancelled) return;
          const entry: RunCache = have ?? { t: d.t, channels: {} };
          entry.t = d.t;
          for (const c of missing) entry.channels[c] = (d as Record<string, (number | null)[]>)[c] ?? [];
          cache.current.set(runId, entry);
          loadedAny = true;
        } catch (err) {
          if (!cancelled) pushToast("error", `读取 run 数据失败：${(err as Error).message}`);
        }
      }
      if (loadedAny && !cancelled) setDataVersion((v) => v + 1);
    })();
    return () => { cancelled = true; };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selected, charts, runs, replayOpen]);

  const selectedRuns = selected
    .map((id) => runs.find((r) => r.run_id === id))
    .filter((r): r is RunListItem => r != null);

  // Extra KPI keys not covered by the ordered list.
  const extraKpiKeys = [
    ...new Set(selectedRuns.flatMap((r) => Object.keys(r.kpis ?? {}))),
  ].filter((k) => !KPI_ROWS.some((row) => row.key === k));

  return (
    <div className="wf-page wf-analysis">
      {/* ── left: run browser ─────────────────────────────────────── */}
      <aside className="wf-col">
        <div className="wf-col-head">
          <span>Run 库（{runs.length}）</span>
          <button className="wf-btn" onClick={refresh}>刷新</button>
        </div>
        <div className="wf-list">
          {runs.map((r) => {
            const col = colorOf(r.run_id);
            return (
              <div key={r.run_id}
                   className={`wf-list-item ${col ? "active" : ""}`}
                   style={col ? { borderLeft: `3px solid ${col}` } : undefined}
                   onClick={() => toggle(r.run_id)}>
                <div className="wf-list-title">
                  {col && <span className="wf-dot" style={{ background: col }} />}
                  {r.label || r.experiment_name || r.run_id}
                </div>
                <div className="wf-list-sub">
                  {r.strategy} · {r.model_type} · {r.duration_s?.toFixed(0)}s
                </div>
                <div className="wf-list-desc">{r.created_at.replace("T", " ")}</div>
                <button className="wf-x" title="删除此 run"
                        onClick={(ev) => { ev.stopPropagation(); remove(r.run_id); }}>✕</button>
              </div>
            );
          })}
          {runs.length === 0 && (
            <div className="wf-empty">还没有 run — 到「试验」页跑一个批量</div>
          )}
        </div>
        <div className="small" style={{ color: "var(--muted)", padding: "6px 2px" }}>
          点击选择（最多 {MAX_SELECT} 个）进行叠加对比
        </div>
      </aside>

      {/* ── main: KPI + charts ────────────────────────────────────── */}
      <section className="wf-col wf-main">
        {selectedRuns.length === 0 ? (
          <div className="wf-empty" style={{ marginTop: 40 }}>
            ← 从左侧选择 1–{MAX_SELECT} 个 run 开始对比分析
          </div>
        ) : (
          <>
            <div className="wf-col-head" data-testid="analysis-workbench-head">
              <span>KPI 对比</span>
              <button className={`wf-btn ${replayOpen ? "primary" : ""}`}
                      onClick={() => setReplayOpen((v) => !v)}>
                {replayOpen ? "⏹ 关闭回放" : "▶ 回放"}
              </button>
            </div>

            {replayOpen && (
              <ReplayPanel
                runs={selectedRuns}
                colors={selectedRuns.map((r) => colorOf(r.run_id) ?? "#888")}
                cache={cache.current}
                version={dataVersion}
                onTime={setReplayT}
                onClose={() => { setReplayOpen(false); setReplayT(null); }}
              />
            )}
            <div className="wf-kpi-wrap" data-testid="analysis-kpi-table">
              <table className="wf-kpi">
                <thead>
                  <tr>
                    <th style={{ textAlign: "left" }}>指标</th>
                    {selectedRuns.map((r) => (
                      <th key={r.run_id}>
                        <span className="wf-dot" style={{ background: colorOf(r.run_id) ?? "#888" }} />
                        {r.label || r.run_id.slice(-6)}
                      </th>
                    ))}
                  </tr>
                </thead>
                <tbody>
                  {[...KPI_ROWS, ...extraKpiKeys.map((k) => ({ key: k, label: k, digits: 3 }))].map((row) => {
                    const vals = selectedRuns.map((r) => r.kpis?.[row.key]);
                    if (vals.every((v) => v == null)) return null;
                    return (
                      <tr key={row.key}>
                        <td>{row.label}</td>
                        {vals.map((v, i) => (
                          <td key={i} className="mono">{v != null ? v.toFixed(row.digits) : "—"}</td>
                        ))}
                      </tr>
                    );
                  })}
                </tbody>
              </table>
            </div>

            <div className="wf-col-head" style={{ marginTop: 12 }}>
              <span>通道叠图</span>
              <div style={{ display: "flex", gap: 6, alignItems: "center", marginLeft: "auto" }}>
                <select className="wf-input" aria-label="分析通道选择" value={pickerCh} onChange={(e) => setPickerCh(e.target.value)}>
                  {Object.keys(CHANNEL_LABELS).map((c) => (
                    <option key={c} value={c}>{CHANNEL_LABELS[c]}</option>
                  ))}
                </select>
                <button className="wf-btn" onClick={() =>
                  setCharts((cur) => (cur.includes(pickerCh) ? cur : [...cur, pickerCh]))
                }>＋ 加图</button>
              </div>
            </div>
            <div className="wf-chiprow">
              {PRESET_CHIPS.map((c) => (
                <button key={c} className={`wf-chip ${charts.includes(c) ? "on" : ""}`}
                        data-testid={`analysis-chip-${c}`}
                        onClick={() => setCharts((cur) =>
                          cur.includes(c) ? cur.filter((x) => x !== c) : [...cur, c])}>
                  {(CHANNEL_LABELS[c] ?? c).replace(/\s*\[.*\]$/, "")}
                </button>
              ))}
            </div>

            <div className="wf-charts" data-testid="analysis-workbench">
              <TrajectoryOverlay
                runs={selectedRuns}
                colors={selectedRuns.map((r) => colorOf(r.run_id) ?? "#888")}
                cache={cache.current}
                version={dataVersion}
              />
              {charts.map((ch) => (
                <OverlayChart
                  key={ch + "|" + selected.join(",")}
                  channel={ch}
                  runs={selectedRuns}
                  colors={selectedRuns.map((r) => colorOf(r.run_id) ?? "#888")}
                  cache={cache.current}
                  version={dataVersion}
                  marker={replayOpen ? replayT : null}
                  onClose={() => setCharts((cur) => cur.filter((x) => x !== ch))}
                />
              ))}
            </div>
          </>
        )}
      </section>
    </div>
  );
}

// ─── one uPlot card: a single channel overlaid across the selected runs ─────

function OverlayChart({ channel, runs, colors, cache, version, marker, onClose }: {
  channel: string;
  runs: RunListItem[];
  colors: string[];
  cache: Map<string, RunCache>;
  version: number;
  marker?: number | null;
  onClose: () => void;
}) {
  const series = runs.map((r, i) => ({
    label: r.label || r.run_id.slice(-6),
    color: colors[i],
  }));

  const getData = (): [number[], ...(number | null)[][]] => {
    // Longest run supplies the time base; others align by sample index.
    let xs: number[] = [];
    for (const r of runs) {
      const c = cache.get(r.run_id);
      if (c && c.t.length > xs.length) xs = c.t;
    }
    const cols = runs.map((r) => {
      const c = cache.get(r.run_id);
      const vals = c?.channels[channel] ?? [];
      if (vals.length >= xs.length) return vals.slice(0, xs.length);
      return [...vals, ...new Array(xs.length - vals.length).fill(null)];
    });
    return [xs, ...cols];
  };

  const { containerRef, plotRef } = useLiveChart(
    series, getData, version, undefined,
    { x: "t [s]", y: CHANNEL_LABELS[channel] ?? channel },
    // Replay-time cursor: a fresh array per render keeps the marker effect
    // firing; useLiveChart redraws without tearing down zoom state.
    { verticalMarkers: marker != null ? [{ x: marker, color: "#fbbf24" }] : [] },
  );

  return (
    <div className="wf-chart-card" data-testid={`analysis-chart-${channel}`}>
      <div className="wf-chart-head">
        <span>{CHANNEL_LABELS[channel] ?? channel}</span>
        <button className="wf-btn" data-testid={`analysis-chart-png-${channel}`} onClick={() => exportPNG(plotRef.current, channel)}>PNG</button>
        <button className="wf-x" aria-label={`关闭${CHANNEL_LABELS[channel] ?? channel}图表`} onClick={onClose}>✕</button>
      </div>
      <div ref={containerRef} className="wf-chart-body" />
    </div>
  );
}

// ─── top-down trajectory overlay (equal-aspect canvas) ──────────────────────

function TrajectoryOverlay({ runs, colors, cache, version }: {
  runs: RunListItem[];
  colors: string[];
  cache: Map<string, RunCache>;
  version: number;
}) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const cv = canvasRef.current;
    const wrap = wrapRef.current;
    if (!cv || !wrap) return;

    const draw = () => {
      const w = wrap.clientWidth, h = wrap.clientHeight;
      const dpr = window.devicePixelRatio || 1;
      cv.width = w * dpr; cv.height = h * dpr;
      cv.style.width = `${w}px`; cv.style.height = `${h}px`;
      const ctx = cv.getContext("2d");
      if (!ctx) return;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);

      // Bounding box over all selected runs.
      let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity, any = false;
      for (const r of runs) {
        const c = cache.get(r.run_id);
        const xs = c?.channels["pose_x"], ys = c?.channels["pose_y"];
        if (!xs || !ys) continue;
        for (let i = 0; i < xs.length; i++) {
          const x = xs[i], y = ys[i];
          if (x == null || y == null) continue;
          any = true;
          if (x < x0) x0 = x; if (x > x1) x1 = x;
          if (y < y0) y0 = y; if (y > y1) y1 = y;
        }
      }
      if (!any) {
        ctx.fillStyle = "#64748b";
        ctx.font = "12px sans-serif";
        ctx.fillText("加载轨迹中…", 12, 20);
        return;
      }
      const pad = 24;
      const spanX = Math.max(x1 - x0, 1), spanY = Math.max(y1 - y0, 1);
      const scale = Math.min((w - 2 * pad) / spanX, (h - 2 * pad) / spanY);
      const ox = (w - spanX * scale) / 2 - x0 * scale;
      // World +Y is left; screen +y is down → flip Y.
      const oy = (h + spanY * scale) / 2 + y0 * scale;
      const px = (x: number) => ox + x * scale;
      const py = (y: number) => oy - y * scale;

      // Light metric grid (10 m).
      ctx.strokeStyle = "rgba(100,116,139,0.18)";
      ctx.lineWidth = 1;
      const grid = 10;
      for (let gx = Math.floor(x0 / grid) * grid; gx <= x1; gx += grid) {
        ctx.beginPath(); ctx.moveTo(px(gx), py(y0)); ctx.lineTo(px(gx), py(y1)); ctx.stroke();
      }
      for (let gy = Math.floor(y0 / grid) * grid; gy <= y1; gy += grid) {
        ctx.beginPath(); ctx.moveTo(px(x0), py(gy)); ctx.lineTo(px(x1), py(gy)); ctx.stroke();
      }

      runs.forEach((r, i) => {
        const c = cache.get(r.run_id);
        const xs = c?.channels["pose_x"], ys = c?.channels["pose_y"];
        if (!xs || !ys) return;
        ctx.strokeStyle = colors[i];
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        let started = false;
        for (let k = 0; k < xs.length; k++) {
          const x = xs[k], y = ys[k];
          if (x == null || y == null) continue;
          if (!started) { ctx.moveTo(px(x), py(y)); started = true; }
          else ctx.lineTo(px(x), py(y));
        }
        ctx.stroke();
        // Start marker.
        const sx = xs.find((v) => v != null), sy = ys.find((v) => v != null);
        if (sx != null && sy != null) {
          ctx.fillStyle = colors[i];
          ctx.beginPath(); ctx.arc(px(sx), py(sy), 3.5, 0, Math.PI * 2); ctx.fill();
        }
      });

      // Scale hint.
      ctx.fillStyle = "#64748b";
      ctx.font = "10px ui-monospace, monospace";
      ctx.fillText(`网格 ${grid} m`, 8, h - 8);
    };

    draw();
    const ro = new ResizeObserver(draw);
    ro.observe(wrap);
    return () => ro.disconnect();
  }, [runs, colors, cache, version]);

  return (
    <div className="wf-chart-card" data-testid="analysis-trajectory">
      <div className="wf-chart-head"><span>轨迹俯视（世界系，等比例）</span></div>
      <div ref={wrapRef} className="wf-chart-body">
        <canvas ref={canvasRef} data-testid="analysis-trajectory-canvas" />
      </div>
    </div>
  );
}
