/**
 * ReplayPanel — artifact-driven replay of selected runs (Phase B-2).
 *
 * Re-enacts up to 6 selected runs simultaneously as coloured "ghost" vehicles
 * (body + 4 wheels at their recorded steer angles) over their dimmed
 * trajectories, with play/pause, speed multiplier and a timeline scrubber.
 * The current replay time is reported to the parent so the overlay charts can
 * draw a synchronized vertical cursor.
 *
 * Pure client-side: data comes from the run artifacts already cached by the
 * analysis page (pose_x/pose_y/pose_psi + delta_fl..rr). Vehicle drawing
 * dimensions come from the first run's experiment snapshot (vehicle overrides
 * applied on top of the LS9 defaults).
 */

import { useEffect, useRef, useState } from "react";

import { RunListItem, getRunMeta } from "@/api/experiments";

interface RunCache {
  t: number[];
  channels: Record<string, (number | null)[]>;
}

interface Dims {
  wheelbase: number;
  track: number;
}

const DEFAULT_DIMS: Dims = { wheelbase: 3.16, track: 1.565 };
const WHEEL_LEN = 0.72;   // drawn wheel footprint [m]
const WHEEL_WID = 0.27;
const SPEEDS = [0.5, 1, 2, 4];

function formatError(e: unknown): string {
  return e instanceof Error ? e.message : String(e);
}

function sampleAt(ts: number[], vs: (number | null)[] | undefined, tq: number): number | null {
  if (!vs || ts.length === 0 || vs.length === 0) return null;
  const n = Math.min(ts.length, vs.length);
  if (tq <= ts[0]) return vs[0];
  if (tq >= ts[n - 1]) return vs[n - 1];
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (ts[mid] <= tq) lo = mid; else hi = mid;
  }
  const v0 = vs[lo], v1 = vs[hi];
  if (v0 == null || v1 == null) return v0 ?? v1;
  const f = (tq - ts[lo]) / Math.max(ts[hi] - ts[lo], 1e-9);
  return v0 + (v1 - v0) * f;
}

export default function ReplayPanel({ runs, colors, cache, version, onTime, onClose }: {
  runs: RunListItem[];
  colors: string[];
  cache: Map<string, RunCache>;
  version: number;
  onTime: (t: number | null) => void;
  onClose: () => void;
}) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);
  const [tCur, setTCur] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [dims, setDims] = useState<Dims>(DEFAULT_DIMS);
  const [dimsWarning, setDimsWarning] = useState<string | null>(null);
  const tCurRef = useRef(0);
  tCurRef.current = tCur;

  const tMax = Math.max(0.1, ...runs.map((r) => {
    const c = cache.get(r.run_id);
    return c && c.t.length ? c.t[c.t.length - 1] : 0;
  }));

  // Vehicle dims from the first run's experiment snapshot (fallback: LS9).
  useEffect(() => {
    if (runs.length === 0) return;
    getRunMeta(runs[0].run_id)
      .then((m) => {
        const ov = (m.experiment?.vehicle?.overrides ?? {}) as Record<string, number>;
        setDims({
          wheelbase: Number(ov.wheelbase) || DEFAULT_DIMS.wheelbase,
          track: Number(ov.track_front) || DEFAULT_DIMS.track,
        });
        setDimsWarning(null);
      })
      .catch((e) => {
        setDims(DEFAULT_DIMS);
        setDimsWarning(`回放尺寸读取失败，使用默认 LS9 尺寸：${formatError(e)}`);
      });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [runs.map((r) => r.run_id).join(",")]);

  // Report replay time to the parent (chart cursor sync); null on unmount.
  useEffect(() => {
    onTime(tCur);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [tCur]);
  useEffect(() => () => onTime(null), [onTime]);

  // Playback loop (rAF; pauses automatically at the end).
  useEffect(() => {
    if (!playing) return;
    let raf = 0;
    let last = performance.now();
    const step = (now: number) => {
      const dt = Math.min(0.1, (now - last) / 1000);
      last = now;
      const next = tCurRef.current + dt * speed;
      if (next >= tMax) {
        setTCur(tMax);
        setPlaying(false);
        return;
      }
      setTCur(next);
      raf = requestAnimationFrame(step);
    };
    raf = requestAnimationFrame(step);
    return () => cancelAnimationFrame(raf);
  }, [playing, speed, tMax]);

  // ---- drawing ---------------------------------------------------------------

  useEffect(() => {
    const cv = canvasRef.current;
    const wrap = wrapRef.current;
    if (!cv || !wrap) return;

    const draw = () => {
      const w = wrap.clientWidth, h = wrap.clientHeight;
      if (w < 10 || h < 10) return;
      const dpr = window.devicePixelRatio || 1;
      cv.width = w * dpr; cv.height = h * dpr;
      cv.style.width = `${w}px`; cv.style.height = `${h}px`;
      const ctx = cv.getContext("2d");
      if (!ctx) return;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);

      // Bounding box over all trajectories (+ vehicle margin).
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
        ctx.fillText("加载回放数据中…", 12, 20);
        return;
      }
      const margin = dims.wheelbase;
      x0 -= margin; x1 += margin; y0 -= margin; y1 += margin;
      const pad = 16;
      const spanX = Math.max(x1 - x0, 1), spanY = Math.max(y1 - y0, 1);
      const scale = Math.min((w - 2 * pad) / spanX, (h - 2 * pad) / spanY);
      const ox = (w - spanX * scale) / 2 - x0 * scale;
      const oy = (h + spanY * scale) / 2 + y0 * scale;
      const px = (x: number) => ox + x * scale;
      const py = (y: number) => oy - y * scale;

      // 10 m grid.
      ctx.strokeStyle = "rgba(100,116,139,0.16)";
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
        if (!c) return;
        const xs = c.channels["pose_x"], ys = c.channels["pose_y"];
        if (!xs || !ys) return;

        // Dimmed full trajectory.
        ctx.strokeStyle = colors[i] + "55";
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        let started = false;
        for (let k = 0; k < xs.length; k++) {
          const x = xs[k], y = ys[k];
          if (x == null || y == null) continue;
          if (!started) { ctx.moveTo(px(x), py(y)); started = true; }
          else ctx.lineTo(px(x), py(y));
        }
        ctx.stroke();

        // Ghost vehicle at tCur.
        const x = sampleAt(c.t, xs, tCur);
        const y = sampleAt(c.t, ys, tCur);
        const psi = sampleAt(c.t, c.channels["pose_psi"], tCur) ?? 0;
        if (x == null || y == null) return;
        const deltas = (["fl", "fr", "rl", "rr"] as const).map(
          (wname) => sampleAt(c.t, c.channels[`delta_${wname}`], tCur) ?? 0,
        );
        drawVehicle(ctx, px, py, x, y, psi, deltas, colors[i], dims);
      });

      ctx.fillStyle = "#64748b";
      ctx.font = "10px ui-monospace, monospace";
      ctx.fillText(`网格 ${grid} m`, 8, h - 8);
    };

    draw();
    const ro = new ResizeObserver(draw);
    ro.observe(wrap);
    return () => ro.disconnect();
  }, [runs, colors, cache, version, tCur, dims]);

  return (
    <div className="wf-chart-card replay-card">
      <div className="wf-chart-head">
        <span>回放（幽灵车叠放 · 车轮显示实际转角）</span>
        <button className="wf-x" data-testid="replay-close" aria-label="关闭回放" onClick={onClose}>✕</button>
      </div>
      {dimsWarning && (
        <div className="wf-small" role="alert" style={{ color: "var(--warn)", padding: "0 10px 6px" }}>
          {dimsWarning}
        </div>
      )}
      <div ref={wrapRef} className="wf-chart-body replay-body">
        <canvas ref={canvasRef} data-testid="replay-canvas" />
      </div>
      <div className="replay-controls">
        <button className="wf-btn" style={{ width: 54 }}
                data-testid="replay-play-toggle"
                onClick={() => {
                  if (!playing && tCur >= tMax - 1e-6) setTCur(0);
                  setPlaying(!playing);
                }}>
          {playing ? "⏸ 暂停" : "▶ 播放"}
        </button>
        <select className="wf-input" style={{ width: 66 }} value={speed}
                data-testid="replay-speed"
                aria-label="回放速度"
                onChange={(e) => setSpeed(Number(e.target.value))}>
          {SPEEDS.map((s) => <option key={s} value={s}>{s}×</option>)}
        </select>
        <input type="range" min={0} max={tMax} step={0.02} value={tCur}
               data-testid="replay-timeline"
               aria-label="回放时间"
               style={{ flex: 1 }}
               onChange={(e) => { setPlaying(false); setTCur(Number(e.target.value)); }} />
        <span className="wf-mono replay-time" data-testid="replay-time">{tCur.toFixed(2)} / {tMax.toFixed(1)} s</span>
      </div>
    </div>
  );
}

function drawVehicle(
  ctx: CanvasRenderingContext2D,
  px: (x: number) => number,
  py: (y: number) => number,
  x: number, y: number, psi: number,
  deltas: number[],
  color: string,
  dims: Dims,
) {
  const L = dims.wheelbase, tr = dims.track;
  const bodyL = L + 1.15, bodyW = tr + 0.32;
  const cp = Math.cos(psi), sp = Math.sin(psi);
  const b2s = (bx: number, by: number): [number, number] => {
    const wx = x + bx * cp - by * sp;
    const wy = y + bx * sp + by * cp;
    return [px(wx), py(wy)];
  };
  const poly = (pts: [number, number][]) => {
    ctx.beginPath();
    pts.forEach(([X, Y], i) => (i === 0 ? ctx.moveTo(X, Y) : ctx.lineTo(X, Y)));
    ctx.closePath();
  };

  // Body outline with a pointed nose so heading is unambiguous.
  const hl = bodyL / 2, hw = bodyW / 2;
  poly([
    b2s(-hl, +hw), b2s(hl * 0.62, +hw), b2s(hl, +hw * 0.45),
    b2s(hl, -hw * 0.45), b2s(hl * 0.62, -hw), b2s(-hl, -hw),
  ]);
  ctx.fillStyle = color + "2e";
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.6;
  ctx.fill();
  ctx.stroke();

  // Wheels at ±L/2, ±track/2, each rotated by its own steer angle.
  const wheelPos: [number, number][] = [
    [+L / 2, +tr / 2], [+L / 2, -tr / 2], [-L / 2, +tr / 2], [-L / 2, -tr / 2],
  ];
  ctx.fillStyle = color;
  wheelPos.forEach(([wx, wy], i) => {
    const d = deltas[i] ?? 0;
    const cd = Math.cos(d), sd = Math.sin(d);
    const corners: [number, number][] = [
      [+WHEEL_LEN / 2, +WHEEL_WID / 2], [+WHEEL_LEN / 2, -WHEEL_WID / 2],
      [-WHEEL_LEN / 2, -WHEEL_WID / 2], [-WHEEL_LEN / 2, +WHEEL_WID / 2],
    ];
    poly(corners.map(([lx, ly]) => {
      const bx = wx + lx * cd - ly * sd;
      const by = wy + lx * sd + ly * cd;
      return b2s(bx, by);
    }));
    ctx.fill();
  });
}
