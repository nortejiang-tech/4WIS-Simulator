/**
 * MeasurePanel — on-canvas measurement tool.
 *
 * Two readouts:
 *   1. Manual ruler — toggle 测量模式, click two points on the 2D canvas to read
 *      the straight-line distance between them (drawn + labelled on the canvas).
 *   2. Trajectory metrics — auto-computed from the recorded trajectory buffer:
 *      bounding-box span (X / Y), total path length, and net displacement.
 *
 * Pure frontend — derived from store state (trajectory in world frame [m]).
 */

import { useEffect, useState } from "react";
import { useSimStore } from "@/store/sim";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

function trajMetrics(traj: number[]) {
  const n = traj.length;
  if (n < 4) return null;
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  let length = 0;
  for (let i = 0; i < n; i += 2) {
    const x = traj[i], y = traj[i + 1];
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
    if (i >= 2) length += Math.hypot(x - traj[i - 2], y - traj[i - 1]);
  }
  const net = Math.hypot(traj[n - 2] - traj[0], traj[n - 1] - traj[1]);
  return { spanX: maxX - minX, spanY: maxY - minY, length, net };
}

export default function MeasurePanel() {
  const measureMode = useSimStore((s) => s.measureMode);
  const setMeasureMode = useSimStore((s) => s.setMeasureMode);
  const measurePts = useSimStore((s) => s.measurePts);
  const clearMeasure = useSimStore((s) => s.clearMeasure);

  // The trajectory buffer is mutated in place (same array reference), so a
  // plain selector never re-renders as it grows. Recompute on a 4 Hz timer.
  const [m, setM] = useState(() => trajMetrics(useSimStore.getState().trajectory));
  useEffect(() => {
    const id = window.setInterval(
      () => setM(trajMetrics(useSimStore.getState().trajectory)), 250,
    );
    return () => window.clearInterval(id);
  }, []);

  let manualDist: number | null = null;
  if (measurePts.length === 2) {
    manualDist = Math.hypot(
      measurePts[1][0] - measurePts[0][0],
      measurePts[1][1] - measurePts[0][1],
    );
  }
  const Row = ({ label, value }: { label: string; value: string }) => (
    <div style={{ display: "flex", justifyContent: "space-between", fontSize: 11, marginBottom: 2 }}>
      <span style={{ color: "var(--muted)" }}>{label}</span>
      <span style={{ fontFamily: "monospace", color: "var(--text)" }}>{value}</span>
    </div>
  );

  return (
    <Panel
      title="测量工具"
      help={HELP.measure}
      badge={measureMode && (
        <span style={{ fontSize: 10, color: "#fbbf24", marginLeft: "auto" }}>▶ 点击画布取两点</span>
      )}
    >
      {/* Manual ruler */}
      <div style={{ display: "flex", gap: 6, alignItems: "center" }}>
        <button
          onClick={() => setMeasureMode(!measureMode)}
          style={measureMode ? {
            background: "rgba(251,191,36,0.15)", border: "1px solid #fbbf24", color: "#fbbf24",
          } : undefined}
        >
          {measureMode ? "测量中…" : "卷尺测量"}
        </button>
        <button onClick={clearMeasure} disabled={measurePts.length === 0}>清除</button>
        <span style={{ marginLeft: "auto", fontFamily: "monospace", fontSize: 13, color: "#fbbf24" }}>
          {manualDist != null
            ? `${manualDist.toFixed(2)} m`
            : measurePts.length === 1 ? "再点一个点…" : "—"}
        </span>
      </div>

      {/* Trajectory metrics */}
      <div style={{ marginTop: 10, borderTop: "1px solid var(--border)", paddingTop: 6 }}>
        <div style={{ fontSize: 11, color: "var(--muted)", marginBottom: 4 }}>轨迹尺寸（自动）</div>
        {m ? (
          <>
            <Row label="X 跨度（东）" value={`${m.spanX.toFixed(2)} m`} />
            <Row label="Y 跨度（北）" value={`${m.spanY.toFixed(2)} m`} />
            <Row label="路径总长" value={`${m.length.toFixed(2)} m`} />
            <Row label="直线位移" value={`${m.net.toFixed(2)} m`} />
          </>
        ) : (
          <div className="panel-small" style={{ color: "var(--muted)" }}>暂无轨迹（先驾驶/激励一段）</div>
        )}
      </div>

      <div className="panel-small" style={{ color: "var(--muted)", marginTop: 6, lineHeight: 1.4 }}>
        卷尺：开启后在 2D 画布点两点测直线距离，第三次点击重新开始。
      </div>
    </Panel>
  );
}
