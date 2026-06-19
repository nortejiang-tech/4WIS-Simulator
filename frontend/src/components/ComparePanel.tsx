/**
 * ComparePanel — A/B run comparison.
 *
 * Snapshot the current run (trajectory + vx/yaw history) into slot A or B,
 * then overlay both trajectories on the 2D/3D canvas and compare their
 * velocity profiles here. Lets you e.g. run a double-lane-change under two
 * steering strategies and see the path + dynamics difference side by side.
 */

import { useEffect, useRef, useState } from "react";
import uPlot from "uplot";
import "uplot/dist/uPlot.min.css";

import { useSimStore } from "@/store/sim";
import type { RunSnapshot } from "@/store/sim";
import { toKmh } from "@/ui/units";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

export const RUN_COLORS = { A: "#34d399", B: "#fb923c" } as const;

type Channel = "vx" | "yaw_rate" | "icr_dev" | "motor_torque";
const CHANNELS: { key: Channel; label: string }[] = [
  { key: "vx",           label: "车速 vₓ (km/h)" },
  { key: "yaw_rate",     label: "横摆角速度 (rad/s)" },
  { key: "icr_dev",      label: "瞬心偏差峰值 (m)" },
  { key: "motor_torque", label: "电机力矩合计 (N·m)" },
];

export default function ComparePanel() {
  const saved = useSimStore((s) => s.savedRuns);
  const showOverlay = useSimStore((s) => s.showOverlay);
  const saveRun = useSimStore((s) => s.saveRun);
  const clearRun = useSimStore((s) => s.clearRun);
  const setShowOverlay = useSimStore((s) => s.setShowOverlay);

  const [channel, setChannel] = useState<Channel>("vx");

  const containerRef = useRef<HTMLDivElement>(null);
  const plotRef = useRef<uPlot | null>(null);

  // (Re)build comparison data whenever either run, or the channel, changes.
  const sig = `${saved.A?.vx.length ?? 0}:${saved.B?.vx.length ?? 0}:${channel}`;

  useEffect(() => {
    if (!containerRef.current) return;
    const opts: uPlot.Options = {
      title: "",
      width: containerRef.current.clientWidth,
      height: containerRef.current.clientHeight,
      legend: { show: false },
      cursor: { show: false },
      scales: { x: { time: false } },
      axes: [
        { stroke: "#94a3b8", grid: { stroke: "#1f2937", width: 1 }, ticks: { stroke: "#1f2937" }, size: 22 },
        { stroke: "#94a3b8", grid: { stroke: "#1f2937", width: 1 }, ticks: { stroke: "#1f2937" }, size: 36 },
      ],
      series: [
        {},
        { label: "A", stroke: RUN_COLORS.A, width: 1.6, points: { show: false } },
        { label: "B", stroke: RUN_COLORS.B, width: 1.6, points: { show: false } },
      ],
    };
    plotRef.current = new uPlot(opts, buildData(channel), containerRef.current);
    const ro = new ResizeObserver((e) => {
      const cr = e[0].contentRect;
      plotRef.current?.setSize({ width: cr.width, height: cr.height });
    });
    ro.observe(containerRef.current);
    return () => { ro.disconnect(); plotRef.current?.destroy(); plotRef.current = null; };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useEffect(() => {
    plotRef.current?.setData(buildData(channel));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [sig]);

  return (
    <Panel title="双跑对比（A / B）" help={HELP.compare}>
      <div style={{ display: "flex", gap: 6, flexWrap: "wrap" }}>
        <button onClick={() => saveRun("A")}>存为 A</button>
        <button onClick={() => saveRun("B")}>存为 B</button>
        <button onClick={() => clearRun("A")} disabled={!saved.A}>清除 A</button>
        <button onClick={() => clearRun("B")} disabled={!saved.B}>清除 B</button>
        <label style={{ display: "flex", alignItems: "center", gap: 4, marginLeft: "auto", fontSize: 12, cursor: "pointer" }}>
          <input type="checkbox" checked={showOverlay} onChange={(e) => setShowOverlay(e.target.checked)} />
          叠加显示
        </label>
      </div>

      <div className="small" style={{ marginTop: 6, display: "flex", gap: 12 }}>
        <span style={{ color: RUN_COLORS.A }}>● A: {saved.A?.label ?? "（空）"}</span>
        <span style={{ color: RUN_COLORS.B }}>● B: {saved.B?.label ?? "（空）"}</span>
      </div>

      <div className="chart-block" style={{ marginTop: 8 }}>
        <h3 style={{ display: "flex", alignItems: "center", gap: 6 }}>
          通道对比
          <select value={channel} onChange={(e) => setChannel(e.target.value as Channel)}
            style={{ fontSize: 11, marginLeft: "auto" }}>
            {CHANNELS.map((c) => <option key={c.key} value={c.key}>{c.label}</option>)}
          </select>
        </h3>
        <div className="chart-container" ref={containerRef} />
      </div>
    </Panel>
  );
}

function buildData(channel: Channel): uPlot.AlignedData {
  const { A, B } = useSimStore.getState().savedRuns;
  const pick = (r: RunSnapshot | null): number[] | undefined => r ? r[channel] : undefined;
  const conv = channel === "vx" ? toKmh : (v: number) => v;
  const aArr = pick(A)?.map(conv), bArr = pick(B)?.map(conv);
  const n = Math.max(aArr?.length ?? 0, bArr?.length ?? 0, 1);
  const xs = new Array(n);
  for (let i = 0; i < n; i++) xs[i] = i;
  const pad = (arr: number[] | undefined): (number | null)[] => {
    const out: (number | null)[] = new Array(n).fill(null);
    if (arr) for (let i = 0; i < arr.length && i < n; i++) out[i] = arr[i];
    return out;
  };
  return [xs, pad(aArr), pad(bArr)] as uPlot.AlignedData;
}
