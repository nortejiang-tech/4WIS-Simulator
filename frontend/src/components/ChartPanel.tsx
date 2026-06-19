/**
 * ChartPanel — four live uPlot charts:
 *   1. Body velocities — vx, yaw rate
 *   2. Wheel steer angles — δ_FL/FR/RL/RR (deg)
 *   3. Wheel steering torques — τ_FL/FR/RL/RR (N·m)
 *   4. Per-wheel ICR deviation vs vehicle ICR (m) — NaN gaps when driving straight
 *
 * Chart construction / live-update plumbing lives in charts/uplotFactory.
 * Each chart has PNG / CSV export of the current ~30 s window.
 */

import type uPlot from "uplot";

import { useSimStore } from "@/store/sim";
import { exportCSV, exportPNG, useLiveChart, type SeriesSpec } from "@/charts/uplotFactory";
import { WHEEL_COLORS } from "@/components/canvas2d/colors";
import { toKmh } from "@/ui/units";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

const WHEEL_SERIES: SeriesSpec[] = ["FL", "FR", "RL", "RR"].map((label, i) => ({
  label,
  color: WHEEL_COLORS[i],
}));

interface BlockSpec {
  title: string;
  filename: string;
  series: SeriesSpec[];
  getData: () => uPlot.AlignedData;
}

function ChartBlock({ spec }: { spec: BlockSpec }) {
  const tLen = useSimStore((s) => s.history.t.length);
  const { containerRef, plotRef } = useLiveChart(spec.series, spec.getData, tLen);
  const header = ["t", ...spec.series.map((s) => s.label)];

  return (
    <div className="chart-item">
      <div className="chart-block">
        <h3 style={{ display: "flex", alignItems: "center" }}>
          {spec.title}
          <span style={{ marginLeft: "auto", display: "flex", gap: 4 }}>
            <button style={{ fontSize: 10 }} title="导出 PNG"
              onClick={() => exportPNG(plotRef.current, spec.filename)}>PNG</button>
            <button style={{ fontSize: 10 }} title="导出 CSV（当前窗口）"
              onClick={() => exportCSV(plotRef.current, spec.filename, header)}>CSV</button>
          </span>
        </h3>
        <div className="chart-container" ref={containerRef} />
      </div>
    </div>
  );
}

const h = () => useSimStore.getState().history;
const toDeg = (a: number[]) => a.map((v) => (v * 180) / Math.PI);

const BLOCKS: BlockSpec[] = [
  {
    title: "车速 vₓ km/h (青) / 横摆角速度 ψ̇ (黄)",
    filename: "sim4wis_velocity",
    series: [
      { label: "vx_kmh", color: "#22d3ee" },
      { label: "yaw_rate", color: "#fbbf24" },
    ],
    getData: () => [h().t, h().vx.map(toKmh), h().yaw_rate] as uPlot.AlignedData,
  },
  {
    title: "车轮转角 δ (°)",
    filename: "sim4wis_delta",
    series: WHEEL_SERIES,
    getData: () => [
      h().t,
      toDeg(h().wheel_delta[0]), toDeg(h().wheel_delta[1]),
      toDeg(h().wheel_delta[2]), toDeg(h().wheel_delta[3]),
    ] as uPlot.AlignedData,
  },
  {
    title: "转向阻力矩 (N·m)",
    filename: "sim4wis_torque",
    series: WHEEL_SERIES,
    getData: () => [
      h().t,
      h().wheel_torque[0], h().wheel_torque[1], h().wheel_torque[2], h().wheel_torque[3],
    ] as uPlot.AlignedData,
  },
  {
    title: "每轮瞬心偏差 (m)",
    filename: "sim4wis_icr_dev",
    series: WHEEL_SERIES,
    getData: () => [
      h().t,
      h().wheel_icr_dev[0], h().wheel_icr_dev[1], h().wheel_icr_dev[2], h().wheel_icr_dev[3],
    ] as uPlot.AlignedData,
  },
  {
    title: "齿条力 (N) — 分体齿条力链",
    filename: "sim4wis_rack_force",
    series: WHEEL_SERIES,
    getData: () => [
      h().t,
      h().wheel_rack_force[0], h().wheel_rack_force[1],
      h().wheel_rack_force[2], h().wheel_rack_force[3],
    ] as uPlot.AlignedData,
  },
  {
    title: "电机需求力矩 (N·m)",
    filename: "sim4wis_motor_torque",
    series: WHEEL_SERIES,
    getData: () => [
      h().t,
      h().wheel_motor_torque[0], h().wheel_motor_torque[1],
      h().wheel_motor_torque[2], h().wheel_motor_torque[3],
    ] as uPlot.AlignedData,
  },
  {
    title: "轮胎侧偏角 α (°) — 仅动力学模型",
    filename: "sim4wis_slip_alpha",
    series: WHEEL_SERIES,
    getData: () => [
      h().t,
      toDeg(h().wheel_slip_alpha[0]), toDeg(h().wheel_slip_alpha[1]),
      toDeg(h().wheel_slip_alpha[2]), toDeg(h().wheel_slip_alpha[3]),
    ] as uPlot.AlignedData,
  },
];

export default function ChartPanel() {
  return (
    <Panel title="实时曲线" help={HELP.chart}>
      {BLOCKS.map((b) => (
        <ChartBlock key={b.filename} spec={b} />
      ))}
    </Panel>
  );
}
