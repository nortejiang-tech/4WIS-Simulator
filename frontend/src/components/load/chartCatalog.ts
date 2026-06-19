// R5 chart catalog: encodes every chart type so the 4 grid slots can pick from
// a dropdown. Each entry is consumed by a small <ChartSlot> wrapper.

import type uPlot from "uplot";
import type { SeriesSpec } from "@/charts/uplotFactory";
import { WHEEL_COLORS } from "@/components/canvas2d/colors";
import { WHEEL_LABELS } from "@/types/sim";
import { toKmh } from "@/ui/units";
import { EXPLANATIONS } from "./chartExplanations";
import { deg, type LoadRow, type LoadSweepResponse } from "./types";

export type ChartId =
  | "torque"
  | "rack_force"
  | "side_force_body_y"
  | "equilibrium"
  | "efficiency"
  | "arm_tie"
  | "tie_rack"
  | "wheel_side_force"
  | "side_force_sum"
  | "motor_torque"
  | "friction_utilization"
  | "fz"
  | "fx_drive"
  | "fy_camber";

export interface ChartConfig {
  id: ChartId;
  label: string;          // dropdown label
  title: (ctx: ChartContext) => string;
  filename: string;
  series: (ctx: ChartContext) => SeriesSpec[];
  data: (ctx: ChartContext) => uPlot.AlignedData;
  valueUnit?: string;
  xLabel: string;
  xUnit: string;
  yAxisLabel: string;
  xAxisLabel: string;
  explanation?: typeof EXPLANATIONS[keyof typeof EXPLANATIONS];
  autoZoom?: boolean;        // primary saturation-prone charts default true
  yScaleFromActual?: boolean; // true → Y range from series 0 only
  supportsAbsoluteValue?: boolean;
  /** When true, draw a vertical marker at the current-speed δ_eq position so
   *  the "free state is not at 0°" is obvious on the curve. */
  showEquilibriumMarker?: boolean;
}

export interface ChartContext {
  result: LoadSweepResponse | null;
  selectedRows: LoadRow[];
  // Profile-speed (linearly-interpolated) snapshot — same length as angleAxis.
  profileRows: InterpolatedRow[];
  // Display label like "30.0km/h" (rendered exact, not snap-to-grid).
  profileSpeedLabel: string;
  // Current-speed δ_eq in degrees (drawn as vertical marker on primary charts).
  profileDeltaEqDeg: number | null;
  // Multi-speed series for "by speed" charts.
  speedSeries: { speed: number; spec: SeriesSpec }[];
  angleAxis: number[]; // in degrees
}

// Interpolated row keeps only the numeric metrics we visualise.
export interface InterpolatedRow {
  requested_angle: number;
  delta_cmd: number;
  torque_steer: number;
  torque_steer_ideal: number;
  rack_force: number;
  rack_force_ideal: number;
  motor_torque: number;
  motor_torque_ideal: number;
  side_force_body_y: number;
  side_force_body_y_ideal: number;
  fz: number;
  fx_drive: number;
  fy_camber: number;
  friction_utilization: number;
  arm_tie_angle: number;
  tie_rack_angle: number;
  geometry_efficiency: number;
}

// ── helpers shared by configs ───────────────────────────────────────────────

const profileDualData = (ctx: ChartContext, actualKey: keyof InterpolatedRow, idealKey: keyof InterpolatedRow): uPlot.AlignedData => [
  ctx.profileRows.map((r) => deg(r.requested_angle)),
  ctx.profileRows.map((r) => Number(r[actualKey] ?? NaN)),
  ctx.profileRows.map((r) => Number(r[idealKey] ?? NaN)),
];

const profileSingleData = (ctx: ChartContext, key: keyof InterpolatedRow, transform?: (v: number) => number): uPlot.AlignedData => [
  ctx.profileRows.map((r) => deg(r.requested_angle)),
  ctx.profileRows.map((r) => {
    const v = Number(r[key] ?? NaN);
    return Number.isFinite(v) ? (transform ? transform(v) : v) : NaN;
  }),
];

const dualSeries = (label: string): SeriesSpec[] => ([
  { label: `${label} 实际`, color: "#60a5fa", width: 1.8 },
  { label: `${label} 悬架原生`, color: "#ef4444", width: 1.6, dash: [6, 4] },
]);

const singleSeries = (label: string, color = "#60a5fa"): SeriesSpec[] => ([
  { label, color, width: 1.8 },
]);

const bySpeedData = (ctx: ChartContext, metric: keyof LoadRow, transform?: (v: number) => number): uPlot.AlignedData => {
  const data: uPlot.AlignedData = [ctx.angleAxis];
  for (const s of ctx.speedSeries) {
    const vals = ctx.angleAxis.map((xDeg) => {
      const row = ctx.selectedRows.find(
        (r) => Math.abs(r.speed - s.speed) < 1e-9 && Math.abs(deg(r.requested_angle) - xDeg) < 1e-9,
      );
      const value = Number(row?.[metric] ?? NaN);
      return Number.isFinite(value) ? (transform ? transform(value) : value) : NaN;
    });
    data.push(vals);
  }
  return data;
};

const bySpeedSeries = (ctx: ChartContext): SeriesSpec[] =>
  ctx.speedSeries.map((s) => s.spec);

// ── registry ────────────────────────────────────────────────────────────────

export const CHART_CATALOG: Record<ChartId, ChartConfig> = {
  torque: {
    id: "torque",
    label: "τ 转向阻力矩",
    title: (ctx) => `τ_steer · ${ctx.profileSpeedLabel}`,
    filename: "load_torque",
    series: (ctx) => dualSeries(ctx.profileSpeedLabel),
    data: (ctx) => profileDualData(ctx, "torque_steer", "torque_steer_ideal"),
    valueUnit: "Nm",
    xLabel: "δ_cmd",
    xUnit: "°",
    xAxisLabel: "δ_cmd (°)",
    yAxisLabel: "τ_steer (Nm)",
    explanation: EXPLANATIONS.torque,
    autoZoom: true,
    yScaleFromActual: true,
    supportsAbsoluteValue: true,
    showEquilibriumMarker: true,
  },
  rack_force: {
    id: "rack_force",
    label: "F_rack 齿条力",
    title: (ctx) => `F_rack · ${ctx.profileSpeedLabel}`,
    filename: "load_rack_force",
    series: (ctx) => dualSeries(ctx.profileSpeedLabel),
    data: (ctx) => profileDualData(ctx, "rack_force", "rack_force_ideal"),
    valueUnit: "N",
    xLabel: "δ_cmd",
    xUnit: "°",
    xAxisLabel: "δ_cmd (°)",
    yAxisLabel: "F_rack (N)",
    explanation: EXPLANATIONS.rackForce,
    autoZoom: true,
    yScaleFromActual: true,
    supportsAbsoluteValue: true,
    showEquilibriumMarker: true,
  },
  side_force_body_y: {
    id: "side_force_body_y",
    label: "Fy_body 单轮侧向力",
    title: (ctx) => `Fy_body · ${ctx.profileSpeedLabel}`,
    filename: "load_side_force_profile",
    series: (ctx) => dualSeries(ctx.profileSpeedLabel),
    data: (ctx) => profileDualData(ctx, "side_force_body_y", "side_force_body_y_ideal"),
    valueUnit: "N",
    xLabel: "δ_cmd",
    xUnit: "°",
    xAxisLabel: "δ_cmd (°)",
    yAxisLabel: "Fy_body (N)",
    explanation: EXPLANATIONS.sideForce,
    autoZoom: true,
    yScaleFromActual: true,
    supportsAbsoluteValue: true,
    showEquilibriumMarker: true,
  },
  motor_torque: {
    id: "motor_torque",
    label: "τ_motor 电机轴扭矩",
    title: (ctx) => `τ_motor · ${ctx.profileSpeedLabel}`,
    filename: "load_motor_torque",
    series: (ctx) => dualSeries(ctx.profileSpeedLabel),
    data: (ctx) => profileDualData(ctx, "motor_torque", "motor_torque_ideal"),
    valueUnit: "Nm",
    xLabel: "δ_cmd",
    xUnit: "°",
    xAxisLabel: "δ_cmd (°)",
    yAxisLabel: "τ_motor (Nm)",
    explanation: EXPLANATIONS.rackForce, // shares physics with rack force
    autoZoom: true,
    yScaleFromActual: true,
    supportsAbsoluteValue: true,
  },
  friction_utilization: {
    id: "friction_utilization",
    label: "附着利用率",
    title: (ctx) => `附着利用率 · ${ctx.profileSpeedLabel}`,
    filename: "load_friction_utilization",
    series: (ctx) => singleSeries(`${ctx.profileSpeedLabel}`, "#fbbf24"),
    data: (ctx) => profileSingleData(ctx, "friction_utilization"),
    valueUnit: "",
    xLabel: "δ_cmd",
    xUnit: "°",
    xAxisLabel: "δ_cmd (°)",
    yAxisLabel: "|F|/(μF_z)",
    explanation: EXPLANATIONS.sideForce,
  },
  fz: {
    id: "fz",
    label: "F_z 垂载",
    title: (ctx) => `F_z · ${ctx.profileSpeedLabel}`,
    filename: "load_fz",
    series: (ctx) => singleSeries(`${ctx.profileSpeedLabel}`, "#a78bfa"),
    data: (ctx) => profileSingleData(ctx, "fz"),
    valueUnit: "N",
    xLabel: "δ_cmd",
    xUnit: "°",
    xAxisLabel: "δ_cmd (°)",
    yAxisLabel: "F_z (N)",
    explanation: EXPLANATIONS.torque,
  },
  fx_drive: {
    id: "fx_drive",
    label: "F_x 驱动力",
    title: (ctx) => `F_x_drive · ${ctx.profileSpeedLabel}`,
    filename: "load_fx_drive",
    series: (ctx) => singleSeries(`${ctx.profileSpeedLabel}`, "#f59e0b"),
    data: (ctx) => profileSingleData(ctx, "fx_drive"),
    valueUnit: "N",
    xLabel: "δ_cmd",
    xUnit: "°",
    xAxisLabel: "δ_cmd (°)",
    yAxisLabel: "F_x_drive (N)",
    explanation: EXPLANATIONS.torque,
  },
  fy_camber: {
    id: "fy_camber",
    label: "F_y camber thrust",
    title: (ctx) => `Fy_camber · ${ctx.profileSpeedLabel}`,
    filename: "load_fy_camber",
    series: (ctx) => singleSeries(`${ctx.profileSpeedLabel}`, "#34d399"),
    data: (ctx) => profileSingleData(ctx, "fy_camber"),
    valueUnit: "N",
    xLabel: "δ_cmd",
    xUnit: "°",
    xAxisLabel: "δ_cmd (°)",
    yAxisLabel: "Fy_camber (N)",
    explanation: EXPLANATIONS.sideForce,
  },
  equilibrium: {
    id: "equilibrium",
    label: "δ_eq 零输出转角 (vs 车速)",
    title: () => "零输出自然转角 δ_eq - 车速",
    filename: "load_equilibrium_angle",
    series: () => [{ label: "δ_eq", color: "#34d399", width: 2.5 }],
    data: (ctx) => {
      const eq = ctx.result?.summary?.per_speed_equilibrium ?? [];
      const xs = eq.map((r) => toKmh(r.speed));
      const ys = eq.map((r) => r.delta_eq_deg ?? NaN);
      return [xs, ys];
    },
    valueUnit: "°",
    xLabel: "v",
    xUnit: "km/h",
    xAxisLabel: "车速 v (km/h)",
    yAxisLabel: "δ_eq (°)",
    explanation: EXPLANATIONS.equilibrium,
  },
  efficiency: {
    id: "efficiency",
    label: "η 几何效率 (多车速)",
    title: () => "几何效率（各车速对比）",
    filename: "load_efficiency",
    series: bySpeedSeries,
    data: (ctx) => bySpeedData(ctx, "geometry_efficiency"),
    valueUnit: "",
    xLabel: "δ_cmd",
    xUnit: "°",
    xAxisLabel: "δ_cmd (°)",
    yAxisLabel: "η_linkage",
    explanation: EXPLANATIONS.efficiency,
  },
  arm_tie: {
    id: "arm_tie",
    label: "梯形臂-拉杆夹角 (多车速)",
    title: () => "梯形臂-拉杆夹角（多车速）",
    filename: "load_arm_tie_angle",
    series: bySpeedSeries,
    data: (ctx) => bySpeedData(ctx, "arm_tie_angle", deg),
    valueUnit: "°",
    xLabel: "δ_cmd",
    xUnit: "°",
    xAxisLabel: "δ_cmd (°)",
    yAxisLabel: "arm-tie (°)",
    explanation: EXPLANATIONS.armTie,
  },
  tie_rack: {
    id: "tie_rack",
    label: "拉杆-齿条夹角 (多车速)",
    title: () => "拉杆-齿条夹角（多车速）",
    filename: "load_tie_rack_angle",
    series: bySpeedSeries,
    data: (ctx) => bySpeedData(ctx, "tie_rack_angle", deg),
    valueUnit: "°",
    xLabel: "δ_cmd",
    xUnit: "°",
    xAxisLabel: "δ_cmd (°)",
    yAxisLabel: "tie-rack (°)",
    explanation: EXPLANATIONS.tieRack,
  },
  wheel_side_force: {
    id: "wheel_side_force",
    label: "四轮侧向力 @最高车速",
    title: () => "四轮侧向力 @最高车速",
    filename: "load_wheel_side_force",
    series: () => WHEEL_LABELS.map((w, i) => ({ label: w, color: WHEEL_COLORS[i] })),
    data: (ctx) => {
      const rows = ctx.result?.rows ?? [];
      const maxSpeed = Math.max(...rows.map((r) => r.speed), 0);
      const angles = Array.from(new Set(rows.map((r) => r.requested_angle))).sort((a, b) => a - b);
      const out: uPlot.AlignedData = [angles.map(deg)];
      for (let w = 0; w < 4; w++) {
        out.push(angles.map((a) => {
          const row = rows.find((r) => r.wheel_index === w && Math.abs(r.speed - maxSpeed) < 1e-9 && Math.abs(r.requested_angle - a) < 1e-9);
          return row?.side_force_body_y ?? NaN;
        }));
      }
      return out;
    },
    valueUnit: "N",
    xLabel: "δ_cmd",
    xUnit: "°",
    xAxisLabel: "δ_cmd (°)",
    yAxisLabel: "Fy_body (N)",
    explanation: EXPLANATIONS.wheelSideForce,
  },
  side_force_sum: {
    id: "side_force_sum",
    label: "左右轮侧向合力 @最高车速",
    title: () => "左右轮侧向合力 @最高车速",
    filename: "load_side_force_sum",
    series: () => [
      { label: "Left", color: "#22d3ee", width: 1.8 },
      { label: "Right", color: "#f97316", width: 1.8 },
      { label: "Total", color: "#fbbf24", width: 1.8 },
    ],
    data: (ctx) => {
      const rows = ctx.result?.rows ?? [];
      const maxSpeed = Math.max(...rows.map((r) => r.speed), 0);
      const angles = Array.from(new Set(rows.map((r) => r.requested_angle))).sort((a, b) => a - b);
      const left: number[] = [];
      const right: number[] = [];
      const total: number[] = [];
      for (const a of angles) {
        const sample = rows.filter((r) => Math.abs(r.speed - maxSpeed) < 1e-9 && Math.abs(r.requested_angle - a) < 1e-9);
        const l = sample.filter((r) => r.wheel_index === 0 || r.wheel_index === 2).reduce((sum, r) => sum + r.side_force_body_y, 0);
        const rr = sample.filter((r) => r.wheel_index === 1 || r.wheel_index === 3).reduce((sum, r) => sum + r.side_force_body_y, 0);
        left.push(l);
        right.push(rr);
        total.push(l + rr);
      }
      return [angles.map(deg), left, right, total];
    },
    valueUnit: "N",
    xLabel: "δ_cmd",
    xUnit: "°",
    xAxisLabel: "δ_cmd (°)",
    yAxisLabel: "ΣFy (N)",
    explanation: EXPLANATIONS.sideForceSum,
  },
};

export const CHART_ORDER: ChartId[] = [
  "torque", "rack_force", "side_force_body_y", "motor_torque",
  "friction_utilization", "fz", "fx_drive", "fy_camber",
  "equilibrium", "efficiency", "arm_tie", "tie_rack",
  "wheel_side_force", "side_force_sum",
];

export const DEFAULT_SLOT_CHARTS: [ChartId, ChartId, ChartId, ChartId] = [
  "torque", "rack_force", "side_force_body_y", "equilibrium",
];
