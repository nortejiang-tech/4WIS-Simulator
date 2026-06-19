import { useCallback, useEffect, useMemo, useState } from "react";
import type uPlot from "uplot";

import { postJSON } from "@/api/http";
import { useSimStore } from "@/store/sim";
import { numInput, selectStyle } from "@/ui/styles";
import { ChartBox } from "./ChartBox";
import { EXPLANATIONS } from "./chartExplanations";
import {
  fmt,
  paramToBody,
  rad,
  rangeValues,
  type VehicleParams,
} from "./types";

interface SensitivityPoint {
  value: number;
  delta_eq: number | null;
  delta_eq_deg: number | null;
  rack_at_zero: number | null;
  torque_at_zero: number | null;
  found: boolean;
}

interface SensitivityResponse {
  vary_param: string;
  speed: number;
  wheel_index: number;
  points: SensitivityPoint[];
}

interface KnobOption {
  path: string;
  label: string;
  scale: number;
  unit: string;
  defaultMin: number;
  defaultMax: number;
  defaultSteps: number;
  step: number;
}

const KNOBS: KnobOption[] = [
  { path: "camber_thrust_coeff", label: "Camber thrust Cγ", scale: 1, unit: "/rad", defaultMin: 0, defaultMax: 2.0, defaultSteps: 11, step: 0.05 },
  { path: "static_toe_front", label: "前轴 toe-in", scale: 180 / Math.PI, unit: "°", defaultMin: -0.3, defaultMax: 0.3, defaultSteps: 13, step: 0.02 },
  { path: "static_toe_rear", label: "后轴 toe-in", scale: 180 / Math.PI, unit: "°", defaultMin: -0.3, defaultMax: 0.3, defaultSteps: 13, step: 0.02 },
  { path: "rolling_resistance_coeff", label: "滚阻 Crr", scale: 1, unit: "-", defaultMin: 0, defaultMax: 0.03, defaultSteps: 13, step: 0.002 },
  { path: "drag_coeff_cd", label: "风阻 Cd", scale: 1, unit: "-", defaultMin: 0, defaultMax: 0.6, defaultSteps: 13, step: 0.02 },
  { path: "frontal_area", label: "迎风面积 A", scale: 1, unit: "m²", defaultMin: 1.5, defaultMax: 4.0, defaultSteps: 13, step: 0.1 },
  { path: "suspension.camber", label: "外倾角 γ", scale: 180 / Math.PI, unit: "°", defaultMin: -2.0, defaultMax: 2.0, defaultSteps: 13, step: 0.1 },
  { path: "suspension.scrub_radius", label: "主销偏置 (scrub)", scale: 1000, unit: "mm", defaultMin: -10, defaultMax: 30, defaultSteps: 13, step: 1 },
  { path: "suspension.caster_angle", label: "主销后倾 ε", scale: 180 / Math.PI, unit: "°", defaultMin: 0, defaultMax: 12.0, defaultSteps: 13, step: 0.5 },
];

interface Props {
  params: VehicleParams | null;
  wheelIndex: number;
  profileSpeedMs: number;
  mu: number;
  angleMaxDeg: number;
  angleSteps: number;
  signal: unknown;
}

export function SensitivityPanel({ params, wheelIndex, profileSpeedMs, mu, angleMaxDeg, angleSteps, signal }: Props) {
  const pushToast = useSimStore((s) => s.pushToast);
  const [knobPath, setKnobPath] = useState(KNOBS[0].path);
  const knob = useMemo(() => KNOBS.find((k) => k.path === knobPath) ?? KNOBS[0], [knobPath]);
  const [rangeMin, setRangeMin] = useState(knob.defaultMin);
  const [rangeMax, setRangeMax] = useState(knob.defaultMax);
  const [steps, setSteps] = useState(knob.defaultSteps);
  const [data, setData] = useState<SensitivityResponse | null>(null);
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    // Reset range when the knob changes.
    setRangeMin(knob.defaultMin);
    setRangeMax(knob.defaultMax);
    setSteps(knob.defaultSteps);
  }, [knob]);

  const run = useCallback(async () => {
    if (!params) return;
    setBusy(true);
    try {
      const valuesUI = rangeValues(rangeMin, rangeMax, steps);
      const values = valuesUI.map((v) => v / knob.scale);
      const angles = rangeValues(-rad(angleMaxDeg), rad(angleMaxDeg), Math.min(angleSteps, 51));
      const resp = await postJSON<SensitivityResponse>(
        "/api/load-analysis/sensitivity",
        {
          params: paramToBody(params),
          vary_param: knob.path,
          values,
          speed: profileSpeedMs,
          angles,
          wheel_index: wheelIndex,
          mu,
        },
        15000,
      );
      setData(resp);
    } catch (e: any) {
      pushToast("error", `敏感度扫描失败：${e?.message ?? e}`);
    } finally {
      setBusy(false);
    }
  }, [angleMaxDeg, angleSteps, knob, mu, params, profileSpeedMs, pushToast, rangeMax, rangeMin, steps, wheelIndex]);

  // Auto-run whenever the upstream sweep refreshes (signal changes) or knob.
  useEffect(() => {
    if (params) void run();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [params, signal, knobPath]);

  const chartData = useMemo((): uPlot.AlignedData => {
    const points = data?.points ?? [];
    return [
      points.map((p) => p.value * knob.scale),
      points.map((p) => p.delta_eq_deg ?? NaN),
    ] as uPlot.AlignedData;
  }, [data, knob.scale]);

  const series = useMemo(() => ([
    { label: "δ_eq (°)", color: "#34d399", width: 1.8 },
  ]), []);

  return (
    <section className="load-sensitivity">
      <div className="load-sensitivity-head">
        <strong>δ_eq 敏感度 · v={(profileSpeedMs * 3.6).toFixed(0)} km/h · 单轮 {["FL","FR","RL","RR"][wheelIndex]}</strong>
        <div className="load-sensitivity-controls">
          <label>
            参数
            <select value={knobPath} onChange={(e) => setKnobPath(e.target.value)} style={selectStyle}>
              {KNOBS.map((k) => <option key={k.path} value={k.path}>{k.label}</option>)}
            </select>
          </label>
          <label>min <input type="number" step={knob.step} value={rangeMin} onChange={(e) => setRangeMin(Number(e.target.value))} style={numInput} /></label>
          <label>max <input type="number" step={knob.step} value={rangeMax} onChange={(e) => setRangeMax(Number(e.target.value))} style={numInput} /></label>
          <label>点数 <input type="number" step={1} min={3} max={41} value={steps} onChange={(e) => setSteps(Number(e.target.value))} style={numInput} /></label>
          <button onClick={run} disabled={busy || !params}>{busy ? "计算中" : "扫描"}</button>
        </div>
      </div>
      <ChartBox
        title={`δ_eq 随 ${knob.label} 变化（${knob.unit}）`}
        filename={`sensitivity_${knob.path.replace(/\./g, "_")}`}
        series={series}
        data={chartData}
        signal={`${data?.points.length ?? 0}:${knobPath}:${rangeMin}:${rangeMax}:${steps}`}
        valueUnit=""
        xLabel={knob.label}
        xUnit={` ${knob.unit}`}
        xAxisLabel={`${knob.label} (${knob.unit})`}
        yAxisLabel="δ_eq (°)"
        explanation={EXPLANATIONS.sensitivity}
      />
      <div className="load-sensitivity-summary">
        {data && data.points.length > 0 && (() => {
          const valid = data.points.filter((p) => p.delta_eq_deg != null);
          if (!valid.length) return <span>没有零交点</span>;
          const lo = valid.reduce((b, p) => p.delta_eq_deg! < b.delta_eq_deg! ? p : b);
          const hi = valid.reduce((b, p) => p.delta_eq_deg! > b.delta_eq_deg! ? p : b);
          const span = (hi.delta_eq_deg ?? 0) - (lo.delta_eq_deg ?? 0);
          return (
            <span>
              δ_eq 范围 {fmt(lo.delta_eq_deg, 3)}° ~ {fmt(hi.delta_eq_deg, 3)}° · 跨度 {fmt(span, 3)}° ·
              对应 rack@0 极差 {fmt((hi.rack_at_zero ?? 0) - (lo.rack_at_zero ?? 0), 0)} N
            </span>
          );
        })()}
      </div>
    </section>
  );
}
