// Three interactive teaching demos for the model page. Each fetches curves from
// the read-only /api/model/demo/* endpoints (the real foundation model) and
// renders with the shared ChartBox. Sliders are debounced.

import { useEffect, useMemo, useRef, useState } from "react";
import type uPlot from "uplot";

import { postJSON } from "@/api/http";
import type { SeriesSpec } from "@/charts/uplotFactory";
import { ChartBox } from "@/components/load/ChartBox";

export type DemoKey = "bicycleGain" | "tireCurve" | "kingpinBreakdown";

function Slider({
  label, value, min, max, step, unit, onChange,
}: {
  label: string; value: number; min: number; max: number; step: number; unit: string;
  onChange: (v: number) => void;
}) {
  return (
    <label className="model-demo-slider">
      <span>{label}</span>
      <input type="range" min={min} max={max} step={step} value={value}
        onChange={(e) => onChange(Number(e.target.value))} />
      <b>{value}{unit}</b>
    </label>
  );
}

function useDebounced<T>(value: T, ms: number): T {
  const [v, setV] = useState(value);
  useEffect(() => {
    const h = window.setTimeout(() => setV(value), ms);
    return () => window.clearTimeout(h);
  }, [value, ms]);
  return v;
}

// ---- Demo 1: bicycle slip gain vs speed --------------------------------------

interface GainResp { points: { speed_kmh: number; slip_gain: number; delta_sat_deg: number }[] }

function BicycleGainDemo() {
  const [refDelta, setRefDelta] = useState(1);
  const d = useDebounced(refDelta, 250);
  const [resp, setResp] = useState<GainResp | null>(null);
  const reqId = useRef(0);

  useEffect(() => {
    const id = ++reqId.current;
    postJSON<GainResp>("/api/model/demo/bicycle-gain", {
      speeds_kmh: Array.from({ length: 21 }, (_, i) => i * 10),
      ref_delta_deg: d,
    }, 10000).then((r) => { if (id === reqId.current) setResp(r); }).catch(() => {});
  }, [d]);

  const data = useMemo((): uPlot.AlignedData => {
    const p = resp?.points ?? [];
    return [p.map((x) => x.speed_kmh), p.map((x) => x.slip_gain), p.map((x) => x.delta_sat_deg)];
  }, [resp]);
  const series: SeriesSpec[] = [
    { label: "滑移增益 |α/δ|", color: "#60a5fa", width: 2 },
    { label: "饱和转角 δ_sat (°)", color: "#f59e0b", width: 2, dash: [6, 4] },
  ];

  return (
    <div className="model-demo">
      <div className="model-demo-controls">
        <Slider label="参考转角 δ" value={refDelta} min={0.5} max={5} step={0.5} unit="°" onChange={setRefDelta} />
      </div>
      <ChartBox title="bicycle 滑移增益 / 饱和转角 vs 车速" filename="demo_bicycle_gain"
        series={series} data={data} signal={`${resp?.points.length ?? 0}:${d}`}
        xLabel="v" xUnit="km/h" xAxisLabel="车速 v (km/h)" yAxisLabel="增益 / δ_sat(°)" />
    </div>
  );
}

// ---- Demo 2: tyre Fy-alpha vs load -------------------------------------------

interface TireResp { mu_fz: number; curve: { alpha_deg: number; fy: number }[] }

function TireCurveDemo() {
  const [fzKn, setFzKn] = useState(7);
  const [mu, setMu] = useState(0.85);
  const fzD = useDebounced(fzKn, 250);
  const muD = useDebounced(mu, 250);
  const [resp, setResp] = useState<TireResp | null>(null);
  const reqId = useRef(0);

  useEffect(() => {
    const id = ++reqId.current;
    postJSON<TireResp>("/api/model/demo/tire-curve", {
      fz: fzD * 1000, mu: muD, alpha_max_deg: 15, points: 81,
    }, 10000).then((r) => { if (id === reqId.current) setResp(r); }).catch(() => {});
  }, [fzD, muD]);

  const data = useMemo((): uPlot.AlignedData => {
    const c = resp?.curve ?? [];
    const cap = resp?.mu_fz ?? NaN;
    return [c.map((x) => x.alpha_deg), c.map((x) => x.fy), c.map(() => cap), c.map(() => -cap)];
  }, [resp]);
  const series: SeriesSpec[] = [
    { label: "Fy (N)", color: "#34d399", width: 2.4 },
    { label: "+μ·Fz", color: "#f87171", width: 1, dash: [4, 4] },
    { label: "−μ·Fz", color: "#f87171", width: 1, dash: [4, 4] },
  ];

  return (
    <div className="model-demo">
      <div className="model-demo-controls">
        <Slider label="垂载 Fz" value={fzKn} min={2} max={12} step={0.5} unit=" kN" onChange={setFzKn} />
        <Slider label="附着 μ" value={mu} min={0.3} max={1.1} step={0.05} unit="" onChange={setMu} />
      </div>
      <ChartBox title="轮胎 Fy 随侧偏角 α（峰值 = μ·Fz）" filename="demo_tire_curve"
        series={series} data={data} signal={`${resp?.curve.length ?? 0}:${fzD}:${muD}`}
        valueUnit="N" xLabel="α" xUnit="°" xAxisLabel="侧偏角 α (°)" yAxisLabel="Fy (N)" />
    </div>
  );
}

// ---- Demo 3: kingpin torque term decomposition -------------------------------

interface KpResp {
  curve: { delta_cmd_deg: number; m_fy: number; m_fx: number; m_mz: number; m_kpi: number; torque_steer: number }[];
}

function KingpinBreakdownDemo() {
  const [speed, setSpeed] = useState(30);
  const [coupling, setCoupling] = useState<"vehicle" | "isolated">("vehicle");
  const d = useDebounced(speed, 250);
  const [resp, setResp] = useState<KpResp | null>(null);
  const reqId = useRef(0);

  useEffect(() => {
    const id = ++reqId.current;
    postJSON<KpResp>("/api/model/demo/kingpin-breakdown", {
      speed_kmh: d, angle_max_deg: 20, points: 61, body_coupling: coupling,
    }, 10000).then((r) => { if (id === reqId.current) setResp(r); }).catch(() => {});
  }, [d, coupling]);

  const data = useMemo((): uPlot.AlignedData => {
    const c = resp?.curve ?? [];
    return [
      c.map((x) => x.delta_cmd_deg),
      c.map((x) => x.m_fy), c.map((x) => x.m_fx),
      c.map((x) => x.m_mz), c.map((x) => x.m_kpi),
      c.map((x) => x.torque_steer),
    ];
  }, [resp]);
  const series: SeriesSpec[] = [
    { label: "Fy×(s+t_m)", color: "#60a5fa", width: 2 },
    { label: "Fx×偏置", color: "#f59e0b", width: 2 },
    { label: "Mz 气胎回正", color: "#a78bfa", width: 2 },
    { label: "KPI 抬升", color: "#34d399", width: 2 },
    { label: "合计 τ_KP", color: "#e5e7eb", width: 2.6 },
  ];

  return (
    <div className="model-demo">
      <div className="model-demo-controls">
        <Slider label="车速" value={speed} min={0} max={200} step={10} unit=" km/h" onChange={setSpeed} />
        <div className="load-bcoup" title="切换两种单轮分析口径">
          <span className="load-bcoup-label">口径</span>
          <div className="load-bcoup-seg" role="group">
            {(["vehicle", "isolated"] as const).map((id) => (
              <button key={id} type="button"
                className={`load-bcoup-btn${coupling === id ? " active" : ""}`}
                onClick={() => setCoupling(id)}>
                {id === "vehicle" ? "整车装载" : "单轮台架"}
              </button>
            ))}
          </div>
        </div>
      </div>
      <ChartBox title={`主销力矩四项分解 vs δ（FL · ${coupling === "vehicle" ? "整车装载" : "单轮台架"}）`}
        filename={`demo_kingpin_breakdown_${coupling}`}
        series={series} data={data} signal={`${resp?.curve.length ?? 0}:${d}:${coupling}`}
        valueUnit="Nm" xLabel="δ_cmd" xUnit="°" xAxisLabel="δ_cmd (°)" yAxisLabel="力矩 (N·m)" />
    </div>
  );
}

export const DEMOS: Record<DemoKey, () => JSX.Element> = {
  bicycleGain: BicycleGainDemo,
  tireCurve: TireCurveDemo,
  kingpinBreakdown: KingpinBreakdownDemo,
};
