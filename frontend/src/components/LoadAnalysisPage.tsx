import { useCallback, useEffect, useMemo, useState } from "react";

import { WHEEL_COLORS } from "@/components/canvas2d/colors";
import { useSimStore } from "@/store/sim";
import { toKmh } from "@/ui/units";

import { ChartSlot } from "./load/ChartSlot";
import {
  CHART_CATALOG,
  DEFAULT_SLOT_CHARTS,
  type ChartContext,
  type ChartId,
  type InterpolatedRow,
} from "./load/chartCatalog";
import { LoadControls } from "./load/LoadControls";
import { LoadKpis } from "./load/LoadKpis";
import { LoadLivePanel } from "./load/LoadLivePanel";
import { LoadParamsEditor } from "./load/LoadParamsEditor";
import { ProfileToolbar } from "./load/ProfileToolbar";
import {
  deg,
  equilibriumAt,
  paramsHash,
  type BodyCoupling,
  type LoadRow,
} from "./load/types";
import { useLoadSweep } from "./load/useLoadSweep";

const NUMERIC_INTERP_KEYS: (keyof InterpolatedRow)[] = [
  "torque_steer", "torque_steer_ideal",
  "rack_force", "rack_force_ideal",
  "motor_torque", "motor_torque_ideal",
  "side_force_body_y", "side_force_body_y_ideal",
  "fz", "fx_drive", "fy_camber",
  "friction_utilization",
  "arm_tie_angle", "tie_rack_angle", "geometry_efficiency",
];

function interpolateProfileRows(
  rows: LoadRow[],          // selectedRows for the wheel of interest
  profileSpeedMs: number,
): InterpolatedRow[] {
  if (rows.length === 0) return [];
  const speedSet = Array.from(new Set(rows.map((r) => r.speed))).sort((a, b) => a - b);
  if (speedSet.length === 0) return [];

  // Bracket the profile speed between the two adjacent swept speeds.
  let loSpeed = speedSet[0];
  let hiSpeed = speedSet[speedSet.length - 1];
  if (profileSpeedMs <= loSpeed) { hiSpeed = loSpeed; }
  else if (profileSpeedMs >= hiSpeed) { loSpeed = hiSpeed; }
  else {
    for (let i = 0; i < speedSet.length - 1; i++) {
      if (speedSet[i] <= profileSpeedMs && profileSpeedMs <= speedSet[i + 1]) {
        loSpeed = speedSet[i];
        hiSpeed = speedSet[i + 1];
        break;
      }
    }
  }
  const t = hiSpeed === loSpeed ? 0 : (profileSpeedMs - loSpeed) / (hiSpeed - loSpeed);

  const angleSet = Array.from(new Set(rows.map((r) => r.requested_angle))).sort((a, b) => a - b);
  const out: InterpolatedRow[] = [];
  for (const angle of angleSet) {
    const lo = rows.find((r) => Math.abs(r.speed - loSpeed) < 1e-9 && Math.abs(r.requested_angle - angle) < 1e-9);
    const hi = rows.find((r) => Math.abs(r.speed - hiSpeed) < 1e-9 && Math.abs(r.requested_angle - angle) < 1e-9);
    if (!lo && !hi) continue;
    const anchor = lo ?? hi!;
    const ip: InterpolatedRow = {
      requested_angle: anchor.requested_angle,
      delta_cmd: anchor.delta_cmd,
      torque_steer: NaN, torque_steer_ideal: NaN,
      rack_force: NaN, rack_force_ideal: NaN,
      motor_torque: NaN, motor_torque_ideal: NaN,
      side_force_body_y: NaN, side_force_body_y_ideal: NaN,
      fz: NaN, fx_drive: NaN, fy_camber: NaN,
      friction_utilization: NaN,
      arm_tie_angle: NaN, tie_rack_angle: NaN, geometry_efficiency: NaN,
    };
    for (const k of NUMERIC_INTERP_KEYS) {
      const loV = lo ? Number((lo as any)[k]) : NaN;
      const hiV = hi ? Number((hi as any)[k]) : NaN;
      if (Number.isFinite(loV) && Number.isFinite(hiV)) {
        (ip as any)[k] = loV + (hiV - loV) * t;
      } else if (Number.isFinite(loV)) {
        (ip as any)[k] = loV;
      } else if (Number.isFinite(hiV)) {
        (ip as any)[k] = hiV;
      }
    }
    out.push(ip);
  }
  return out;
}

export default function LoadAnalysisPage() {
  const live = useSimStore((s) => s.state);
  const {
    params, profiles, result, busy,
    setParams, runSweep, loadProfile, saveProfile, applyProfileToSim,
  } = useLoadSweep();

  const [selectedProfile, setSelectedProfile] = useState("LS9");
  const [wheelIndex, setWheelIndex] = useState(0);
  const [mode, setMode] = useState("single_wheel");
  const [mu, setMu] = useState(0.85);
  // Default to 200 km/h max so the high-speed aero / drag effects are visible
  // out of the box (previously capped at 80 → user couldn't see plateau drift).
  const [speedMaxKmh, setSpeedMaxKmh] = useState(200);
  // Finer grid → smoother slider interpolation across the wider speed range.
  const [speedSteps, setSpeedSteps] = useState(21);
  const [profileSpeedKmh, setProfileSpeedKmh] = useState(30);
  const [angleMaxDeg, setAngleMaxDeg] = useState(35);
  const [angleSteps, setAngleSteps] = useState(101);
  // v0.8.1: single-wheel analysis framing — vehicle (default, bicycle-coupled)
  // or isolated (bench, α=−δ). Toggle in the toolbar; touching it triggers
  // a full re-sweep via the dep-list below.
  const [bodyCoupling, setBodyCoupling] = useState<BodyCoupling>("vehicle");

  // 4-slot grid layout: dropdown selections per slot.
  const [slotCharts, setSlotCharts] = useState<[ChartId, ChartId, ChartId, ChartId]>(DEFAULT_SLOT_CHARTS);
  const setSlot = (i: number) => (id: ChartId) => setSlotCharts((s) => {
    const next = [...s] as [ChartId, ChartId, ChartId, ChartId];
    next[i] = id;
    return next;
  });

  const inputs = useMemo(() => ({
    wheelIndex, mode, mu, speedMaxKmh, speedSteps, profileSpeedKmh, angleMaxDeg, angleSteps,
    bodyCoupling,
  }), [wheelIndex, mode, mu, speedMaxKmh, speedSteps, profileSpeedKmh, angleMaxDeg, angleSteps, bodyCoupling]);

  const doSweep = useCallback(() => runSweep(inputs), [runSweep, inputs]);

  // First-load auto-sweep once params arrive.
  useEffect(() => {
    if (params && !result && !busy) void doSweep();
  }, [params, result, busy, doSweep]);

  // Re-sweep on any sweep input change (debounced). Profile-speed changes don't
  // need a re-sweep — they're handled by row interpolation — but other inputs
  // (mode, mu, angle range, ...) do.
  useEffect(() => {
    if (!params) return;
    const handle = window.setTimeout(() => { void doSweep(); }, 350);
    return () => window.clearTimeout(handle);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [wheelIndex, mode, mu, speedMaxKmh, speedSteps, angleMaxDeg, angleSteps, bodyCoupling]);

  const exportRows = () => {
    const rows = result?.rows ?? [];
    if (!rows.length) return;
    const eq = result?.summary?.per_speed_equilibrium ?? [];
    const eqByKey: Record<string, number | null> = {};
    for (const e of eq) {
      eqByKey[(Math.round(e.speed * 1e6) / 1e6).toString()] = e.delta_eq_deg ?? null;
    }
    const cols = [...Object.keys(rows[0]) as (keyof LoadRow)[], "delta_eq_deg_at_speed"];
    const lines = [cols.join(",")];
    for (const row of rows) {
      const key = (Math.round(row.speed * 1e6) / 1e6).toString();
      const extra = eqByKey[key];
      lines.push([
        ...cols.slice(0, -1).map((c) => String(row[c as keyof LoadRow] ?? "")),
        extra == null ? "" : String(extra),
      ].join(","));
    }
    const blob = new Blob([lines.join("\n")], { type: "text/csv" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    const ts = new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19);
    a.href = url;
    a.download = `sim4wis_load_${ts}.csv`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const selectedRows = useMemo(
    () => (result?.rows ?? []).filter((r) => r.wheel_index === wheelIndex),
    [result, wheelIndex],
  );
  const effectiveProfileSpeedKmh = Math.max(0, Math.min(profileSpeedKmh, speedMaxKmh));
  const effectiveProfileSpeedMs = effectiveProfileSpeedKmh / 3.6;

  // chartSignal must change ONLY when fresh data arrives. Using the UI
  // bodyCoupling state would change at click-time (before the re-sweep finishes)
  // and then not change again when data arrives — leaving uPlot with the
  // previous mode's data. Using result.body_coupling solves this: it stays the
  // old value until the new sweep completes, then flips.
  const resultCoupling = result?.body_coupling ?? "vehicle";
  const chartSignal = `${selectedRows.length}:${wheelIndex}:${mode}:${speedMaxKmh}:${profileSpeedKmh}:${angleMaxDeg}:${resultCoupling}:${paramsHash(params)}`;

  const speedSeries = useMemo(() => {
    const speeds = Array.from(new Set(selectedRows.map((r) => r.speed))).slice(0, 6);
    return speeds.map((s, i) => ({
      speed: s,
      spec: { label: `${toKmh(s).toFixed(0)}km/h`, color: WHEEL_COLORS[i % WHEEL_COLORS.length], width: 1.6 },
    }));
  }, [selectedRows]);

  const angleAxis = useMemo(
    () => Array.from(new Set(selectedRows.map((r) => r.requested_angle))).sort((a, b) => a - b).map(deg),
    [selectedRows],
  );

  // R1: linear-interpolate profile rows so the slider response is truly
  // continuous, not stepped between sampled swept speeds.
  const profileRows = useMemo(
    () => interpolateProfileRows(selectedRows, effectiveProfileSpeedMs),
    [selectedRows, effectiveProfileSpeedMs],
  );
  const profileSpeedLabel = `${effectiveProfileSpeedKmh.toFixed(1)}km/h`;

  const equilibriumBySpeed = result?.summary?.per_speed_equilibrium ?? [];
  const profileEquilibrium = equilibriumAt(equilibriumBySpeed, effectiveProfileSpeedMs);

  const chartCtx: ChartContext = useMemo(() => ({
    result,
    selectedRows,
    profileRows,
    profileSpeedLabel,
    profileDeltaEqDeg: profileEquilibrium?.delta_eq_deg ?? null,
    speedSeries,
    angleAxis,
  }), [result, selectedRows, profileRows, profileSpeedLabel, profileEquilibrium, speedSeries, angleAxis]);

  const summary = result?.summary ?? {};
  const liveSummary = live?.side_force_summary ?? null;

  // sanity: drop any slot whose chart id is no longer in the registry.
  useEffect(() => {
    setSlotCharts((s) => s.map((id) => CHART_CATALOG[id] ? id : "torque") as [ChartId, ChartId, ChartId, ChartId]);
  }, []);

  return (
    <main className="load-page">
      <ProfileToolbar
        profiles={profiles}
        selected={selectedProfile}
        onSelect={setSelectedProfile}
        onLoad={() => loadProfile(selectedProfile)}
        onApply={() => applyProfileToSim(selectedProfile)}
        onSave={(name) => saveProfile(name)}
        onExport={exportRows}
        busy={busy}
        rows={result?.rows ?? []}
      />

      <section className="load-layout">
        <LoadParamsEditor
          params={params}
          onChange={setParams}
          onResetDefault={() => loadProfile("LS9")}
          resetDisabled={busy}
        />

        <section className="load-workspace">
          <LoadControls
            wheelIndex={wheelIndex} setWheelIndex={setWheelIndex}
            mode={mode} setMode={setMode}
            mu={mu} setMu={setMu}
            speedMaxKmh={speedMaxKmh} setSpeedMaxKmh={setSpeedMaxKmh}
            speedSteps={speedSteps} setSpeedSteps={setSpeedSteps}
            profileSpeedKmh={profileSpeedKmh} setProfileSpeedKmh={setProfileSpeedKmh}
            angleMaxDeg={angleMaxDeg} setAngleMaxDeg={setAngleMaxDeg}
            angleSteps={angleSteps} setAngleSteps={setAngleSteps}
            bodyCoupling={bodyCoupling} setBodyCoupling={setBodyCoupling}
            onRun={doSweep}
            busy={busy}
            paramsReady={!!params}
          />

          {/* R5: 2×2 chart grid with dropdown selectors. Each slot can display
              any chart from the catalog. Default = τ, F_rack, Fy_body, δ_eq. */}
          <section className="load-slot-grid">
            {slotCharts.map((id, i) => (
              <ChartSlot
                key={i}
                slotIndex={i}
                chartId={id}
                onChange={setSlot(i)}
                ctx={chartCtx}
                signal={chartSignal}
                bodyCoupling={resultCoupling}
              />
            ))}
          </section>

          <div className="load-model-note">
            蓝色实线 = 考虑轮胎 friction circle 后的实际值；红色虚线 = 假定无饱和限制时的悬架/几何"本征"响应。
            Y 轴自适应跟随蓝线；横轴自动聚焦到饱和前的线性区，可在每个图上独立关闭。
            鼠标可框选放大局部细节，双击或点"复位"还原。
          </div>

          <LoadKpis
            peakRack={summary.peak_abs_rack_force}
            minEfficiency={summary.min_geometry_efficiency}
            maxUtilization={summary.max_friction_utilization}
            profileEquilibrium={profileEquilibrium}
          />

          <LoadLivePanel
            params={params}
            wheelIndex={wheelIndex}
            wheels={live?.wheels}
            summary={liveSummary}
          />

          {(summary.warnings ?? []).length > 0 && (
            <div className="load-warnings">
              {(summary.warnings ?? []).map((w) => <span key={w}>{w}</span>)}
            </div>
          )}
        </section>
      </section>
    </main>
  );
}
