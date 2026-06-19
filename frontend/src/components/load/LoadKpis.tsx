import { MetricInfo } from "./MetricInfo";
import {
  KPI_DELTA_EQ, KPI_MAX_UTIL, KPI_MIN_EFF,
  KPI_PEAK_MOTOR, KPI_PEAK_RACK, KPI_RACK_AT_ZERO,
} from "./metricExplanations";
import { fmt, type PerSpeedEquilibrium } from "./types";

interface Props {
  peakRack?: number;
  peakMotor?: number;
  minEfficiency?: number;
  maxUtilization?: number;
  profileEquilibrium: PerSpeedEquilibrium | null;
}

export function LoadKpis({
  peakRack, peakMotor, minEfficiency, maxUtilization, profileEquilibrium,
}: Props) {
  const utilPct = maxUtilization != null && Number.isFinite(maxUtilization)
    ? maxUtilization * 100
    : null;
  return (
    <div className="load-kpi-row">
      <div className="load-kpi">
        <span>峰值齿条力<MetricInfo explanation={KPI_PEAK_RACK} /></span>
        <b>{fmt(peakRack, 0)} N</b>
      </div>
      <div className="load-kpi">
        <span>峰值电机力矩<MetricInfo explanation={KPI_PEAK_MOTOR} /></span>
        <b>{fmt(peakMotor, 2)} Nm</b>
      </div>
      <div className="load-kpi">
        <span>最低几何效率<MetricInfo explanation={KPI_MIN_EFF} /></span>
        <b>{fmt(minEfficiency, 3)}</b>
      </div>
      <div className="load-kpi">
        <span>最大附着利用<MetricInfo explanation={KPI_MAX_UTIL} /></span>
        <b>{fmt(utilPct, 1)}%</b>
      </div>
      <div className="load-kpi">
        <span>零输出自然转角<MetricInfo explanation={KPI_DELTA_EQ} /></span>
        <b>{fmt(profileEquilibrium?.delta_eq_deg, 2)}°</b>
      </div>
      <div className="load-kpi">
        <span>0°保持齿条力<MetricInfo explanation={KPI_RACK_AT_ZERO} /></span>
        <b>{fmt(profileEquilibrium?.rack_at_zero, 0)} N</b>
      </div>
    </div>
  );
}
