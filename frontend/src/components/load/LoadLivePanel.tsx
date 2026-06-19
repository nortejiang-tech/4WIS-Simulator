import { WHEEL_LABELS } from "@/types/sim";
import { MetricInfo } from "./MetricInfo";
import {
  LIVE_DELTA, LIVE_ETA, LIVE_FY_BODY, LIVE_RACK,
  LIVE_SIDE_LEFT, LIVE_SIDE_RIGHT, LIVE_SIDE_TOTAL, LIVE_SOURCE,
  LIVE_TAU,
} from "./metricExplanations";
import { RackSteerMechanism } from "./RackSteerMechanism";
import { deg, fmt, type VehicleParams } from "./types";

interface Props {
  params: VehicleParams | null;
  wheelIndex: number;
  wheels: any[] | undefined;
  summary: { left?: number; right?: number; total?: number; source?: string } | null;
}

export function LoadLivePanel({ params, wheelIndex, wheels, summary }: Props) {
  return (
    <section className="load-live-grid">
      <div className="load-live-card wide">
        <h3>实时四轮负载</h3>
        <div className="load-wheel-grid">
          {WHEEL_LABELS.map((w, i) => {
            const wheel = wheels?.[i];
            return (
              <div className="load-wheel-card" key={w}>
                <strong>{w}</strong>
                <span>
                  δ {fmt(wheel ? deg(wheel.delta) : undefined, 1)}°
                  <MetricInfo explanation={LIVE_DELTA} />
                </span>
                <span>
                  τ {fmt(wheel?.torque_steer, 1)} Nm
                  <MetricInfo explanation={LIVE_TAU} />
                </span>
                <span>
                  Rack {fmt(wheel?.rack_force, 0)} N
                  <MetricInfo explanation={LIVE_RACK} />
                </span>
                <span>
                  Fy_body {fmt(wheel?.side_force_body_y, 0)} N
                  <MetricInfo explanation={LIVE_FY_BODY} />
                </span>
                <span>
                  η {fmt(wheel?.linkage_efficiency, 3)}
                  <MetricInfo explanation={LIVE_ETA} />
                </span>
              </div>
            );
          })}
        </div>
        <div className="load-side-sum">
          <span>
            左侧 {fmt(summary?.left, 0)} N
            <MetricInfo explanation={LIVE_SIDE_LEFT} />
          </span>
          <span> · </span>
          <span>
            右侧 {fmt(summary?.right, 0)} N
            <MetricInfo explanation={LIVE_SIDE_RIGHT} />
          </span>
          <span> · </span>
          <span>
            合力 {fmt(summary?.total, 0)} N
            <MetricInfo explanation={LIVE_SIDE_TOTAL} />
          </span>
          <span className="load-side-source">
            {summary?.source ?? "waiting"}
            <MetricInfo explanation={LIVE_SOURCE} />
          </span>
        </div>
      </div>
      <div className="load-live-card">
        <h3>齿条-车轮几何</h3>
        <RackSteerMechanism params={params} wheelIndex={wheelIndex} />
      </div>
    </section>
  );
}
