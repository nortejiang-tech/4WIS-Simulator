import type { VerticalMarker } from "@/charts/uplotFactory";
import { ChartBox } from "./ChartBox";
import { CHART_CATALOG, CHART_ORDER, type ChartContext, type ChartId } from "./chartCatalog";

interface Props {
  slotIndex: number;
  chartId: ChartId;
  onChange: (id: ChartId) => void;
  ctx: ChartContext;
  signal: string;
}

export function ChartSlot({ slotIndex, chartId, onChange, ctx, signal }: Props) {
  const cfg = CHART_CATALOG[chartId];
  const headerSlot = (
    <select
      className="load-chart-slot-select"
      value={chartId}
      onChange={(e) => onChange(e.target.value as ChartId)}
      title="切换该格显示的图表"
    >
      {CHART_ORDER.map((id) => (
        <option key={id} value={id}>{CHART_CATALOG[id].label}</option>
      ))}
    </select>
  );
  const series = cfg.series(ctx);
  const data = cfg.data(ctx);

  // S3: vertical δ_eq marker on the primary τ/F_rack/Fy charts — makes the
  // "free state is at δ ≠ 0°" claim visually obvious instead of being lost in
  // a wide X range.
  const verticalMarkers: VerticalMarker[] | undefined =
    cfg.showEquilibriumMarker && ctx.profileDeltaEqDeg != null && Number.isFinite(ctx.profileDeltaEqDeg)
      ? [{
          x: ctx.profileDeltaEqDeg,
          label: `δ_eq=${ctx.profileDeltaEqDeg.toFixed(3)}°`,
          color: "#fbbf24",
          dash: [5, 4],
        }]
      : undefined;

  // Force-remount on chart-id change so uPlot starts fresh and never carries
  // over the empty-init state. ChartBox internally also keys by series labels,
  // but charts like "equilibrium" with constant labels never remount otherwise.
  return (
    <ChartBox
      key={`${chartId}-${slotIndex}`}
      title={cfg.title(ctx)}
      filename={`${cfg.filename}_slot${slotIndex + 1}`}
      series={series}
      data={data}
      signal={`${signal}|slot${slotIndex}|${chartId}`}
      valueUnit={cfg.valueUnit}
      xLabel={cfg.xLabel}
      xUnit={cfg.xUnit}
      xAxisLabel={cfg.xAxisLabel}
      yAxisLabel={cfg.yAxisLabel}
      explanation={cfg.explanation}
      autoZoomReferenceIndex={cfg.autoZoom ? 0 : null}
      yScaleReferenceIndex={cfg.yScaleFromActual ? 0 : null}
      supportsAbsoluteValue={cfg.supportsAbsoluteValue}
      headerSlot={headerSlot}
      verticalMarkers={verticalMarkers}
    />
  );
}
