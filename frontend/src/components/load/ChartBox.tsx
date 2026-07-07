import { useCallback, useEffect, useMemo, useState } from "react";
import type uPlot from "uplot";

import { exportCSV, exportPNG, useLiveChart, type AxisLabels, type SeriesSpec, type VerticalMarker } from "@/charts/uplotFactory";
import { fmt, interpolateAt, type NumericSeries } from "./types";
import { renderInlineMath } from "./latex";
import "./ChartBox.css";
import "./LoadExplanation.css";

interface ChartBoxProps {
  title: string;
  filename: string;
  series: SeriesSpec[];
  data: uPlot.AlignedData;
  signal: unknown;
  valueUnit?: string;
  xLabel?: string;
  xUnit?: string;
  yAxisLabel?: string;
  xAxisLabel?: string;
  priority?: boolean;
  explanation?: ExplanationContent;
  /** Detect tire saturation on the actual (clipped) series and zoom X to the
   * pre-saturation region. Pass the series index used as the saturation ref. */
  autoZoomReferenceIndex?: number | null;
  /** Y axis auto-scales to this series only (typically the "actual" one), so
   *  the off-screen "ideal" linear extrapolation doesn't squish the actual curve. */
  yScaleReferenceIndex?: number | null;
  /** Allow toggling display to |y| (energy minimum is then a true minimum). */
  supportsAbsoluteValue?: boolean;
  /** Toolbar slot (e.g. chart-type dropdown when this is in a grid). */
  headerSlot?: React.ReactNode;
  /** Vertical reference lines drawn on the plot canvas. */
  verticalMarkers?: VerticalMarker[];
}

export interface ExplanationContent {
  title: string;
  sections: { heading?: string; body: string; figure?: string }[];
}

function detectSaturationXRange(
  data: uPlot.AlignedData,
  refIdx: number,
): [number, number] | null {
  const xs = data[0] as NumericSeries;
  const ys = data[refIdx + 1] as NumericSeries | undefined;
  if (!xs || !ys || xs.length < 5) return null;
  let absMax = 0;
  for (let i = 0; i < ys.length; i++) {
    const v = Number(ys[i]);
    if (Number.isFinite(v)) absMax = Math.max(absMax, Math.abs(v));
  }
  if (absMax < 1e-6) return null;
  const thr = 0.92 * absMax;
  let neg: number | null = null;
  let pos: number | null = null;
  for (let i = 0; i < xs.length; i++) {
    const x = Number(xs[i]);
    const v = Number(ys[i]);
    if (!Number.isFinite(x) || !Number.isFinite(v)) continue;
    if (Math.abs(v) >= thr) {
      if (x < 0 && (neg == null || x > neg)) neg = x;
      if (x > 0 && (pos == null || x < pos)) pos = x;
    }
  }
  if (neg == null && pos == null) return null;
  // Tighter zoom (0.6×) so 0.1° δ_eq offsets become visually obvious instead
  // of being lost in a ±10° pan view. Saturation shoulder is still in frame.
  const radius = Math.max(Math.abs(neg ?? -1), Math.abs(pos ?? 1)) * 0.6;
  if (radius < 0.5) return null;
  let xMin = Infinity, xMax = -Infinity;
  for (let i = 0; i < xs.length; i++) {
    const x = Number(xs[i]);
    if (Number.isFinite(x)) { if (x < xMin) xMin = x; if (x > xMax) xMax = x; }
  }
  return [Math.max(-radius, xMin), Math.min(radius, xMax)];
}

interface ToggleState {
  visibility: boolean[];
  autoZoom: boolean;
  absValue: boolean;
  userZoomed: boolean;
  setVisibility: React.Dispatch<React.SetStateAction<boolean[]>>;
  setAutoZoom: React.Dispatch<React.SetStateAction<boolean>>;
  setAbsValue: React.Dispatch<React.SetStateAction<boolean>>;
  setUserZoomed: React.Dispatch<React.SetStateAction<boolean>>;
}

function ChartBoxInner(props: Required<Pick<ChartBoxProps, "title" | "filename" | "series" | "data" | "signal">> & {
  valueUnit?: string;
  xLabel: string;
  xUnit: string;
  yAxisLabel?: string;
  xAxisLabel?: string;
  priority?: boolean;
  explanation?: ExplanationContent;
  autoZoomReferenceIndex: number | null;
  yScaleReferenceIndex: number | null;
  supportsAbsoluteValue: boolean;
  headerSlot?: React.ReactNode;
  verticalMarkers?: VerticalMarker[];
  toggleState: ToggleState;
}) {
  const {
    title, filename, series, data, signal, valueUnit, xLabel, xUnit, yAxisLabel, xAxisLabel,
    priority, explanation, autoZoomReferenceIndex, yScaleReferenceIndex, supportsAbsoluteValue, headerSlot, verticalMarkers,
    toggleState,
  } = props;
  const [cursorText, setCursorText] = useState("");
  const [helpOpen, setHelpOpen] = useState(false);
  // Toggle states (visibility, autoZoom, absValue, userZoomed) are *lifted*
  // to the outer ChartBox so they survive ChartBoxInner remounts. The inner
  // would otherwise reset every time the series labels change (e.g. profile
  // speed changes "30.0km/h 实际" → "35.0km/h 实际").
  const { visibility, setVisibility, autoZoom, setAutoZoom, absValue, setAbsValue, userZoomed, setUserZoomed } = toggleState;

  // Apply absolute-value transform when toggled.
  const effectiveData = useMemo((): uPlot.AlignedData => {
    if (!absValue) return data;
    return data.map((col, i) => {
      if (i === 0) return col;
      return (col as Array<number | null | undefined>).map((v) =>
        v != null && Number.isFinite(v) ? Math.abs(v as number) : v);
    }) as uPlot.AlignedData;
  }, [data, absValue]);

  const handleCursor = useCallback((plot: uPlot) => {
    const cursorLeft = Number(plot.cursor.left);
    if (!Number.isFinite(cursorLeft) || cursorLeft < 0 || cursorLeft > plot.bbox.width) {
      setCursorText("");
      return;
    }
    const x = plot.posToVal(cursorLeft, "x");
    if (!Number.isFinite(x)) {
      setCursorText("");
      return;
    }
    const xs = plot.data[0] as NumericSeries;
    const values = series.map((s, i) => {
      if (!visibility[i]) return null;
      const y = interpolateAt(xs, plot.data[i + 1] as NumericSeries, x);
      if (y == null || !Number.isFinite(y)) return `${s.label}: --`;
      return `${s.label}: ${fmt(y, Math.abs(y) >= 100 ? 0 : 2)}${valueUnit ? ` ${valueUnit}` : ""}`;
    }).filter(Boolean);
    setCursorText(`${xLabel} ${fmt(x, 2)}${xUnit} · ${values.join(" · ")}`);
  }, [series, valueUnit, xLabel, xUnit, visibility]);

  const axisLabels: AxisLabels = {
    x: xAxisLabel ?? `${xLabel}${xUnit ? ` (${xUnit.trim()})` : ""}`,
    y: yAxisLabel ?? (valueUnit ? `value (${valueUnit})` : undefined),
  };

  const effectiveSeries = useMemo(
    () => series.map((s, i) => ({ ...s, show: visibility[i] })),
    [series, visibility],
  );

  const { containerRef, plotRef } = useLiveChart(
    effectiveSeries,
    () => effectiveData,
    `${signal}|abs=${absValue}`,
    handleCursor,
    axisLabels,
    { yScaleReferenceIndex, dragZoom: true, verticalMarkers },
    () => setUserZoomed(false),
  );

  useEffect(() => {
    const plot = plotRef.current;
    if (!plot) return;
    for (let i = 0; i < visibility.length; i++) {
      plot.setSeries(i + 1, { show: visibility[i] });
    }
  }, [visibility, plotRef]);

  // Track whether the user has manually zoomed (drag-select). Once they have,
  // skip auto-zoom so we don't fight them.
  useEffect(() => {
    const plot = plotRef.current;
    if (!plot) return;
    const onSelect = () => setUserZoomed(true);
    plot.hooks.setSelect = plot.hooks.setSelect || [];
    plot.hooks.setSelect.push(onSelect);
  }, [plotRef]);

  useEffect(() => {
    const plot = plotRef.current;
    if (!plot) return;
    if (userZoomed) return;
    if (autoZoomReferenceIndex == null) {
      // No auto-zoom requested at all — leave the scale alone so uPlot's
      // built-in auto-range kicks in. (Setting min/max to null would freeze
      // the X axis at [null, null] and the curve never gets drawn.)
      return;
    }
    if (!autoZoom) {
      // User turned auto-zoom off — fall back to uPlot auto-range.
      plot.setScale("x", { min: plot.data[0][0] ?? 0, max: plot.data[0][plot.data[0].length - 1] ?? 1 });
      return;
    }
    const range = detectSaturationXRange(effectiveData, autoZoomReferenceIndex);
    if (range) plot.setScale("x", { min: range[0], max: range[1] });
    else {
      const xs = plot.data[0];
      if (xs && xs.length) {
        plot.setScale("x", { min: xs[0] ?? 0, max: xs[xs.length - 1] ?? 1 });
      }
    }
  }, [autoZoom, autoZoomReferenceIndex, effectiveData, signal, plotRef, userZoomed]);

  const toggle = (i: number) => setVisibility((v) => v.map((b, j) => (j === i ? !b : b)));

  const resetZoom = () => {
    setUserZoomed(false);
    const plot = plotRef.current;
    if (plot) {
      plot.setScale("x", { min: null as any, max: null as any });
      plot.setScale("y", { min: null as any, max: null as any });
    }
  };

  return (
    <section className={`load-chart-panel${priority ? " priority" : ""}`}>
      <div className="load-chart-head">
        <span className="load-chart-title">
          {headerSlot}
          <span>{title}</span>
        </span>
        <span className="load-chart-cursor">{cursorText || "悬停读数"}</span>
        <span className="load-chart-actions">
          {explanation && (
            <button className="load-chart-help-btn" onClick={() => setHelpOpen(true)} title="原理说明">
              原理
            </button>
          )}
          <button onClick={() => exportPNG(plotRef.current, filename)}>PNG</button>
          <button onClick={() => exportCSV(plotRef.current, filename, ["x", ...series.map((s) => s.label)])}>
            CSV
          </button>
        </span>
      </div>
      <div className="load-chart-toolbar">
        <div className="load-chart-legend">
          {series.map((s, i) => (
            <label key={s.label} className={`load-chart-legend-item${visibility[i] ? "" : " off"}`}>
              <input type="checkbox" checked={visibility[i] ?? true} onChange={() => toggle(i)} />
              <span className="legend-swatch" style={{ background: s.color, borderStyle: s.dash ? "dashed" : "solid" }} />
              <span>{s.label}</span>
            </label>
          ))}
        </div>
        <div className="load-chart-flags">
          {supportsAbsoluteValue && (
            <label className="load-chart-zoom-toggle" title="显示 |y|，便于看「能量最低点」">
              <input type="checkbox" checked={absValue} onChange={(e) => setAbsValue(e.target.checked)} />
              <span>|绝对值|</span>
            </label>
          )}
          {autoZoomReferenceIndex != null && (
            <label className="load-chart-zoom-toggle">
              <input type="checkbox" checked={autoZoom} onChange={(e) => { setAutoZoom(e.target.checked); setUserZoomed(false); }} />
              <span>自动聚焦</span>
            </label>
          )}
          {userZoomed && (
            <button className="load-chart-zoom-reset" onClick={resetZoom} title="重置缩放">复位</button>
          )}
        </div>
      </div>
      <div className="load-chart">
        <div className="load-chart-host" ref={containerRef} />
      </div>
      {explanation && helpOpen && (
        <ExplanationModal explanation={explanation} onClose={() => setHelpOpen(false)} />
      )}
    </section>
  );
}

function ExplanationModal({ explanation, onClose }: { explanation: ExplanationContent; onClose: () => void }) {
  const containerRef = (el: HTMLDivElement | null) => {
    if (el) renderInlineMath(el);
  };
  return (
    <div className="load-explanation-backdrop" onClick={onClose}>
      <div className="load-explanation-modal" onClick={(e) => e.stopPropagation()}>
        <header>
          <h3>{explanation.title}</h3>
          <button onClick={onClose} aria-label="关闭">✕</button>
        </header>
        <div className="load-explanation-body" ref={containerRef}>
          {explanation.sections.map((sec, idx) => (
            <section key={idx}>
              {sec.heading && <h4>{sec.heading}</h4>}
              <div className="load-explanation-text" dangerouslySetInnerHTML={{ __html: sec.body }} />
              {sec.figure && (
                <div className="load-explanation-figure" dangerouslySetInnerHTML={{ __html: sec.figure }} />
              )}
            </section>
          ))}
        </div>
      </div>
    </div>
  );
}

export function ChartBox(props: ChartBoxProps) {
  // T1: lift toggle state out of ChartBoxInner so it survives Inner remounts
  // (which happen when series labels change, e.g. "30km/h 实际" → "65km/h
  // 实际"). Without this the user's checkboxes reset every slider move.
  const seriesCount = props.series.length;
  const [visibility, setVisibility] = useState<boolean[]>(
    () => props.series.map((s) => s.show !== false),
  );
  const [autoZoom, setAutoZoom] = useState(true);
  const [absValue, setAbsValue] = useState(false);
  const [userZoomed, setUserZoomed] = useState(false);

  useEffect(() => {
    setVisibility((prev) => {
      if (prev.length === seriesCount) return prev;
      // Series count changed — preserve overlap, default new ones to "shown".
      return Array.from({ length: seriesCount }, (_, i) =>
        prev[i] ?? (props.series[i]?.show !== false));
    });
  }, [seriesCount, props.series]);

  // Treat zero-length columns as empty too — uPlot init with empty data then
  // setData later doesn't reliably render the line, so we wait for real data.
  const xs = props.data[0] as ArrayLike<unknown> | undefined;
  const noData = seriesCount === 0 || props.data.length <= 1 || !xs || xs.length === 0;
  if (noData) {
    return (
      <section className={`load-chart-panel${props.priority ? " priority" : ""}`}>
        <div className="load-chart-head">
          <span className="load-chart-title">
            {props.headerSlot}
            <span>{props.title}</span>
          </span>
        </div>
        <div className="load-chart load-chart-empty">等待计算</div>
      </section>
    );
  }
  const key = props.series.map((s) => s.label).join("|");
  const toggleState: ToggleState = {
    visibility, setVisibility,
    autoZoom, setAutoZoom,
    absValue, setAbsValue,
    userZoomed, setUserZoomed,
  };
  return (
    <ChartBoxInner
      key={key}
      title={props.title}
      filename={props.filename}
      series={props.series}
      data={props.data}
      signal={props.signal}
      valueUnit={props.valueUnit}
      xLabel={props.xLabel ?? "δ"}
      xUnit={props.xUnit ?? "°"}
      yAxisLabel={props.yAxisLabel}
      xAxisLabel={props.xAxisLabel}
      priority={props.priority}
      explanation={props.explanation}
      autoZoomReferenceIndex={props.autoZoomReferenceIndex ?? null}
      yScaleReferenceIndex={props.yScaleReferenceIndex ?? null}
      supportsAbsoluteValue={props.supportsAbsoluteValue ?? false}
      headerSlot={props.headerSlot}
      verticalMarkers={props.verticalMarkers}
      toggleState={toggleState}
    />
  );
}
