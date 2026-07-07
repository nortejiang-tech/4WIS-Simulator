// Shared uPlot helpers — option factory, a React hook for live charts, and
// PNG / CSV export of whatever a chart currently displays.

import { useEffect, useRef } from "react";
import uPlot from "uplot";
import "uplot/dist/uPlot.min.css";

export interface SeriesSpec {
  label: string;
  color: string;
  width?: number;
  dash?: number[];   // e.g. [6, 4] for dashed
  show?: boolean;    // initial visibility (default true)
}

export interface AxisLabels {
  x?: string;
  y?: string;
}

export interface VerticalMarker {
  x: number;
  label?: string;
  color?: string;
  dash?: number[];
}

export interface OptionsExtras {
  /** Use this series index (1-based of series passed in; 0 = first user series)
   *  as the sole reference for Y auto-scaling. Useful when a second "ideal"
   *  series can be off-screen-large and would otherwise squish the actual data. */
  yScaleReferenceIndex?: number | null;
  /** Enable mouse-drag-to-zoom over the X axis. */
  dragZoom?: boolean;
  /** Vertical reference lines (e.g. δ_eq position) drawn via uPlot draw hook. */
  verticalMarkers?: VerticalMarker[];
}

export function makeOptions(
  width: number,
  height: number,
  series: SeriesSpec[],
  labels: AxisLabels = {},
  extras: OptionsExtras = {},
): uPlot.Options {
  const yRef = extras.yScaleReferenceIndex;
  const dragZoom = extras.dragZoom ?? true;
  return {
    title: "",
    width,
    height,
    legend: { show: false },
    cursor: {
      show: true,
      x: true,
      y: false,
      drag: {
        x: dragZoom,
        y: false,
        setScale: dragZoom,
        uni: 16,
      },
      points: {
        show: false,
        size: 7,
        width: 1,
        fill: "#0b1220",
      },
    },
    scales: {
      x: { time: false },
      y: yRef != null ? {
        range: (u, _min, _max) => {
          const idx = yRef + 1; // series array is 1-based (data[0] = X)
          const data = u.data[idx] as ArrayLike<number | null | undefined> | undefined;
          if (!data) return [_min, _max];
          let lo = Infinity, hi = -Infinity;
          for (let i = 0; i < data.length; i++) {
            const v = Number(data[i]);
            if (Number.isFinite(v)) { if (v < lo) lo = v; if (v > hi) hi = v; }
          }
          if (!Number.isFinite(lo) || !Number.isFinite(hi)) return [_min, _max];
          if (lo === hi) { lo -= 1; hi += 1; }
          const pad = (hi - lo) * 0.08;
          return [lo - pad, hi + pad];
        },
      } : {},
    },
    axes: [
      {
        stroke: "#94a3b8",
        grid: { stroke: "rgba(71, 85, 105, 0.4)", width: 1, dash: [4, 4] },
        ticks: { stroke: "rgba(71, 85, 105, 0.6)", width: 1 },
        size: labels.x ? 50 : 34,
        label: labels.x,
        labelSize: labels.x ? 14 : undefined,
        labelFont: '11px ui-sans-serif, system-ui',
        font: '11px ui-monospace, SFMono-Regular, Menlo, monospace',
      },
      {
        stroke: "#94a3b8",
        grid: { stroke: "rgba(71, 85, 105, 0.4)", width: 1, dash: [4, 4] },
        ticks: { stroke: "rgba(71, 85, 105, 0.6)", width: 1 },
        size: labels.y ? 64 : 54,
        label: labels.y,
        labelSize: labels.y ? 14 : undefined,
        labelFont: '11px ui-sans-serif, system-ui',
        font: '11px ui-monospace, SFMono-Regular, Menlo, monospace',
      },
    ],
    series: [
      {},
      ...series.map((s) => ({
        label: s.label,
        stroke: s.color,
        width: s.width ?? 1.4,
        dash: s.dash,
        show: s.show !== false,
        points: { show: false },
      })),
    ],
  };
}

function getXBounds(plot: uPlot): [number, number] | null {
  const xScale = plot.scales.x;
  const scaleMin = Number(xScale.min);
  const scaleMax = Number(xScale.max);
  if (Number.isFinite(scaleMin) && Number.isFinite(scaleMax) && scaleMax > scaleMin) {
    return [scaleMin, scaleMax];
  }
  const xs = plot.data[0] as ArrayLike<number | null | undefined> | undefined;
  if (!xs || xs.length === 0) return null;
  let min = Infinity;
  let max = -Infinity;
  for (let i = 0; i < xs.length; i++) {
    const x = Number(xs[i]);
    if (!Number.isFinite(x)) continue;
    if (x < min) min = x;
    if (x > max) max = x;
  }
  return Number.isFinite(min) && Number.isFinite(max) && max > min ? [min, max] : null;
}

function writeXScaleDiagnostics(el: HTMLElement, plot: uPlot) {
  const bounds = getXBounds(plot);
  if (!bounds) {
    delete el.dataset.chartXMin;
    delete el.dataset.chartXMax;
    delete el.dataset.chartXSpan;
    return;
  }
  const [min, max] = bounds;
  el.dataset.chartXMin = String(min);
  el.dataset.chartXMax = String(max);
  el.dataset.chartXSpan = String(max - min);
}

/**
 * Imperative live chart hook: builds the uPlot instance once, resizes with
 * its container, and calls setData whenever `signal` changes.
 */
export function useLiveChart(
  series: SeriesSpec[],
  getData: () => uPlot.AlignedData,
  signal: unknown,
  onCursor?: (plot: uPlot) => void,
  axisLabels?: AxisLabels,
  extras?: OptionsExtras,
  onZoomReset?: () => void,
) {
  const containerRef = useRef<HTMLDivElement>(null);
  const plotRef = useRef<uPlot | null>(null);
  const cursorRef = useRef<typeof onCursor>(onCursor);
  const zoomResetRef = useRef<typeof onZoomReset>(onZoomReset);
  // Vertical markers are kept on a mutable ref so they can update without
  // tearing down uPlot (which would clobber zoom/pan state).
  const markersRef = useRef<VerticalMarker[]>(extras?.verticalMarkers ?? []);

  useEffect(() => {
    cursorRef.current = onCursor;
  }, [onCursor]);

  useEffect(() => {
    zoomResetRef.current = onZoomReset;
  }, [onZoomReset]);

  useEffect(() => {
    markersRef.current = extras?.verticalMarkers ?? [];
    if (plotRef.current) plotRef.current.redraw();
  }, [extras?.verticalMarkers]);

  useEffect(() => {
    if (!containerRef.current) return;
    const el = containerRef.current;
    const updateScaleDiagnostics = (plot: uPlot) => writeXScaleDiagnostics(el, plot);
    const updateCursorDiagnostics = (plot: uPlot) => {
      const left = plot.cursor.left;
      if (typeof left !== "number" || !Number.isFinite(left) || left < 0) {
        delete el.dataset.chartCursorLeft;
        delete el.dataset.chartCursorX;
        return;
      }
      const x = plot.posToVal(left, "x");
      el.dataset.chartCursorLeft = String(left);
      if (Number.isFinite(x)) el.dataset.chartCursorX = String(x);
    };
    const options = makeOptions(el.clientWidth, el.clientHeight, series, axisLabels, extras);
    options.hooks = {
      setCursor: [
        (plot) => {
          updateCursorDiagnostics(plot);
          cursorRef.current?.(plot);
        },
      ],
      setScale: [
        (plot) => {
          updateScaleDiagnostics(plot);
        },
      ],
      draw: [
        (plot) => {
          updateScaleDiagnostics(plot);
          const markers = markersRef.current;
          if (!markers || markers.length === 0) return;
          const ctx = plot.ctx;
          ctx.save();
          for (const m of markers) {
            if (!Number.isFinite(m.x)) continue;
            const xPx = plot.valToPos(m.x, "x", true);
            if (!Number.isFinite(xPx)) continue;
            const top = plot.bbox.top;
            const bot = plot.bbox.top + plot.bbox.height;
            const left = plot.bbox.left;
            const right = plot.bbox.left + plot.bbox.width;
            if (xPx < left - 1 || xPx > right + 1) continue;
            ctx.beginPath();
            ctx.setLineDash(m.dash ?? [5, 4]);
            ctx.lineWidth = 1.5;
            ctx.strokeStyle = m.color ?? "#fbbf24";
            ctx.moveTo(xPx + 0.5, top);
            ctx.lineTo(xPx + 0.5, bot);
            ctx.stroke();
            ctx.setLineDash([]);
            if (m.label) {
              ctx.font = "11px ui-sans-serif, system-ui";
              ctx.fillStyle = m.color ?? "#fbbf24";
              ctx.textAlign = "left";
              ctx.textBaseline = "top";
              ctx.fillText(m.label, xPx + 5, top + 4);
            }
          }
          ctx.restore();
        },
      ],
    };
    plotRef.current = new uPlot(options, getData(), el);
    const resetZoom = () => {
      const plot = plotRef.current;
      if (!plot) return;
      plot.setScale("x", { min: null as any, max: null as any });
      plot.setScale("y", { min: null as any, max: null as any });
      updateScaleDiagnostics(plot);
      zoomResetRef.current?.();
    };
    const over = plotRef.current.root.querySelector(".u-over") as HTMLElement | null;
    over?.addEventListener("dblclick", resetZoom);
    updateScaleDiagnostics(plotRef.current);
    const ro = new ResizeObserver((entries) => {
      const cr = entries[0].contentRect;
      plotRef.current?.setSize({ width: cr.width, height: cr.height });
      if (plotRef.current) updateScaleDiagnostics(plotRef.current);
    });
    ro.observe(el);
    return () => {
      over?.removeEventListener("dblclick", resetZoom);
      ro.disconnect();
      plotRef.current?.destroy();
      plotRef.current = null;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useEffect(() => {
    plotRef.current?.setData(getData());
    if (plotRef.current && containerRef.current) writeXScaleDiagnostics(containerRef.current, plotRef.current);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [signal]);

  return { containerRef, plotRef };
}

// ---- export helpers ----

function download(filename: string, href: string) {
  const a = document.createElement("a");
  a.href = href;
  a.download = filename;
  a.click();
}

export function exportPNG(plot: uPlot | null, filename: string) {
  if (!plot) return;
  const src = plot.root.querySelector("canvas") as HTMLCanvasElement | null;
  if (!src) return;
  // Compose onto an opaque background so dark-theme charts stay readable.
  const out = document.createElement("canvas");
  out.width = src.width;
  out.height = src.height;
  const ctx = out.getContext("2d");
  if (!ctx) return;
  ctx.fillStyle = "#0b1220";
  ctx.fillRect(0, 0, out.width, out.height);
  ctx.drawImage(src, 0, 0);
  download(`${filename}.png`, out.toDataURL("image/png"));
}

export function exportCSV(plot: uPlot | null, filename: string, header: string[]) {
  if (!plot) return;
  const data = plot.data;
  if (!data || data.length === 0) return;
  const n = data[0].length;
  const lines: string[] = [header.join(",")];
  for (let i = 0; i < n; i++) {
    const row: string[] = [];
    for (let c = 0; c < data.length; c++) {
      const v = data[c][i];
      row.push(v == null || Number.isNaN(v as number) ? "" : String(v));
    }
    lines.push(row.join(","));
  }
  const blob = new Blob([lines.join("\n")], { type: "text/csv" });
  const url = URL.createObjectURL(blob);
  download(`${filename}.csv`, url);
  URL.revokeObjectURL(url);
}
