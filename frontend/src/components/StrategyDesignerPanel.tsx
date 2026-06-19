/**
 * StrategyDesignerPanel — WYSIWYG no-code steering angle allocation designer.
 *
 * Two modes:
 *   "curve"  — 4 editable piecewise-linear curves (one per wheel).
 *              X = normalised steering input (−1…+1), Y = wheel angle (−60°…+60°).
 *              Drag control points; click empty area to add; right-click to delete.
 *   "icr"    — drag the instantaneous centre of rotation (ICR) on a vehicle
 *              top-view canvas. Wheel angles are computed from the geometry.
 *
 * When "user_js" is the active strategy, the panel evaluates its current mode's
 * angles on every state update and sends a steer_cmd over the WebSocket.
 */

import { useCallback, useEffect, useRef, useState } from "react";
import { useSimStore } from "@/store/sim";
import { sendMessage, setDriverMode } from "@/api/ws";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

// ─── Types ───────────────────────────────────────────────────────────────────

const WHEEL_KEYS = ["fl", "fr", "rl", "rr"] as const;
type WheelKey = (typeof WHEEL_KEYS)[number];

interface Pt { x: number; y: number }          // x ∈ [−1,1] normalised steer, y in °
type CurveMap = Record<WheelKey, Pt[]>;
type AngleMap = Record<WheelKey, number>;       // degrees

// ─── Constants ────────────────────────────────────────────────────────────────

const STEER_LIMIT = 35;                         // ° — shaded region
const VIEW_LIMIT  = 60;                         // ° — canvas y-axis extent
const CV_H        = 88;                         // curve canvas pixel height
const PT_R        = 5;                          // control-point dot radius px
const DEG         = Math.PI / 180;
const STORAGE_KEY = "sim4wis_strategy_designer_v1";
const KCURVE_KEY  = "sim4wis_kvx_schedule_v1";

// k(vx) schedule editor extents.
const KV_VMAX = 200;   // km/h (x-axis)
const KV_KMAX = 1;     // |ratio| (y-axis)

interface KPt { v: number; k: number }   // v in km/h, k = δ_rear/δ_front

// Default schedule = the backend zero-sideslip ratio for the LS9 calibration
// (analytic k(vx); counter-phase at low speed, crossing to in-phase ~59 km/h).
const DEFAULT_KCURVE: KPt[] = [
  { v: 0,   k: -1.04 },
  { v: 20,  k: -0.82 },
  { v: 40,  k: -0.38 },
  { v: 60,  k: 0.01 },
  { v: 90,  k: 0.39 },
  { v: 130, k: 0.64 },
  { v: 200, k: 0.81 },
];

function loadKCurve(): KPt[] {
  try {
    const raw = localStorage.getItem(KCURVE_KEY);
    if (raw) {
      const arr = JSON.parse(raw);
      if (Array.isArray(arr) && arr.length >= 2) return arr;
    }
  } catch { /* ignore */ }
  return DEFAULT_KCURVE.map((p) => ({ ...p }));
}

const WHEEL_LABELS: Record<WheelKey, string> = {
  fl: "前左 FL", fr: "前右 FR", rl: "后左 RL", rr: "后右 RR",
};

// Vehicle half-dimensions (body frame, metres)
const V = { a: 1.25, b: 1.25, tf: 0.8, tr: 0.8 };

// Wheel positions in body frame [forward, left]
const WHEEL_POS: Record<WheelKey, [number, number]> = {
  fl: [ V.a,  V.tf],
  fr: [ V.a, -V.tf],
  rl: [-V.b,  V.tr],
  rr: [-V.b, -V.tr],
};

// ICR canvas geometry
const IC = { W: 320, H: 230, CX: 160, CY: 115, PPM: 28 } as const;

// ─── Presets ──────────────────────────────────────────────────────────────────

const PRESETS: Record<string, () => CurveMap> = {
  ackermann: () => ({
    fl: [{ x: -1, y: -40 }, { x: 0, y: 0 }, { x: 1, y: 40 }],
    fr: [{ x: -1, y: -30 }, { x: 0, y: 0 }, { x: 1, y: 30 }],
    rl: [{ x: -1, y:   0 }, { x: 0, y: 0 }, { x: 1, y:  0 }],
    rr: [{ x: -1, y:   0 }, { x: 0, y: 0 }, { x: 1, y:  0 }],
  }),
  crab: () => Object.fromEntries(
    WHEEL_KEYS.map((w) => [w, [{ x: -1, y: -STEER_LIMIT }, { x: 0, y: 0 }, { x: 1, y: STEER_LIMIT }]])
  ) as CurveMap,
  rear_counter: () => ({
    fl: [{ x: -1, y: -STEER_LIMIT }, { x: 0, y: 0 }, { x: 1, y:  STEER_LIMIT      }],
    fr: [{ x: -1, y: -STEER_LIMIT }, { x: 0, y: 0 }, { x: 1, y:  STEER_LIMIT      }],
    rl: [{ x: -1, y:  STEER_LIMIT * 0.5 }, { x: 0, y: 0 }, { x: 1, y: -STEER_LIMIT * 0.5 }],
    rr: [{ x: -1, y:  STEER_LIMIT * 0.5 }, { x: 0, y: 0 }, { x: 1, y: -STEER_LIMIT * 0.5 }],
  }),
  pivot: () => ({
    fl: [{ x: -1, y: -STEER_LIMIT }, { x: 0, y: 0 }, { x: 1, y:  STEER_LIMIT }],
    fr: [{ x: -1, y:  STEER_LIMIT }, { x: 0, y: 0 }, { x: 1, y: -STEER_LIMIT }],
    rl: [{ x: -1, y:  STEER_LIMIT }, { x: 0, y: 0 }, { x: 1, y: -STEER_LIMIT }],
    rr: [{ x: -1, y: -STEER_LIMIT }, { x: 0, y: 0 }, { x: 1, y:  STEER_LIMIT }],
  }),
};

const PRESET_LABELS: Record<string, string> = {
  ackermann: "阿克曼", crab: "蟹行", rear_counter: "后轮反向", pivot: "零半径", custom: "自定义",
};

// ─── Math helpers ─────────────────────────────────────────────────────────────

function evalCurve(pts: Pt[], x: number): number {
  if (!pts.length) return 0;
  if (x <= pts[0].x) return pts[0].y;
  if (x >= pts[pts.length - 1].x) return pts[pts.length - 1].y;
  for (let i = 0; i < pts.length - 1; i++) {
    if (x >= pts[i].x && x <= pts[i + 1].x) {
      const t = (x - pts[i].x) / (pts[i + 1].x - pts[i].x);
      return pts[i].y + t * (pts[i + 1].y - pts[i].y);
    }
  }
  return 0;
}

function clamp(v: number, lo: number, hi: number) {
  return Math.max(lo, Math.min(hi, v));
}

function anglesFromCurves(curves: CurveMap, steer: number): AngleMap {
  return Object.fromEntries(
    WHEEL_KEYS.map((w) => [w, clamp(evalCurve(curves[w], steer), -VIEW_LIMIT, VIEW_LIMIT)])
  ) as AngleMap;
}

// δ_wheel = atan2(px − icr_x, icr_y − py) where all coords are body-frame [fwd, left].
// The raw atan2 assumes CCW travel (ICR on the left); when the ICR is on the
// right the consistent heading is the *opposite* perpendicular, so we wrap the
// result into the wheel-line range (−90°,90°] (a wheel pointing "back" is the
// same steering line as pointing "forward"). Without this, a right-side ICR
// produced ~±162° → clamped to ±60° → wrong direction & orientation.
function wrapPmHalf(deg: number): number {
  if (deg > 90) return deg - 180;
  if (deg < -90) return deg + 180;
  return deg;
}
function anglesFromICR(icr_x: number, icr_y: number): AngleMap {
  return Object.fromEntries(
    WHEEL_KEYS.map((w) => {
      const [px, py] = WHEEL_POS[w];
      const deg = wrapPmHalf(Math.atan2(px - icr_x, icr_y - py) / DEG);
      return [w, clamp(deg, -VIEW_LIMIT, VIEW_LIMIT)];
    })
  ) as AngleMap;
}

function turningRadius(a: AngleMap): number | null {
  const avg = WHEEL_KEYS.reduce((s, w) => s + Math.tan(a[w] * DEG), 0) / 4;
  return Math.abs(avg) > 0.005 ? clamp(Math.abs(1.25 / avg), 0, 99) : null;
}

function loadCurves(): CurveMap {
  try {
    const s = localStorage.getItem(STORAGE_KEY);
    if (s) return JSON.parse(s) as CurveMap;
  } catch {}
  return PRESETS.ackermann();
}

// ─── Canvas: curve editor ─────────────────────────────────────────────────────

function renderCurve(
  canvas: HTMLCanvasElement, pts: Pt[], steer: number, dragIdx: number
): void {
  const W = canvas.width, H = canvas.height;
  if (!W || !H) return;
  const ctx = canvas.getContext("2d")!;
  ctx.clearRect(0, 0, W, H);

  const toP = (x: number, y: number) => ({
    px: ((x + 1) / 2) * W,
    py: ((1 - y / VIEW_LIMIT) / 2) * H,
  });

  // Grid
  ctx.strokeStyle = "rgba(45,52,72,0.6)";
  ctx.lineWidth = 0.5;
  for (const gx of [-0.5, 0, 0.5]) {
    const { px } = toP(gx, 0);
    ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, H); ctx.stroke();
  }
  for (const gy of [-VIEW_LIMIT, -STEER_LIMIT, 0, STEER_LIMIT, VIEW_LIMIT]) {
    const { py } = toP(0, gy);
    ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(W, py); ctx.stroke();
  }
  // ±STEER_LIMIT band tint
  const { py: bt } = toP(0, STEER_LIMIT), { py: bb } = toP(0, -STEER_LIMIT);
  ctx.fillStyle = "rgba(34,211,238,0.04)";
  ctx.fillRect(0, bt, W, bb - bt);
  // Axes
  ctx.strokeStyle = "rgba(61,74,99,0.9)"; ctx.lineWidth = 1;
  const { px: ax, py: ay } = toP(0, 0);
  ctx.beginPath(); ctx.moveTo(ax, 0); ctx.lineTo(ax, H); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0, ay); ctx.lineTo(W, ay); ctx.stroke();

  if (pts.length < 2) return;

  // Curve line
  ctx.beginPath(); ctx.strokeStyle = "#22d3ee"; ctx.lineWidth = 2;
  for (let i = 0; i <= 200; i++) {
    const x = -1 + i / 100;
    const { px, py } = toP(x, evalCurve(pts, x));
    i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
  }
  ctx.stroke();

  // Cursor
  const { px: cX, py: cY } = toP(steer, evalCurve(pts, steer));
  ctx.setLineDash([3, 3]); ctx.strokeStyle = "rgba(250,204,21,0.55)"; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(cX, 0); ctx.lineTo(cX, H); ctx.stroke();
  ctx.setLineDash([]);
  ctx.beginPath(); ctx.arc(cX, cY, 5, 0, Math.PI * 2);
  ctx.fillStyle = "#facc15"; ctx.fill();

  // Control points
  pts.forEach((pt, i) => {
    const { px, py } = toP(pt.x, pt.y);
    ctx.beginPath(); ctx.arc(px, py, PT_R, 0, Math.PI * 2);
    ctx.fillStyle = i === dragIdx ? "#fff" : "#22d3ee"; ctx.fill();
    ctx.strokeStyle = "#0a0d14"; ctx.lineWidth = 1.5; ctx.stroke();
  });
}

// ─── Canvas: ICR drag view ────────────────────────────────────────────────────

function renderICR(
  canvas: HTMLCanvasElement,
  angles: AngleMap,
  icrCv: { x: number; y: number } | null
): void {
  const { W, H, CX, CY, PPM } = IC;
  const ctx = canvas.getContext("2d")!;
  ctx.clearRect(0, 0, W, H);

  // body → canvas: fwd=up, left=left
  const toC = (bx: number, by: number) => ({ cx: CX - by * PPM, cy: CY - bx * PPM });

  // Grid (every 1 m)
  ctx.strokeStyle = "rgba(45,52,72,0.45)"; ctx.lineWidth = 0.5;
  for (let d = -6; d <= 6; d++) {
    const { cx } = toC(0, d);
    ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, H); ctx.stroke();
  }
  for (let d = -5; d <= 5; d++) {
    const { cy } = toC(d, 0);
    ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(W, cy); ctx.stroke();
  }
  // Axes
  ctx.strokeStyle = "rgba(61,74,99,0.7)"; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(CX, 0); ctx.lineTo(CX, H); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0, CY); ctx.lineTo(W, CY); ctx.stroke();

  // Vehicle body
  const bw = (V.tf + V.tr) * PPM + 18, bh = (V.a + V.b) * PPM + 18;
  ctx.fillStyle = "#2d3448"; ctx.strokeStyle = "#3d4a63"; ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.rect(CX - bw / 2, CY - bh / 2, bw, bh);
  ctx.fill(); ctx.stroke();
  // Forward arrow
  ctx.fillStyle = "#22d3ee";
  ctx.beginPath();
  ctx.moveTo(CX, CY - bh / 2 - 10);
  ctx.lineTo(CX - 5, CY - bh / 2);
  ctx.lineTo(CX + 5, CY - bh / 2);
  ctx.closePath(); ctx.fill();

  // Wheels + ICR lines
  WHEEL_KEYS.forEach((w) => {
    const [bx, by] = WHEEL_POS[w];
    const { cx: wcx, cy: wcy } = toC(bx, by);
    const rad = angles[w] * DEG;

    // Wheel rectangle, rotated around its centre
    ctx.save(); ctx.translate(wcx, wcy); ctx.rotate(-rad);
    ctx.fillStyle = "#22d3ee"; ctx.strokeStyle = "#0a0d14"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.rect(-5, -13, 10, 26); ctx.fill(); ctx.stroke();
    ctx.restore();

    // Line to ICR
    if (icrCv) {
      ctx.setLineDash([3, 4]);
      ctx.strokeStyle = "rgba(250,204,21,0.22)"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(wcx, wcy); ctx.lineTo(icrCv.x, icrCv.y); ctx.stroke();
      ctx.setLineDash([]);
    }
  });

  // ICR dot
  if (icrCv) {
    ctx.beginPath(); ctx.arc(icrCv.x, icrCv.y, 7, 0, Math.PI * 2);
    ctx.fillStyle = "#facc15"; ctx.fill();
    ctx.strokeStyle = "#0a0d14"; ctx.lineWidth = 1.5; ctx.stroke();
  }

  // Radius label
  const R = turningRadius(angles);
  ctx.font = "11px monospace"; ctx.fillStyle = "#facc15"; ctx.textAlign = "left";
  ctx.fillText(R === null ? "R = ∞" : `R ≈ ${R.toFixed(1)} m`, 6, 14);
}

// ─── Sub-component: WheelCurveEditor ──────────────────────────────────────────

interface EditorProps {
  wheel: WheelKey;
  pts: Pt[];
  steer: number;
  onChange: (pts: Pt[]) => void;
  onCustomize: () => void;
}

function WheelCurveEditor({ wheel, pts, steer, onChange, onCustomize }: EditorProps) {
  const cvRef = useRef<HTMLCanvasElement>(null);
  const ptsRef = useRef(pts); ptsRef.current = pts;
  const steerRef = useRef(steer); steerRef.current = steer;
  const dragRef = useRef<{ idx: number } | null>(null);

  // Redraw on every change
  useEffect(() => {
    const cv = cvRef.current; if (!cv) return;
    cv.width = cv.offsetWidth || 150; cv.height = CV_H;
    renderCurve(cv, pts, steer, dragRef.current?.idx ?? -1);
  }, [pts, steer]);

  // Responsive resize
  useEffect(() => {
    const cv = cvRef.current; if (!cv) return;
    const ro = new ResizeObserver(() => {
      cv.width = cv.offsetWidth || 150; cv.height = CV_H;
      renderCurve(cv, ptsRef.current, steerRef.current, dragRef.current?.idx ?? -1);
    });
    ro.observe(cv);
    return () => ro.disconnect();
  }, []);

  const toP = (x: number, y: number, W: number, H: number) => ({
    px: ((x + 1) / 2) * W, py: ((1 - y / VIEW_LIMIT) / 2) * H,
  });
  const fromP = (px: number, py: number, W: number, H: number): Pt => ({
    x: clamp(px / W * 2 - 1, -1, 1),
    y: clamp((1 - py / H * 2) * VIEW_LIMIT, -VIEW_LIMIT * 1.15, VIEW_LIMIT * 1.15),
  });

  const onMD = (e: React.MouseEvent<HTMLCanvasElement>) => {
    e.preventDefault();
    const cv = cvRef.current!;
    const r = cv.getBoundingClientRect();
    const px = e.clientX - r.left, py = e.clientY - r.top;
    const W = cv.offsetWidth, H = CV_H;
    const cur = ptsRef.current;

    let best = -1, minD = 14;
    cur.forEach((pt, i) => {
      const { px: cx, py: cy } = toP(pt.x, pt.y, W, H);
      const d = Math.hypot(px - cx, py - cy);
      if (d < minD) { minD = d; best = i; }
    });

    if (best >= 0) {
      dragRef.current = { idx: best };
    } else {
      const np = fromP(px, py, W, H);
      np.x = clamp(np.x, -0.99, 0.99);
      const newPts = [...cur, np].sort((a, b) => a.x - b.x);
      const newIdx = newPts.indexOf(np);
      dragRef.current = { idx: newIdx };
      onChange(newPts);
      onCustomize();
    }
  };

  const onMM = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!dragRef.current) return;
    const cv = cvRef.current!;
    const r = cv.getBoundingClientRect();
    const { idx } = dragRef.current;
    const cur = [...ptsRef.current];
    const pos = fromP(e.clientX - r.left, e.clientY - r.top, cv.offsetWidth, CV_H);
    const minX = idx > 0 ? cur[idx - 1].x + 0.02 : -1;
    const maxX = idx < cur.length - 1 ? cur[idx + 1].x - 0.02 : 1;
    cur[idx] = { x: clamp(pos.x, minX, maxX), y: pos.y };
    onChange(cur);
  };

  const onMU = () => { dragRef.current = null; };

  const onCM = (e: React.MouseEvent<HTMLCanvasElement>) => {
    e.preventDefault();
    const cur = ptsRef.current;
    if (cur.length <= 2) return;
    const cv = cvRef.current!;
    const r = cv.getBoundingClientRect();
    const px = e.clientX - r.left, py = e.clientY - r.top;
    const W = cv.offsetWidth, H = CV_H;
    let best = -1, minD = 14;
    cur.forEach((pt, i) => {
      const { px: cx, py: cy } = toP(pt.x, pt.y, W, H);
      const d = Math.hypot(px - cx, py - cy);
      if (d < minD) { minD = d; best = i; }
    });
    if (best >= 0) onChange(cur.filter((_, i) => i !== best));
  };

  return (
    <div>
      <div style={{ fontSize: 10, color: "var(--muted)", marginBottom: 2 }}>{WHEEL_LABELS[wheel]}</div>
      <canvas
        ref={cvRef}
        style={{ display: "block", width: "100%", height: CV_H, cursor: "crosshair",
                 borderRadius: 3, border: "1px solid var(--border)" }}
        onMouseDown={onMD} onMouseMove={onMM} onMouseUp={onMU}
        onMouseLeave={onMU} onContextMenu={onCM}
      />
    </div>
  );
}

// ─── Sub-component: ICRDragCanvas ─────────────────────────────────────────────

interface ICRCanvasProps {
  angles: AngleMap;
  onAnglesChange: (a: AngleMap) => void;
}

function ICRDragCanvas({ angles, onAnglesChange }: ICRCanvasProps) {
  const cvRef = useRef<HTMLCanvasElement>(null);
  const icrRef = useRef<{ x: number; y: number } | null>(null);
  const dragging = useRef(false);
  const anglesRef = useRef(angles); anglesRef.current = angles;

  const getPos = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    const cv = cvRef.current!;
    const r = cv.getBoundingClientRect();
    const sx = IC.W / r.width, sy = IC.H / r.height;
    return { x: (e.clientX - r.left) * sx, y: (e.clientY - r.top) * sy };
  }, []);

  const updateICR = useCallback((cx: number, cy: number) => {
    icrRef.current = { x: cx, y: cy };
    // canvas → body frame
    const bx = -(cy - IC.CY) / IC.PPM;
    const by = -(cx - IC.CX) / IC.PPM;
    onAnglesChange(anglesFromICR(bx, by));
  }, [onAnglesChange]);

  useEffect(() => {
    const cv = cvRef.current; if (!cv) return;
    renderICR(cv, angles, icrRef.current);
  }, [angles]);

  return (
    <div>
      <canvas
        ref={cvRef}
        width={IC.W} height={IC.H}
        style={{ display: "block", width: "100%", cursor: "crosshair",
                 borderRadius: 4, border: "1px solid var(--border)" }}
        onMouseDown={(e) => {
          dragging.current = true;
          const { x, y } = getPos(e);
          updateICR(x, y);
        }}
        onMouseMove={(e) => {
          if (!dragging.current) return;
          const { x, y } = getPos(e);
          updateICR(x, y);
        }}
        onMouseUp={() => { dragging.current = false; }}
        onMouseLeave={() => { dragging.current = false; }}
        onDoubleClick={() => {
          icrRef.current = null;
          onAnglesChange(Object.fromEntries(WHEEL_KEYS.map((w) => [w, 0])) as AngleMap);
        }}
      />
      <div style={{ fontSize: 10, color: "var(--muted)", marginTop: 3 }}>
        拖动黄点设置 ICR 位置（转向中心）· 双击复位直行
      </div>
    </div>
  );
}

// ─── Sub-component: AngleReadout ──────────────────────────────────────────────

function AngleReadout({ angles, R }: { angles: AngleMap; R: number | null }) {
  return (
    <div style={{ marginTop: 8, borderTop: "1px solid var(--border)", paddingTop: 6 }}>
      {WHEEL_KEYS.map((w) => {
        const deg = angles[w];
        const pct = Math.abs(deg) / VIEW_LIMIT * 50;
        const off = deg >= 0 ? 50 : 50 - pct;
        return (
          <div key={w} style={{ display: "flex", alignItems: "center", gap: 4, marginBottom: 2 }}>
            <span style={{ fontSize: 10, color: "var(--muted)", width: 26 }}>{w.toUpperCase()}</span>
            <div style={{ flex: 1, height: 7, background: "var(--bg)", borderRadius: 2,
                          overflow: "hidden", position: "relative" }}>
              <div style={{
                position: "absolute", left: `${off}%`, width: `${pct}%`,
                height: "100%", borderRadius: 2,
                background: deg >= 0 ? "#22d3ee" : "#f87171",
              }} />
              <div style={{ position: "absolute", left: "50%", top: 0,
                            width: 1, height: "100%", background: "var(--border)" }} />
            </div>
            <span style={{ fontFamily: "monospace", fontSize: 11, width: 46, textAlign: "right" }}>
              {deg >= 0 ? "+" : ""}{deg.toFixed(1)}°
            </span>
          </div>
        );
      })}
      <div style={{ fontFamily: "monospace", fontSize: 10, color: "#facc15",
                    marginTop: 3, textAlign: "right" }}>
        {R === null ? "R = ∞ (直行)" : `R ≈ ${R.toFixed(1)} m`}
      </div>
    </div>
  );
}

// ─── Sub-component: KvxScheduleEditor (method ① k(vx) curve) ──────────────────

const KV_H = 130;

function renderKvx(cv: HTMLCanvasElement, pts: KPt[], curKmh: number, dragIdx: number) {
  const ctx = cv.getContext("2d"); if (!ctx) return;
  const W = cv.width, H = cv.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = "#0a0e14"; ctx.fillRect(0, 0, W, H);
  const toPx = (v: number) => (v / KV_VMAX) * W;
  const toPy = (k: number) => ((1 - k / KV_KMAX) / 2) * H;

  // Grid: vertical speed lines every 40 km/h
  ctx.strokeStyle = "#1c2330"; ctx.lineWidth = 1;
  for (let v = 0; v <= KV_VMAX; v += 40) {
    ctx.beginPath(); ctx.moveTo(toPx(v), 0); ctx.lineTo(toPx(v), H); ctx.stroke();
  }
  // Zero-ratio line (phase boundary)
  ctx.strokeStyle = "#475569"; ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.moveTo(0, toPy(0)); ctx.lineTo(W, toPy(0)); ctx.stroke();
  ctx.setLineDash([]);

  // Phase labels
  ctx.font = "9px sans-serif"; ctx.textAlign = "left";
  ctx.fillStyle = "#22d3ee"; ctx.fillText("同相 +", 4, 11);
  ctx.fillStyle = "#f59e0b"; ctx.fillText("反相 −", 4, H - 5);

  // Current-speed marker
  if (curKmh > 0.5) {
    const cx = toPx(Math.min(curKmh, KV_VMAX));
    ctx.strokeStyle = "#39d353"; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, H); ctx.stroke();
  }

  // Curve
  ctx.strokeStyle = "#22d3ee"; ctx.lineWidth = 2; ctx.beginPath();
  pts.forEach((p, i) => {
    const x = toPx(p.v), y = toPy(p.k);
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.stroke();

  // Control points
  pts.forEach((p, i) => {
    ctx.fillStyle = i === dragIdx ? "#fef08a" : "#22d3ee";
    ctx.beginPath(); ctx.arc(toPx(p.v), toPy(p.k), 4, 0, Math.PI * 2); ctx.fill();
  });

  // Axis labels
  ctx.fillStyle = "#8b949e"; ctx.font = "9px monospace"; ctx.textAlign = "right";
  ctx.fillText(`${KV_VMAX} km/h`, W - 3, H - 4);
}

function KvxScheduleEditor(
  { pts, onChange, onReset, currentKmh }:
  { pts: KPt[]; onChange: (p: KPt[]) => void; onReset: () => void; currentKmh: number },
) {
  const cvRef = useRef<HTMLCanvasElement>(null);
  const ptsRef = useRef(pts); ptsRef.current = pts;
  const curRef = useRef(currentKmh); curRef.current = currentKmh;
  const dragRef = useRef<{ idx: number } | null>(null);

  useEffect(() => {
    const cv = cvRef.current; if (!cv) return;
    cv.width = cv.offsetWidth || 280; cv.height = KV_H;
    renderKvx(cv, pts, currentKmh, dragRef.current?.idx ?? -1);
  }, [pts, currentKmh]);

  useEffect(() => {
    const cv = cvRef.current; if (!cv) return;
    const ro = new ResizeObserver(() => {
      cv.width = cv.offsetWidth || 280; cv.height = KV_H;
      renderKvx(cv, ptsRef.current, curRef.current, dragRef.current?.idx ?? -1);
    });
    ro.observe(cv);
    return () => ro.disconnect();
  }, []);

  const toPx = (v: number, W: number) => (v / KV_VMAX) * W;
  const toPy = (k: number, H: number) => ((1 - k / KV_KMAX) / 2) * H;
  const fromP = (px: number, py: number, W: number, H: number): KPt => ({
    v: clamp((px / W) * KV_VMAX, 0, KV_VMAX),
    k: clamp((1 - (py / H) * 2) * KV_KMAX, -KV_KMAX, KV_KMAX),
  });

  const pick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const cv = cvRef.current!; const r = cv.getBoundingClientRect();
    const px = e.clientX - r.left, py = e.clientY - r.top;
    const W = cv.offsetWidth, H = KV_H;
    let best = -1, minD = 14;
    ptsRef.current.forEach((p, i) => {
      const d = Math.hypot(px - toPx(p.v, W), py - toPy(p.k, H));
      if (d < minD) { minD = d; best = i; }
    });
    return { px, py, W, H, best };
  };

  const onMD = (e: React.MouseEvent<HTMLCanvasElement>) => {
    e.preventDefault();
    const { px, py, W, H, best } = pick(e);
    if (best >= 0) { dragRef.current = { idx: best }; return; }
    const np = fromP(px, py, W, H);
    const newPts = [...ptsRef.current, np].sort((a, b) => a.v - b.v);
    dragRef.current = { idx: newPts.indexOf(np) };
    onChange(newPts);
  };
  const onMM = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!dragRef.current) return;
    const cv = cvRef.current!; const r = cv.getBoundingClientRect();
    const { idx } = dragRef.current;
    const cur = [...ptsRef.current];
    const pos = fromP(e.clientX - r.left, e.clientY - r.top, cv.offsetWidth, KV_H);
    const minV = idx > 0 ? cur[idx - 1].v + 1 : 0;
    const maxV = idx < cur.length - 1 ? cur[idx + 1].v - 1 : KV_VMAX;
    cur[idx] = { v: clamp(pos.v, minV, maxV), k: pos.k };
    onChange(cur);
  };
  const onMU = () => { dragRef.current = null; };
  const onCM = (e: React.MouseEvent<HTMLCanvasElement>) => {
    e.preventDefault();
    if (ptsRef.current.length <= 2) return;
    const { best } = pick(e);
    if (best >= 0) onChange(ptsRef.current.filter((_, i) => i !== best));
  };

  return (
    <div>
      <div style={{ display: "flex", alignItems: "center", marginBottom: 3 }}>
        <span style={{ fontSize: 10, color: "var(--muted)" }}>
          后轮/前轮比 k 随车速 vx 的调度曲线
        </span>
        <button style={{ fontSize: 10, marginLeft: "auto", padding: "1px 7px" }}
          onClick={onReset}>恢复默认</button>
      </div>
      <canvas ref={cvRef}
        style={{ display: "block", width: "100%", height: KV_H, cursor: "crosshair",
                 borderRadius: 3, border: "1px solid var(--border)" }}
        onMouseDown={onMD} onMouseMove={onMM} onMouseUp={onMU}
        onMouseLeave={onMU} onContextMenu={onCM} />
      <div style={{ fontSize: 10, color: "var(--muted)", marginTop: 4 }}>
        点空白加点 · 拖动调整 · 右键删除 · 绿线=当前车速 · k&lt;0 反相(低速灵活)，k&gt;0 同相(高速稳定)
        <br />需选「{`后轮·车速调度`}」策略才生效(右上「切到车速调度」)。
      </div>
    </div>
  );
}

// ─── Main component ───────────────────────────────────────────────────────────

export default function StrategyDesignerPanel() {
  const [mode, setMode] = useState<"curve" | "icr" | "schedule">("curve");
  const [curves, setCurves] = useState<CurveMap>(loadCurves);
  const [preset, setPreset] = useState("ackermann");
  const [localSteer, setLocalSteer] = useState(0);
  const [icrAngles, setIcrAngles] = useState<AngleMap>(
    () => Object.fromEntries(WHEEL_KEYS.map((w) => [w, 0])) as AngleMap
  );
  const [kCurve, setKCurve] = useState<KPt[]>(loadKCurve);

  const activeStrategy = useSimStore((s) => s.state?.strategy);
  const isActive = activeStrategy === "user_js";
  const scheduleActive = activeStrategy === "rear_wheel_steer";
  const simSteer = useSimStore((s) => s.state?.driver?.steering ?? 0);
  const simVxKmh = useSimStore((s) => (s.state?.velocity?.vx ?? 0) * 3.6);
  const steer = isActive ? simSteer : localSteer;

  const angles: AngleMap = mode === "curve"
    ? anglesFromCurves(curves, steer)
    : icrAngles;
  const R = turningRadius(angles);

  // Persist curves
  useEffect(() => {
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(curves)); } catch {}
  }, [curves]);

  // Persist + push the k(vx) schedule to the rws_speed_schedule backend strategy.
  useEffect(() => {
    try { localStorage.setItem(KCURVE_KEY, JSON.stringify(kCurve)); } catch {}
    if (scheduleActive) {
      const sorted = [...kCurve].sort((a, b) => a.v - b.v);
      setDriverMode({ rws_mode: "speed_schedule", k_curve: sorted.map((p) => [p.v, p.k]) });
    }
  }, [kCurve, scheduleActive]);

  // Send steer_cmd — use refs to avoid stale closure in the subscribe callback
  const curvesRef   = useRef(curves);   curvesRef.current   = curves;
  const icrRef      = useRef(icrAngles); icrRef.current      = icrAngles;
  const isActiveRef = useRef(isActive); isActiveRef.current = isActive;
  const modeRef     = useRef(mode);     modeRef.current     = mode;

  useEffect(() => {
    return useSimStore.subscribe((s) => {
      if (!isActiveRef.current || !s.state) return;
      const sv = s.state.driver?.steering ?? 0;
      const a = modeRef.current === "curve"
        ? anglesFromCurves(curvesRef.current, sv)
        : icrRef.current;
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      sendMessage({ type: "steer_cmd" as any,
        fl: a.fl * DEG, fr: a.fr * DEG, rl: a.rl * DEG, rr: a.rr * DEG } as any);
    });
  }, []); // mount once; refs stay current

  const setCurveWheel = useCallback((w: WheelKey, pts: Pt[]) =>
    setCurves((prev) => ({ ...prev, [w]: pts })), []);

  const markCustom = useCallback(() => setPreset("custom"), []);

  const applyPreset = (key: string) => {
    if (key in PRESETS) { setCurves(PRESETS[key]()); setPreset(key); }
  };

  return (
    <Panel
      title="策略设计器"
      help={HELP.designer}
      badge={mode === "schedule" ? (
        scheduleActive ? (
          <span style={{ fontSize: 10, color: "#22d3ee", marginLeft: "auto" }}>▶ 激活</span>
        ) : (
          <button
            style={{ fontSize: 10, marginLeft: "auto", padding: "2px 8px" }}
            // eslint-disable-next-line @typescript-eslint/no-explicit-any
            onClick={() => sendMessage({ type: "strategy", name: "rear_wheel_steer" } as any)}
          >
            切到车速调度
          </button>
        )
      ) : isActive ? (
        <span style={{ fontSize: 10, color: "#22d3ee", marginLeft: "auto" }}>▶ 激活</span>
      ) : (
        <button
          style={{ fontSize: 10, marginLeft: "auto", padding: "2px 8px" }}
          // eslint-disable-next-line @typescript-eslint/no-explicit-any
          onClick={() => sendMessage({ type: "strategy", name: "user_js" } as any)}
        >
          切换到 user_js
        </button>
      )}
    >

      {/* Mode tabs */}
      <div style={{ display: "flex", gap: 4, marginBottom: 8 }}>
        {(["curve", "icr", "schedule"] as const).map((m) => (
          <button key={m}
            style={{
              fontSize: 11, padding: "2px 10px",
              background: mode === m ? "rgba(34,211,238,0.12)" : "var(--surface)",
              border: `1px solid ${mode === m ? "#22d3ee" : "var(--border)"}`,
              color: mode === m ? "#22d3ee" : "var(--muted)",
              borderRadius: 4, cursor: "pointer",
            }}
            onClick={() => setMode(m)}
          >
            {m === "curve" ? "曲线映射" : m === "icr" ? "ICR 拖拽" : "k(vx) 调度"}
          </button>
        ))}
      </div>

      {mode === "curve" && (
        <>
          {/* Preset row */}
          <div style={{ display: "flex", gap: 3, flexWrap: "wrap", marginBottom: 6 }}>
            {Object.keys(PRESET_LABELS).map((key) => (
              <button key={key}
                style={{
                  fontSize: 10, padding: "1px 7px",
                  background: preset === key ? "rgba(34,211,238,0.12)" : "var(--surface)",
                  border: `1px solid ${preset === key ? "rgba(34,211,238,0.5)" : "var(--border)"}`,
                  color: preset === key ? "#22d3ee" : "var(--muted)",
                  borderRadius: 3, cursor: "pointer",
                }}
                onClick={() => applyPreset(key)}
              >
                {PRESET_LABELS[key]}
              </button>
            ))}
          </div>

          {/* 2×2 curve grid */}
          <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 6 }}>
            {WHEEL_KEYS.map((w) => (
              <WheelCurveEditor key={w} wheel={w} pts={curves[w]}
                steer={steer} onChange={(pts) => setCurveWheel(w, pts)}
                onCustomize={markCustom} />
            ))}
          </div>

          {/* Steering slider (only when not live) */}
          {!isActive && (
            <div style={{ display: "flex", alignItems: "center", gap: 6, marginTop: 7 }}>
              <span style={{ fontSize: 10, color: "var(--muted)", width: 56, flexShrink: 0 }}>转向预览</span>
              <input type="range" min={-100} max={100}
                value={Math.round(localSteer * 100)} step={1}
                style={{ flex: 1, accentColor: "#22d3ee" }}
                onChange={(e) => setLocalSteer(Number(e.target.value) / 100)} />
              <span style={{ fontFamily: "monospace", fontSize: 11, width: 44,
                             textAlign: "right", color: "#22d3ee" }}>
                {(localSteer >= 0 ? "+" : "") + localSteer.toFixed(2)}
              </span>
            </div>
          )}

          <div style={{ fontSize: 10, color: "var(--muted)", marginTop: 5 }}>
            点击添加控制点 · 拖动调整 · 右键删除 · 范围 ±{VIEW_LIMIT}°
          </div>
        </>
      )}

      {mode === "icr" && (
        <ICRDragCanvas angles={icrAngles} onAnglesChange={setIcrAngles} />
      )}

      {mode === "schedule" && (
        <KvxScheduleEditor pts={kCurve} onChange={setKCurve}
          onReset={() => setKCurve(DEFAULT_KCURVE.map((p) => ({ ...p })))}
          currentKmh={simVxKmh} />
      )}

      {mode !== "schedule" && <AngleReadout angles={angles} R={R} />}
    </Panel>
  );
}
