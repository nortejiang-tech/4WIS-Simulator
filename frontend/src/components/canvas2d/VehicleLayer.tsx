/**
 * VehicleLayer — the top-down vehicle rendering for Canvas2D.
 *
 * Split out of Canvas2D so the scene composition there stays readable, and so
 * the silhouette data (vehicleShape.ts) is shared with the 3D viewport.
 *
 * Local frame inside the returned <Group>: the parent applies
 * `rotation={-psiDeg + ROT}`, which leaves us with
 *     local +X = body forward,  local −Y = body left
 * so body-frame (bx, by) maps to local pixels (bx·pxm, −by·pxm), exactly as
 * the previous inline implementation did.
 *
 * Wheel tint follows effective μ (canvas2d/colors.ts::wheelMuColor) — the same
 * signal the 3D view and the HUD use. Slip angle drives an extra warning ring
 * only, so kinematic runs (α ≡ 0) look identical to before.
 */

import { Group, Line, Circle, Rect, Ellipse, Arc } from "react-konva";

import { wheelMuColor } from "./colors";
import {
  PALETTE,
  WHEEL,
  bodyDimensions,
  bodyOutline,
  cabinOutline,
  lampPositions,
  windshieldQuad,
} from "../vehicleShape";
import type { SimStateMessage, WheelState } from "@/types/sim";

/** Body-frame metres → flat local pixel array for <Line points>. */
function toPx(points: [number, number][], pxm: number): number[] {
  const out: number[] = [];
  for (const [bx, by] of points) out.push(bx * pxm, -by * pxm);
  return out;
}

// ---------------------------------------------------------------------------
// Wheel
// ---------------------------------------------------------------------------

interface WheelProps {
  w: WheelState;
  tireRadius: number;
  pxm: number;
  spin: number;
  pal: typeof PALETTE["dark"];
}

function Wheel({ w, tireRadius, pxm, spin, pal }: WheelProps) {
  const cx = w.pos_body[0] * pxm;
  const cy = -w.pos_body[1] * pxm;
  const len = 2 * tireRadius * pxm;
  const wid = 2 * tireRadius * WHEEL.widthRatio * pxm;
  const rimR = (tireRadius * WHEEL.rimRatio) * pxm;
  const hubR = (tireRadius * WHEEL.hubRatio) * pxm;
  const deltaDeg = (w.delta * 180) / Math.PI;

  const tyre = wheelMuColor(w.mu);
  // Slip warning: ring brightens past ~4°, saturates near ~9°.
  const alpha = Math.abs(w.slip_alpha ?? 0);
  const slipT = Math.min(Math.max((alpha - 0.07) / 0.09, 0), 1);

  // Below ~10 px the detail is illegible — draw a simplified tyre instead.
  const detailed = len > 11;

  return (
    <Group x={cx} y={cy} rotation={-deltaDeg}>
      {/* Contact shadow */}
      <Rect
        x={-len / 2 + 1.5} y={-wid / 2 + 1.5}
        width={len} height={wid}
        fill={pal.shadow}
        cornerRadius={wid * 0.32}
      />
      {/* Tyre carcass — shaded across its width so it reads as round */}
      <Rect
        x={-len / 2} y={-wid / 2}
        width={len} height={wid}
        fillLinearGradientStartPoint={{ x: 0, y: -wid / 2 }}
        fillLinearGradientEndPoint={{ x: 0, y: wid / 2 }}
        fillLinearGradientColorStops={[0, "#0b1220", 0.5, tyre, 1, "#0b1220"]}
        stroke={pal.tyreEdge}
        strokeWidth={0.9}
        cornerRadius={wid * 0.32}
      />
      {detailed && (
        <>
          {/* Rim seen edge-on from above */}
          <Ellipse
            radiusX={rimR} radiusY={wid * 0.30}
            fill="rgba(148,163,184,0.16)"
            stroke={pal.rim}
            strokeWidth={0.7}
          />
          {/* Spokes rotate with wheel spin, so rolling is legible at a glance */}
          {Array.from({ length: WHEEL.spokes }).map((_, i) => {
            const a = (i * (2 * Math.PI)) / WHEEL.spokes + spin;
            return (
              <Line
                key={i}
                points={[0, 0, Math.cos(a) * rimR * 0.82, Math.sin(a) * wid * 0.23]}
                stroke={pal.rim}
                strokeWidth={0.6}
                opacity={0.7}
              />
            );
          })}
          <Circle radius={hubR} fill={pal.hub} />
        </>
      )}
      {/* Rolling-direction tick (kept from the original rendering) */}
      <Line
        points={[len / 2, 0, len / 2 + Math.max(5, len * 0.16), 0]}
        stroke={pal.tyreEdge}
        strokeWidth={1.4}
        opacity={0.75}
      />
      {/* Slip warning halo — invisible in kinematic mode (α ≡ 0) */}
      {slipT > 0.01 && (
        <Ellipse
          radiusX={len * 0.62}
          radiusY={wid * 0.95}
          stroke="#f87171"
          strokeWidth={1.2 + slipT}
          opacity={0.25 + 0.5 * slipT}
          dash={[4, 3]}
        />
      )}
    </Group>
  );
}

// ---------------------------------------------------------------------------
// Body shell
// ---------------------------------------------------------------------------

function Body({
  length, width, pxm, pal, detailed,
}: { length: number; width: number; pxm: number; pal: typeof PALETTE["dark"]; detailed: boolean }) {
  const outline = toPx(bodyOutline(length, width), pxm);
  const halfW = (width * pxm) / 2;
  const halfL = (length * pxm) / 2;

  if (!detailed) {
    // Zoomed far out: a clean silhouette reads better than micro-detail.
    return (
      <Line
        points={outline}
        closed
        tension={0.12}
        fill={pal.shellMid}
        stroke={pal.edge}
        strokeWidth={1.4}
      />
    );
  }

  const cabin = toPx(cabinOutline(length, width), pxm);
  const glass = toPx(windshieldQuad(length, width), pxm);
  const lamps = lampPositions(length, width);
  const lampR = Math.max(1.5, width * pxm * 0.05);

  return (
    <>
      {/* Drop shadow */}
      <Line
        points={outline.map((v, i) => (i % 2 === 0 ? v + 2.5 : v + 3))}
        closed tension={0.12}
        fill={pal.shadow}
      />
      {/* Painted shell — cross-body gradient suggests a rounded, lit surface */}
      <Line
        points={outline}
        closed tension={0.12}
        fillLinearGradientStartPoint={{ x: 0, y: -halfW }}
        fillLinearGradientEndPoint={{ x: 0, y: halfW }}
        fillLinearGradientColorStops={[0, pal.shellLight, 0.45, pal.shellMid, 1, pal.shellDark]}
        stroke={pal.edge}
        strokeWidth={1.6}
        shadowColor={pal.edge}
        shadowBlur={10}
        shadowOpacity={0.3}
        perfectDrawEnabled={false}
      />
      {/* Greenhouse */}
      <Line
        points={cabin}
        closed tension={0.2}
        fill={pal.cabinFill}
        stroke={pal.cabinEdge}
        strokeWidth={1}
      />
      {/* Windshield highlight */}
      <Line points={glass} closed fill={pal.glass} />
      {/* Centre crease — helps read heading when zoomed out */}
      <Line points={[-halfL * 0.5, 0, halfL * 0.84, 0]} stroke={pal.crease} strokeWidth={1} />
      {/* Head-lamp beam cones — cheap but sells "this end is the front" */}
      {lamps.head.map(([bx, by], i) => (
        <Arc
          key={`hb${i}`}
          x={bx * pxm} y={-by * pxm}
          innerRadius={0} outerRadius={halfL * 0.5}
          angle={26} rotation={-13}
          fill={pal.headlightGlow}
          opacity={0.55}
        />
      ))}
      {lamps.head.map(([bx, by], i) => (
        <Circle
          key={`h${i}`}
          x={bx * pxm} y={-by * pxm}
          radius={lampR}
          fill={pal.headlight}
          shadowColor={pal.headlight} shadowBlur={8} shadowOpacity={0.85}
          perfectDrawEnabled={false}
        />
      ))}
      {lamps.tail.map(([bx, by], i) => (
        <Rect
          key={`t${i}`}
          x={bx * pxm - lampR} y={-by * pxm - lampR * 0.5}
          width={lampR * 2} height={lampR}
          fill={pal.taillight}
          cornerRadius={lampR * 0.35}
          shadowColor={pal.taillight} shadowBlur={6} shadowOpacity={0.75}
          perfectDrawEnabled={false}
        />
      ))}
    </>
  );
}

// ---------------------------------------------------------------------------
// Velocity vector
// ---------------------------------------------------------------------------

function VelocityVector({
  vx, vy, vMax, pxm,
}: { vx: number; vy: number; vMax: number; pxm: number }) {
  const speed = Math.hypot(vx, vy);
  if (speed < 0.25) return null;
  const lenM = Math.min(speed / Math.max(vMax, 1e-3), 1) * 5.5;
  const ang = Math.atan2(vy, vx);
  const ex = Math.cos(ang) * lenM * pxm;
  const ey = -Math.sin(ang) * lenM * pxm;
  const dirA = Math.atan2(-ey, ex);
  const hl = Math.max(5, lenM * pxm * 0.15);

  return (
    <>
      <Line points={[0, 0, ex, ey]} stroke="#4ade80" strokeWidth={2} opacity={0.85} lineCap="round" />
      <Line
        points={[
          ex, ey,
          ex - Math.cos(dirA - 0.4) * hl, ey + Math.sin(dirA - 0.4) * hl,
          ex - Math.cos(dirA + 0.4) * hl, ey + Math.sin(dirA + 0.4) * hl,
        ]}
        closed fill="#4ade80" opacity={0.9}
      />
    </>
  );
}

// ---------------------------------------------------------------------------
// Public layer
// ---------------------------------------------------------------------------

interface VehicleLayerProps {
  state: SimStateMessage;
  /** Screen position of the body origin. */
  x: number;
  y: number;
  /** Konva rotation in degrees (parent already folded in ROT). */
  rotationDeg: number;
  pxm: number;
  /** Accumulated wheel rotation [rad] per wheel, for spoke animation. */
  spin: number[];
  theme: "dark" | "light";
  showVelocity?: boolean;
}

export default function VehicleLayer({
  state, x, y, rotationDeg, pxm, spin, theme, showVelocity = true,
}: VehicleLayerProps) {
  const p = state.params;
  const trackMax = Math.max(p.track_front, p.track_rear);
  const { length, width } = bodyDimensions(p.wheelbase, trackMax);
  const pal = PALETTE[theme];
  const detailed = length * pxm > 46;

  return (
    <Group x={x} y={y} rotation={rotationDeg} listening={false}>
      <Body length={length} width={width} pxm={pxm} pal={pal} detailed={detailed} />
      {state.wheels.map((w, i) => (
        <Wheel key={i} w={w} tireRadius={p.tire_radius} pxm={pxm} spin={spin[i] ?? 0} pal={pal} />
      ))}
      {showVelocity && (
        <VelocityVector
          vx={state.velocity.vx}
          vy={state.velocity.vy}
          vMax={p.v_max}
          pxm={pxm}
        />
      )}
    </Group>
  );
}
