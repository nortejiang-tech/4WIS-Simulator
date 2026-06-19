/**
 * IcrLayer — steering-geometry overlay:
 *   - each wheel's perpendicular dashed line (locus of its possible ICR)
 *   - per-wheel steering centre: vehicle ICR projected onto that line
 *     (diamond, wheel-coloured) + a short segment showing the deviation
 *     (turns red when the wheel disagrees with the body motion by > 0.3 m)
 *   - vehicle ICR markers (red dot = actual, blue dashed ring = target)
 */

import { Circle, Line, Rect } from "react-konva";

import type { SimStateMessage } from "@/types/sim";
import { WHEEL_COLORS } from "./colors";
import type { Camera2D } from "./projection";
import { bodyToWorld, worldToScreen } from "./projection";

const PERP_LEN = 60;        // [m] half-length of the perpendicular lines
const DEV_WARN = 0.3;       // [m] deviation that turns the marker red

export default function IcrLayer({ state, cam }: { state: SimStateMessage; cam: Camera2D }) {
  const pose = state.pose;

  const actual = toScreen(state.icr_vehicle_body);
  const target = toScreen(state.icr_target_body);

  function toScreen(p: [number | null, number | null]): [number, number] | null {
    const [x, y] = p;
    if (x == null || y == null) return null;
    const w = bodyToWorld(pose, x, y);
    return worldToScreen(cam, w.x, w.y);
  }

  return (
    <>
      {/* Wheel perpendicular lines */}
      {state.wheels.map((w, i) => {
        const wp = bodyToWorld(pose, w.pos_body[0], w.pos_body[1]);
        const ang = pose.psi + w.delta;
        const px = -Math.sin(ang), py = Math.cos(ang);
        const [x1, y1] = worldToScreen(cam, wp.x - px * PERP_LEN, wp.y - py * PERP_LEN);
        const [x2, y2] = worldToScreen(cam, wp.x + px * PERP_LEN, wp.y + py * PERP_LEN);
        return (
          <Line
            key={`perp-${i}`}
            points={[x1, y1, x2, y2]}
            stroke="#fbbf24" strokeWidth={1} dash={[7, 5]} opacity={0.45}
            listening={false}
          />
        );
      })}

      {/* Per-wheel steering centres + deviation segments */}
      {state.wheels.map((w, i) => {
        const q = w.icr_body;
        if (!q || q[0] == null || q[1] == null) return null;
        const qs = toScreen(q as [number, number]);
        if (!qs) return null;
        if (qs[0] < -50 || qs[0] > cam.W + 50 || qs[1] < -50 || qs[1] > cam.H + 50) return null;
        const dev = Math.abs(w.icr_dev ?? 0);
        const warn = dev > DEV_WARN;
        return (
          <Rect
            key={`wicr-${i}`}
            x={qs[0]} y={qs[1]}
            width={7} height={7}
            offsetX={3.5} offsetY={3.5}
            rotation={45}
            fill={warn ? "#ef4444" : WHEEL_COLORS[i]}
            stroke="#0f172a" strokeWidth={1}
            listening={false}
          />
        );
      })}
      {/* Deviation segments: vehicle ICR → each wheel's projected centre */}
      {actual && state.wheels.map((w, i) => {
        const q = w.icr_body;
        if (!q || q[0] == null || q[1] == null) return null;
        const qs = toScreen(q as [number, number]);
        if (!qs) return null;
        const dev = Math.abs(w.icr_dev ?? 0);
        if (dev < 1e-3) return null;   // coincident — skip the segment
        const warn = dev > DEV_WARN;
        return (
          <Line
            key={`wdev-${i}`}
            points={[actual[0], actual[1], qs[0], qs[1]]}
            stroke={warn ? "#ef4444" : WHEEL_COLORS[i]}
            strokeWidth={1.5}
            opacity={0.8}
            listening={false}
          />
        );
      })}

      {/* Strategy-target ICR (blue dashed ring) */}
      {target && (
        <Circle
          x={target[0]} y={target[1]} radius={8}
          stroke="#60a5fa" strokeWidth={2} dash={[4, 3]}
          listening={false}
        />
      )}
      {/* Actual ICR (red filled dot) */}
      {actual && (
        <Circle
          x={actual[0]} y={actual[1]} radius={5}
          fill="#ef4444" stroke="#fef2f2" strokeWidth={1.5}
          listening={false}
        />
      )}
    </>
  );
}
