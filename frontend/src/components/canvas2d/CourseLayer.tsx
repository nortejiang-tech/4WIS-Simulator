/**
 * CourseLayer — the maneuver's ground furniture in the 2D view: painted lines
 * (lane edges, ISO test-section boxes, start/finish) and the cones/poles.
 *
 * Top-down, a cone and a slalom pole should not look alike — you need to know
 * which markers you may straddle and which you must go around. Cones are drawn
 * as filled discs at their real base radius, poles as a small ringed dot with a
 * white core, both scaled by the camera so they stay to scale when zooming.
 *
 * Everything is `listening={false}`; the layer is memoised on the plan object,
 * so panning only re-projects.
 */

import { memo } from "react";
import { Circle, Group, Line } from "react-konva";

import type { PathPlan } from "@/types/sim";
import type { Camera2D } from "./projection";
import { worldToScreen } from "./projection";

function CourseLayer({ path, cam }: { path: PathPlan; cam: Camera2D }) {
  const S = (x: number, y: number) => worldToScreen(cam, x, y);
  const flat = (pts: [number, number][]) => {
    const out: number[] = [];
    for (const [x, y] of pts) { const [sx, sy] = S(x, y); out.push(sx, sy); }
    return out;
  };
  const dash = [cam.pxm * 1.6, cam.pxm * 1.6];

  return (
    <Group listening={false}>
      {(path.marks ?? []).map((m, i) => (
        <Line
          key={`mk${i}`}
          points={flat(m.points)}
          stroke={m.color}
          strokeWidth={Math.max(1, m.width * cam.pxm)}
          dash={m.dash ? dash : undefined}
          opacity={0.9}
          lineCap="butt"
          lineJoin="round"
          listening={false}
        />
      ))}
      {(path.cones ?? []).map((c, i) => {
        const [sx, sy] = S(c.x, c.y);
        if (c.kind === "pole") {
          const r = Math.max(2.5, 0.16 * cam.pxm);
          return (
            <Group key={`c${i}`} listening={false}>
              <Circle x={sx} y={sy} r={r} fill={c.color} />
              <Circle x={sx} y={sy} r={r * 0.42} fill="#f8fafc" />
            </Group>
          );
        }
        // Base radius of a 75 cm cone is ~0.17 m; scale with the cone's height.
        const r = Math.max(2.5, 0.17 * (c.height / 0.75) * cam.pxm);
        return (
          <Circle
            key={`c${i}`}
            x={sx} y={sy} r={r}
            fill={c.color}
            stroke="#f8fafc"
            strokeWidth={Math.min(2, r * 0.35)}
            listening={false}
          />
        );
      })}
    </Group>
  );
}

export default memo(CourseLayer);
