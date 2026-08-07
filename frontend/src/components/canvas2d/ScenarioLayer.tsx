/**
 * ScenarioLayer — draws the static driving scenario (roads, markings, traffic
 * lights, track edges) under the vehicle in the 2D view.
 *
 * Everything is `listening={false}` (no hit-testing) and surfaces are single
 * filled Konva polylines, so even the larger tracks stay cheap to redraw as the
 * follow-camera pans each frame.
 */

import { memo } from "react";
import { Line, Rect, Circle, Group } from "react-konva";

import type { Scenario } from "@/types/sim";
import type { Camera2D } from "./projection";
import { worldToScreen } from "./projection";

const TL_COLOR: Record<string, string> = { red: "#ef4444", yellow: "#facc15", green: "#22c55e" };

function ScenarioLayer({ scenario, cam }: { scenario: Scenario; cam: Camera2D }) {
  const S = (x: number, y: number) => worldToScreen(cam, x, y);
  const flat = (pts: [number, number][]) => {
    const out: number[] = [];
    for (const [x, y] of pts) { const [sx, sy] = S(x, y); out.push(sx, sy); }
    return out;
  };
  const dash = [cam.pxm * 2, cam.pxm * 2];
  // Resolve a line's dash array: a metre-rhythm `dash_pattern` wins over the
  // legacy boolean `dash`; both are scaled to pixels by cam.pxm.
  const dashFor = (ln: Scenario["lines"][number]): number[] | undefined => {
    if (ln.dash_pattern) return [ln.dash_pattern[0] * cam.pxm, ln.dash_pattern[1] * cam.pxm];
    return ln.dash ? dash : undefined;
  };

  return (
    <Group listening={false}>
      {scenario.surfaces.map((s, i) => {
        const outlined = s.kind === "building" || s.kind === "landmark";
        return (
          <Line
            key={`s${i}`}
            points={flat(s.points)}
            closed
            fill={s.color}
            stroke={outlined ? "#1c1f25" : undefined}
            strokeWidth={outlined ? 1 : 0}
            listening={false}
          />
        );
      })}
      {scenario.lines.map((ln, i) => (
        <Line
          key={`l${i}`}
          points={flat(ln.points)}
          stroke={ln.color}
          strokeWidth={Math.max(1, ln.width * cam.pxm)}
          dash={dashFor(ln)}
          lineCap="butt"
          lineJoin="round"
          listening={false}
        />
      ))}
      {scenario.markers.map((m, i) => {
        const [sx, sy] = S(m.x, m.y);
        if (m.type === "traffic_light") {
          const state = (m.meta?.state as string) || "red";
          const r = Math.max(3, 0.6 * cam.pxm);
          return (
            <Group key={`m${i}`} listening={false}>
              <Rect x={sx - r * 0.9} y={sy - r * 2.6} width={r * 1.8} height={r * 5.2}
                cornerRadius={r * 0.5} fill="#15181d" stroke="#0a0d14" strokeWidth={1} />
              <Circle x={sx} y={sy} r={r} fill={TL_COLOR[state] ?? "#ef4444"}
                shadowColor={TL_COLOR[state] ?? "#ef4444"} shadowBlur={r} />
            </Group>
          );
        }
        if (m.type === "distance") {
          // Distance post: a small tick mark + the metre value.
          const meters = typeof m.meta?.m === "number" ? String(m.meta.m) : "";
          const h = Math.max(8, 1.2 * cam.pxm);
          return (
            <Group key={`m${i}`} listening={false}>
              <Rect x={sx - 0.6} y={sy - h} width={1.6} height={h} fill="#e8e8e8" />
              {meters && (
                <Circle x={sx + 0.2} y={sy - h} r={Math.max(7, 2.2 * cam.pxm)}
                  fill="#1c1f25" stroke="#e8e8e8" strokeWidth={1} />
              )}
            </Group>
          );
        }
        return null;
      })}
    </Group>
  );
}

export default memo(ScenarioLayer);
