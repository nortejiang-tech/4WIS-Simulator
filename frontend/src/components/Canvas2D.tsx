/**
 * Canvas2D — top-down 2D viewport (container + layer composition).
 *
 * World convention (matches docs/design.md §2):
 *   X-forward (east), Y-left (north), Z-up. Yaw ψ CCW about Z.
 * Projection lives in canvas2d/projection.ts; the heavier sub-layers
 * (disturbances, steering geometry, HUD) are split into canvas2d/.
 *
 * Canvas click routing:
 *   - waypoint edit mode  → add draft waypoint
 *   - disturbance place mode → POST a new disturbance at the clicked point
 *   - otherwise, clicks on disturbance shapes select them for editing
 */

import { useEffect, useMemo, useRef, useState } from "react";
import { Stage, Layer, Rect, Line, Circle, Text } from "react-konva";

import { useSimStore } from "@/store/sim";
import { createDisturbance, updateDisturbance } from "@/api/scene";
import CourseLayer from "./canvas2d/CourseLayer";
import DisturbanceLayer from "./canvas2d/DisturbanceLayer";
import ScenarioLayer from "./canvas2d/ScenarioLayer";
import IcrLayer from "./canvas2d/IcrLayer";
import VehicleLayer from "./canvas2d/VehicleLayer";
import Hud from "./canvas2d/Hud";
import type { Camera2D } from "./canvas2d/projection";
import { ROT, screenToWorld, worldToScreen } from "./canvas2d/projection";

const TRAIL_OPACITY = 0.75;

// ---------- Grid ----------

function Grid({ cam, cell, section }: { cam: Camera2D; cell: string; section: string }) {
  const { W, H, camX, camY, pxm } = cam;
  const S = (wx: number, wy: number) => worldToScreen(cam, wx, wy);
  const halfX = H / 2 / pxm, halfY = W / 2 / pxm;
  const minX = Math.floor(camX - halfX), maxX = Math.ceil(camX + halfX);
  const minY = Math.floor(camY - halfY), maxY = Math.ceil(camY + halfY);

  const lines: { points: number[]; stroke: string; w: number }[] = [];
  for (let x = minX; x <= maxX; x++) {
    const heavy = x % 5 === 0;
    const [ax, ay] = S(x, minY), [bx, by] = S(x, maxY);
    lines.push({ points: [ax, ay, bx, by], stroke: heavy ? section : cell, w: heavy ? 1.0 : 0.6 });
  }
  for (let y = minY; y <= maxY; y++) {
    const heavy = y % 5 === 0;
    const [ax, ay] = S(minX, y), [bx, by] = S(maxX, y);
    lines.push({ points: [ax, ay, bx, by], stroke: heavy ? section : cell, w: heavy ? 1.0 : 0.6 });
  }
  const [ox, oy] = S(0, 0), [xx, xy] = S(1, 0), [yx, yy] = S(0, 1);
  return (
    <>
      {lines.map((l, i) => (
        <Line key={i} points={l.points} stroke={l.stroke} strokeWidth={l.w} listening={false} />
      ))}
      {/* World axes — short red(+X) / green(+Y) tick at world origin */}
      <Line points={[ox, oy, xx, xy]} stroke="#ef4444" strokeWidth={2} listening={false} />
      <Line points={[ox, oy, yx, yy]} stroke="#22c55e" strokeWidth={2} listening={false} />
    </>
  );
}

// ---------- Canvas2D ----------

export default function Canvas2D() {
  const state = useSimStore((s) => s.state);
  const trajectory = useSimStore((s) => s.trajectory);
  const path = useSimStore((s) => s.path);
  const scenario = useSimStore((s) => s.scenario);
  const editMode = useSimStore((s) => s.editMode);
  const draft = useSimStore((s) => s.draftWaypoints);
  const addDraftWaypoint = useSimStore((s) => s.addDraftWaypoint);
  const distPlaceType = useSimStore((s) => s.distPlaceType);
  const measureMode = useSimStore((s) => s.measureMode);
  const measurePts = useSimStore((s) => s.measurePts);
  const addMeasurePt = useSimStore((s) => s.addMeasurePt);
  const selectedDistId = useSimStore((s) => s.selectedDistId);
  const setSelectedDistId = useSimStore((s) => s.setSelectedDistId);
  const setDistPlaceType = useSimStore((s) => s.setDistPlaceType);
  const pushToast = useSimStore((s) => s.pushToast);
  const savedRuns = useSimStore((s) => s.savedRuns);
  const showOverlay = useSimStore((s) => s.showOverlay);
  const theme = useSimStore((s) => s.theme);
  const pxm = useSimStore((s) => s.view2dPxm);
  const containerRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ w: 800, h: 600 });

  // Accumulated wheel rotation, so the 2D wheels' spokes visibly spin.
  // Mirrors the same integration Canvas3D does in its useFrame loop.
  const spinRef = useRef<number[]>([0, 0, 0, 0]);
  const lastSpinT = useRef(0);
  const simT = state?.t ?? 0;
  useEffect(() => {
    const st = useSimStore.getState().state;
    if (!st) return;
    const dt = st.t - lastSpinT.current;
    lastSpinT.current = st.t;
    if (dt > 0 && dt < 0.5) {
      const s = spinRef.current;
      for (let i = 0; i < 4; i++) {
        s[i] = (s[i] + (st.wheels[i]?.omega ?? 0) * dt) % (Math.PI * 2);
      }
    }
  }, [simT]);

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const update = () => {
      const r = el.getBoundingClientRect();
      setSize({ w: r.width, h: r.height });
    };
    update();
    const ro = new ResizeObserver(update);
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  // Esc exits disturbance-place mode.
  useEffect(() => {
    if (!distPlaceType) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") setDistPlaceType(null);
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [distPlaceType, setDistPlaceType]);

  const W = size.w;
  const H = size.h;

  // Memoise trajectory transformation
  const trajPoints = useMemo(() => {
    if (!state) return [] as number[];
    const camX = state.pose.x, camY = state.pose.y;
    const out: number[] = new Array(trajectory.length);
    for (let i = 0; i < trajectory.length; i += 2) {
      out[i] = W / 2 - (trajectory[i + 1] - camY) * pxm;
      out[i + 1] = H / 2 - (trajectory[i] - camX) * pxm;
    }
    return out;
  }, [trajectory, state, pxm, W, H]);

  // Never mount the Konva <Stage> at a degenerate size. A shadowed/gradient shape
  // (VehicleLayer) drawn while the stage is momentarily 0-sized makes Konva blit a
  // 0×0 buffer canvas → "drawImage ... width or height of 0" throws, the view's
  // ErrorBoundary catches it and rebuilds, and on slower/DPI-scaled machines the
  // crash→rebuild loop reads as severe flicker. The container keeps its ref so the
  // ResizeObserver still measures and re-renders once a real size arrives.
  if (!state || W < 4 || H < 4) {
    return (
      <div ref={containerRef} className="canvas-container">
        <div className="canvas-loading">等待仿真数据…</div>
      </div>
    );
  }

  const cam: Camera2D = { camX: state.pose.x, camY: state.pose.y, pxm, W, H };
  const S = (wx: number, wy: number) => worldToScreen(cam, wx, wy);

  const [vsx, vsy] = S(state.pose.x, state.pose.y);
  const psiDeg = (state.pose.psi * 180) / Math.PI;

  const handleStageClick = (e: { target: { getStage: () => any } }) => {
    const stage = e.target.getStage();
    const p = stage?.getPointerPosition();
    if (!p) return;
    const [wx, wy] = screenToWorld(cam, p.x, p.y);
    if (measureMode) {
      addMeasurePt(wx, wy);
      return;
    }
    if (editMode) {
      addDraftWaypoint(wx, wy);
      return;
    }
    if (distPlaceType) {
      createDisturbance(distPlaceType, { x: wx, y: wy })
        .then((d) => setSelectedDistId(d.id))
        .catch((err) => pushToast("error", `放置扰动失败：${err.message ?? err}`));
      return;
    }
    // Plain click on empty canvas clears the disturbance selection.
    if (selectedDistId) setSelectedDistId(null);
  };

  const handleDistDragEnd = (id: string, wx: number, wy: number) => {
    updateDisturbance(id, { x: wx, y: wy })
      .catch((err) => pushToast("error", `移动扰动失败：${err.message ?? err}`));
  };

  // Path polyline (world → screen)
  const pathPoints: number[] = [];
  if (path) {
    for (const [x, y] of path.points) {
      const [sxv, syv] = S(x, y);
      pathPoints.push(sxv, syv);
    }
  }
  const draftScreen = draft.map(([x, y]) => {
    const [sxv, syv] = S(x, y);
    return { x: sxv, y: syv };
  });
  const draftLine: number[] = [];
  for (const d of draftScreen) draftLine.push(d.x, d.y);

  // A/B comparison overlay trajectories (world → screen)
  const runScreenPoints = (traj: number[] | undefined): number[] => {
    if (!traj) return [];
    const out: number[] = new Array(traj.length);
    for (let i = 0; i < traj.length; i += 2) {
      const [sxv, syv] = S(traj[i], traj[i + 1]);
      out[i] = sxv;
      out[i + 1] = syv;
    }
    return out;
  };

  const runA = showOverlay ? runScreenPoints(savedRuns.A?.trajectory) : [];
  const runB = showOverlay ? runScreenPoints(savedRuns.B?.trajectory) : [];
  const crosshair = editMode || distPlaceType != null || measureMode;

  // Measurement overlay: world points → screen, plus the metric distance.
  const measureScreen = measurePts.map(([x, y]) => S(x, y));
  const measureLine: number[] = [];
  for (const [sx, sy] of measureScreen) measureLine.push(sx, sy);
  let measureDist = 0;
  let measureMid: [number, number] | null = null;
  if (measurePts.length === 2) {
    const dx = measurePts[1][0] - measurePts[0][0];
    const dy = measurePts[1][1] - measurePts[0][1];
    measureDist = Math.hypot(dx, dy);
    measureMid = [
      (measureScreen[0][0] + measureScreen[1][0]) / 2,
      (measureScreen[0][1] + measureScreen[1][1]) / 2,
    ];
  }

  return (
    <div ref={containerRef} className="canvas-container">
      <Stage
        width={W}
        height={H}
        onClick={handleStageClick}
        onTap={handleStageClick}
        style={crosshair ? { cursor: "crosshair" } : undefined}
      >
        <Layer>
          {/* Background */}
          <Rect x={0} y={0} width={W} height={H} fill={theme === "dark" ? "#0a0f1c" : "#f1f5f9"} listening={false} />
          {/* Grid */}
          <Grid
            cam={cam}
            cell={theme === "dark" ? "#152033" : "#dbe3ee"}
            section={theme === "dark" ? "#1e293b" : "#c2cdda"}
          />
          {/* Static scenario (roads / markings / lights) over grid, under everything else */}
          {scenario && <ScenarioLayer scenario={scenario} cam={cam} />}
          {/* Disturbances (under trajectory, over grid) */}
          <DisturbanceLayer
            disturbances={state.scene?.disturbances ?? []}
            cam={cam}
            selectable={!editMode && distPlaceType == null}
            selectedId={selectedDistId}
            onSelect={(id) => setSelectedDistId(id)}
            onDragEnd={handleDistDragEnd}
          />
          {/* Maneuver course: painted marks + cones, under the reference line */}
          {path && <CourseLayer path={path} cam={cam} />}
          {/* Reference path (step 17) */}
          {pathPoints.length >= 4 && (
            <Line
              points={pathPoints}
              stroke="#f472b6"
              strokeWidth={2}
              dash={[10, 6]}
              opacity={0.85}
              listening={false}
              lineCap="round"
              lineJoin="round"
              closed={path?.closed}
            />
          )}
          {/* Waypoint editor draft */}
          {draftLine.length >= 4 && (
            <Line points={draftLine} stroke="#fb7185" strokeWidth={1.5} dash={[4, 4]} listening={false} />
          )}
          {draftScreen.map((d, i) => (
            <Circle key={`wp-${i}`} x={d.x} y={d.y} radius={4} fill="#fb7185" stroke="#fff" strokeWidth={1} listening={false} />
          ))}
          {/* A/B comparison overlays (under live trajectory) */}
          {runA.length >= 4 && (
            <Line points={runA} stroke="#34d399" strokeWidth={2} opacity={0.7} listening={false} lineCap="round" lineJoin="round" />
          )}
          {runB.length >= 4 && (
            <Line points={runB} stroke="#fb923c" strokeWidth={2} opacity={0.7} listening={false} lineCap="round" lineJoin="round" />
          )}
          {/* Trajectory */}
          {trajPoints.length >= 4 && (
            <Line
              points={trajPoints}
              stroke="#22d3ee"
              strokeWidth={1.6}
              opacity={TRAIL_OPACITY}
              listening={false}
              lineCap="round"
              lineJoin="round"
            />
          )}
          {/* Vehicle (body + wheels + velocity vector); +ROT keeps forward up */}
          <VehicleLayer
            state={state}
            x={vsx}
            y={vsy}
            rotationDeg={-psiDeg + ROT}
            pxm={pxm}
            spin={spinRef.current}
            theme={theme}
          />
          {/* Steering geometry: perpendicular lines + per-wheel/vehicle ICR */}
          <IcrLayer state={state} cam={cam} />

          {/* Measurement overlay */}
          {measureMode && measureLine.length >= 4 && (
            <Line points={measureLine} stroke="#fbbf24" strokeWidth={2} dash={[6, 4]} listening={false} />
          )}
          {measureMode && measureScreen.map(([sx, sy], i) => (
            <Circle key={`m-${i}`} x={sx} y={sy} radius={5} fill="#fbbf24" stroke="#1a1208" strokeWidth={1.5} listening={false} />
          ))}
          {measureMode && measureMid && (
            <Text
              x={measureMid[0] + 8}
              y={measureMid[1] - 8}
              text={`${measureDist.toFixed(2)} m`}
              fontSize={13}
              fontStyle="bold"
              fill="#fbbf24"
              listening={false}
            />
          )}
        </Layer>
      </Stage>

      <Hud state={state} />
    </div>
  );
}
