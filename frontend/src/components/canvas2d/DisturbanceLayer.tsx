/**
 * DisturbanceLayer — renders scene disturbances and (optionally) lets the
 * user select / drag them for GUI editing.
 *
 * Selection: click a region → onSelect(id). The selected region gets a bright
 * outline and becomes draggable; onDragEnd reports the new world centre.
 */

import { Group, Line, Rect, Text } from "react-konva";
import type Konva from "konva";

import type { DisturbanceMsg } from "@/types/sim";
import { muToColor } from "./colors";
import type { Camera2D } from "./projection";
import { ROT, screenToWorld, worldToScreen } from "./projection";

const MONO = "ui-monospace, SFMono-Regular, Menlo, monospace";

interface LayerProps {
  disturbances: DisturbanceMsg[];
  cam: Camera2D;
  selectable: boolean;
  selectedId: string | null;
  onSelect?: (id: string) => void;
  onDragEnd?: (id: string, wx: number, wy: number) => void;
}

export default function DisturbanceLayer({
  disturbances, cam, selectable, selectedId, onSelect, onDragEnd,
}: LayerProps) {
  return (
    <>
      {disturbances.map((d) => (
        <DisturbanceShape
          key={d.id}
          d={d}
          cam={cam}
          selectable={selectable}
          selected={selectable && d.id === selectedId}
          onSelect={onSelect}
          onDragEnd={onDragEnd}
        />
      ))}
    </>
  );
}

interface ShapeProps {
  d: DisturbanceMsg;
  cam: Camera2D;
  selectable: boolean;
  selected: boolean;
  onSelect?: (id: string) => void;
  onDragEnd?: (id: string, wx: number, wy: number) => void;
}

function DisturbanceShape({ d, cam, selectable, selected, onSelect, onDragEnd }: ShapeProps) {
  const [cx, cy] = worldToScreen(cam, d.x, d.y);
  const psiDeg = (d.heading * 180) / Math.PI - ROT;
  const w = d.length * cam.pxm;   // along-axis in pixels
  const h = d.width * cam.pxm;    // cross-axis in pixels

  const handleDragEnd = (e: Konva.KonvaEventObject<DragEvent>) => {
    const { x, y } = e.target.position();
    const [wx, wy] = screenToWorld(cam, x, y);
    onDragEnd?.(d.id, wx, wy);
  };

  return (
    <Group
      x={cx}
      y={cy}
      rotation={-psiDeg}
      listening={selectable}
      draggable={selected}
      onClick={() => onSelect?.(d.id)}
      onTap={() => onSelect?.(d.id)}
      onDragEnd={handleDragEnd}
    >
      <ShapeBody d={d} w={w} h={h} />
      {selected && (
        <Rect
          x={-w / 2 - 3} y={-h / 2 - 3}
          width={w + 6} height={h + 6}
          stroke="#38bdf8"
          strokeWidth={2}
          dash={[6, 4]}
          listening={false}
        />
      )}
    </Group>
  );
}

function ShapeBody({ d, w, h }: { d: DisturbanceMsg; w: number; h: number }) {
  if (d.type === "ice_patch") {
    return (
      <>
        <Rect
          x={-w / 2} y={-h / 2} width={w} height={h}
          fill={muToColor(d.mu)}
          stroke="rgba(255,255,255,0.25)" strokeWidth={1} dash={[6, 4]} cornerRadius={4}
        />
        <Text
          x={-w / 2 + 6} y={-h / 2 + 4}
          text={`ice  μ=${d.mu.toFixed(2)}`}
          fontSize={11} fill="rgba(226,232,240,0.7)" fontFamily={MONO}
          listening={false}
        />
      </>
    );
  }
  if (d.type === "speed_bump") {
    return (
      <>
        <Rect
          x={-w / 2} y={-h / 2} width={w} height={h}
          fill="rgba(251,191,36,0.30)" stroke="rgba(251,191,36,0.7)" strokeWidth={2}
        />
        {Array.from({ length: 4 }).map((_, i) => (
          <Line
            key={i}
            points={[-w / 2 + i * (w / 4), -h / 2, -w / 2 + (i + 1) * (w / 4), h / 2]}
            stroke="rgba(251,191,36,0.5)" strokeWidth={1}
            listening={false}
          />
        ))}
        <Text
          x={-w / 2 + 6} y={-h / 2 + 4}
          text={`bump ${(d.height * 1000).toFixed(0)}mm`}
          fontSize={11} fill="rgba(226,232,240,0.9)" fontFamily={MONO}
          listening={false}
        />
      </>
    );
  }
  if (d.type === "slope") {
    const grade_pct = Math.tan(d.angle) * 100;
    return (
      <>
        <Rect
          x={-w / 2} y={-h / 2} width={w} height={h}
          fill="rgba(168,85,247,0.18)" stroke="rgba(168,85,247,0.6)" strokeWidth={1} dash={[3, 3]}
        />
        <Line points={[-w / 2 + 12, 0, w / 2 - 12, 0]} stroke="rgba(168,85,247,0.8)" strokeWidth={2} listening={false} />
        <Line points={[w / 2 - 18, -5, w / 2 - 12, 0, w / 2 - 18, 5]} stroke="rgba(168,85,247,0.8)" strokeWidth={2} listening={false} />
        <Text
          x={-w / 2 + 6} y={-h / 2 + 4}
          text={`slope ${grade_pct.toFixed(1)}%`}
          fontSize={11} fill="rgba(226,232,240,0.9)" fontFamily={MONO}
          listening={false}
        />
      </>
    );
  }
  // split_mu: two halves stacked across the long axis
  return (
    <>
      <Rect
        x={-w / 2} y={-h / 2} width={w} height={h / 2}
        fill={muToColor(d.mu_left)} stroke="rgba(255,255,255,0.18)" strokeWidth={1}
      />
      <Rect
        x={-w / 2} y={0} width={w} height={h / 2}
        fill={muToColor(d.mu_right)} stroke="rgba(255,255,255,0.18)" strokeWidth={1}
      />
      <Text
        x={-w / 2 + 6} y={-h / 2 + 4}
        text={`split-μ  L=${d.mu_left.toFixed(2)}  R=${d.mu_right.toFixed(2)}`}
        fontSize={11} fill="rgba(226,232,240,0.7)" fontFamily={MONO}
        listening={false}
      />
    </>
  );
}
