/**
 * SVG drag plumbing shared by the vehicle-geometry diagrams.
 *
 * `useVbPointer` converts client coordinates → the SVG's viewBox coordinates
 * (via the live screen CTM, so it survives resizing/scaling). `Handle` is a
 * draggable dot that reports the current viewBox point to `onDrag`; each diagram
 * maps that back to a parameter through its own inverse scale.
 */

import { RefObject, useCallback, useRef, useState } from "react";

export function useVbPointer(svgRef: RefObject<SVGSVGElement>) {
  return useCallback((clientX: number, clientY: number) => {
    const svg = svgRef.current;
    if (!svg) return { x: 0, y: 0 };
    const pt = svg.createSVGPoint();
    pt.x = clientX; pt.y = clientY;
    const ctm = svg.getScreenCTM();
    if (!ctm) return { x: 0, y: 0 };
    const p = pt.matrixTransform(ctm.inverse());
    return { x: p.x, y: p.y };
  }, [svgRef]);
}

export function Handle({
  cx, cy, r = 6, svgRef, onDrag, label, className = "",
}: {
  cx: number; cy: number; r?: number;
  svgRef: RefObject<SVGSVGElement>;
  onDrag: (vb: { x: number; y: number }) => void;
  label?: string;
  className?: string;
}) {
  const toVb = useVbPointer(svgRef);
  const [active, setActive] = useState(false);
  const draggingRef = useRef(false);

  const down = (e: React.PointerEvent) => {
    e.stopPropagation();
    try { (e.target as Element).setPointerCapture(e.pointerId); } catch { /* headless / unsupported */ }
    draggingRef.current = true;
    setActive(true);
  };
  const move = (e: React.PointerEvent) => {
    if (!draggingRef.current) return;
    onDrag(toVb(e.clientX, e.clientY));
  };
  const up = (e: React.PointerEvent) => {
    draggingRef.current = false;
    setActive(false);
    try { (e.target as Element).releasePointerCapture(e.pointerId); } catch { /* ignore */ }
  };

  return (
    <g className={`vg-handle ${active ? "active" : ""} ${className}`}>
      <circle cx={cx} cy={cy} r={r + 8} className="vg-handle-hit"
        onPointerDown={down} onPointerMove={move} onPointerUp={up} onPointerCancel={up} />
      <circle cx={cx} cy={cy} r={r} className="vg-handle-dot" />
      {label && <title>{label}</title>}
    </g>
  );
}
