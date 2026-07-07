/**
 * Viewport — owns the 2D/3D view-mode toggle and renders the active view.
 *
 * Both views share the same Zustand store (live sim state + trajectory), so
 * switching is instantaneous and stateless. The 2D Canvas (Konva) stays the
 * default; the 3D view (Three.js / R3F) is opt-in and can be turned off on
 * low-end machines.
 */

import { lazy, Suspense, useState } from "react";

import Canvas2D from "@/components/Canvas2D";
import "./Viewport.css";

// three.js is heavy (~1 MB) — only load it when the 3D view is opened.
const Canvas3D = lazy(() => import("@/components/Canvas3D"));

export type ViewMode = "2d" | "3d";

export default function Viewport() {
  const [mode, setMode] = useState<ViewMode>("2d");

  return (
    <div className="viewport-root">
      <div className="view-switch">
        <button
          className={mode === "2d" ? "active" : ""}
          onClick={() => setMode("2d")}
        >
          2D
        </button>
        <button
          className={mode === "3d" ? "active" : ""}
          onClick={() => setMode("3d")}
        >
          3D
        </button>
      </div>
      {mode === "2d" ? (
        <Canvas2D />
      ) : (
        <Suspense
          fallback={<div className="canvas-container"><div className="canvas-loading">加载 3D 视图…</div></div>}
        >
          <Canvas3D />
        </Suspense>
      )}
    </div>
  );
}
