// World ↔ screen projection for the 2D top-down view.
//
// World: X-forward (east), Y-left (north). Screen: the camera follows the
// vehicle; world +X maps to screen-up, +Y to screen-left:
//   sx = W/2 − (wy − camY)·pxm
//   sy = H/2 − (wx − camX)·pxm

export interface Camera2D {
  camX: number;
  camY: number;
  pxm: number;   // pixels per metre
  W: number;     // viewport width [px]
  H: number;     // viewport height [px]
}

/** Extra screen rotation (deg) so heading ψ renders with forward pointing up. */
export const ROT = -90;

export function worldToScreen(cam: Camera2D, wx: number, wy: number): [number, number] {
  return [
    cam.W / 2 - (wy - cam.camY) * cam.pxm,
    cam.H / 2 - (wx - cam.camX) * cam.pxm,
  ];
}

export function screenToWorld(cam: Camera2D, sx: number, sy: number): [number, number] {
  return [
    cam.camX - (sy - cam.H / 2) / cam.pxm,
    cam.camY - (sx - cam.W / 2) / cam.pxm,
  ];
}

export function bodyToWorld(
  pose: { x: number; y: number; psi: number },
  bx: number,
  by: number,
): { x: number; y: number } {
  const c = Math.cos(pose.psi), s = Math.sin(pose.psi);
  return { x: pose.x + c * bx - s * by, y: pose.y + s * bx + c * by };
}
