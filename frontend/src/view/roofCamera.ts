/**
 * Roof-camera configuration — the driving view.
 *
 * The 3D viewport has three cameras. Two are orbit cameras (free / target-
 * locked); this module owns the third, a rig rigidly mounted to the vehicle
 * body and aimed forward. It is the view you drive from with a wheel: you see
 * the bodywork you have to place between the cones, without the mirror-less
 * blind spots of a true cockpit view.
 *
 * Rig geometry, all in BODY frame metres (x forward, y left, z up):
 *
 *        height  ── camera height above ground
 *              │        pitch (down from horizontal)
 *      ┌───────●╌╌╌╌╌╌╌╌────────▶ aim
 *      │  back │
 *   ───┴───────┴────────────────  vehicle, seen from the side
 *
 * `back` slides the camera along −x, so 0 sits above the CG and larger values
 * pull it back over the tail (and, past the bodywork, into a chase camera).
 *
 * `smooth` is a first-order lag on the rig's *heading only* — never on its
 * position, which stays welded to the body so the camera can't trail the car it
 * is mounted on. At 0 every yaw twitch is visible (best for judging what the
 * car is actually doing); larger values let the view swing into a turn a beat
 * late, which is calmer to drive but hides small yaw motion.
 *
 * `attitude` scales how much of the multibody roll/pitch reaches the camera.
 * Some is worth having — it is the cue that tells you the car is loaded up —
 * but at 1.0 the horizon moves enough to make cone placement harder.
 */

export interface RoofCamConfig {
  height: number;    // m above ground
  back: number;      // m behind the CG
  pitch: number;     // deg below horizontal
  fov: number;       // deg, vertical
  smooth: number;    // s, pose lag time-constant (0 = rigid)
  attitude: number;  // 0…1 share of body roll/pitch fed to the camera
}

export type CamMode3d = "orbit" | "follow" | "roof";

export const CAM_MODE_LABEL: Record<CamMode3d, string> = {
  orbit: "自由",
  follow: "跟随",
  roof: "车顶",
};

export function defaultRoofCam(): RoofCamConfig {
  // Tuned so the bodywork takes roughly the bottom quarter of the frame: enough
  // to place the car against a cone, not enough to hide the next gate.
  return { height: 2.1, back: 0.8, pitch: 6, fov: 68, smooth: 0.06, attitude: 0.35 };
}

/** Named rig positions — the same camera, moved along the body. */
export const ROOF_PRESETS: { key: string; label: string; hint: string; cfg: Partial<RoofCamConfig> }[] = [
  { key: "roof", label: "车顶", hint: "车顶上方前视 — 看得到机头和两侧车身，绕桩/移线判位最准",
    cfg: { height: 2.1, back: 0.8, pitch: 6, fov: 68 } },
  { key: "hood", label: "引擎盖", hint: "贴近机头，视野最贴地，车身占画面少、看桩最远",
    cfg: { height: 1.35, back: -0.4, pitch: 4, fov: 72 } },
  { key: "chase", label: "追车", hint: "退到车后上方，能同时看到四个轮子的转角",
    cfg: { height: 3.2, back: 7.0, pitch: 12, fov: 60 } },
];

export const ROOF_LIMITS: Record<keyof RoofCamConfig, { min: number; max: number; step: number; unit: string; label: string }> = {
  height: { min: 0.8, max: 6, step: 0.05, unit: "m", label: "高度" },
  back: { min: -2, max: 12, step: 0.1, unit: "m", label: "后移" },
  pitch: { min: -5, max: 35, step: 1, unit: "°", label: "俯角" },
  fov: { min: 40, max: 100, step: 1, unit: "°", label: "视场角" },
  smooth: { min: 0, max: 0.4, step: 0.01, unit: "s", label: "平滑" },
  attitude: { min: 0, max: 1, step: 0.05, unit: "", label: "姿态联动" },
};

const LS_KEY = "4wis_roof_cam_v1";
const LS_MODE = "4wis_cam_mode_3d";

function clampCfg(c: RoofCamConfig): RoofCamConfig {
  const out = { ...c };
  (Object.keys(ROOF_LIMITS) as (keyof RoofCamConfig)[]).forEach((k) => {
    const { min, max } = ROOF_LIMITS[k];
    const v = Number(out[k]);
    out[k] = Number.isFinite(v) ? Math.max(min, Math.min(max, v)) : defaultRoofCam()[k];
  });
  return out;
}

export function loadRoofCam(): RoofCamConfig {
  try {
    const raw = localStorage.getItem(LS_KEY);
    if (raw) return clampCfg({ ...defaultRoofCam(), ...JSON.parse(raw) });
  } catch { /* ignore */ }
  return defaultRoofCam();
}

export function saveRoofCam(c: RoofCamConfig): void {
  try { localStorage.setItem(LS_KEY, JSON.stringify(c)); } catch { /* ignore */ }
}

export function loadCamMode(): CamMode3d {
  try {
    const v = localStorage.getItem(LS_MODE);
    if (v === "orbit" || v === "follow" || v === "roof") return v;
  } catch { /* ignore */ }
  return "follow";
}

export function saveCamMode(m: CamMode3d): void {
  try { localStorage.setItem(LS_MODE, m); } catch { /* ignore */ }
}

/** `C` cycles 自由 → 跟随 → 车顶 → 自由. */
export function nextCamMode(m: CamMode3d): CamMode3d {
  return m === "orbit" ? "follow" : m === "follow" ? "roof" : "orbit";
}
