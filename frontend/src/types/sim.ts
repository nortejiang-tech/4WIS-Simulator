// Shared TypeScript types — must match what the backend sends/receives over WS.

export interface WheelState {
  delta: number;          // actual steer angle [rad]
  delta_cmd: number;      // commanded steer angle [rad]
  omega: number;          // wheel spin angular velocity [rad/s]
  fz: number;             // vertical load [N]
  torque_steer: number;   // steering resistance torque [N·m]
  mu?: number;            // effective surface μ at this wheel
  susp_defl?: number;     // suspension compression vs static [m] (multibody)
  pos_body: [number, number]; // wheel position in body frame [m]
  // Per-wheel steering centre: vehicle ICR projected onto this wheel's
  // perpendicular line (body frame), and the signed deviation [m].
  icr_body?: [number | null, number | null];
  icr_dev?: number | null;
  // Split-rack force chain (分体齿条力链)
  rack_force?: number;           // rack axial force [N]
  motor_torque_demand?: number;  // motor shaft torque demand [N·m]
  linkage_arm_tie_angle?: number | null; // steering arm vs tie rod [rad]
  linkage_tie_rack_angle?: number | null; // tie rod vs rack axis [rad]
  linkage_efficiency?: number;   // combined hardpoint/rack efficiency [-]
  tire_fx?: number;              // tyre force in wheel frame [N]
  tire_fy?: number;              // tyre force in wheel frame [N]
  side_force_body_y?: number;    // lateral force in body-Y [N]
  force_source?: string;         // tire_model | kinematic_estimate
  // Tyre slip diagnostics (dynamic models only; 0 for kinematic)
  slip_alpha?: number;           // side-slip angle α [rad]
  slip_kappa?: number;           // longitudinal slip ratio κ [-]
  locked?: boolean;              // friction-brake lockup flag (v0.100)
  // Friction budget (v0.101). The operating point itself is (tire_fx, tire_fy)
  // above; these are the parts the client cannot derive on its own.
  grip_capacity?: number;        // μ·Fz — radius of the friction circle [N]
  grip_util?: number;            // |F| / (μ·Fz) ∈ [0,1]
  grip_margin_lat?: number;      // extra |Fy| still available at this Fx [N]
  grip_margin_long?: number;     // extra |Fx| still available at this Fy [N]
  grip_alpha_peak?: number;      // |α| at peak lateral force [rad]
  grip_kappa_peak?: number;      // |κ| at peak longitudinal force [-]
  // Past the peak the tyre is not just "nearly out of grip" — it is on the
  // falling side, where more slip returns LESS force. Utilisation alone cannot
  // tell the two apart, which is why these ride alongside it.
  grip_beyond_peak_lat?: boolean;
  grip_beyond_peak_long?: boolean;
}

/** Vehicle-level grip: body specific force vs the μ·g envelope (g-g diagram). */
export interface AccelState {
  ax: number;         // body longitudinal specific force [m/s²]
  ay: number;         // body lateral specific force [m/s²]
  envelope: number;   // μ_avg · g — radius of the g-g envelope [m/s²]
  valid: boolean;     // false on models without tyres (kinematic)
}

export interface Pose { x: number; y: number; psi: number }
export interface Attitude { z: number; roll: number; pitch: number }
export interface Velocity { vx: number; vy: number; yaw_rate: number }
export interface SideForceSummary {
  left: number;
  right: number;
  total: number;
  source: string;
}

export interface DriverEcho {
  throttle: number;
  brake?: number;        // [0,1] friction-brake pedal (v0.100)
  gear?: number;         // -1 = R, 0 = N, 1 = D (v0.100)
  steering: number;
  handbrake: number;
  // Resolved steering-feel outputs for this frame (v0.100). Sent by the
  // backend so the HUD never has to re-derive the mapping — a front-end copy
  // cannot see the μ-aware soft limit and so misreports the angle actually
  // commanded.
  steer_ratio?: number;          // i(v), the live gear ratio
  steer_delta_eff_deg?: number;  // δ_eff after ratio + soft limit
  mode_params: Record<string, unknown>;
}

export interface VehicleParamsLite {
  wheelbase: number;
  track_front: number;
  track_rear: number;
  tire_radius: number;
  steer_limit: number;
  v_max: number;
  steer_wheel_range?: number;     // deg (v0.100 steering feel)
  // Resolved ratios — the raw params default to 0 meaning "auto-derive from
  // steer_wheel_range", so the backend sends the computed values.
  steer_ratio_low?: number;
  steer_ratio_high?: number;
  steer_ratio_v_ref?: number;     // m/s, ratio transition reference speed
}

export interface DisturbanceBase {
  id: string;
  type: string;
  x: number;
  y: number;
  width: number;
  length: number;
  heading: number;
}
export interface IcePatchD extends DisturbanceBase {
  type: "ice_patch";
  mu: number;   // absolute μ inside the patch
}
export interface SplitMuD extends DisturbanceBase {
  type: "split_mu";
  mu_left: number;
  mu_right: number;
}
export interface SpeedBumpD extends DisturbanceBase {
  type: "speed_bump";
  height: number;
  stiffness: number;
}
export interface SlopeD extends DisturbanceBase {
  type: "slope";
  angle: number;
}
export type DisturbanceMsg = IcePatchD | SplitMuD | SpeedBumpD | SlopeD;

export interface SceneSnapshot {
  base_mu: number;
  surface: string;
  disturbances: DisturbanceMsg[];
}

export interface SimStateMessage {
  type: "state";
  t: number;             // simulation time [s]
  wall: number;          // server wall-clock at emission
  strategy: string;
  driver: DriverEcho;
  pose: Pose;
  attitude?: Attitude;   // z/roll/pitch — nonzero only for the multibody model
  velocity: Velocity;
  accel?: AccelState;    // body specific force + μ·g envelope (v0.101)
  wheels: WheelState[];  // length 4 — FL, FR, RL, RR
  side_force_summary?: SideForceSummary;
  icr_vehicle_body: [number | null, number | null];
  icr_target_body: [number | null, number | null];
  params: VehicleParamsLite;
  scene: SceneSnapshot | null;
  model_type?: string;   // "kinematic" | "simplified_dynamic" | "multibody"
  path_version?: number; // bumps when the reference path changes
  scenario_version?: number; // bumps when the active scenario changes
  fault_active?: boolean; // true when at least one fault is active
}

// Reference path (fetched from REST when path_version changes)

/** A course marker. `kind` picks the glyph both viewports draw. */
export interface PathCone {
  x: number;
  y: number;
  kind: "cone" | "pole" | string;
  color: string;
  height: number;   // metres
}

/** A painted ground line (lane edge, test-section box, start/finish). */
export interface PathMark {
  points: [number, number][];
  color: string;
  width: number;    // metres
  dash: boolean;
}

export interface PathPlan {
  name: string;
  label: string;   // human-readable maneuver name
  notes: string;   // the geometry that was actually laid out, in one line
  closed: boolean;
  points: [number, number][];  // world frame
  cones: PathCone[];           // ground markers
  marks: PathMark[];           // painted ground lines
}

// Static driving scenario (fetched from REST when scenario_version changes)
export interface ScenarioSurface { points: [number, number][]; color: string; kind: string }
export interface ScenarioLine {
  points: [number, number][];
  color: string;
  width: number;
  dash: boolean;
  /** Optional (on, off) dash rhythm in metres; overrides the boolean `dash`. */
  dash_pattern?: [number, number] | null;
}
export interface ScenarioMarker { type: string; x: number; y: number; heading: number; meta: Record<string, unknown> }
/** A named spawn pose offered by a scenario (x, y, heading). */
export interface ScenarioSpawn { name: string; x: number; y: number; heading: number }
export interface Scenario {
  name: string;
  label: string;
  surfaces: ScenarioSurface[];
  lines: ScenarioLine[];
  markers: ScenarioMarker[];
  spawn: [number, number, number];
  /** Named spawn points (optional; legacy scenarios have none). */
  spawns?: ScenarioSpawn[];
  /** Named anchor poses path templates can be placed on. */
  anchors?: Record<string, [number, number, number]>;
}

// Client → Server messages
export interface DriverMsg {
  type: "driver";
  throttle?: number;
  brake?: number;       // [0,1] friction-brake pedal (v0.100)
  gear?: number;        // -1 = R, 0 = N, 1 = D (v0.100)
  steering?: number;
  handbrake?: number;   // 0 | 1 parking brake
  mode_params?: Record<string, unknown>;
}
export interface StrategyMsg { type: "strategy"; name: string }
export interface ResetMsg { type: "reset" }
export type ClientMessage = DriverMsg | StrategyMsg | ResetMsg;

export const WHEEL_LABELS = ["FL", "FR", "RL", "RR"] as const;
export type WheelLabel = typeof WHEEL_LABELS[number];
