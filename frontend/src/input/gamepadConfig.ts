/**
 * Gamepad mapping configuration — the "mechanism" behind the direct-control
 * modes (Phase: gamepad UX). Three top-level modes:
 *
 *   assisted   sticks → (throttle, steering) → the active strategy (default).
 *   direct     per-wheel steering via a selectable *grouping* (front/rear,
 *              left/right, per-wheel, crab). Drives the `manual_wheel` strategy.
 *   holonomic  left stick = translation (forward + crab), right stick = yaw.
 *              Drives the `manual_body` strategy.
 *
 * The direct mode's grouping only changes which named channels are active and
 * how they fan out to the four wheels — every layout is one binding table, so
 * "front/rear independent" and "left/right independent" are the same mechanism
 * with different bindings. Standard-mapping axis indices: 0=LX 1=LY 2=RX 3=RY.
 */

export type GpMode = "assisted" | "direct" | "holonomic";
export type GpGrouping = "front_rear" | "left_right" | "per_wheel" | "crab";

export interface AxisBinding {
  axis: number;     // gamepad axis index; -1 = unbound
  invert: boolean;
  expo: number;     // 0 = linear … 0.8 = soft centre (cubic blend)
}

export interface ThrottleBinding {
  source: "triggers" | "axis";
  posBtn: number;   // forward trigger button index (standard RT = 7)
  negBtn: number;   // reverse trigger button index (standard LT = 6)
  axis: AxisBinding;
}

export interface GamepadConfig {
  mode: GpMode;
  grouping: GpGrouping;
  deadzone: number;      // 0 … 0.4
  sensitivity: number;   // steer gain 0.2 … 1.5
  throttle: ThrottleBinding;
  channels: Record<string, AxisBinding>;
}

const AX = (axis: number, invert = false, expo = 0): AxisBinding => ({ axis, invert, expo });

export function defaultGamepadConfig(): GamepadConfig {
  return {
    mode: "assisted",
    grouping: "front_rear",
    deadzone: 0.08,
    sensitivity: 1.0,
    throttle: { source: "triggers", posBtn: 7, negBtn: 6, axis: AX(1, true) },
    channels: {
      // assisted
      steering: AX(0),
      // direct · front/rear
      front: AX(0),
      rear: AX(2),
      // direct · left/right
      left: AX(0),
      right: AX(2),
      // direct · per-wheel (left stick = left side X前/Y后, right stick = right side)
      fl: AX(0), rl: AX(1), fr: AX(2), rr: AX(3),
      // direct · crab (single axis, all four wheels同角)
      crab_all: AX(0),
      // holonomic
      forward: AX(1, true), crab: AX(0), yaw: AX(2),
    },
  };
}

/** Channels the calibration panel should expose for the current mode/grouping. */
export function activeChannels(cfg: GamepadConfig): { key: string; label: string }[] {
  if (cfg.mode === "assisted") return [{ key: "steering", label: "转向" }];
  if (cfg.mode === "holonomic")
    return [
      { key: "forward", label: "前进/后退 (vx)" },
      { key: "crab", label: "横移 (vy)" },
      { key: "yaw", label: "自转 (ω)" },
    ];
  // direct
  switch (cfg.grouping) {
    case "front_rear": return [{ key: "front", label: "前轴 FL+FR" }, { key: "rear", label: "后轴 RL+RR" }];
    case "left_right": return [{ key: "left", label: "左侧 FL+RL" }, { key: "right", label: "右侧 FR+RR" }];
    case "per_wheel": return [
      { key: "fl", label: "FL 左前" }, { key: "rl", label: "RL 左后" },
      { key: "fr", label: "FR 右前" }, { key: "rr", label: "RR 右后" },
    ];
    case "crab": return [{ key: "crab_all", label: "四轮同角 (蟹行)" }];
  }
}

// ---- axis reading -----------------------------------------------------------

function applyExpo(v: number, expo: number): number {
  const e = Math.max(0, Math.min(0.9, expo));
  return (1 - e) * v + e * v * v * v;
}

/** Read one bound axis with deadzone → expo → sensitivity, clamped to [-1,1]. */
export function readAxis(
  gp: Gamepad, b: AxisBinding, deadzone: number, sensitivity = 1,
): number {
  if (b.axis < 0) return 0;
  let v = gp.axes[b.axis] ?? 0;
  if (b.invert) v = -v;
  const dz = Math.max(0, Math.min(0.45, deadzone));
  if (Math.abs(v) < dz) return 0;
  v = Math.sign(v) * (Math.abs(v) - dz) / (1 - dz);
  v = applyExpo(v, b.expo) * sensitivity;
  return Math.max(-1, Math.min(1, v));
}

export function readThrottle(gp: Gamepad, cfg: GamepadConfig): number {
  const t = cfg.throttle;
  if (t.source === "axis") return readAxis(gp, t.axis, cfg.deadzone);
  const rt = gp.buttons[t.posBtn]?.value ?? 0;
  const lt = gp.buttons[t.negBtn]?.value ?? 0;
  let v = rt - lt;
  const dz = Math.max(0, Math.min(0.45, cfg.deadzone));
  if (Math.abs(v) < dz) v = 0;
  return Math.max(-1, Math.min(1, v));
}

export interface GamepadOutput {
  throttle: number;
  steering?: number;                 // assisted
  wheelNorm?: [number, number, number, number]; // direct (FL,FR,RL,RR)
  body?: { vx: number; vy: number; yaw: number }; // holonomic
}

/** First connected gamepad, or null. */
export function firstGamepad(): Gamepad | null {
  if (typeof navigator === "undefined" || !navigator.getGamepads) return null;
  return Array.from(navigator.getGamepads()).find((g) => g && g.connected) ?? null;
}

/** Compute the driver output for the current config from the live gamepad. */
export function computeGamepadOutput(gp: Gamepad, cfg: GamepadConfig): GamepadOutput {
  const throttle = readThrottle(gp, cfg);
  const rd = (k: string) => readAxis(gp, cfg.channels[k], cfg.deadzone, cfg.sensitivity);

  if (cfg.mode === "assisted") {
    return { throttle, steering: rd("steering") };
  }
  if (cfg.mode === "holonomic") {
    return { throttle: 0, body: { vx: rd("forward"), vy: rd("crab"), yaw: rd("yaw") } };
  }
  // direct — fan the grouping channels out to (FL, FR, RL, RR)
  let fl = 0, fr = 0, rl = 0, rr = 0;
  switch (cfg.grouping) {
    case "front_rear": { const f = rd("front"), r = rd("rear"); fl = fr = f; rl = rr = r; break; }
    case "left_right": { const l = rd("left"), r = rd("right"); fl = rl = l; fr = rr = r; break; }
    case "per_wheel": fl = rd("fl"); rl = rd("rl"); fr = rd("fr"); rr = rd("rr"); break;
    case "crab": { const a = rd("crab_all"); fl = fr = rl = rr = a; break; }
  }
  return { throttle, wheelNorm: [fl, fr, rl, rr] };
}

/** Which backend strategy a mode requires (null = leave current, for assisted). */
export function requiredStrategy(mode: GpMode): string | null {
  if (mode === "direct") return "manual_wheel";
  if (mode === "holonomic") return "manual_body";
  return null;
}

const LS_KEY = "4wis_gamepad_config_v1";

export function loadGamepadConfig(): GamepadConfig {
  try {
    const raw = localStorage.getItem(LS_KEY);
    if (raw) {
      const parsed = JSON.parse(raw);
      // Merge onto defaults so new channels/fields survive old saved configs.
      const d = defaultGamepadConfig();
      return {
        ...d, ...parsed,
        throttle: { ...d.throttle, ...(parsed.throttle ?? {}) },
        channels: { ...d.channels, ...(parsed.channels ?? {}) },
      };
    }
  } catch { /* ignore */ }
  return defaultGamepadConfig();
}

export function saveGamepadConfig(cfg: GamepadConfig): void {
  try { localStorage.setItem(LS_KEY, JSON.stringify(cfg)); } catch { /* ignore */ }
}
