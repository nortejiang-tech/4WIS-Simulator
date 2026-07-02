/**
 * KeyboardInput — listens to window-level keydown/keyup, computes a smoothed
 * (throttle, steering) input pair, and pushes it to the backend over the
 * WebSocket at ~50 Hz. Also wires number-key strategy switching and `R` for
 * reset.
 *
 * Smoothing:
 *   * Holding W/A/S/D ramps the corresponding axis toward ±1 with time-constant
 *     τ_ramp ~ 0.18 s, so the vehicle doesn't snap to full throttle/steering.
 *   * Releasing returns the axis toward 0 with τ_return ~ 0.25 s (slightly
 *     slower so a tap leaves a tiny residual).
 *
 * Gamepad (Web Gamepad API, "standard" mapping):
 *   * This component is the single writer of driver input — the gamepad is
 *     polled inside the same rAF loop and merged additively with the keyboard
 *     axes (so an idle stick doesn't cancel W/S and vice versa).
 *   * Steering = −axes[0] (left stick X; stick left = steer left = +1).
 *   * Throttle = RT − LT analog triggers (buttons 7/6); if the device has no
 *     analog triggers (some wheels), falls back to −axes[1] (left stick Y).
 *   * Deadzone 0.08 per axis; no extra smoothing (sticks are already analog).
 */

import { useEffect, useRef } from "react";
import { resetSim, setDriver, setStrategy } from "@/api/ws";
import { useSimStore } from "@/store/sim";

const PUSH_INTERVAL_MS = 20;     // 50 Hz to backend
const TAU_RAMP = 0.18;           // s
const TAU_RETURN = 0.25;         // s
// Hold-speed: how fast a held key drives the *persistent* target (units/sec).
const HOLD_RATE = 0.6;
// Steering auto-return: max decay rate [1/s] at slider=1 (= 1.5× the baseline
// 1/TAU_RETURN ≈ 4/s). slider=0 → 0 = hold (no return).
const STEER_RETURN_RATE_MAX = 1.5 / TAU_RETURN;

const HOTKEYS_STRATEGY: Record<string, number> = {
  Digit1: 0, Digit2: 1, Digit3: 2, Digit4: 3, Digit5: 4,
};

const GP_DEADZONE = 0.08;

function deadzone(v: number): number {
  if (Math.abs(v) < GP_DEADZONE) return 0;
  // Rescale so the response is continuous at the deadzone edge.
  const s = Math.sign(v);
  return s * (Math.abs(v) - GP_DEADZONE) / (1 - GP_DEADZONE);
}

/** Read (throttle, steering) from the first connected gamepad, or null. */
function readGamepad(): { throttle: number; steering: number } | null {
  if (typeof navigator === "undefined" || !navigator.getGamepads) return null;
  const gp = Array.from(navigator.getGamepads()).find((g) => g && g.connected);
  if (!gp) return null;
  const steering = deadzone(-(gp.axes[0] ?? 0));
  // Prefer analog triggers (standard mapping: 6 = LT, 7 = RT).
  const rt = gp.buttons[7]?.value ?? 0;
  const lt = gp.buttons[6]?.value ?? 0;
  let throttle = deadzone(rt - lt);
  if (rt === 0 && lt === 0) {
    // No trigger input — fall back to left stick Y (push forward = +throttle).
    throttle = deadzone(-(gp.axes[1] ?? 0));
  }
  return { throttle, steering };
}

export default function KeyboardInput() {
  const pressed = useRef<Record<string, boolean>>({});
  const throttle = useRef(0);
  const steering = useRef(0);
  const last = useRef(performance.now());
  const lastPush = useRef(performance.now());

  // Strategies need to be read inside handlers — keep a ref that's updated
  // whenever the store's list changes.
  const strategiesRef = useRef<string[]>([]);
  const holdSpeedRef = useRef(false);
  const steerReturnRef = useRef(0.667);
  const zeroReqRef = useRef(0);
  const cruiseOnRef = useRef(false);
  const cruiseSpeedRef = useRef(5);
  const vMaxRef = useRef(20);
  const gamepadEnabledRef = useRef(true);
  useEffect(() => useSimStore.subscribe((st) => {
    strategiesRef.current = st.strategies;
    holdSpeedRef.current = st.holdSpeed;
    steerReturnRef.current = st.steerReturn;
    cruiseOnRef.current = st.cruiseOn;
    cruiseSpeedRef.current = st.cruiseSpeed;
    vMaxRef.current = st.state?.params.v_max ?? 20;
    gamepadEnabledRef.current = st.gamepadEnabled;
    // A bump in zeroRequest = UI asked us to zero the persistent targets.
    if (st.zeroRequest !== zeroReqRef.current) {
      zeroReqRef.current = st.zeroRequest;
      throttle.current = 0;
      steering.current = 0;
    }
  }), []);

  useEffect(() => {
    const down = (e: KeyboardEvent) => {
      if (e.repeat) return;
      // Don't swallow typing in inputs (none yet, but future-proof).
      const tgt = e.target as HTMLElement | null;
      if (tgt && /^(INPUT|TEXTAREA|SELECT)$/.test(tgt.tagName)) return;

      if (e.code === "KeyR") {
        resetSim();
        return;
      }
      const idx = HOTKEYS_STRATEGY[e.code];
      if (idx !== undefined) {
        const s = strategiesRef.current[idx];
        if (s) setStrategy(s);
        return;
      }
      pressed.current[e.code] = true;
    };
    const up = (e: KeyboardEvent) => {
      pressed.current[e.code] = false;
    };
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    return () => {
      window.removeEventListener("keydown", down);
      window.removeEventListener("keyup", up);
    };
  }, []);

  // Track gamepad connect/disconnect so the UI can show the device name.
  useEffect(() => {
    const sync = () => {
      const gp = navigator.getGamepads
        ? Array.from(navigator.getGamepads()).find((g) => g && g.connected)
        : null;
      useSimStore.getState().setGamepadId(gp ? gp.id : null);
    };
    sync();
    window.addEventListener("gamepadconnected", sync);
    window.addEventListener("gamepaddisconnected", sync);
    return () => {
      window.removeEventListener("gamepadconnected", sync);
      window.removeEventListener("gamepaddisconnected", sync);
    };
  }, []);

  // Integration loop with rAF
  useEffect(() => {
    let raf = 0;
    const step = () => {
      const now = performance.now();
      const dt = Math.min(0.1, (now - last.current) / 1000);
      last.current = now;

      const p = pressed.current;
      const clamp = (v: number) => Math.max(-1, Math.min(1, v));
      const ramp = (cur: number, tgt: number, tau: number) => {
        const alpha = 1 - Math.exp(-dt / tau);
        return cur + (tgt - cur) * alpha;
      };

      // --- Throttle (speed) ---
      if (cruiseOnRef.current) {
        // Fixed-speed cruise overrides W/S: command throttle = target/v_max.
        const vmax = vMaxRef.current || 20;
        throttle.current = clamp(cruiseSpeedRef.current / vmax);
      } else if (holdSpeedRef.current) {
        // Hold-speed: W/S nudge a persistent target speed; release = hold.
        if (p["KeyW"]) throttle.current = clamp(throttle.current + HOLD_RATE * dt);
        if (p["KeyS"]) throttle.current = clamp(throttle.current - HOLD_RATE * dt);
        if (p["Space"]) throttle.current = 0; // quick stop
      } else {
        // Momentary: ramp toward key, return to 0 on release.
        let targetThr = 0;
        if (p["KeyW"]) targetThr += 1;
        if (p["KeyS"]) targetThr -= 1;
        if (p["Space"]) targetThr = 0;
        throttle.current = ramp(throttle.current, targetThr, targetThr === 0 ? TAU_RETURN : TAU_RAMP);
        if (Math.abs(throttle.current) < 0.005) throttle.current = 0;
      }

      // --- Steering (independent of hold-speed) ---
      // While A/D held: ramp toward ±1. Released: decay toward 0 at the
      // adjustable return rate (0 = hold, 1 = 1.5× the baseline rate).
      let targetSteer = 0;
      if (p["KeyA"]) targetSteer += 1;
      if (p["KeyD"]) targetSteer -= 1;
      if (targetSteer !== 0) {
        steering.current = ramp(steering.current, targetSteer, TAU_RAMP);
      } else {
        const rate = steerReturnRef.current * STEER_RETURN_RATE_MAX;
        if (rate > 0) {
          steering.current *= Math.exp(-rate * dt);
          if (Math.abs(steering.current) < 0.005) steering.current = 0;
        }
      }

      // --- Gamepad (merged additively; idle stick leaves keyboard in charge) ---
      let outThrottle = throttle.current;
      let outSteering = steering.current;
      if (gamepadEnabledRef.current) {
        const gp = readGamepad();
        if (gp) {
          if (!cruiseOnRef.current) outThrottle = clamp(outThrottle + gp.throttle);
          outSteering = clamp(outSteering + gp.steering);
        }
      }

      // Throttle output rate to the backend
      if (now - lastPush.current >= PUSH_INTERVAL_MS) {
        setDriver(outThrottle, outSteering);
        lastPush.current = now;
      }

      raf = requestAnimationFrame(step);
    };
    raf = requestAnimationFrame(step);
    return () => cancelAnimationFrame(raf);
  }, []);

  return null;
}
