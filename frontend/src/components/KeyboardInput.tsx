/**
 * KeyboardInput — listens to window-level keydown/keyup, computes a smoothed
 * driver command, and pushes it to the backend over the WebSocket at ~50 Hz.
 * Also wires number-key strategy switching and `R` for reset.
 *
 * Longitudinal input is a *direction-intent* state machine (v0.100):
 *   * W = "I want to go forward": while moving forward → throttle; while
 *     moving backward → brake. Once stopped, W held = forward throttle.
 *   * S = "I want to go backward": while moving backward → reverse throttle;
 *     while moving forward → brake. To *engage* reverse from a stop, the
 *     vehicle must be stopped AND S released-then-pressed again (no accidental
 *     drop into reverse from a held brake).
 *   * Space = parking brake (handbrake), not "release throttle".
 *
 * The same intent logic applies to the gamepad / wheel pedals: the front-end
 * resolves (forward-intent, backward-intent) into (throttle, brake, gear) and
 * sends the resolved physical channels; the backend stays pure-physics.
 *
 * Steering still ramps W/A/S/D-style toward ±1 with τ_ramp ~ 0.18 s and
 * returns to 0 with τ_return ~ 0.25 s.
 */

import { useEffect, useRef } from "react";
import { resetSim, setDriver, setStrategy } from "@/api/ws";
import { computeGamepadOutput, firstGamepad, GamepadConfig } from "@/input/gamepadConfig";
import { useSimStore } from "@/store/sim";

const PUSH_INTERVAL_MS = 20;     // 50 Hz to backend
const TAU_RAMP = 0.18;           // s
const TAU_RETURN = 0.25;         // s
// Steering auto-return: max decay rate [1/s] at slider=1 (= 1.5× the baseline
// 1/TAU_RETURN ≈ 4/s). slider=0 → 0 = hold (no return).
const STEER_RETURN_RATE_MAX = 1.5 / TAU_RETURN;
// A vehicle is "stopped" below this speed; direction changes only engage then.
const STOP_SPEED = 0.3;          // m/s
// Hold-speed: how fast a held key drives the *persistent* target (units/sec).
const HOLD_RATE = 0.6;

const HOTKEYS_STRATEGY: Record<string, number> = {
  Digit1: 0, Digit2: 1, Digit3: 2, Digit4: 3, Digit5: 4,
};

type Direction = 1 | -1;   // forward | reverse

export default function KeyboardInput() {
  const pressed = useRef<Record<string, boolean>>({});
  // Smoothed drive (throttle ∈ [0,1]) and brake (∈ [0,1]) channels.
  const throttle = useRef(0);
  const brake = useRef(0);
  const steering = useRef(0);
  // Direction-intent state machine.
  const direction = useRef<Direction>(1);
  // Track whether the "other" intent key has been released since stopping, so
  // a held brake doesn't auto-flip into reverse — only a fresh press does.
  const fwdKeyArmed = useRef(true);   // W may engage forward from a stop
  const revKeyArmed = useRef(true);   // S may engage reverse from a stop
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
  const vxRef = useRef(0);            // current longitudinal speed (m/s)
  const gamepadEnabledRef = useRef(true);
  const gamepadCfgRef = useRef<GamepadConfig>(useSimStore.getState().gamepadConfig);
  useEffect(() => useSimStore.subscribe((st) => {
    strategiesRef.current = st.strategies;
    holdSpeedRef.current = st.holdSpeed;
    steerReturnRef.current = st.steerReturn;
    cruiseOnRef.current = st.cruiseOn;
    cruiseSpeedRef.current = st.cruiseSpeed;
    vMaxRef.current = st.state?.params.v_max ?? 20;
    vxRef.current = st.state?.velocity.vx ?? 0;
    gamepadEnabledRef.current = st.gamepadEnabled;
    gamepadCfgRef.current = st.gamepadConfig;
    // A bump in zeroRequest = UI asked us to zero the persistent targets.
    if (st.zeroRequest !== zeroReqRef.current) {
      zeroReqRef.current = st.zeroRequest;
      throttle.current = 0;
      brake.current = 0;
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
      // Releasing a direction key re-arms it for a fresh from-stop engagement.
      if (e.code === "KeyW") fwdKeyArmed.current = true;
      if (e.code === "KeyS") revKeyArmed.current = true;
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

  // Resolve (W-held, S-held, current vx) → (throttle target, brake target,
  // direction). This is the single source of truth for longitudinal intent;
  // the gamepad feeds the same (forwardIntent, backwardIntent) booleans.
  function resolveIntent(fwdIntent: boolean, backIntent: boolean, vx: number,
                        ): { thrTgt: number; brkTgt: number; gear: number } {
    const stopped = Math.abs(vx) < STOP_SPEED;
    // ── Direction engagement (only from a stop, on a fresh press) ──
    if (stopped) {
      if (fwdIntent && fwdKeyArmed.current) { direction.current = 1; revKeyArmed.current = true; }
      if (backIntent && revKeyArmed.current) { direction.current = -1; fwdKeyArmed.current = true; }
    }
    // Disarm the opposite key once we're committed to a direction so a held
    // opposite key keeps braking (not flipping) until released. The re-arm
    // only happens on key release (handled in keyup).
    if (direction.current === 1 && backIntent) revKeyArmed.current = false;
    if (direction.current === -1 && fwdIntent) fwdKeyArmed.current = false;

    const dir = direction.current;
    // "Rolling in the selected direction" — the case where the opposite key
    // has something to brake. Both branches ask the same question, so both
    // use movingWith: in R the car is rolling backwards (vx < 0), which is
    // movingWith, not movingAgainst. Testing movingAgainst in the reverse
    // branch made it permanently false, so W did nothing at all while
    // reversing.
    const movingWith = dir === 1 ? vx > STOP_SPEED : vx < -STOP_SPEED;

    // ── Resolve target channels ──
    if (dir === 1) {
      // Forward direction.
      if (fwdIntent) return { thrTgt: 1, brkTgt: 0, gear: 1 };          // W = drive
      if (backIntent) return { thrTgt: 0, brkTgt: movingWith ? 1 : 0, gear: 1 }; // S = brake
      return { thrTgt: 0, brkTgt: 0, gear: 1 };                          // coast
    } else {
      // Reverse direction.
      if (backIntent) return { thrTgt: 1, brkTgt: 0, gear: -1 };        // S = reverse drive
      if (fwdIntent) return { thrTgt: 0, brkTgt: movingWith ? 1 : 0, gear: -1 }; // W = brake
      return { thrTgt: 0, brkTgt: 0, gear: -1 };                         // coast
    }
  }

  // Integration loop with rAF
  useEffect(() => {
    let raf = 0;
    const step = () => {
      const now = performance.now();
      const dt = Math.min(0.1, (now - last.current) / 1000);
      last.current = now;

      const p = pressed.current;
      // Two distinct ranges — do not conflate them. `clampUnit` is for the
      // bipolar axes (steering, holonomic vx/vy/yaw: -1 = full one way,
      // +1 = full the other); `clamp01` is for the unipolar pedals
      // (throttle, brake). Collapsing both onto [0,1] and patching the
      // difference with an `x * 2 - 1` rescale at the call site sent
      // steering = -1 (full right lock) whenever the wheel was centred.
      const clampUnit = (v: number) => Math.max(-1, Math.min(1, v));
      const clamp01 = (v: number) => Math.max(0, Math.min(1, v));
      const ramp = (cur: number, tgt: number, tau: number) => {
        const alpha = 1 - Math.exp(-dt / tau);
        return cur + (tgt - cur) * alpha;
      };

      // --- Gamepad (config-driven; see input/gamepadConfig.ts) ---
      // Polled before the intent resolve so the keyboard and the pedals feed
      // ONE pass of the state machine. Stepping it twice per frame (once for
      // the keyboard, again in the push block) advanced the direction latch
      // on stale inputs.
      const cfg = gamepadCfgRef.current;
      const gp = gamepadEnabledRef.current ? firstGamepad() : null;
      const out = gp ? computeGamepadOutput(gp, cfg) : null;

      // --- Longitudinal intent (keyboard + gamepad, merged) ---
      const handbrake = p["Space"] ? 1 : 0;
      let fwdIntent = !!p["KeyW"];
      let backIntent = !!p["KeyS"];
      let thrScale = 1.0;   // gamepad can scale the drive target

      // The gamepad's signed throttle axis (>0 forward, <0 back) feeds the
      // SAME intent state machine as W/S, so a wheel's pedals behave
      // identically to the keyboard.
      if (out && !cruiseOnRef.current && out.throttle !== 0) {
        if (out.throttle > 0) fwdIntent = true;
        else backIntent = true;
        thrScale = Math.abs(out.throttle);
      }

      if (cruiseOnRef.current) {
        // Fixed-speed cruise overrides intent: command throttle = target/v_max.
        const vmax = vMaxRef.current || 20;
        throttle.current = clamp01(cruiseSpeedRef.current / vmax);
        brake.current = 0;
      } else if (holdSpeedRef.current) {
        // Hold-speed: W/S nudge a persistent target speed, release = hold.
        // Kept alongside the intent state machine because the ControlPanel
        // still offers the toggle; the direction latch supplies the gear so
        // S past zero rolls into reverse instead of clamping at a standstill.
        if (fwdIntent) throttle.current = clamp01(throttle.current + HOLD_RATE * dt);
        if (backIntent) throttle.current = clamp01(throttle.current - HOLD_RATE * dt);
        if (throttle.current <= 0 && backIntent) direction.current = -1;
        if (throttle.current <= 0 && fwdIntent) direction.current = 1;
        // Space is the parking brake now, so the old "Space = quick stop"
        // shortcut is gone; the brake channel carries it instead.
        brake.current = ramp(brake.current, handbrake ? 1 : 0, TAU_RAMP);
        if (brake.current < 0.005) brake.current = 0;
      } else {
        const { thrTgt, brkTgt } = resolveIntent(fwdIntent, backIntent, vxRef.current);
        throttle.current = ramp(throttle.current, thrTgt * thrScale,
                                thrTgt === 0 ? TAU_RETURN : TAU_RAMP);
        brake.current = ramp(brake.current, brkTgt, brkTgt === 0 ? TAU_RETURN : TAU_RAMP);
        if (throttle.current < 0.005) throttle.current = 0;
        if (brake.current < 0.005) brake.current = 0;
      }

      // --- Steering ---
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

      if (now - lastPush.current >= PUSH_INTERVAL_MS) {
        const gear = direction.current;
        if (out && cfg.mode === "direct") {
          setDriver(throttle.current, 0, { wheel_norm: out.wheelNorm },
            { brake: brake.current, gear, handbrake });
        } else if (out && cfg.mode === "holonomic") {
          const b = out.body!;
          // vx_frac is signed (manual_body drives it straight into ±v_max), so
          // it must clamp to [-1,1] — clamping to [0,1] removed holonomic
          // reverse translation entirely. The keyboard contribution follows
          // the selected direction rather than always adding forwards.
          const kbd = gear < 0 ? -throttle.current : throttle.current;
          setDriver(0, 0,
            { vx_frac: clampUnit(b.vx + kbd), vy_frac: b.vy, yaw_frac: b.yaw },
            { brake: brake.current, gear, handbrake });
        } else {
          // Assisted: gamepad steering merges additively with the keyboard.
          let outSteering = steering.current;
          if (out) outSteering = clampUnit(outSteering + (out.steering ?? 0));
          setDriver(throttle.current, outSteering, undefined,
            { brake: brake.current, gear, handbrake });
        }
        lastPush.current = now;
      }

      raf = requestAnimationFrame(step);
    };
    raf = requestAnimationFrame(step);
    return () => cancelAnimationFrame(raf);
  }, []);

  return null;
}
