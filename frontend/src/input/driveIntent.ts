/**
 * Direction-intent state machine — pure, so it can be tested.
 *
 * Resolves "which way does the driver want to go" from the two longitudinal
 * intents (forward / backward, whether they come from W/S, a gamepad trigger
 * or a wheel's pedals) plus the current speed, into the physical channels the
 * backend expects: throttle, brake and gear.
 *
 * Extracted out of KeyboardInput because it is where the longitudinal feel
 * actually lives, and because every bug this component has had was in here:
 * a reverse branch that tested the wrong direction predicate (so W did nothing
 * at all while reversing), and a latch that could flip into reverse from a
 * held brake. Component-local logic with no way to test it is how those
 * survived; this file exists to be exercised directly.
 *
 * The state is explicit rather than captured in refs so a test can drive it
 * step by step.
 */

export interface IntentState {
  /** Currently engaged direction: +1 forward (D), -1 reverse (R). */
  direction: 1 | -1;
  /** W may engage forward from a stop (cleared until the key is released). */
  fwdArmed: boolean;
  /** S may engage reverse from a stop. */
  revArmed: boolean;
}

export interface IntentOutput {
  /** Drive pedal ∈ {0, 1} — the ramp is applied by the caller. */
  thrTgt: number;
  /** Brake pedal ∈ {0, 1}. */
  brkTgt: number;
  /** -1 = R, +1 = D. */
  gear: number;
}

/** A vehicle is "stopped" below this speed; direction changes engage only then. */
export const STOP_SPEED = 0.3;   // m/s

export function initialIntentState(): IntentState {
  return { direction: 1, fwdArmed: true, revArmed: true };
}

/** Releasing a direction key re-arms it for a fresh from-stop engagement. */
export function releaseForward(s: IntentState): void { s.fwdArmed = true; }
export function releaseBackward(s: IntentState): void { s.revArmed = true; }

/**
 * One step of the machine. Mutates `s` and returns the resolved channels.
 *
 * Rules:
 *   * A direction only engages from a standstill, and only on a *fresh* press
 *     — holding S to stop the car must not then drop it into reverse.
 *   * Once a direction is committed, the opposite key is the brake.
 *   * Braking only makes sense while actually rolling in the engaged
 *     direction; at rest the opposite key does nothing until it re-engages.
 */
export function resolveIntent(
  s: IntentState,
  fwdIntent: boolean,
  backIntent: boolean,
  vx: number,
): IntentOutput {
  const stopped = Math.abs(vx) < STOP_SPEED;

  if (stopped) {
    if (fwdIntent && s.fwdArmed) { s.direction = 1; s.revArmed = true; }
    if (backIntent && s.revArmed) { s.direction = -1; s.fwdArmed = true; }
  }
  // Disarm the opposite key once committed, so a held opposite key keeps
  // braking rather than flipping direction until it is released.
  if (s.direction === 1 && backIntent) s.revArmed = false;
  if (s.direction === -1 && fwdIntent) s.fwdArmed = false;

  const dir = s.direction;
  // "Rolling in the engaged direction" — the case where the opposite key has
  // something to brake. BOTH branches ask this same question: in R the car
  // rolls backwards (vx < 0), which is still moving *with* the gear. Testing
  // the opposite predicate in the reverse branch made it permanently false,
  // so W did nothing whatsoever while reversing.
  const movingWith = dir === 1 ? vx > STOP_SPEED : vx < -STOP_SPEED;

  if (dir === 1) {
    if (fwdIntent) return { thrTgt: 1, brkTgt: 0, gear: 1 };
    if (backIntent) return { thrTgt: 0, brkTgt: movingWith ? 1 : 0, gear: 1 };
    return { thrTgt: 0, brkTgt: 0, gear: 1 };
  }
  if (backIntent) return { thrTgt: 1, brkTgt: 0, gear: -1 };
  if (fwdIntent) return { thrTgt: 0, brkTgt: movingWith ? 1 : 0, gear: -1 };
  return { thrTgt: 0, brkTgt: 0, gear: -1 };
}

/**
 * Throttle curve. `expo` 0 = linear; higher stretches the low end, where
 * nearly all driving happens when the axis spans 0…v_max (200 km/h by
 * default). Applied to the value sent, not to the ramp state, so the shaping
 * is a lens on the pedal rather than something the smoothing integrates.
 */
export function shapePedal(value: number, expo: number): number {
  if (expo <= 0) return value;
  return Math.pow(Math.max(0, value), 1 + expo);
}
