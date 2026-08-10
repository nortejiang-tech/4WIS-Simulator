/**
 * Steering ramp — how a held A/D key becomes a steering-wheel angle.
 *
 * ## The problem with a first-order ramp
 *
 * This used to be the same exponential lag the pedals use:
 *
 *     steering += (target − steering) · (1 − e^(−dt/τ)),  τ = 0.18 s
 *
 * That shape is exactly wrong for a steering wheel. Its rate is **highest at
 * the instant of the keypress** — (1−0)/τ = 5.6 units/s, which on the default
 * 540° wheel is ≈1500 °/s, an emergency-avoidance rate — and then decays
 * asymptotically. On the first 60 Hz frame the wheel jumps 8.9% of full travel,
 * about 24° of hand wheel in one frame, before crawling the rest of the way:
 * 63% at 0.18 s, 90% at 0.41 s, 99% at 0.83 s.
 *
 * So turn-in snapped and the rest of the movement dragged. Nothing about that
 * matches a hand on a rim, which starts from rest, winds at a roughly constant
 * rate, and slows as it reaches where the driver wanted it. That mismatch is
 * the "初始转向不自然" report.
 *
 * ## What replaces it
 *
 * A rate-limited (trapezoidal) profile: the *rate* accelerates to a limit,
 * holds, and decelerates into the target — the wheel has an angular
 * acceleration rather than a step response.
 *
 *     first 60 Hz frame:  a·dt² = 0.0044 (≈1.2° of hand wheel) — 20× gentler
 *     full lock:          0.19 s ramp-up + 0.15 s at rate + 0.19 s ramp-down
 *
 * Mid-travel pace is close to what it was (0.54 vs 0.63 at 0.18 s), so the
 * keyboard is no less usable; it is only the ends of the movement that changed,
 * which is where all the artificial-feeling motion was.
 *
 * Self-centring is left as an exponential decay — that one *is* the right
 * shape, because the aligning torque falls with the angle, and its rate is the
 * user's 转向回正速度 setting.
 *
 * Pure and separately tested, like the direction-intent machine next door.
 */

/** Peak rate of change of normalised steering [1/s]. 3.0 ≈ 810 °/s at a 540° wheel. */
export const STEER_RATE_MAX = 3.0;
/** How fast the rate itself builds [1/s²] — the "hand accelerating the rim" term. */
export const STEER_ACCEL = 16;

export interface SteerRampState {
  /** Normalised steering ∈ [−1, 1]; + = left. */
  angle: number;
  /** Current rate of change [1/s], signed. */
  rate: number;
}

export function initialSteerRamp(): SteerRampState {
  return { angle: 0, rate: 0 };
}

/** Zero both the angle and its rate — used by the panel's "zero inputs" action. */
export function zeroSteerRamp(s: SteerRampState): void {
  s.angle = 0;
  s.rate = 0;
}

/**
 * One step toward a held target (±1 while A/D is down).
 *
 * The deceleration test uses only the rate *component along the direction of
 * travel*: when the driver reverses lock, the existing rate points the wrong
 * way and must be wound back through zero, not treated as an overshoot risk.
 */
export function stepSteerToward(s: SteerRampState, target: number, dt: number): void {
  const gap = target - s.angle;
  const dir = Math.sign(gap);
  const along = s.rate * dir;                       // > 0 = closing on the target
  const braking = along > 0 ? (along * along) / (2 * STEER_ACCEL) : 0;
  const wantRate = dir === 0 || Math.abs(gap) <= braking ? 0 : dir * STEER_RATE_MAX;

  const dv = STEER_ACCEL * dt;
  s.rate += Math.max(-dv, Math.min(dv, wantRate - s.rate));
  s.angle += s.rate * dt;

  // Land on the target rather than oscillating around it.
  if (dir !== 0 && Math.sign(target - s.angle) !== dir) {
    s.angle = target;
    s.rate = 0;
  }
  s.angle = Math.max(-1, Math.min(1, s.angle));
}

/**
 * One step of self-centring, with no key held.
 *
 * `rate` [1/s] is the decay constant, 0 = hold the current angle (the
 * 转向回正速度 slider's left end).
 */
export function stepSteerReturn(s: SteerRampState, rate: number, dt: number): void {
  s.rate = 0;
  if (rate <= 0) return;
  s.angle *= Math.exp(-rate * dt);
  if (Math.abs(s.angle) < 0.005) s.angle = 0;
}
