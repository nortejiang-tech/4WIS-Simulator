/**
 * Tests — steering ramp shape.
 *
 * The complaint these pin is about *turn-in*, so the assertions are mostly
 * about the first few frames after the key goes down, plus the properties that
 * make the profile trapezoidal rather than exponential.
 *
 * `OLD_FIRST_FRAME` is what the replaced first-order ramp did, kept as the
 * reference the change is measured against.
 */

import { describe, expect, it } from "vitest";
import {
  STEER_ACCEL,
  STEER_RATE_MAX,
  initialSteerRamp,
  stepSteerReturn,
  stepSteerToward,
  zeroSteerRamp,
} from "./steerRamp";

const DT = 1 / 60;
/** 1 − e^(−dt/0.18) — the old ramp's first frame, 8.9% of full travel. */
const OLD_FIRST_FRAME = 1 - Math.exp(-DT / 0.18);

function hold(target: number, seconds: number, s = initialSteerRamp()) {
  const trace: number[] = [];
  for (let t = 0; t < seconds; t += DT) {
    stepSteerToward(s, target, DT);
    trace.push(s.angle);
  }
  return { s, trace };
}

describe("turn-in", () => {
  it("starts from rest instead of stepping", () => {
    const { trace } = hold(1, 0.02);
    // Semi-implicit Euler: rate reaches a·dt first, so the first frame covers
    // a·dt² — 0.44% of full travel, about 1.2° at a 540° wheel.
    expect(trace[0]).toBeCloseTo(STEER_ACCEL * DT * DT, 6);
    // The whole point: ~20× smaller than the old first-order ramp's first frame.
    expect(trace[0]).toBeLessThan(OLD_FIRST_FRAME / 15);
  });

  it("accelerates — the second frame moves further than the first", () => {
    const { trace } = hold(1, 0.06);
    const steps = trace.map((v, i) => v - (i ? trace[i - 1] : 0));
    expect(steps[1]).toBeGreaterThan(steps[0]);
    expect(steps[2]).toBeGreaterThan(steps[1]);
  });

  it("holds a constant rate through the middle of the travel", () => {
    const { trace } = hold(1, 0.35);
    // Once the rate limit is reached (≈0.19 s) the angle advances linearly.
    const mid = trace.slice(14, 20);
    const steps = mid.map((v, i) => v - (i ? mid[i - 1] : mid[0]));
    const cruise = STEER_RATE_MAX * DT;
    steps.slice(1).forEach((d) => expect(d).toBeCloseTo(cruise, 4));
  });

  it("is still quick enough to drive with", () => {
    // Half lock inside a third of a second, full lock inside 0.6 s.
    expect(hold(1, 0.3).s.angle).toBeGreaterThan(0.5);
    expect(hold(1, 0.6).s.angle).toBeCloseTo(1, 3);
  });
});

describe("reaching the stop", () => {
  it("decelerates into full lock without overshoot or oscillation", () => {
    const { s, trace } = hold(1, 1.0);
    expect(s.angle).toBeLessThanOrEqual(1);
    expect(Math.max(...trace)).toBeLessThanOrEqual(1);
    // Monotone — a first-order ramp is monotone too, but a rate-limited one
    // without the braking term would ring around the target.
    trace.slice(1).forEach((v, i) => expect(v).toBeGreaterThanOrEqual(trace[i]));
    expect(s.rate).toBeCloseTo(0, 6);
  });
});

describe("reversing lock", () => {
  it("winds the rate back through zero rather than jumping", () => {
    const { s } = hold(1, 0.25);            // moving left at the rate limit
    expect(s.rate).toBeGreaterThan(0);
    const rates: number[] = [];
    for (let i = 0; i < 30; i++) {
      stepSteerToward(s, -1, DT);
      rates.push(s.rate);
    }
    // The rate crosses zero once and never changes by more than a·dt.
    rates.forEach((r, i) => {
      const prev = i ? rates[i - 1] : STEER_RATE_MAX;
      expect(Math.abs(r - prev)).toBeLessThanOrEqual(STEER_ACCEL * DT + 1e-9);
    });
    expect(rates[rates.length - 1]).toBeLessThan(0);
  });
});

describe("self-centring", () => {
  it("decays toward zero and clears the residual", () => {
    const { s } = hold(1, 0.6);
    for (let i = 0; i < 300; i++) stepSteerReturn(s, 6, DT);
    expect(s.angle).toBe(0);
    expect(s.rate).toBe(0);
  });

  it("holds the angle when the return rate is zero", () => {
    const { s } = hold(1, 0.3);
    const held = s.angle;
    for (let i = 0; i < 60; i++) stepSteerReturn(s, 0, DT);
    expect(s.angle).toBe(held);
  });

  it("drops the stored rate so the next press starts from rest", () => {
    const { s } = hold(1, 0.25);
    stepSteerReturn(s, 4, DT);
    expect(s.rate).toBe(0);
  });
});

describe("zeroing", () => {
  it("clears angle and rate together", () => {
    const { s } = hold(1, 0.3);
    zeroSteerRamp(s);
    expect(s.angle).toBe(0);
    expect(s.rate).toBe(0);
  });
});
