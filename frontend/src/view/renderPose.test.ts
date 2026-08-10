/**
 * Tests — render-clock pose reconstruction.
 *
 * These pin the actual complaint: driving in the roof camera looked like it
 * hitched several times a second. The mechanism is a 66.7 Hz state stream
 * sampled by a display that does not run at 66.7 Hz, so the assertions are
 * about *frame-to-frame* motion, not about any single pose being right.
 *
 * `rawWorst` in each case is what the old renderer did — read
 * `state.pose` straight into the transform — and is asserted alongside the
 * reconstruction so the numbers stay honest if the constants are retuned.
 */

import { describe, expect, it } from "vitest";
import type { SimStateMessage } from "@/types/sim";
import { createPoseTrack } from "./renderPose";

const PUSH_DT = 0.015;      // backend: round(dt_push/dt_sim)=3 steps at 200 Hz
const FRAME_DT = 1 / 60;    // a 60 Hz display

/** Minimal state frame — the tracker only reads t, pose, velocity, wheels. */
function frame(t: number, x: number, y: number, psi: number,
               vx: number, vy: number, r: number, delta = 0): SimStateMessage {
  return {
    t,
    pose: { x, y, psi },
    velocity: { vx, vy, yaw_rate: r },
    wheels: [0, 1, 2, 3].map(() => ({ delta })),
  } as unknown as SimStateMessage;
}

/**
 * Drive a straight line at `v` and return, per rendered frame, how far the
 * pose moved and how far it ended up from the true position.
 */
function runStraight(v: number, frames: number) {
  const track = createPoseTrack();
  let simT = 0;
  let simX = 0;
  let latest = frame(simT, simX, 0, 0, v, 0, 0);

  const steps: number[] = [];
  const errors: number[] = [];
  const rawSteps: number[] = [];
  let prev: number | null = null;
  let prevRaw: number | null = null;

  for (let i = 1; i <= frames; i++) {
    const now = i * FRAME_DT;
    // Deliver every backend sample whose time has come.
    while (simT + PUSH_DT <= now) {
      simT += PUSH_DT;
      simX += v * PUSH_DT;
      latest = frame(simT, simX, 0, 0, v, 0, 0);
    }
    const p = track.sample(latest, now, FRAME_DT);
    if (prev !== null) steps.push(p.x - prev);
    if (prevRaw !== null) rawSteps.push(latest.pose.x - prevRaw);
    prev = p.x;
    prevRaw = latest.pose.x;
    errors.push(Math.abs(p.x - v * now));
  }
  const settled = (a: number[]) => a.slice(Math.floor(a.length / 2));
  const spread = (a: number[]) => Math.max(...a) - Math.min(...a);
  return {
    stepSpread: spread(settled(steps)),
    rawStepSpread: spread(settled(rawSteps)),
    maxError: Math.max(...settled(errors)),
    nominalStep: v * FRAME_DT,
  };
}

describe("straight-line motion", () => {
  it("removes the sample-boundary steps a 66.7 Hz stream leaves on a 60 Hz display", () => {
    const r = runStraight(5, 240);   // 5 m/s = 18 km/h
    // Raw sampling alternates one-sample and two-sample frames: the per-frame
    // displacement swings by a whole sample period of travel (7.5 cm).
    expect(r.rawStepSpread).toBeGreaterThan(0.5 * r.nominalStep);
    // Reconstructed motion is near-uniform — under 2% of a frame's travel.
    expect(r.stepSpread).toBeLessThan(0.02 * r.nominalStep);
  });

  it("does not trade the judder for lag", () => {
    // Extrapolating one τ ahead cancels the smoother's own lag, so the drawn
    // car sits on the true position rather than trailing it.
    expect(runStraight(5, 240).maxError).toBeLessThan(0.05);   // < 5 cm at 18 km/h
    expect(runStraight(25, 240).maxError).toBeLessThan(0.25);  // < 25 cm at 90 km/h
  });
});

describe("cornering", () => {
  it("follows a steady turn without drifting off the circle", () => {
    const track = createPoseTrack();
    const v = 10;
    const r = 0.5;                    // rad/s → 20 m radius
    let simT = 0;
    let psi = 0;
    let x = 0;
    let y = 0;
    let worst = 0;

    for (let i = 1; i <= 300; i++) {
      const now = i * FRAME_DT;
      while (simT + PUSH_DT <= now) {
        // Same integration order the backend uses: velocity, then pose, then ψ.
        x += PUSH_DT * v * Math.cos(psi);
        y += PUSH_DT * v * Math.sin(psi);
        psi += PUSH_DT * r;
        simT += PUSH_DT;
      }
      const p = track.sample(frame(simT, x, y, psi, v, 0, r), now, FRAME_DT);
      // Distance from the true circle centre (0, 20) must stay at the radius.
      const rad = Math.hypot(p.x - 0, p.y - v / r);
      if (i > 60) worst = Math.max(worst, Math.abs(rad - v / r));
    }
    expect(worst).toBeLessThan(0.15);   // 15 cm on a 20 m radius
  });
});

describe("degenerate frame times", () => {
  // Found the hard way: the renderer's first frame reports dt = 0, which made
  // the lag term dt·(1−α)/α a 0/0. The NaN went straight into the roof
  // camera's position and blanked the viewport for the rest of the session —
  // silently, with no console error.
  it("survives a zero-length first frame", () => {
    const track = createPoseTrack();
    const p = track.sample(frame(0, 0, 0, 0, 5, 0, 0), 0, 0);
    expect(Number.isFinite(p.x)).toBe(true);
    expect(Number.isFinite(p.y)).toBe(true);
    expect(Number.isFinite(p.psi)).toBe(true);
    // …and stays finite once it is running.
    for (let i = 1; i <= 10; i++) {
      const q = track.sample(frame(i * PUSH_DT, i * 0.075, 0, 0, 5, 0, 0), i * FRAME_DT, 0);
      expect(Number.isFinite(q.x)).toBe(true);
    }
  });

  it("survives a frame time long enough to be a tab switch", () => {
    const track = createPoseTrack();
    track.sample(frame(0, 0, 0, 0, 5, 0, 0), 0, FRAME_DT);
    const p = track.sample(frame(PUSH_DT, 0.075, 0, 0, 5, 0, 0), 30, 30);
    expect(Number.isFinite(p.x)).toBe(true);
    expect(p.delta.every(Number.isFinite)).toBe(true);
  });
});

describe("discontinuities", () => {
  it("snaps on a reset instead of sliding across the map", () => {
    const track = createPoseTrack();
    for (let i = 1; i <= 60; i++) {
      track.sample(frame(i * PUSH_DT, i * 0.075, 0, 0, 5, 0, 0), i * FRAME_DT, FRAME_DT);
    }
    // Reset: sim time rewinds and the pose returns to the origin.
    const p = track.sample(frame(0, 0, 0, 0, 0, 0, 0), 61 * FRAME_DT, FRAME_DT);
    expect(Math.abs(p.x)).toBeLessThan(1e-9);
    expect(Math.abs(p.psi)).toBeLessThan(1e-9);
  });

  it("coasts on the last sample when the stream stalls, then holds", () => {
    const track = createPoseTrack();
    const stalled = frame(1.0, 10, 0, 0, 5, 0, 0);
    let last = 0;
    for (let i = 1; i <= 60; i++) last = track.sample(stalled, i * FRAME_DT, FRAME_DT).x;
    // Bounded by EXTRAP_MAX + TAU: never more than ~0.6 m of invented travel.
    expect(last).toBeGreaterThan(10);
    expect(last).toBeLessThan(10.7);
  });
});

describe("steer angle", () => {
  it("ramps between samples instead of stepping once per packet", () => {
    const track = createPoseTrack();
    const RATE = 3.3;                  // rad/s at the road wheel — brisk turn-in
    const steps: number[] = [];
    const rawSteps: number[] = [];
    const errors: number[] = [];
    let prev = 0;
    let prevRaw = 0;
    let simT = 0;
    let latest = frame(0, 0, 0, 0, 0, 0, 0, 0);
    for (let i = 1; i <= 120; i++) {
      const now = i * FRAME_DT;
      while (simT + PUSH_DT <= now) {
        simT += PUSH_DT;
        latest = frame(simT, 0, 0, 0, 0, 0, 0, RATE * simT);
      }
      const p = track.sample(latest, now, FRAME_DT);
      if (i > 40) {
        steps.push(p.delta[0] - prev);
        rawSteps.push(latest.wheels[0].delta - prevRaw);
        errors.push(Math.abs(p.delta[0] - RATE * now));
      }
      prev = p.delta[0];
      prevRaw = latest.wheels[0].delta;
    }
    const nominal = RATE * FRAME_DT;
    const spread = (a: number[]) => Math.max(...a) - Math.min(...a);
    // Raw δ jumps a whole sample's worth on some frames and not at all on others.
    expect(spread(rawSteps)).toBeGreaterThan(0.5 * nominal);
    // Reconstructed δ advances by a near-constant amount every frame. The
    // steady pattern spans ~2.4%; the bound leaves room for the occasional
    // frame where 15 ms and 1/60 s drift far enough apart that a different
    // number of samples lands (worst measured 7.7%, against ±100% raw).
    expect(spread(steps)).toBeLessThan(0.1 * nominal);
    // … and the rate estimated across samples keeps it on the true angle,
    // rather than trailing it by τ the way plain smoothing would (0.066 rad).
    expect(Math.max(...errors)).toBeLessThan(0.01);
  });
});
