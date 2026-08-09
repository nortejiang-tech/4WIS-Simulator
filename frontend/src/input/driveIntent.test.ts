/**
 * Tests — direction-intent state machine.
 *
 * This file exists because the front end had no unit tests at all, and three
 * of the five blocking defects found in the v0.100 review lived in exactly
 * this logic. Each of those is pinned below by name.
 */

import { describe, expect, it } from "vitest";
import {
  initialIntentState,
  releaseBackward,
  releaseForward,
  resolveIntent,
  shapePedal,
  STOP_SPEED,
} from "./driveIntent";

const MOVING_FWD = 10;
const MOVING_REV = -10;
const STOPPED = 0;

describe("forward driving", () => {
  it("W drives forward from a standstill", () => {
    const s = initialIntentState();
    expect(resolveIntent(s, true, false, STOPPED)).toEqual({ thrTgt: 1, brkTgt: 0, gear: 1 });
  });

  it("S brakes while rolling forward", () => {
    const s = initialIntentState();
    expect(resolveIntent(s, false, true, MOVING_FWD)).toEqual({ thrTgt: 0, brkTgt: 1, gear: 1 });
  });

  it("releasing everything coasts", () => {
    const s = initialIntentState();
    expect(resolveIntent(s, false, false, MOVING_FWD)).toEqual({ thrTgt: 0, brkTgt: 0, gear: 1 });
  });
});

describe("reverse driving", () => {
  /** Drive to a stop, release S, press it again — the documented way into R. */
  function engageReverse() {
    const s = initialIntentState();
    resolveIntent(s, false, true, MOVING_FWD);   // S braking
    resolveIntent(s, false, true, STOPPED);      // still held at rest
    releaseBackward(s);                          // let go
    resolveIntent(s, false, true, STOPPED);      // fresh press
    return s;
  }

  it("engages R only on a fresh press after stopping", () => {
    const s = engageReverse();
    expect(s.direction).toBe(-1);
  });

  it("does NOT drop into reverse from a held brake", () => {
    // The whole point of the arming latch: holding S to stop the car must
    // leave it in D, or every hard stop ends up in reverse.
    const s = initialIntentState();
    resolveIntent(s, false, true, MOVING_FWD);
    resolveIntent(s, false, true, 1.0);
    resolveIntent(s, false, true, STOPPED);
    expect(s.direction).toBe(1);
  });

  it("S drives backwards once R is engaged", () => {
    const s = engageReverse();
    expect(resolveIntent(s, false, true, MOVING_REV)).toEqual({ thrTgt: 1, brkTgt: 0, gear: -1 });
  });

  it("W BRAKES while reversing — regression: reverse used the wrong predicate", () => {
    // The reverse branch tested "moving against the gear", which is
    // permanently false in R (the car rolls backwards, i.e. WITH the gear).
    // W therefore did nothing at all while reversing.
    const s = engageReverse();
    const out = resolveIntent(s, true, false, MOVING_REV);
    expect(out.brkTgt).toBe(1);
    expect(out.thrTgt).toBe(0);
    expect(out.gear).toBe(-1);
  });

  it("W does not brake once already stopped in R", () => {
    const s = engageReverse();
    expect(resolveIntent(s, true, false, STOPPED).brkTgt).toBe(0);
  });

  it("holding W while reversing does not flip back to D", () => {
    const s = engageReverse();
    resolveIntent(s, true, false, MOVING_REV);   // braking
    resolveIntent(s, true, false, STOPPED);      // still held at rest
    expect(s.direction).toBe(-1);
  });

  it("releasing W then pressing it returns to D", () => {
    const s = engageReverse();
    resolveIntent(s, true, false, MOVING_REV);
    resolveIntent(s, true, false, STOPPED);
    releaseForward(s);
    resolveIntent(s, true, false, STOPPED);
    expect(s.direction).toBe(1);
  });
});

describe("symmetry between the two directions", () => {
  it("both branches brake under the same condition", () => {
    // Forward: S brakes while rolling forward. Reverse: W brakes while rolling
    // backward. Neither brakes at rest. Asserting the pair together is what
    // would have caught the reverse-branch defect immediately.
    const fwd = initialIntentState();
    const rev = initialIntentState();
    rev.direction = -1;

    expect(resolveIntent(fwd, false, true, MOVING_FWD).brkTgt).toBe(1);
    expect(resolveIntent(rev, true, false, MOVING_REV).brkTgt).toBe(1);

    const fwd2 = initialIntentState();
    const rev2 = initialIntentState();
    rev2.direction = -1;
    expect(resolveIntent(fwd2, false, true, STOPPED).brkTgt).toBe(0);
    expect(resolveIntent(rev2, true, false, STOPPED).brkTgt).toBe(0);
  });
});

describe("threshold behaviour", () => {
  it("does not brake below the stop threshold", () => {
    const s = initialIntentState();
    expect(resolveIntent(s, false, true, STOP_SPEED * 0.5).brkTgt).toBe(0);
  });

  it("brakes just above it", () => {
    const s = initialIntentState();
    expect(resolveIntent(s, false, true, STOP_SPEED * 1.5).brkTgt).toBe(1);
  });

  it("never returns a gear of 0 — neutral is not reachable from the keys", () => {
    const s = initialIntentState();
    for (const [f, b, v] of [
      [true, false, 0], [false, true, 0], [true, true, 5],
      [false, false, -5], [true, true, -5],
    ] as [boolean, boolean, number][]) {
      expect(Math.abs(resolveIntent(s, f, b, v).gear)).toBe(1);
    }
  });

  it("both keys at once never commands drive and brake together", () => {
    const s = initialIntentState();
    for (const v of [-10, -1, 0, 1, 10]) {
      const out = resolveIntent(s, true, true, v);
      expect(out.thrTgt * out.brkTgt).toBe(0);
    }
  });
});

describe("pedal curve", () => {
  it("is the identity at expo 0", () => {
    for (const v of [0, 0.25, 0.5, 0.75, 1]) {
      expect(shapePedal(v, 0)).toBe(v);
    }
  });

  it("keeps the endpoints and lowers the middle", () => {
    expect(shapePedal(0, 1)).toBe(0);
    expect(shapePedal(1, 1)).toBe(1);
    expect(shapePedal(0.5, 1)).toBeCloseTo(0.25, 10);
    expect(shapePedal(0.5, 1)).toBeLessThan(0.5);
  });

  it("is monotonic — more pedal always means more", () => {
    for (const expo of [0, 0.5, 1, 2]) {
      let prev = -1;
      for (let i = 0; i <= 100; i++) {
        const v = shapePedal(i / 100, expo);
        expect(v).toBeGreaterThan(prev);
        prev = v;
      }
    }
  });

  it("stays within [0,1]", () => {
    for (const expo of [0, 1, 2]) {
      for (const v of [0, 0.3, 1]) {
        const out = shapePedal(v, expo);
        expect(out).toBeGreaterThanOrEqual(0);
        expect(out).toBeLessThanOrEqual(1);
      }
    }
  });
});
