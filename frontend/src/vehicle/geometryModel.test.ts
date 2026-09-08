import { describe, it, expect } from "vitest";
import { axleState, vehicleGeom } from "./geometryModel";

describe("displayed turning geometry", () => {
  it("locates the body-centre circle from the outer wheel without reversing the half-track", () => {
    const a = axleState(null, 0.04);
    const L = 3.16, track = 1.565;
    const rearRadius = Math.sqrt(a.turnRadius ** 2 - (L / 2) ** 2);
    expect(Math.atan2(L, rearRadius + track / 2)).toBeCloseTo(a.outer, 12);
  });
  it("respects the wider axle when showing the symmetric-4WIS minimum radius", () => {
    const g = vehicleGeom({ track_front: 1.5, track_rear: 2.4 });
    expect(Math.atan2(g.L / 2, g.minTurnRadius - g.tr / 2)).toBeCloseTo(g.steerLimit, 12);
  });
});
