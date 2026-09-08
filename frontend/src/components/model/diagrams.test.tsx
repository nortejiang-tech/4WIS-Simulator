import { describe, expect, it } from "vitest";
import { renderToStaticMarkup } from "react-dom/server";
import { DIAGRAMS } from "./diagrams";

describe("body-frame teaching diagram", () => {
  it("locates front/left wheels along its declared X-right/Y-up axes", () => {
    const Diagram = DIAGRAMS.bodyFrame;
    const markup = renderToStaticMarkup(<Diagram />);
    const wheels = new Map<string, [number, number]>();
    for (const m of markup.matchAll(/<g[^>]*transform="translate\(([-\d.]+)[ ,]+([-\d.]+)\)[^"]*"[^>]*>[\s\S]*?<text[^>]*>(FL|FR|RL|RR)<\/text>[\s\S]*?<\/g>/g)) {
      wheels.set(m[3], [Number(m[1]), Number(m[2])]);
    }
    expect(wheels.size).toBe(4);
    const fl = wheels.get("FL")!, fr = wheels.get("FR")!;
    const rl = wheels.get("RL")!, rr = wheels.get("RR")!;
    expect(fl[0]).toBeGreaterThan(rl[0]);
    expect(fr[0]).toBeGreaterThan(rr[0]);
    expect(fl[1]).toBeLessThan(fr[1]);
    expect(rl[1]).toBeLessThan(rr[1]);
    expect(fl[0]).toBe(fr[0]);
    expect(rl[0]).toBe(rr[0]);
    const origin = markup.match(/<circle cx="([\d.]+)" cy="([\d.]+)"[^>]+class="model-dot"/)!;
    expect(Number(origin[1])).toBe((fl[0] + rl[0]) / 2);
    expect(Number(origin[2])).toBe((fl[1] + fr[1]) / 2);
  });
});
