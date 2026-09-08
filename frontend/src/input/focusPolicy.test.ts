import { describe, expect, it } from "vitest";
import { canDriveInput } from "./focusPolicy";

const ready = { page: "run", armed: true, online: true, visible: true, focused: true, editing: false, paused: false, scripted: false };
describe("manual input boundary", () => {
  it("allows a focused, connected driving station", () => expect(canDriveInput(ready)).toBe(true));
  it.each(["agent", "experiment", "script", "analysis", "vehicle", "scene"])("never drives from the %s workspace", page => {
    expect(canDriveInput({ ...ready, page })).toBe(false);
  });
  it.each(["armed", "online", "visible", "focused"] as const)("releases when %s is lost", key => {
    expect(canDriveInput({ ...ready, [key]: false })).toBe(false);
  });
  it.each(["editing", "paused", "scripted"] as const)("does not compete with %s", key => {
    expect(canDriveInput({ ...ready, [key]: true })).toBe(false);
  });
});
