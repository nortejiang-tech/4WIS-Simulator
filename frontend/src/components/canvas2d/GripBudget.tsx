/**
 * GripBudget — the driving-time counterpart to GripPanel's friction circles.
 *
 * One stacked bar per wheel, splitting each tyre's μ·Fz into what longitudinal
 * force is spending, what lateral force is spending, and what is left. Reading
 * four circles while actually driving is too slow; a bar you can take in
 * peripherally is not, and the stack happens to be exactly the X / Y / Z
 * decomposition — the bar's *length* is μ·Fz, so it grows and shrinks with load
 * transfer just like the circle radius does.
 *
 * Note the two segments are drawn to |Fx| + |Fy|, which exceeds the resultant
 * |F| whenever both are non-zero. That is intentional: it shows how the budget
 * is being *spent* per axis. The remaining segment is sized from the true
 * resultant, so "left over" stays honest.
 */

import type { WheelState } from "@/types/sim";

const LABELS = ["FL", "FR", "RL", "RR"] as const;

export default function GripBudget({ wheels }: { wheels: WheelState[] }) {
  // Common scale: the biggest capacity on the car right now, so bar lengths
  // are comparable between wheels rather than each self-normalising.
  let refCap = 1;
  for (const w of wheels) refCap = Math.max(refCap, w.grip_capacity ?? 0);

  return (
    <div className="hud-row" style={{ flexDirection: "column", alignItems: "stretch", gap: 2 }}>
      <span className="hud-label">附着余量</span>
      {wheels.map((w, i) => {
        const cap = w.grip_capacity ?? 0;
        const util = Math.min(1, w.grip_util ?? 0);
        const beyond = !!(w.grip_beyond_peak_lat || w.grip_beyond_peak_long);
        const fx = Math.abs(w.tire_fx ?? 0);
        const fy = Math.abs(w.tire_fy ?? 0);
        const barPct = (cap / refCap) * 100;          // bar length ∝ μ·Fz
        const longPct = cap > 1 ? (fx / cap) * 100 : 0;
        const latPct = cap > 1 ? (fy / cap) * 100 : 0;
        const col = beyond ? "#f87171" : util >= 0.9 ? "#fb923c"
          : util >= 0.7 ? "#fbbf24" : "#4ade80";
        return (
          <div key={i} style={{ display: "flex", alignItems: "center", gap: 4 }}>
            <span className="hud-value hud-mono hud-small"
                  style={{ width: 20, color: "var(--muted)" }}>{LABELS[i]}</span>
            <div style={{
              flex: 1, height: 7, background: "rgba(255,255,255,0.07)",
              borderRadius: 2, overflow: "hidden",
            }}>
              <div style={{ width: `${barPct}%`, height: "100%", display: "flex",
                            background: "rgba(255,255,255,0.10)", borderRadius: 2 }}>
                <div style={{ width: `${longPct}%`, background: "#60a5fa" }}
                     title="纵向已用" />
                <div style={{ width: `${latPct}%`, background: col }}
                     title="侧向已用" />
              </div>
            </div>
            <span className="hud-value hud-mono hud-small"
                  style={{ width: 34, textAlign: "right", color: col }}>
              {beyond ? "过峰" : `${Math.round((1 - util) * 100)}%`}
            </span>
          </div>
        );
      })}
    </div>
  );
}
