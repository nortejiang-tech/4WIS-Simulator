/**
 * ScorePanel — automated strategy-validation metrics.
 *
 * Scans the rolling history buffer and reports quantitative scores for the
 * current run: ICR-consistency, yaw response, actuator load and tyre slip.
 * Capture the current scores into slot A or B to compare two strategies /
 * maneuvers numerically (pairs with ExcitationPanel for repeatable runs).
 *
 * Pure frontend — derived entirely from the data already in the store.
 */

import { useEffect, useState } from "react";
import { useSimStore } from "@/store/sim";
import { toKmh } from "@/ui/units";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

interface Score {
  icrPeak: number;    // max |ICR deviation| over wheels & time [m]
  icrRms: number;     // RMS ICR deviation [m]
  yawPeak: number;    // peak |yaw rate| [°/s]
  vyPeak: number;     // peak |lateral velocity| [km/h]
  energy: number;     // Σ Σ|motor torque|·dt  [N·m·s]
  rackPeak: number;   // peak |rack force| [N]
  slipPeak: number;   // peak |side-slip angle| [°]
  samples: number;    // history length used
}

const R2D = 180 / Math.PI;

function computeScore(): Score {
  const h = useSimStore.getState().history;
  const n = h.t.length;
  const s: Score = {
    icrPeak: 0, icrRms: 0, yawPeak: 0, vyPeak: 0,
    energy: 0, rackPeak: 0, slipPeak: 0, samples: n,
  };
  if (n === 0) return s;

  let icrSq = 0, icrCnt = 0;
  for (let i = 0; i < n; i++) {
    const yr = Math.abs(h.yaw_rate[i] ?? 0) * R2D;
    if (yr > s.yawPeak) s.yawPeak = yr;
    const vy = toKmh(Math.abs(h.vy[i] ?? 0));
    if (vy > s.vyPeak) s.vyPeak = vy;
    const dt = i > 0 ? Math.max(0, (h.t[i] - h.t[i - 1])) : 0;
    for (let w = 0; w < 4; w++) {
      const dev = h.wheel_icr_dev[w][i];
      if (dev != null && !Number.isNaN(dev)) {
        const a = Math.abs(dev);
        if (a > s.icrPeak) s.icrPeak = a;
        icrSq += a * a; icrCnt++;
      }
      const rf = Math.abs(h.wheel_rack_force[w][i] ?? 0);
      if (rf > s.rackPeak) s.rackPeak = rf;
      const sl = Math.abs(h.wheel_slip_alpha[w][i] ?? 0) * R2D;
      if (sl > s.slipPeak) s.slipPeak = sl;
      s.energy += Math.abs(h.wheel_motor_torque[w][i] ?? 0) * dt;
    }
  }
  s.icrRms = icrCnt > 0 ? Math.sqrt(icrSq / icrCnt) : 0;
  return s;
}

const ROWS: { key: keyof Score; label: string; unit: string; digits: number; betterLow: boolean }[] = [
  { key: "icrPeak",  label: "瞬心偏差峰值", unit: "m",     digits: 3, betterLow: true },
  { key: "icrRms",   label: "瞬心偏差 RMS", unit: "m",     digits: 3, betterLow: true },
  { key: "yawPeak",  label: "横摆角速度峰值", unit: "°/s", digits: 1, betterLow: false },
  { key: "vyPeak",   label: "侧向速度峰值", unit: "km/h",  digits: 2, betterLow: false },
  { key: "energy",   label: "转向能耗代理", unit: "N·m·s", digits: 1, betterLow: true },
  { key: "rackPeak", label: "齿条力峰值",   unit: "N",     digits: 0, betterLow: true },
  { key: "slipPeak", label: "侧偏角峰值",   unit: "°",     digits: 2, betterLow: true },
];

export default function ScorePanel() {
  const [live, setLive] = useState(true);
  const [cur, setCur] = useState<Score>(() => computeScore());
  const [slotA, setSlotA] = useState<{ s: Score; label: string } | null>(null);
  const [slotB, setSlotB] = useState<{ s: Score; label: string } | null>(null);

  // Recompute periodically while live; cheap enough at 4 Hz.
  useEffect(() => {
    if (!live) return;
    const id = window.setInterval(() => setCur(computeScore()), 250);
    return () => window.clearInterval(id);
  }, [live]);

  const refresh = () => setCur(computeScore());
  const strat = () => useSimStore.getState().state?.strategy ?? "—";
  const capture = (slot: "A" | "B") => {
    const snap = { s: computeScore(), label: strat() };
    slot === "A" ? setSlotA(snap) : setSlotB(snap);
  };

  const fmt = (v: number, d: number) => (Number.isFinite(v) ? v.toFixed(d) : "—");

  return (
    <Panel
      title="策略评分"
      help={HELP.score}
      badge={
        <>
          <label style={{ fontSize: 10, color: "var(--muted)", marginLeft: "auto",
                          display: "flex", alignItems: "center", gap: 3, cursor: "pointer" }}>
            <input type="checkbox" checked={live} onChange={(e) => setLive(e.target.checked)} />
            实时
          </label>
          {!live && (
            <button style={{ fontSize: 10 }} onClick={refresh}>刷新</button>
          )}
        </>
      }
    >

      <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 11 }}>
        <thead>
          <tr style={{ color: "var(--muted)", textAlign: "right" }}>
            <th style={{ textAlign: "left", fontWeight: 400, paddingBottom: 3 }}>指标</th>
            <th style={{ fontWeight: 400 }}>当前</th>
            <th style={{ fontWeight: 400, color: "#34d399" }}>A</th>
            <th style={{ fontWeight: 400, color: "#fb923c" }}>B</th>
          </tr>
        </thead>
        <tbody>
          {ROWS.map((r) => {
            const cv = cur[r.key] as number;
            const av = slotA?.s[r.key] as number | undefined;
            const bv = slotB?.s[r.key] as number | undefined;
            // winner highlight between A and B
            let aWin = false, bWin = false;
            if (av != null && bv != null && Number.isFinite(av) && Number.isFinite(bv) && av !== bv) {
              const aBetter = r.betterLow ? av < bv : av > bv;
              aWin = aBetter; bWin = !aBetter;
            }
            return (
              <tr key={r.key} style={{ borderTop: "1px solid var(--border)" }}>
                <td style={{ padding: "3px 0", color: "var(--text)" }}>
                  {r.label}
                  <span style={{ color: "var(--muted)", fontSize: 9 }}> {r.unit}</span>
                </td>
                <td style={{ textAlign: "right", fontFamily: "monospace" }}>{fmt(cv, r.digits)}</td>
                <td style={{ textAlign: "right", fontFamily: "monospace",
                             color: aWin ? "#34d399" : "var(--muted)" }}>
                  {av != null ? fmt(av, r.digits) : "—"}
                </td>
                <td style={{ textAlign: "right", fontFamily: "monospace",
                             color: bWin ? "#fb923c" : "var(--muted)" }}>
                  {bv != null ? fmt(bv, r.digits) : "—"}
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>

      <div style={{ display: "flex", gap: 6, marginTop: 8, flexWrap: "wrap" }}>
        <button style={{ fontSize: 11 }} onClick={() => capture("A")}>捕获为 A</button>
        <button style={{ fontSize: 11 }} onClick={() => capture("B")}>捕获为 B</button>
        <button style={{ fontSize: 11 }} disabled={!slotA && !slotB}
          onClick={() => { setSlotA(null); setSlotB(null); }}>清除 A/B</button>
      </div>

      <div className="small" style={{ marginTop: 6, color: "var(--muted)", lineHeight: 1.5 }}>
        基于最近 {cur.samples} 帧历史。
        {slotA && <span style={{ color: "#34d399" }}> A={slotA.label}</span>}
        {slotB && <span style={{ color: "#fb923c" }}> B={slotB.label}</span>}
        <br />绿/橙高亮为 A/B 中更优的一方（瞬心偏差/能耗/载荷越小越好）。
      </div>
    </Panel>
  );
}
