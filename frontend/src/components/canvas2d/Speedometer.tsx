/**
 * Speedometer — the prominent speed readout for manual driving.
 *
 * A 240° arc gauge with the number carrying the actual reading and the arc
 * carrying the sense of "how far along the range am I". The two do different
 * jobs: the digits are what you check deliberately, the arc is what you track
 * peripherally while looking at the road.
 *
 * When a speed target is active (cruise or the hold-speed assist) it is drawn
 * as a tick on the arc plus a small label, so closing on the target is visible
 * without reading two numbers and subtracting.
 *
 * Scale note: the arc spans 0…v_max, which on the default vehicle is 200 km/h.
 * Most driving happens in the bottom third of that, which is exactly the
 * complaint behind the speed-control work — the gauge shows the problem
 * honestly rather than hiding it behind a rescaled dial.
 */

const SIZE = 132;
const R = 52;
const SWEEP = 240;                  // degrees of arc
const START = 150;                  // degrees, CSS/SVG convention (0 = +x, CW)

function polar(cx: number, cy: number, r: number, deg: number): [number, number] {
  const a = (deg * Math.PI) / 180;
  return [cx + r * Math.cos(a), cy + r * Math.sin(a)];
}

function arcPath(cx: number, cy: number, r: number, from: number, to: number): string {
  const [x0, y0] = polar(cx, cy, r, from);
  const [x1, y1] = polar(cx, cy, r, to);
  const large = Math.abs(to - from) > 180 ? 1 : 0;
  return `M ${x0} ${y0} A ${r} ${r} 0 ${large} 1 ${x1} ${y1}`;
}

export default function Speedometer({
  speedKmh, maxKmh, targetKmh, label,
}: {
  speedKmh: number;
  maxKmh: number;
  targetKmh?: number | null;
  label?: string | null;
}) {
  const c = SIZE / 2;
  const max = Math.max(maxKmh, 1);
  const frac = Math.min(1, Math.max(0, Math.abs(speedKmh) / max));
  const end = START + SWEEP * frac;

  const ticks = [];
  for (let i = 0; i <= 4; i++) {
    const deg = START + (SWEEP * i) / 4;
    const [xa, ya] = polar(c, c, R - 6, deg);
    const [xb, yb] = polar(c, c, R, deg);
    ticks.push(<line key={i} x1={xa} y1={ya} x2={xb} y2={yb}
                     stroke="var(--border)" strokeWidth={1.2} />);
  }

  let targetMark = null;
  if (targetKmh != null && targetKmh > 0) {
    const tf = Math.min(1, targetKmh / max);
    const deg = START + SWEEP * tf;
    const [xa, ya] = polar(c, c, R - 9, deg);
    const [xb, yb] = polar(c, c, R + 3, deg);
    targetMark = <line x1={xa} y1={ya} x2={xb} y2={yb}
                       stroke="#38bdf8" strokeWidth={2.2} strokeLinecap="round" />;
  }

  return (
    <div style={{ display: "flex", flexDirection: "column", alignItems: "center" }}>
      <svg width={SIZE} height={SIZE * 0.78} viewBox={`0 0 ${SIZE} ${SIZE * 0.78}`}
           role="img" aria-label={`车速 ${Math.round(speedKmh)} km/h`}>
        <path d={arcPath(c, c, R, START, START + SWEEP)} fill="none"
              stroke="var(--border)" strokeWidth={7} strokeLinecap="round" opacity={0.45} />
        {frac > 0.001 && (
          <path d={arcPath(c, c, R, START, end)} fill="none"
                stroke="var(--accent, #38bdf8)" strokeWidth={7} strokeLinecap="round" />
        )}
        {ticks}
        {targetMark}
        <text x={c} y={c + 4} textAnchor="middle"
              style={{ fontSize: 30, fontWeight: 650, fill: "var(--fg)",
                       fontVariantNumeric: "tabular-nums" }}>
          {Math.round(Math.abs(speedKmh))}
        </text>
        <text x={c} y={c + 20} textAnchor="middle"
              style={{ fontSize: 10, fill: "var(--muted)", letterSpacing: 0.5 }}>
          km/h
        </text>
      </svg>
      {label && (
        <div className="panel-small hud-mono" style={{ color: "#38bdf8", marginTop: -6 }}>
          {label}
        </div>
      )}
    </div>
  );
}
