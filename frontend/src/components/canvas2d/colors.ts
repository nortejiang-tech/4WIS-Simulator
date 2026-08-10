// Shared colour helpers for the 2D/3D views and charts.

/** Per-wheel accent colours — FL, FR, RL, RR (matches the chart series). */
export const WHEEL_COLORS = ["#60a5fa", "#a78bfa", "#f472b6", "#34d399"] as const;

/** Wheel tint by effective μ — normal slate, warming to red/indigo as grip drops. */
export function wheelMuColor(mu?: number): string {
  if (mu == null || mu >= 0.85) return "#1e293b";
  if (mu >= 0.5) return "#78350f";   // caution (dark amber)
  if (mu >= 0.3) return "#7f1d1d";   // low (dark red)
  return "#4c1d95";                  // icy (indigo)
}

/**
 * Text colour for μ readouts. The nominal case defers to the theme's text
 * token rather than a fixed near-white — the HUD panel follows the theme, and
 * a hard-coded #e2e8f0 disappeared on the light panel.
 */
export function muTextColor(mu?: number): string {
  if (mu == null || mu >= 0.85) return "var(--text, #e2e8f0)";
  if (mu >= 0.5) return "#fbbf24";
  if (mu >= 0.3) return "#f87171";
  return "#a78bfa";
}

/** Region fill by μ — high = faint blue (OK), low = red/indigo (slick/icy). */
export function muToColor(mu: number): string {
  if (mu >= 0.9) return "rgba(96,165,250,0.16)";
  if (mu >= 0.6) return "rgba(251,191,36,0.20)";
  if (mu >= 0.3) return "rgba(239,68,68,0.22)";
  return "rgba(99,102,241,0.30)";
}
