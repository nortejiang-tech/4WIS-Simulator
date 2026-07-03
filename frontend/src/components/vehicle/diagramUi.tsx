/** Shared readout + flag chrome for the vehicle-geometry diagrams. */

import type { Flag } from "@/vehicle/geometryModel";

export function Flags({ flags }: { flags: Flag[] }) {
  if (flags.length === 0) {
    return <div className="vg-flags"><span className="vg-flag ok">✓ 几何合理</span></div>;
  }
  return (
    <div className="vg-flags">
      {flags.map((f, i) => (
        <span key={i} className={`vg-flag ${f.level}`}>{f.level === "bad" ? "✕" : "⚠"} {f.text}</span>
      ))}
    </div>
  );
}

export function Readouts({ items }: { items: [string, string][] }) {
  return (
    <div className="vg-readouts">
      {items.map(([k, v]) => (
        <div key={k} className="vg-readout"><span className="vg-rk">{k}</span><span className="vg-rv">{v}</span></div>
      ))}
    </div>
  );
}

export function DiagramCard({ title, hint, children }: {
  title: string; hint?: string; children: React.ReactNode;
}) {
  return (
    <section className="vg-card">
      <div className="vg-card-head">
        <span className="vg-card-title">{title}</span>
        {hint && <span className="vg-card-hint">{hint}</span>}
      </div>
      {children}
    </section>
  );
}
