import { useState } from "react";
import { renderInlineMath } from "./latex";
import type { ExplanationContent } from "./ChartBox";
import "./MetricInfo.css";
import "./LoadExplanation.css";

interface Props {
  explanation: ExplanationContent;
}

/** Compact ⓘ button that opens a per-metric explanation modal with KaTeX
 *  rendering — used in the KPI strip and the real-time wheel panel. */
export function MetricInfo({ explanation }: Props) {
  const [open, setOpen] = useState(false);
  return (
    <>
      <button
        className="metric-info-btn"
        onClick={(e) => { e.stopPropagation(); setOpen(true); }}
        title={`原理：${explanation.title}`}
        aria-label="原理说明"
      >
        ⓘ
      </button>
      {open && <MetricInfoModal explanation={explanation} onClose={() => setOpen(false)} />}
    </>
  );
}

function MetricInfoModal({ explanation, onClose }: { explanation: ExplanationContent; onClose: () => void }) {
  const containerRef = (el: HTMLDivElement | null) => {
    if (el) renderInlineMath(el);
  };
  return (
    <div className="load-explanation-backdrop" onClick={onClose}>
      <div className="load-explanation-modal" onClick={(e) => e.stopPropagation()}>
        <header>
          <h3>{explanation.title}</h3>
          <button onClick={onClose} aria-label="关闭">✕</button>
        </header>
        <div className="load-explanation-body" ref={containerRef}>
          {explanation.sections.map((sec, idx) => (
            <section key={idx}>
              {sec.heading && <h4>{sec.heading}</h4>}
              <div className="load-explanation-text" dangerouslySetInnerHTML={{ __html: sec.body }} />
              {sec.figure && (
                <div className="load-explanation-figure" dangerouslySetInnerHTML={{ __html: sec.figure }} />
              )}
            </section>
          ))}
        </div>
      </div>
    </div>
  );
}
