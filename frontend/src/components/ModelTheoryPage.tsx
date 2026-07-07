import { useEffect, useRef } from "react";

import { renderInlineMath } from "@/components/load/latex";
import {
  ADVANCED_PARAMETER_GROUPS,
  CORE_PARAMETER_GROUPS,
} from "@/vehicle/parameterGroups";
import { CHAPTERS, type Chapter } from "./model/modelChapters";
import { DIAGRAMS } from "./model/diagrams";
import { DEMOS } from "./model/demos";
import "./ModelTheoryPage.css";

const coreNames = CORE_PARAMETER_GROUPS.flatMap((g) => g.fields.map(([, label]) => label));
const advancedNames = ADVANCED_PARAMETER_GROUPS.flatMap((g) => g.fields.map(([, label]) => label));

function Html({ html, className }: { html: string; className?: string }) {
  const ref = useRef<HTMLDivElement | null>(null);
  useEffect(() => {
    if (ref.current) renderInlineMath(ref.current);
  }, [html]);
  return <div ref={ref} className={className} dangerouslySetInnerHTML={{ __html: html }} />;
}

function ChapterBlock({ chapter }: { chapter: Chapter }) {
  const Diagram = chapter.diagram ? DIAGRAMS[chapter.diagram] : null;
  const Demo = chapter.demo ? DEMOS[chapter.demo] : null;
  return (
    <section className="model-chapter" id={`ch-${chapter.id}`}>
      <header className="model-chapter-head">
        <span className="model-chapter-num">{chapter.num}</span>
        <h2>{chapter.title}</h2>
      </header>

      {Diagram && <div className="model-figure"><Diagram /></div>}

      <Html className="model-intuition" html={chapter.intuitionHtml} />

      {chapter.formulaHtml && (
        <div className="model-keyformula">
          <span className="model-tag">关键公式</span>
          <Html html={chapter.formulaHtml} />
        </div>
      )}

      {chapter.calloutHtml && <Html className="model-callout" html={chapter.calloutHtml} />}

      {chapter.derivationHtml && (
        <details className="model-derivation">
          <summary>展开推导 / 工程边界</summary>
          <Html html={chapter.derivationHtml} />
        </details>
      )}

      {Demo && (
        <div className="model-demo-wrap">
          <span className="model-tag model-tag-live">交互演示</span>
          <Demo />
        </div>
      )}
    </section>
  );
}

export default function ModelTheoryPage() {
  return (
    <main className="model-page">
      <section className="model-hero">
        <div>
          <p className="model-kicker">4WIS foundation model</p>
          <h1>四轮独立转向整车数学模型</h1>
          <p>
            从车体坐标系、单轮运动学、轮胎力、主销力矩到齿条/电机负载——这一页由浅入深讲清
            项目统一后的底层物理。每一节先给白话直觉（产品/管理也能看懂），再给关键公式，
            最后可展开教科书级推导（工程师可直接引用）。三个交互演示直接调用真实后端模型。
          </p>
        </div>
        <nav className="model-toc">
          {CHAPTERS.map((c) => (
            <a key={c.id} href={`#ch-${c.id}`}><span>{c.num}</span>{c.title}</a>
          ))}
        </nav>
      </section>

      {CHAPTERS.map((c) => <ChapterBlock key={c.id} chapter={c} />)}

      <section className="model-chapter">
        <header className="model-chapter-head">
          <span className="model-chapter-num">附</span>
          <h2>参数清单</h2>
        </header>
        <div className="model-grid two">
          <article>
            <h3>核心参数（默认可见）</h3>
            <p className="model-chip-list">
              {coreNames.map((n) => <span key={n}>{n}</span>)}
            </p>
          </article>
          <article>
            <h3>高级参数（默认折叠）</h3>
            <p className="model-chip-list">
              {advancedNames.map((n) => <span key={n}>{n}</span>)}
            </p>
          </article>
        </div>
      </section>
    </main>
  );
}
