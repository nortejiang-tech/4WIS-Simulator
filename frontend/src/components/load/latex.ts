// Q5: in-place LaTeX rendering helper for the chart explanation modal.
//
// We use KaTeX's auto-render utility to find $...$ / $$...$$ in already-
// rendered HTML and substitute MathML/CSS-rendered output. Loaded lazily so
// the load page doesn't carry a 250 KB font-bundle in initial render.

import "katex/dist/katex.min.css";
// auto-render ships only as JS; declare its signature inline.
// @ts-expect-error katex/dist/contrib/auto-render.mjs has no .d.ts
import renderMathInElement from "katex/dist/contrib/auto-render.mjs";

export function renderInlineMath(root: HTMLElement): void {
  try {
    renderMathInElement(root, {
      delimiters: [
        { left: "$$", right: "$$", display: true },
        { left: "$", right: "$", display: false },
        { left: "\\(", right: "\\)", display: false },
        { left: "\\[", right: "\\]", display: true },
      ],
      throwOnError: false,
      strict: "ignore",
    });
  } catch (e) {
    // KaTeX render failures shouldn't break the explanation panel; the raw
    // LaTeX falls through and is at least readable.
    console.warn("KaTeX render failed:", e);
  }
}
