"""Default sweep report.

Deliberately modest. The two research reports in this repository are ~2 200
lines of bespoke rendering on top of 126 lines of shared helpers, and that
ratio is not an accident: the structure of a real report *is* part of the
research. Trying to generate one automatically produces something that looks
like a report and argues nothing.

So this renders what is mechanically true and nothing more — the question, the
provenance, the grid, the deltas, and the verdicts — and a bespoke
`build_report.py` remains the way to write an actual paper, now with the study
layer as its data source instead of a private harness.

One rule is enforced rather than trusted: **the report states no conclusion the
study did not measure.** The only sentences that read as findings are the
criteria, which were written before the runs.
"""

from __future__ import annotations

import html
import math
import sys
from pathlib import Path
from typing import Any

from sim4wis.paths import studies_dir
from sim4wis.study.result import StudyResult
from sim4wis.study.spec import StudySpec

# scripts/ is not a package; the shared helpers live there next to the studies
# that use them.
_SCRIPTS = Path(__file__).resolve().parents[4] / "scripts"
if str(_SCRIPTS) not in sys.path:                    # pragma: no cover - import plumbing
    sys.path.insert(0, str(_SCRIPTS))

try:                                                 # pragma: no cover - optional
    from reporting import ReportDocument, html_table, report_section
    _HAVE_HELPERS = True
except Exception:                                    # noqa: BLE001
    _HAVE_HELPERS = False


def _fmt(v: Any, digits: int = 4) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        if math.isnan(v):
            return "NaN"
        return f"{v:.{digits}g}"
    return html.escape(str(v))


def _table(headers: list[str], rows: list[list[Any]]) -> str:
    if _HAVE_HELPERS:
        return html_table(headers, rows)
    head = "".join(f"<th>{html.escape(str(h))}</th>" for h in headers)
    body = "".join(
        "<tr>" + "".join(f"<td>{r if isinstance(r, str) else html.escape(str(r))}</td>"
                         for r in row) + "</tr>"
        for row in rows
    )
    return f"<table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table>"


def _section(title: str, body: str) -> str:
    if _HAVE_HELPERS:
        return report_section(title, body)
    return f"<section><h2>{html.escape(title)}</h2>{body}</section>"


_CSS = """
body{font:15px/1.65 -apple-system,BlinkMacSystemFont,"Segoe UI","PingFang SC",sans-serif;
     max-width:1100px;margin:2rem auto;padding:0 1.5rem;color:#0f172a}
h1{font-size:1.6rem;margin-bottom:.2rem} h2{font-size:1.15rem;margin-top:2rem}
.q{color:#334155;font-size:1.05rem;margin:.2rem 0 1.2rem}
table{border-collapse:collapse;width:100%;margin:.6rem 0;font-size:13.5px}
th,td{border:1px solid #e2e8f0;padding:5px 9px;text-align:right}
th:first-child,td:first-child{text-align:left}
th{background:#f8fafc;font-weight:600}
.pass{color:#15803d;font-weight:600}.fail{color:#b91c1c;font-weight:600}
.meta{color:#64748b;font-size:12.5px;font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
.warn{background:#fffbeb;border-left:3px solid #f59e0b;padding:.5rem .8rem;margin:.5rem 0}
.note{color:#64748b;font-size:13px}
"""


def render_html(spec: StudySpec, result: StudyResult) -> str:
    parts: list[str] = []

    prov = result.provenance
    parts.append(_section("出处", _table(
        ["字段", "值"],
        [
            ["study", result.study],
            ["spec_digest", f"<code>{_fmt(result.spec_digest)}</code>"],
            ["模型", result.model],
            ["策略", _fmt(prov.get("strategy"))],
            ["版本", _fmt(prov.get("sim4wis_version"))],
            ["git", f"<code>{_fmt(prov.get('git_sha'))}</code>"],
            ["params_hash", f"<code>{_fmt(prov.get('params_hash'))}</code>"],
            ["dt / 记录率", f"{_fmt(prov.get('dt'))} s / {_fmt(prov.get('record_hz'))} Hz"],
            ["生成时间", _fmt(prov.get("created_at"))],
        ],
    )))

    # Every report says what its numbers are worth. Cheap to add, and the
    # alternative is a reader assuming correlation that does not exist.
    parts.append(_section("适用边界", (
        "<p class='note'>本结果由 4WIS Simulator 产生，经解析闭式解与内部一致性验证，"
        "golden 基线逐位可复现；<b>尚未与实车或台架数据做相关性验证</b>。"
        "启用转向系统被控对象层时，其参数为工程估计值而非实测硬件。</p>"
    )))

    axes = list(spec.sweep)
    headers = [*axes, "run", *result.metric_names]
    rows: list[list[Any]] = []
    for r in result.rows:
        cells: list[Any] = [_fmt(r.coords.get(a)) for a in axes]
        cells.append(f"<span class='meta'>{_fmt(r.run_id)}</span>")
        for m in result.metric_names:
            cells.append(_fmt(r.metrics.get(m)) if m in r.metrics else "—")
        rows.append(cells)
    parts.append(_section("结果", _table(headers, rows)))

    cmp_block = result.comparison or {}
    if cmp_block.get("groups"):
        gb = cmp_block["group_by"]
        gheaders = [gb, "n", *[f"{m} (均值 / 极差)" for m in result.metric_names]]
        grows: list[list[Any]] = []
        for g in cmp_block["groups"]:
            row: list[Any] = [_fmt(g[gb]), g["n"]]
            for m in result.metric_names:
                s = g["metrics"].get(m)
                row.append("—" if s is None else f"{_fmt(s['mean'])} / {_fmt(s['spread'])}")
            grows.append(row)
        parts.append(_section(f"按 {gb} 分组", _table(gheaders, grows)))

    if cmp_block.get("deltas"):
        ref = ", ".join(f"{k}={v}" for k, v in (cmp_block.get("against") or {}).items())
        dheaders = ["cell", *[f"Δ {m}" for m in result.metric_names],
                    *[f"Δ% {m}" for m in result.metric_names]]
        drows: list[list[Any]] = []
        for d in cmp_block["deltas"]:
            row: list[Any] = [d["label"]]
            for m in result.metric_names:
                e = d["metrics"].get(m)
                row.append("—" if e is None else _fmt(e["delta"]))
            for m in result.metric_names:
                e = d["metrics"].get(m)
                row.append("—" if e is None or e["delta_pct"] is None
                           else f"{e['delta_pct']:+.1f}%")
            drows.append(row)
        parts.append(_section(
            f"相对基准（{ref}）",
            "<p class='note'>Δ% 在基准为 0 时无定义，留空。</p>" + _table(dheaders, drows),
        ))

    if result.verdicts:
        vrows = []
        for v in result.verdicts:
            mark = "<span class='pass'>PASS</span>" if v.passed else "<span class='fail'>FAIL</span>"
            detail = v.note
            if not v.passed and v.worst_label:
                detail = (f"最差：{html.escape(v.worst_label)} = {_fmt(v.worst_value)}"
                          + (f"；{detail}" if detail else ""))
            vrows.append([
                f"{html.escape(v.metric)} {html.escape(v.must)}",
                html.escape(v.at), mark, f"{v.n_failed}/{v.n_checked}", detail or "—",
            ])
        parts.append(_section("判据", "<p class='note'>判据在运行之前写定，是本报告中唯一"
                                      "作为结论的陈述。</p>"
                              + _table(["判据", "范围", "结果", "失败/检查", "说明"], vrows)))

    errs = [(r.label, m, e) for r in result.rows for m, e in r.errors.items()]
    if errs:
        parts.append(_section("缺失的测量", _table(
            ["cell", "指标", "原因"],
            [[html.escape(a), html.escape(b), html.escape(c)] for a, b, c in errs[:60]],
        )))

    if result.warnings:
        warn = "".join(f"<div class='warn'>{html.escape(w)}</div>" for w in result.warnings)
        parts.append(_section("告警", warn))

    body = (
        f"<h1>{html.escape(result.study)}</h1>"
        f"<p class='q'>{html.escape(result.question)}</p>"
        + "".join(parts)
    )

    if _HAVE_HELPERS:
        return ReportDocument(title=result.study, styles=_CSS).render(body)
    return (
        "<!doctype html><html lang='zh'><head><meta charset='utf-8'>"
        f"<title>{html.escape(result.study)}</title><style>{_CSS}</style></head>"
        f"<body>{body}</body></html>"
    )


def render(spec: StudySpec, result: StudyResult, study_id: str) -> Path:
    """Write the report and return its path."""
    out = spec.report.out if (spec.report and spec.report.out) else None
    path = Path(out) if out else studies_dir() / study_id / "report.html"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render_html(spec, result), encoding="utf-8")
    return path
