"""Rendering the compliance table.

Two entry points: a fragment that drops into a study report, and a standalone
document for a sizing pass. Both render the same table, because a requirement
means the same thing whichever run produced the number.

The table shows **margin**, not just a verdict. "Passed" and "passed with 2% of
the limit left" are different engineering situations and only one of them
survives a tolerance stack, so a column that collapses them is a column that
hides the thing worth knowing. Rows are ordered worst-first for the same
reason: what is not met is what the reader came for.
"""

from __future__ import annotations

import html
import sys
from pathlib import Path
from typing import Any

from sim4wis.targets.compliance import (
    MARGINAL,
    MET,
    NOT_EVALUATED,
    VIOLATED,
    ComplianceReport,
)

_SCRIPTS = Path(__file__).resolve().parents[4] / "scripts"
if str(_SCRIPTS) not in sys.path:                    # pragma: no cover - import plumbing
    sys.path.insert(0, str(_SCRIPTS))

try:                                                 # pragma: no cover - optional
    from reporting import ReportDocument, html_cell, html_table, report_section
    _HAVE_HELPERS = True
except Exception:                                    # noqa: BLE001
    _HAVE_HELPERS = False


def _raw(markup: str) -> Any:
    """A cell whose content is markup.

    `html_table` escapes cell bodies, which is the right default and the reason
    it exists; markup has to opt out explicitly. The fallback table below takes
    strings raw already, so this is a no-op there.
    """
    return html_cell(markup, raw=True) if _HAVE_HELPERS else markup

#: Worst first. A compliance table is read for what failed.
_ORDER = {VIOLATED: 0, NOT_EVALUATED: 1, MARGINAL: 2, MET: 3}

_CSS = """
body{font:15px/1.65 -apple-system,BlinkMacSystemFont,"Segoe UI","PingFang SC",sans-serif;
     max-width:1180px;margin:2rem auto;padding:0 1.5rem;color:#0f172a}
h1{font-size:1.6rem;margin-bottom:.2rem} h2{font-size:1.15rem;margin-top:2rem}
table{border-collapse:collapse;width:100%;margin:.6rem 0;font-size:13.5px}
th,td{border:1px solid #e2e8f0;padding:5px 9px;text-align:right;vertical-align:top}
th:first-child,td:first-child,th:nth-child(2),td:nth-child(2){text-align:left}
th:last-child,td:last-child{text-align:left}
th{background:#f8fafc;font-weight:600}
.met{color:#15803d;font-weight:600}.marginal{color:#b45309;font-weight:600}
.violated{color:#b91c1c;font-weight:600}.not_evaluated{color:#64748b;font-weight:600}
.verdict{font-size:1.1rem;padding:.7rem 1rem;border-radius:6px;margin:.8rem 0}
.v-compliant{background:#f0fdf4;border-left:4px solid #15803d}
.v-non_compliant{background:#fef2f2;border-left:4px solid #b91c1c}
.v-incomplete{background:#fffbeb;border-left:4px solid #f59e0b}
.note{color:#64748b;font-size:13px}
.src{color:#64748b;font-size:12px}
"""


def _table(headers: list[str], rows: list[list[Any]]) -> str:
    if _HAVE_HELPERS:
        return html_table(headers, rows)
    head = "".join(f"<th>{html.escape(str(h))}</th>" for h in headers)
    body = "".join(
        "<tr>" + "".join(f"<td>{c}</td>" for c in row) + "</tr>" for row in rows
    )
    return f"<table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table>"


def _section(title: str, body: str) -> str:
    if _HAVE_HELPERS:
        return report_section(title, body)
    return f"<section><h2>{html.escape(title)}</h2>{body}</section>"


def _num(v: float | None, unit: str = "") -> str:
    if v is None:
        return "—"
    return f"{v:.4g}{(' ' + unit) if unit and unit != '—' else ''}"


def _row_cells(d: dict[str, Any]) -> list[Any]:
    detail = html.escape(d["note"]) if d["note"] else ""
    if d["status"] in (VIOLATED, MARGINAL) and d["worst_label"]:
        where = f"最差：{html.escape(str(d['worst_label']))}"
        if d["checked"] > 1:
            where += f"（{d['violated'] or d['marginal']}/{d['checked']} 个测量点）"
        detail = where + ("；" + detail if detail else "")
    sev = "" if d["severity"] == "must" else " <span class='src'>(should)</span>"
    return [
        _raw(f"<b>{html.escape(d['id'])}</b>{sev}<br><span class='src'>"
             f"{html.escape(d['metric'])}</span>"),
        html.escape(d["at"]),
        html.escape(d["target"] or "—"),
        html.escape(d["limit"]),
        _num(d["worst_value"], d["unit"]),
        "—" if d["margin_pct"] is None else f"{d['margin_pct']:+.1f}%",
        _raw(f"<span class='{d['status']}'>{d['status_label']}</span>"),
        _raw(detail or "—"),
    ]


def _margins_cells(a: dict[str, Any]) -> list[Any]:
    def fmt(v: float | None) -> str:
        return "—" if v is None else f"{v:.4g}"

    def fmt_pct(v: float | None) -> str:
        return "—" if v is None else f"{v:+.1f}%"

    flipped = "; ".join(
        e for e in (a.get("flip_low_entries") or []) + (a.get("flip_high_entries") or [])
    ) or "—"
    fitted = "—"
    if a.get("fitted") is not None:
        state = {True: "在存活区间内", False: "<b>超出存活区间</b>", None: "—"}[
            a.get("fitted_inside")]
        fitted = f"{fmt(a['fitted'])}（{state}）"
    return [
        html.escape(a["axis"]),
        _num(a["nominal"]),
        _raw(f"{fmt(a['flip_low'])}<br><span class='src'>{fmt_pct(a['flip_low_pct'])}</span>"),
        _raw(f"{fmt(a['flip_high'])}<br><span class='src'>{fmt_pct(a['flip_high_pct'])}</span>"),
        _raw(fitted),
        html.escape(flipped),
    ]


def render_margins(margins: dict[str, Any]) -> str:
    """The parameter-space margin section (C5/C3c).

    Sits next to the compliance table and answers the question that one
    cannot: not "how close is the number" but "how wrong may the parameters
    be before this verdict is gone".
    """
    rows = [_margins_cells(a) for a in margins.get("axes", [])]
    table = _table(
        ["参数轴", "标称值", "向下翻转", "向上翻转", "标定值 (C3b)", "翻转的条目"],
        rows,
    )
    nominal = margins.get("nominal_violated") or []
    legend = (
        "<p class='note'>单轴扰动下的二分：参数从标称值向边界推进，"
        "「违反的强制条目集合」首次变化的交叉点。↓/↑ 无翻转（—）表示在该方向整个"
        f"边界内判定签名不变。当前判定已违反：{html.escape('、'.join(nominal) or '无')}。"
        "翻转条目标明去向：<b>→ 达标</b>才是合规，"
        "<b>→ 未评估</b>是测量失去可信度（运行超出作动器能力、被拒绝判定）——"
        "违反消失了，但什么也没被证明。联合扰动（两个参数同时错）不在此表的承诺范围内。</p>"
    )
    return _section("参数空间余量", table + legend)


def render_fragment(report: ComplianceReport | dict[str, Any]) -> str:
    """The compliance section, for embedding in a larger report.

    Takes the report's dict form as readily as the object. That is deliberate:
    a stored study renders the verdict it was given, not one re-derived from a
    target library that may have been edited since — a compliance record that
    silently re-computes is a compliance record nobody can audit.
    """
    d = report.to_dict() if isinstance(report, ComplianceReport) else report
    c = d["counts"]
    coverage = d["coverage"]
    head = (
        f"<div class='verdict v-{d['verdict']}'>"
        f"<b>{html.escape(d['verdict_label'])}</b> —— 目标集 "
        f"<code>{html.escape(d['ref'])}</code>"
        f"（{html.escape(d['applies_to'])}）<br>"
        f"<span class='note'>{c['must']} 条强制要求，已评估 "
        f"{c['must_evaluated']}（覆盖 {coverage:.0%}）；"
        f"超标 {c['violated']}，边际 {c['marginal']}，未评估 {c['not_evaluated']}。"
        f"数据来源：{html.escape('、'.join(d.get('measured_from') or []) or '无')}"
        f"　digest <code>{html.escape(str(d['digest']))}</code></span></div>"
    )
    rows = sorted(d["rows"], key=lambda r: (_ORDER[r["status"]], r["id"]))
    table = _table(
        ["需求", "工况", "目标", "限值", "实测", "余量", "判定", "说明"],
        [_row_cells(r) for r in rows],
    )
    legend = (
        "<p class='note'>余量为距限值的相对裕度：单边限值取未用掉的比例，"
        "带状要求取半带宽的未用比例（带中心 100%，带边 0%）；限值为 0 或等式要求时无定义，留空。"
        "<b>未评估</b>不是通过 —— 它表示这条要求本次没有测到，或测量值不可信。</p>"
    )
    sources = _table(
        ["需求", "依据", "理由"],
        [[r["id"], r["source"], r["rationale"] or "—"] for r in d["rows"]],
    )
    return _section("目标符合性", head + table + legend + _section("目标依据", sources))


def render_html(report: ComplianceReport | dict[str, Any], *,
                title: str = "", notes: str = "",
                margins: dict[str, Any] | None = None) -> str:
    """A standalone compliance document."""
    d = report.to_dict() if isinstance(report, ComplianceReport) else report
    heading = title or f"目标符合性报告 — {d['ref']}"
    body = f"<h1>{html.escape(heading)}</h1>"
    if notes:
        body += f"<p class='note'>{html.escape(notes)}</p>"
    body += (
        "<p class='note'>本结果由 4WIS Simulator 产生，经解析闭式解与内部一致性验证，"
        "golden 基线逐位可复现；<b>尚未与实车或台架数据做相关性验证</b>。"
        "目标带本身的依据见下表「目标依据」一栏。</p>"
    )
    body += render_fragment(d)
    if margins:
        body += render_margins(margins)
    if _HAVE_HELPERS:
        return ReportDocument(title=heading, styles=_CSS).render(body)
    return (
        "<!doctype html><html lang='zh'><head><meta charset='utf-8'>"
        f"<title>{html.escape(heading)}</title><style>{_CSS}</style></head>"
        f"<body>{body}</body></html>"
    )


def render(report: ComplianceReport | dict[str, Any], path: Path, **kw: Any) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render_html(report, **kw), encoding="utf-8")
    return path
