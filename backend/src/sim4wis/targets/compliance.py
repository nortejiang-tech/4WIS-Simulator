"""Compliance — one table that answers "does this configuration meet its targets".

The evaluator is small. What it is careful about is the difference between
three outcomes that a naive implementation collapses into two:

    VIOLATED        we measured it and it is outside the limit
    MARGINAL        inside the limit, outside what we were aiming for
    NOT EVALUATED   we did not measure it, or the measurement is not trustworthy

The third is the one that matters most and is the easiest to lose. A target set
covers a whole product; any one study or sizing pass exercises part of it.
Scoring the untested part as FAIL makes the table useless and it will be
ignored; scoring it as PASS makes the table a lie. So coverage is a first-class
output and the set-level verdict distinguishes **non-compliant** (something
failed) from **incomplete** (something was never checked). An engineering
review runs on that distinction; a green tick that means "we did not look" is
how requirements get signed off on nothing.

Ordering of the set verdict is deliberate: a violation outranks a gap. If
something failed, that is the headline, whatever else is missing.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from sim4wis.study.criteria import SelectorError, matches_coords, parse_selector
from sim4wis.targets.measure import Measurement
from sim4wis.targets.spec import Band, TargetEntry, TargetSet

MET = "met"
MARGINAL = "marginal"
VIOLATED = "violated"
NOT_EVALUATED = "not_evaluated"

#: Rendered labels, kept next to the constants so the table and the API cannot
#: drift apart.
STATUS_LABEL = {
    MET: "达标",
    MARGINAL: "边际",
    VIOLATED: "超标",
    NOT_EVALUATED: "未评估",
}


@dataclass
class ComplianceRow:
    """One requirement's outcome."""

    entry: TargetEntry
    status: str
    n_checked: int = 0
    n_violated: int = 0
    n_marginal: int = 0
    #: The tightest cell — smallest headroom against the limit.
    worst_label: str | None = None
    worst_value: float | None = None
    worst_margin: float | None = None
    note: str = ""

    @property
    def blocking(self) -> bool:
        return self.entry.severity == "must"

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.entry.id,
            "metric": self.entry.metric,
            "at": self.entry.at,
            "unit": self.entry.unit,
            "severity": self.entry.severity,
            "limit": self.entry.limit_band.describe(),
            "target": (self.entry.target_band.describe()
                       if self.entry.target_band else None),
            "status": self.status,
            "status_label": STATUS_LABEL[self.status],
            "checked": self.n_checked,
            "violated": self.n_violated,
            "marginal": self.n_marginal,
            "worst_label": self.worst_label,
            "worst_value": self.worst_value,
            "margin_pct": (None if self.worst_margin is None
                           else round(self.worst_margin * 100, 1)),
            "source": self.entry.source,
            "rationale": self.entry.rationale,
            "note": self.note,
        }


@dataclass
class ComplianceReport:
    """The table, plus the verdict it adds up to."""

    target_set: str
    version: int
    digest: str
    applies_to: str
    rows: list[ComplianceRow] = field(default_factory=list)
    measured_from: list[str] = field(default_factory=list)

    # ---- verdict ------------------------------------------------------------

    def _must(self) -> list[ComplianceRow]:
        return [r for r in self.rows if r.blocking]

    @property
    def n_must(self) -> int:
        return len(self._must())

    @property
    def n_must_evaluated(self) -> int:
        return sum(1 for r in self._must() if r.status != NOT_EVALUATED)

    @property
    def coverage(self) -> float:
        must = self._must()
        return (self.n_must_evaluated / len(must)) if must else 1.0

    @property
    def verdict(self) -> str:
        must = self._must()
        if any(r.status == VIOLATED for r in must):
            return "non_compliant"
        if any(r.status == NOT_EVALUATED for r in must):
            return "incomplete"
        return "compliant"

    @property
    def verdict_label(self) -> str:
        return {
            "compliant": "符合",
            "non_compliant": "不符合",
            "incomplete": "覆盖不全",
        }[self.verdict]

    def to_dict(self) -> dict[str, Any]:
        return {
            "target_set": self.target_set,
            "version": self.version,
            "ref": f"{self.target_set}@{self.version}",
            "digest": self.digest,
            "applies_to": self.applies_to,
            "measured_from": self.measured_from,
            "verdict": self.verdict,
            "verdict_label": self.verdict_label,
            "coverage": round(self.coverage, 3),
            "counts": {
                "total": len(self.rows),
                "must": self.n_must,
                "must_evaluated": self.n_must_evaluated,
                "violated": sum(1 for r in self.rows if r.status == VIOLATED),
                "marginal": sum(1 for r in self.rows if r.status == MARGINAL),
                "not_evaluated": sum(1 for r in self.rows if r.status == NOT_EVALUATED),
            },
            "rows": [r.to_dict() for r in self.rows],
        }


def _evaluate_entry(entry: TargetEntry, pool: list[Measurement]) -> ComplianceRow:
    try:
        filters = parse_selector(entry.at)
    except SelectorError as e:
        return ComplianceRow(entry, NOT_EVALUATED, note=str(e))

    same_metric = [m for m in pool if m.metric == entry.metric]
    if not same_metric:
        return ComplianceRow(
            entry, NOT_EVALUATED,
            note=f"没有 {entry.metric!r} 的测量值 —— 本次运行未产生该指标",
        )

    # Naming a coordinate nothing carries is a different mistake from a filter
    # that simply excluded every cell, and the fix is different too, so say
    # which one it is.
    available: set[str] = set()
    for m in same_metric:
        available.update(m.at)
    unknown = [k for k, _ in filters if k not in available]
    if unknown:
        return ComplianceRow(
            entry, NOT_EVALUATED,
            note=(f"工况选择器引用了本次运行不存在的维度：{', '.join(unknown)}；"
                  f"可用维度：{', '.join(sorted(available)) or '无'}"),
        )

    selected = [m for m in same_metric if matches_coords(m.at, filters)]
    if not selected:
        return ComplianceRow(
            entry, NOT_EVALUATED,
            note=f"工况 {entry.at!r} 未匹配到任何测量点",
        )

    trusted = [m for m in selected if m.trusted]
    if not trusted:
        reasons = sorted({m.untrusted_reason for m in selected if m.untrusted_reason})
        return ComplianceRow(
            entry, NOT_EVALUATED, n_checked=0,
            note="测量值不可信，不予判定：" + "；".join(reasons),
        )

    limit: Band = entry.limit_band
    target: Band | None = entry.target_band

    violated: list[Measurement] = []
    marginal: list[Measurement] = []
    worst: tuple[float, Measurement] | None = None
    worst_undef: Measurement | None = None

    for m in trusted:
        inside_limit = limit.contains(m.value)
        if not inside_limit:
            violated.append(m)
        elif target is not None and not target.contains(m.value):
            marginal.append(m)
        margin = limit.margin(m.value)
        if margin is None:
            if not inside_limit and worst_undef is None:
                worst_undef = m
        elif worst is None or margin < worst[0]:
            worst = (margin, m)

    if violated:
        status = VIOLATED
    elif marginal:
        status = MARGINAL
    else:
        status = MET

    note = ""
    skipped = len(selected) - len(trusted)
    if skipped:
        note = f"{skipped} 个测量点因不可信被排除"

    if worst is not None:
        worst_margin, worst_m = worst[0], worst[1]
    elif worst_undef is not None:
        worst_margin, worst_m = None, worst_undef
    else:
        worst_margin, worst_m = None, trusted[0]

    return ComplianceRow(
        entry=entry, status=status, n_checked=len(trusted),
        n_violated=len(violated), n_marginal=len(marginal),
        worst_label=worst_m.label or worst_m.source,
        worst_value=worst_m.value, worst_margin=worst_margin, note=note,
    )


def evaluate(target_set: TargetSet, measurements: list[Measurement]) -> ComplianceReport:
    """Check every requirement in the set against everything measured."""
    report = ComplianceReport(
        target_set=target_set.name,
        version=target_set.version,
        digest=target_set.digest(),
        applies_to=target_set.applies_to,
        measured_from=sorted({m.source.split(":")[0] for m in measurements if m.source}),
    )
    report.rows = [_evaluate_entry(e, measurements) for e in target_set.entries]
    return report
