"""Result types — the shape a study hands back.

One `Row` per grid cell. Rows carry their coordinates in the study's own
vocabulary, which is what lets grouping, criteria and the report all be written
without anyone re-parsing labels.

`StudyResult.to_summary()` is the token-budgeted view: it is what an agent
sees, and it is deliberately not the traces. One 10 s run is ~670 samples over
~35 channels; a fifteen-cell grid of those is far past any useful context
window. Traces are fetched per run, downsampled, on request.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class Row:
    """One grid cell's outcome."""

    coords: dict[str, Any]
    label: str
    run_id: str | None = None
    metrics: dict[str, float] = field(default_factory=dict)
    errors: dict[str, str] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {
            "coords": self.coords,
            "label": self.label,
            "run_id": self.run_id,
            "metrics": self.metrics,
        }
        if self.errors:
            d["errors"] = self.errors
        return d


@dataclass
class Verdict:
    """One criterion's outcome."""

    metric: str
    must: str
    at: str
    passed: bool
    n_checked: int
    n_failed: int
    #: The worst offending cell, so a failure names something concrete.
    worst_label: str | None = None
    worst_value: float | None = None
    note: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "metric": self.metric,
            "must": self.must,
            "at": self.at,
            "passed": self.passed,
            "checked": self.n_checked,
            "failed": self.n_failed,
            "worst_label": self.worst_label,
            "worst_value": self.worst_value,
            "note": self.note,
        }


@dataclass
class StudyResult:
    study: str
    question: str
    model: str
    spec_digest: str
    provenance: dict[str, Any] = field(default_factory=dict)
    metric_names: list[str] = field(default_factory=list)
    rows: list[Row] = field(default_factory=list)
    comparison: dict[str, Any] | None = None
    verdicts: list[Verdict] = field(default_factory=list)
    #: Target-set compliance, when the spec named one. See sim4wis.targets.
    compliance: dict[str, Any] | None = None
    warnings: list[str] = field(default_factory=list)
    report_path: str | None = None
    elapsed_s: float = 0.0

    def to_summary(self) -> dict[str, Any]:
        """The compact view — no traces, no channels."""
        return {
            "study": self.study,
            "question": self.question,
            "model": self.model,
            "spec_digest": self.spec_digest,
            "provenance": self.provenance,
            "metrics": self.metric_names,
            "rows": [r.to_dict() for r in self.rows],
            "comparison": self.comparison,
            "verdicts": [v.to_dict() for v in self.verdicts],
            "compliance": self.compliance,
            "warnings": self.warnings,
            "report_path": self.report_path,
            "elapsed_s": round(self.elapsed_s, 3),
            "all_passed": all(v.passed for v in self.verdicts) if self.verdicts else None,
            "compliant": (self.compliance or {}).get("verdict"),
        }
