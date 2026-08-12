"""StudySpec — one research question, declared.

The whole point of this schema is that a study becomes a *file*: reviewable,
diffable, re-runnable, and attachable to the conclusion it produced. A study
that only exists as a script is a study nobody can check.

Relationship to `Experiment`: a StudySpec **contains** one, as `baseline`. The
sweep then produces concrete Experiments from it by dotted-path override — the
same mechanism `experiment/batch.py` already uses — so everything downstream
(SimSession, KPIs, the run store, the GUI's analysis page) is untouched and a
study's runs are ordinary runs.

Two fields carry most of the design intent:

`sweep[*].bind`
    Research variables and Experiment paths are not the same vocabulary. The
    question is about "rear authority"; the Experiment field is
    `vehicle.overrides.rear_steer_limit_deg`. Binding once keeps every table,
    grouping and criterion in the reader's vocabulary instead of the schema's.

`criteria`
    The anti-hallucination field. It forces the conclusion to be written in
    falsifiable form *before* the runs happen, and turns it into PASS/FAIL
    afterwards — rather than letting a narrative be assembled from whatever the
    numbers turned out to be.
"""

from __future__ import annotations

import hashlib
import json
import re
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from sim4wis.experiment.schema import Experiment
from sim4wis.vehicle.model_registry import is_known_model, model_ids

# `must` clauses: an optional abs(), a comparison, a number.
#   "< 20"   ">= 0.5"   "abs < 1.0"   "== 0"
_MUST_RE = re.compile(r"^\s*(abs\s+)?(<=|>=|<|>|==|!=)\s*(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*$")


class SolveFor(BaseModel):
    """A sweep axis whose values are solved for rather than listed.

    Declared in the schema from day one so specs written now stay valid, but
    the runner refuses it until the solver phase lands — see
    docs/agent_interface_design.md §4 for why this is harder than a bisection
    (a_y is not monotone in steer angle past the friction peak).
    """

    model_config = ConfigDict(protected_namespaces=())

    metric: str
    target: float
    tol: float = Field(0.02, gt=0.0)


class SweepAxis(BaseModel):
    """One dimension of the grid.

    `bind` is the dotted path into the baseline Experiment's JSON form that
    this axis writes, exactly as `experiment/batch.py` expects.
    """

    model_config = ConfigDict(protected_namespaces=())

    values: list[Any] | None = None
    bind: str = Field(..., min_length=1)
    solve_for: SolveFor | None = None
    bracket: tuple[float, float] | None = None
    max_iter: int = Field(12, ge=1, le=100)
    cache: str | None = None

    @model_validator(mode="after")
    def _one_source(self) -> SweepAxis:
        if (self.values is None) == (self.solve_for is None):
            raise ValueError("a sweep axis needs exactly one of `values` or `solve_for`")
        if self.values is not None and not self.values:
            raise ValueError("`values` must not be empty")
        if self.solve_for is not None and self.bracket is None:
            raise ValueError("`solve_for` requires a `bracket` to search within")
        return self


class ExprMetric(BaseModel):
    """A derived metric written as an expression over a run's channels.

    The safe tier: no statements, no imports, no attribute access — see
    `metrics.py`. Anything this cannot express (correlation-based phase
    estimation, injected-noise sensitivity) needs the plugin tier.
    """

    model_config = ConfigDict(protected_namespaces=())

    name: str = Field(..., min_length=1, max_length=64)
    expr: str = Field(..., min_length=1)
    unit: str = ""
    description: str = ""


class CompareSpec(BaseModel):
    """Cross-run aggregation: how to group the grid and what to measure against."""

    model_config = ConfigDict(protected_namespaces=())

    group_by: str | None = None
    #: Coordinates selecting the reference cell(s), e.g. {"rear_limit_deg": 2}.
    against: dict[str, Any] | None = None


class Criterion(BaseModel):
    """A falsifiable claim about the result.

    `must` is a small comparison language rather than a Python expression on
    purpose: a criterion that can run arbitrary code is a criterion nobody can
    review at a glance.

    `at` selects which cells must satisfy it — "all", or an equality filter on
    sweep coordinates such as "speed_kmh == 140".
    """

    model_config = ConfigDict(protected_namespaces=())

    metric: str = Field(..., min_length=1)
    must: str = Field(..., min_length=1)
    at: str = "all"

    @model_validator(mode="after")
    def _parse_must(self) -> Criterion:
        if not _MUST_RE.match(self.must):
            raise ValueError(
                f"unparseable `must` clause: {self.must!r}. "
                "Expected e.g. '< 20', '>= 0.5', 'abs < 1.0'"
            )
        return self


class ReportSpec(BaseModel):
    model_config = ConfigDict(protected_namespaces=())

    template: Literal["sweep"] = "sweep"
    out: str | None = None


class StudySpec(BaseModel):
    """The complete declaration of one study."""

    model_config = ConfigDict(protected_namespaces=())

    study: str = Field(..., min_length=1, max_length=64, pattern=r"^[A-Za-z0-9_\-]+$")
    question: str = Field(..., min_length=1)
    #: Required and without a default — a default is how an agent ends up
    #: asking the kinematic model a dynamics question without noticing.
    model: str
    baseline: Experiment
    sweep: dict[str, SweepAxis] = Field(default_factory=dict)
    metrics: list[str | ExprMetric] = Field(default_factory=list)
    compare: CompareSpec | None = None
    criteria: list[Criterion] = Field(default_factory=list)
    #: A target set to judge this study's results against, as `name` or
    #: `name@version`.
    #:
    #: Distinct from `criteria`, and the distinction is the point. A criterion
    #: belongs to this question and is thrown away with it; a target belongs to
    #: the product, is versioned and sourced, and is the same document every
    #: other study is judged against. Both can be present: the criteria say
    #: whether the study answered its question, the targets say whether the
    #: configuration is acceptable. See sim4wis.targets.
    targets: str | None = None
    report: ReportSpec | None = None

    # ---- validation --------------------------------------------------------

    @model_validator(mode="after")
    def _check(self) -> StudySpec:
        if not is_known_model(self.model):
            raise ValueError(f"unknown model {self.model!r}; known: {', '.join(model_ids())}")

        # The study's model wins, but silently overriding an explicitly set
        # baseline.model_type would make the spec lie about what it ran.
        if "model_type" in self.baseline.model_fields_set and self.baseline.model_type != self.model:
            raise ValueError(
                f"study declares model {self.model!r} but baseline.model_type is "
                f"{self.baseline.model_type!r} — remove one of them"
            )

        if not self.metrics:
            raise ValueError("a study with no metrics measures nothing")

        names = self.metric_names()
        dupes = {n for n in names if names.count(n) > 1}
        if dupes:
            raise ValueError(f"duplicate metric name(s): {', '.join(sorted(dupes))}")

        for c in self.criteria:
            if c.metric not in names:
                raise ValueError(
                    f"criterion refers to metric {c.metric!r}, which the study does not compute"
                )

        if self.compare is not None:
            if self.compare.group_by is not None and self.compare.group_by not in self.sweep:
                raise ValueError(
                    f"compare.group_by={self.compare.group_by!r} is not a sweep axis; "
                    f"axes: {', '.join(sorted(self.sweep)) or 'none'}"
                )
            for key in (self.compare.against or {}):
                if key not in self.sweep:
                    raise ValueError(f"compare.against refers to unknown axis {key!r}")
        return self

    # ---- helpers -----------------------------------------------------------

    def metric_names(self) -> list[str]:
        return [m if isinstance(m, str) else m.name for m in self.metrics]

    def expr_metrics(self) -> list[ExprMetric]:
        return [m for m in self.metrics if isinstance(m, ExprMetric)]

    def builtin_metrics(self) -> list[str]:
        return [m for m in self.metrics if isinstance(m, str)]

    def solver_axes(self) -> list[str]:
        return [k for k, ax in self.sweep.items() if ax.solve_for is not None]

    def digest(self) -> str:
        """Stable hash of the spec — the identity a conclusion is pinned to.

        Sorted keys so a reordered YAML is the same study, and truncated to 16
        hex chars because it is a label, not a security claim.
        """
        payload = json.dumps(self.model_dump(mode="json"), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]


def parse_must(clause: str) -> tuple[bool, str, float]:
    """Split a validated `must` clause into (use_abs, operator, threshold)."""
    m = _MUST_RE.match(clause)
    if m is None:                                    # pragma: no cover - schema guards this
        raise ValueError(f"unparseable `must` clause: {clause!r}")
    return bool(m.group(1)), m.group(2), float(m.group(3))
