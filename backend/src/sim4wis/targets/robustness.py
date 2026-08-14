"""Parameter-space margins (C5 · C3c) — how wrong can the parameters get
before a verdict flips.

The compliance table already reports margin in **output space**: the measured
value against its limit. That answers "how close is this number" but not the
question a review actually argues about: *the parameters are engineering
estimates — how far can they be off before this PASS becomes a FAIL?* This
module answers it in **parameter space**: one axis at a time, the steering
parameter is pushed away from its nominal value until the set of violated
must-entries changes, and the crossing point is bisected to a few digits.

What "flips" is deliberately the **violation signature** — the set of
must-entries read as violated — not the set-level verdict. A met→marginal
wobble is not a flip; a violation appearing or disappearing is, in either
direction: for a compliant set the interesting flip is a failure appearing
(how much slack the parameters have), for today's eps_actuator it is a
failure disappearing (how much motor — or how little friction — it would
take).

Method and its limits. Each axis is probed alone, along the ray from nominal
toward its bound; the crossing is found by bisection, so what is reported is
a crossing the bisection brackets, with the invariant that the signature at
the inside endpoint matches nominal and the outside endpoint does not.
Non-monotone signatures (saturation regimes can do this) still yield an
honest crossing, not necessarily the nearest one. Joint perturbations — two
parameters wrong at once — are out of scope for a first version and are what
the calibration workbench's residuals are for.

The C3c link: a 3.1b fit record (`sim4wis calibrate fit --json`) carries
identified values for some of these axes. `annotate_with_fit` reads it and
marks, per axis, whether the identified value sits inside the interval where
the verdict survives — turning "the parameters are estimates" into a
comparison the review can act on: a margin that does not cover the
calibration shift is a margin that has already been spent.
"""

from __future__ import annotations

import dataclasses
import json
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from sim4wis.core.state import VehicleParams
from sim4wis.steering.sizing import SizingScenario, size_actuator
from sim4wis.targets import compliance as compliance_mod
from sim4wis.targets import measure as measure_mod
from sim4wis.targets.spec import TargetSet


@dataclass(frozen=True)
class RobustAxis:
    """One steering scalar the verdict is stressed against."""

    #: Dotted name as used by the calibration workbench (e.g.
    #: "rack.coulomb_friction_n"), so a fit record and a margin report speak
    #: about the same quantity without a translation table.
    name: str
    section: str
    field: str
    lo: float
    hi: float


#: The default stress set: the motor capability side (what a supplier
#: tolerance moves) and the load side (what the calibration workbench
#: identifies). Bounds bracket plausible hardware, matching the fittable set
#: in calibration.identify where they overlap.
DEFAULT_AXES: tuple[RobustAxis, ...] = (
    RobustAxis("motor.peak_torque", "motor", "peak_torque", 2.0, 20.0),
    RobustAxis("motor.continuous_torque", "motor", "continuous_torque", 1.0, 12.0),
    RobustAxis("motor.no_load_speed_rpm", "motor", "no_load_speed_rpm",
               2000.0, 20000.0),
    RobustAxis("rack.coulomb_friction_n", "rack", "coulomb_friction_n", 0.0, 1200.0),
    RobustAxis("rack.viscous_n_per_mps", "rack", "viscous_n_per_mps", 100.0, 3000.0),
    RobustAxis("column.torsion_stiffness_nm_per_deg", "column",
               "torsion_stiffness_nm_per_deg", 0.5, 6.0),
)


def axis_value(params: VehicleParams, axis: RobustAxis) -> float:
    sub = getattr(params.steering_system, axis.section)
    return float(getattr(sub, axis.field))


def with_axis_value(params: VehicleParams, axis: RobustAxis,
                    value: float) -> VehicleParams:
    """A copy of the params with one steering scalar replaced."""
    sub = getattr(params.steering_system, axis.section)
    return dataclasses.replace(
        params,
        steering_system=dataclasses.replace(
            params.steering_system,
            **{axis.section: dataclasses.replace(sub, **{axis.field: float(value)})},
        ),
    )


def violation_signature(report: compliance_mod.ComplianceReport) -> frozenset[str]:
    """The must-entries read as violated — the thing whose change is a flip."""
    return frozenset(
        r.entry.id for r in report.rows
        if r.blocking and r.status == compliance_mod.VIOLATED
    )


def _statuses_at_flip(report: compliance_mod.ComplianceReport,
                      changed: frozenset[str]) -> list[str]:
    """What the changed entries became at the flip point.

    A violation can leave the signature two ways: the entry got met, or the
    measurement stopped being trusted and the entry got not_evaluated. Only
    the first is compliance; the second is the run giving up. The margin
    table has to say which, or "lower the motor and the failures disappear"
    becomes advice.
    """
    return sorted(
        f"{r.entry.id} → {compliance_mod.STATUS_LABEL[r.status]}"
        for r in report.rows if r.entry.id in changed
    )


@dataclass
class AxisMargin:
    axis: str
    nominal: float
    bounds: tuple[float, float]
    #: Value searching *down* where the signature changed; None if it never
    # did within bounds.
    flip_low: float | None = None
    #: Entries that appeared or disappeared in the signature there.
    flip_low_entries: list[str] | None = None
    flip_high: float | None = None
    flip_high_entries: list[str] | None = None
    #: C3b annotation: identified value and whether the verdict survives it.
    fitted: float | None = None
    fitted_inside: bool | None = None
    n_evals: int = 0

    def to_dict(self) -> dict[str, Any]:
        def pct(flip: float | None) -> float | None:
            if flip is None or abs(self.nominal) < 1e-12:
                return None
            return round((flip - self.nominal) / abs(self.nominal) * 100, 1)

        return {
            "axis": self.axis,
            "nominal": round(self.nominal, 4),
            "bounds": list(self.bounds),
            "flip_low": None if self.flip_low is None else round(self.flip_low, 4),
            "flip_low_pct": pct(self.flip_low),
            "flip_low_entries": self.flip_low_entries or [],
            "flip_high": None if self.flip_high is None else round(self.flip_high, 4),
            "flip_high_pct": pct(self.flip_high),
            "flip_high_entries": self.flip_high_entries or [],
            "fitted": self.fitted,
            "fitted_inside": self.fitted_inside,
        }

    @property
    def verdict_interval(self) -> tuple[float, float]:
        """Where the signature holds: the flips, or the bounds if it never
        flipped."""
        return (self.flip_low if self.flip_low is not None else self.bounds[0],
                self.flip_high if self.flip_high is not None else self.bounds[1])


@dataclass
class RobustnessReport:
    target_set: str
    axes: list[AxisMargin] = dataclasses.field(default_factory=list)
    nominal_violated: list[str] = dataclasses.field(default_factory=list)
    n_evals: int = 0
    wall_s: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "target_set": self.target_set,
            "nominal_violated": self.nominal_violated,
            "n_evals": self.n_evals,
            "wall_s": round(self.wall_s, 1),
            "axes": [a.to_dict() for a in self.axes],
        }


def axis_margins(
    params: VehicleParams,
    target_set: TargetSet,
    axes: tuple[RobustAxis, ...] = DEFAULT_AXES,
    *,
    scenarios: tuple[SizingScenario, ...] | None = None,
    bisect_steps: int = 6,
    progress: Callable[[str, int], None] | None = None,
) -> RobustnessReport:
    """Bisect each axis in both directions for the violation-signature flip.

    Every evaluation is a full sizing pass and compliance check on a params
    copy with one axis moved; `scenarios` narrows the pass (tests use one
    worst case) and `bisect_steps` trades precision for passes.
    """

    def evaluate_at(p: VehicleParams) -> compliance_mod.ComplianceReport:
        req = size_actuator(p, scenarios) if scenarios is not None else size_actuator(p)
        return compliance_mod.evaluate(target_set, measure_mod.from_sizing(req))

    def signature(p: VehicleParams) -> frozenset[str]:
        return violation_signature(evaluate_at(p))

    t0 = time.perf_counter()
    nominal_sig = signature(params)
    report = RobustnessReport(
        target_set=f"{target_set.name}@{target_set.version}",
        nominal_violated=sorted(nominal_sig),
    )
    n_evals = 1

    def probe(direction: int, axis: RobustAxis, nominal: float
              ) -> tuple[float | None, list[str], int]:
        """Bisect toward the bound until the signature changes.

        Invariant: `inside` carries the nominal signature, `outside` a
        different one. Returns (flip value or None, entries with the status
        they took at the flip, evals).
        """
        bound = axis.hi if direction > 0 else axis.lo
        if signature(with_axis_value(params, axis, bound)) == nominal_sig:
            return None, [], 1
        inside_v, outside_v = nominal, bound
        spent = 1
        for _ in range(bisect_steps):
            mid = 0.5 * (inside_v + outside_v)
            if signature(with_axis_value(params, axis, mid)) == nominal_sig:
                inside_v = mid
            else:
                outside_v = mid
            spent += 1
        flipped_report = evaluate_at(with_axis_value(params, axis, outside_v))
        changed = nominal_sig ^ violation_signature(flipped_report)
        return outside_v, _statuses_at_flip(flipped_report, changed), spent + 1

    for axis in axes:
        nominal = axis_value(params, axis)
        margin = AxisMargin(axis=axis.name, nominal=nominal,
                            bounds=(axis.lo, axis.hi))
        margin.flip_low, margin.flip_low_entries, spent_lo = probe(-1, axis, nominal)
        margin.flip_high, margin.flip_high_entries, spent_hi = probe(+1, axis, nominal)
        margin.n_evals = spent_lo + spent_hi
        n_evals += margin.n_evals
        report.axes.append(margin)
        if progress is not None:
            progress(axis.name, n_evals)
    report.n_evals = n_evals
    report.wall_s = time.perf_counter() - t0
    return report


def annotate_with_fit(report: RobustnessReport, fit_record_path: str | Path) -> int:
    """Mark each axis with its identified value and whether the verdict
    survives it. Returns the number of axes annotated."""
    record = json.loads(Path(fit_record_path).read_text(encoding="utf-8"))
    fitted = {name: float(p["fitted"])
              for name, p in (record.get("params") or {}).items()}
    n = 0
    for margin in report.axes:
        if margin.axis not in fitted:
            continue
        margin.fitted = fitted[margin.axis]
        lo, hi = margin.verdict_interval
        margin.fitted_inside = lo <= margin.fitted <= hi
        n += 1
    return n
