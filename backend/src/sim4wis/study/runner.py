"""Study runner — expand, run, measure, aggregate, judge.

The orchestration is deliberately thin, because everything underneath it
already exists: expansion produces the same variant dicts `/api/batch` takes,
execution goes through `experiment/batch.run_batch_sync`, and the runs it
produces are ordinary runs that the GUI's analysis page lists like any other.
A study is a *view over runs*, not a second way to run things.

`dry_run` deserves its own note. It answers "is this spec valid, and what will
it cost" without executing anything — parsing every expression, resolving every
bind, checking every selector, and counting the grid. For an agent that is the
difference between finding out about a typo now and finding out after a few
hundred runs; for a human it is the difference between a review and a wait.
"""

from __future__ import annotations

import logging
import time
from typing import Any

from sim4wis.experiment import store as run_store
from sim4wis.experiment.batch import run_batch_sync
from sim4wis.experiment.session import (
    SCALAR_CHANNELS,
    WHEEL_CHANNELS,
    WHEELS,
    resolve_vehicle_params,
)
from sim4wis.project.params_codec import params_to_dict
from sim4wis.study import compare as compare_mod
from sim4wis.study import criteria as criteria_mod
from sim4wis.study import store as study_store
from sim4wis.study.expand import baseline_experiment, expand, grid_size, validate_binds
from sim4wis.study.metrics import BUILTIN, ExpressionError, collect, validate_expression
from sim4wis.study.result import Row, StudyResult
from sim4wis.study.spec import StudySpec

logger = logging.getLogger(__name__)

#: Rough wall-clock per simulated second, measured on the dev machine for the
#: default 5 ms step. Only ever used to warn about cost, never to decide.
_SECONDS_PER_SIM_SECOND = 0.05


def expected_channels() -> list[str]:
    """Channel names every headless run produces, without running one."""
    return list(SCALAR_CHANNELS) + [f"{b}_{w}" for b in WHEEL_CHANNELS for w in WHEELS]


def dry_run(spec: StudySpec) -> dict[str, Any]:
    """Validate a spec and estimate its cost. Runs nothing."""
    problems: list[str] = []
    warnings: list[str] = []

    solver = spec.solver_axes()
    if solver:
        problems.append(
            f"solver axes are not implemented yet: {', '.join(sorted(solver))} "
            "(see docs/agent_interface_design.md §4)"
        )

    problems.extend(validate_binds(spec))

    channels = expected_channels()
    for name in spec.builtin_metrics():
        if name not in BUILTIN:
            problems.append(
                f"unknown metric {name!r}; known: {', '.join(sorted(BUILTIN))}"
            )
    for m in spec.expr_metrics():
        try:
            validate_expression(m.expr, channels)
        except ExpressionError as e:
            problems.append(str(e))

    for c in spec.criteria:
        try:
            filters = criteria_mod.parse_selector(c.at)
        except criteria_mod.SelectorError as e:
            problems.append(str(e))
            continue
        for key, _ in filters:
            if key not in spec.sweep:
                problems.append(
                    f"criterion selector {c.at!r} names unknown axis {key!r}"
                )

    # Step-response metrics need a step-kind segment to be defined at all.
    step_metrics = {"yaw_gain_dps", "yaw_rise_time_s", "yaw_overshoot_pct", "yaw_settling_time_s"}
    wanted_step = step_metrics.intersection(spec.metric_names())
    if wanted_step:
        kinds = {s.steer.kind for s in spec.baseline.maneuver.steps}
        if "step" not in kinds:
            warnings.append(
                f"{', '.join(sorted(wanted_step))} are step-response metrics but the "
                f"baseline manoeuvre has no step-kind segment (kinds: "
                f"{', '.join(sorted(kinds)) or 'none'}) — they will be missing from every cell"
            )

    n = grid_size(spec) if not problems else 0
    sim_seconds = n * spec.baseline.maneuver.total_duration
    return {
        "ok": not problems,
        "problems": problems,
        "warnings": warnings,
        "grid": n,
        "runs": n,
        "sim_seconds": round(sim_seconds, 2),
        "est_wall_seconds": round(sim_seconds * _SECONDS_PER_SIM_SECOND, 1),
        "metrics": spec.metric_names(),
        "axes": list(spec.sweep),
        "cells": [c.label for c in expand(spec)] if not problems else [],
        "spec_digest": spec.digest(),
    }


def run_sync(spec: StudySpec, *, write_report: bool = True) -> tuple[str, StudyResult]:
    """Execute a study end to end. Returns (study_id, result)."""
    started = time.time()
    check = dry_run(spec)
    if not check["ok"]:
        raise ValueError("; ".join(check["problems"]))

    cells = expand(spec)
    base = baseline_experiment(spec)
    variants = [c.to_variant() for c in cells]

    job = run_batch_sync(base, variants)
    if job.status == "error":
        raise RuntimeError(f"batch failed: {job.error}")

    by_label = {lbl: (rid, k) for rid, lbl, k in zip(job.run_ids, job.labels, job.kpis, strict=True)}

    names = spec.metric_names()
    exprs = spec.expr_metrics()
    # Only the channels the expressions actually reference are read back, and
    # only when there are expressions at all — the CSV is the widest thing in
    # the pipeline.
    want_channels = expected_channels() if exprs else []

    rows: list[Row] = []
    warnings: list[str] = list(check["warnings"])
    for cell in cells:
        run_id, kpis = by_label.get(cell.label, (None, {}))
        t: list[float] | None = None
        channels: dict[str, list[float]] | None = None
        if exprs and run_id:
            try:
                raw = run_store.load_run_channels(run_id, want_channels)
                t = [float(x) for x in raw.pop("t", [])]
                channels = {k: v for k, v in raw.items()}
            except Exception as e:                   # noqa: BLE001 - recorded per row
                warnings.append(f"{cell.label}: could not read channels: {e}")
        mv = collect(names, exprs, kpis, t, channels)
        rows.append(Row(
            coords=cell.coords, label=cell.label, run_id=run_id,
            metrics=mv.values, errors=mv.errors,
        ))

    comparison = None
    if spec.compare is not None:
        comparison = compare_mod.build(
            rows, names, list(spec.sweep), spec.compare.group_by, spec.compare.against
        )
        warnings.extend(comparison.pop("warnings", []))

    verdicts = criteria_mod.evaluate(spec.criteria, rows, list(spec.sweep))

    resolved = params_to_dict(resolve_vehicle_params(base))
    result = StudyResult(
        study=spec.study,
        question=spec.question,
        model=spec.model,
        spec_digest=spec.digest(),
        provenance=study_store.provenance(spec, resolved),
        metric_names=names,
        rows=rows,
        comparison=comparison,
        verdicts=verdicts,
        warnings=warnings,
        elapsed_s=time.time() - started,
    )

    study_id = study_store.new_study_id(spec.study)
    if write_report and spec.report is not None:
        from sim4wis.study.report import render

        try:
            result.report_path = str(render(spec, result, study_id))
        except Exception as e:                       # noqa: BLE001 - a study is not its report
            logger.exception("Report rendering failed for study %s", spec.study)
            result.warnings.append(f"report rendering failed: {type(e).__name__}: {e}")

    study_store.save(study_id, spec, result)
    return study_id, result
