"""Cross-run aggregation — the part every bespoke study rewrote.

Two operations, both of which the existing studies open-code:

`group_by`
    Collapse the grid onto one axis and report, per metric, the spread across
    the other axes. This is what answers "does the ranking survive the speed
    sweep?" — and in the decoupling study the answer was that four of six
    control laws change sign inside the sweep, which a single-speed table hides
    completely.

`against`
    Express every cell relative to a reference cell, as an absolute delta and a
    percentage. "8° of authority buys 12% over 2°" is the sentence an actuator
    spec is written from; "8° gives 4.31" is not.

A note on the percentage: it is undefined when the reference is zero, and this
returns None rather than inf. A table full of `inf%` looks like a bug in the
study; a blank says "this comparison does not mean anything here".
"""

from __future__ import annotations

import math
from typing import Any

from sim4wis.study.result import Row


def _key_of(row: Row, axes: list[str]) -> tuple:
    return tuple(row.coords.get(a) for a in axes)


def _close(a: Any, b: Any) -> bool:
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
        return math.isclose(float(a), float(b), rel_tol=1e-9, abs_tol=1e-12)
    return a == b


def find_reference(rows: list[Row], against: dict[str, Any]) -> list[Row]:
    """Cells matching the reference coordinates (may be several: a slice)."""
    out = []
    for r in rows:
        if all(k in r.coords and _close(r.coords[k], v) for k, v in against.items()):
            out.append(r)
    return out


def group(rows: list[Row], group_by: str, metrics: list[str]) -> list[dict[str, Any]]:
    """Aggregate each metric over the cells sharing a `group_by` value."""
    order: list[Any] = []
    buckets: dict[Any, list[Row]] = {}
    for r in rows:
        if group_by not in r.coords:
            continue
        k = r.coords[group_by]
        if k not in buckets:
            buckets[k] = []
            order.append(k)
        buckets[k].append(r)

    out: list[dict[str, Any]] = []
    for k in order:
        members = buckets[k]
        entry: dict[str, Any] = {group_by: k, "n": len(members), "metrics": {}}
        for m in metrics:
            vals = [r.metrics[m] for r in members if m in r.metrics and not math.isnan(r.metrics[m])]
            if not vals:
                continue
            entry["metrics"][m] = {
                "mean": sum(vals) / len(vals),
                "min": min(vals),
                "max": max(vals),
                "spread": max(vals) - min(vals),
                "n": len(vals),
            }
        out.append(entry)
    return out


def deltas(
    rows: list[Row],
    reference: list[Row],
    metrics: list[str],
    axes: list[str],
    against: dict[str, Any],
) -> list[dict[str, Any]]:
    """每个 cell 相对参考的 Δ 与 Δ%.

    When `against` pins only some axes, the reference is a slice rather than a
    single cell, and each row is compared against the reference cell sharing
    its remaining coordinates. That is what makes "relative to 2° at the same
    speed" work instead of "relative to 2° at 60 km/h" for every speed.
    """
    free = [a for a in axes if a not in against]
    ref_by_free: dict[tuple, Row] = {}
    for r in reference:
        ref_by_free.setdefault(_key_of(r, free), r)

    out: list[dict[str, Any]] = []
    for r in rows:
        ref = ref_by_free.get(_key_of(r, free))
        if ref is None:
            continue
        entry: dict[str, Any] = {
            "label": r.label,
            "coords": r.coords,
            "reference_label": ref.label,
            "metrics": {},
        }
        for m in metrics:
            if m not in r.metrics or m not in ref.metrics:
                continue
            v, rv = r.metrics[m], ref.metrics[m]
            if math.isnan(v) or math.isnan(rv):
                continue
            pct = None if rv == 0 else (v - rv) / abs(rv) * 100.0
            entry["metrics"][m] = {"value": v, "reference": rv, "delta": v - rv, "delta_pct": pct}
        out.append(entry)
    return out


def build(
    rows: list[Row],
    metrics: list[str],
    axes: list[str],
    group_by: str | None,
    against: dict[str, Any] | None,
) -> dict[str, Any]:
    """The comparison block of a study result."""
    out: dict[str, Any] = {}
    warnings: list[str] = []

    if group_by:
        out["group_by"] = group_by
        out["groups"] = group(rows, group_by, metrics)

    if against:
        ref = find_reference(rows, against)
        if not ref:
            warnings.append(
                f"compare.against={against} matched no cell — no deltas computed"
            )
        else:
            out["against"] = against
            out["reference_labels"] = [r.label for r in ref]
            out["deltas"] = deltas(rows, ref, metrics, axes, against)

    if warnings:
        out["warnings"] = warnings
    return out
