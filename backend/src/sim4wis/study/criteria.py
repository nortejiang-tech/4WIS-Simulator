"""Criteria — turning a claim into a verdict.

A study's conclusion should be decided by something written down before the
runs happened. `criteria` is that something, and this module is what makes it
binding.

Two rules shape the implementation:

* **A criterion that cannot be checked fails.** If the metric is missing from
  every selected cell, or the selector matches nothing, that is a FAIL with a
  note — not a silent PASS over an empty set. Vacuous truth is how a broken
  study reports success.
* **A failure names a cell.** "yaw_overshoot_pct must be < 20" is not
  actionable; "failed at rear_limit_deg=8 speed_kmh=140 (28.4)" is.
"""

from __future__ import annotations

import math
import re
from typing import Any

from sim4wis.study.result import Row, Verdict
from sim4wis.study.spec import Criterion, parse_must

#: `at` selectors: "all", or one or more equality filters joined by "and".
_EQ_RE = re.compile(r"^\s*([A-Za-z_][A-Za-z_0-9]*)\s*==\s*(.+?)\s*$")


class SelectorError(ValueError):
    """An `at` selector that cannot be parsed or names an unknown axis."""


def _coerce(text: str) -> Any:
    text = text.strip().strip("'\"")
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    if text.lower() in ("true", "false"):
        return text.lower() == "true"
    return text


def parse_selector(at: str) -> list[tuple[str, Any]]:
    """Parse an `at` clause into equality filters. "all" → no filters."""
    if at.strip().lower() in ("all", "*", ""):
        return []
    filters: list[tuple[str, Any]] = []
    for part in re.split(r"\s+and\s+", at.strip(), flags=re.IGNORECASE):
        m = _EQ_RE.match(part)
        if m is None:
            raise SelectorError(
                f"unparseable selector {at!r}. Expected 'all' or "
                "'axis == value' (optionally joined by 'and')"
            )
        filters.append((m.group(1), _coerce(m.group(2))))
    return filters


def _matches(row: Row, filters: list[tuple[str, Any]]) -> bool:
    for key, want in filters:
        if key not in row.coords:
            return False
        have = row.coords[key]
        if isinstance(have, float) or isinstance(want, float):
            try:
                if not math.isclose(float(have), float(want), rel_tol=1e-9, abs_tol=1e-12):
                    return False
                continue
            except (TypeError, ValueError):
                return False
        if have != want:
            return False
    return True


def _holds(value: float, use_abs: bool, op: str, threshold: float) -> bool:
    v = abs(value) if use_abs else value
    if op == "<":
        return v < threshold
    if op == "<=":
        return v <= threshold
    if op == ">":
        return v > threshold
    if op == ">=":
        return v >= threshold
    if op == "==":
        return math.isclose(v, threshold, rel_tol=1e-9, abs_tol=1e-12)
    return not math.isclose(v, threshold, rel_tol=1e-9, abs_tol=1e-12)


def evaluate(criteria: list[Criterion], rows: list[Row], axes: list[str]) -> list[Verdict]:
    """Check every criterion against the grid."""
    verdicts: list[Verdict] = []
    for c in criteria:
        try:
            filters = parse_selector(c.at)
        except SelectorError as e:
            verdicts.append(Verdict(c.metric, c.must, c.at, False, 0, 0, note=str(e)))
            continue

        unknown = [k for k, _ in filters if k not in axes]
        if unknown:
            verdicts.append(Verdict(
                c.metric, c.must, c.at, False, 0, 0,
                note=f"selector names unknown axis/axes: {', '.join(unknown)}; "
                     f"axes are {', '.join(axes) or 'none'}",
            ))
            continue

        selected = [r for r in rows if _matches(r, filters)]
        if not selected:
            verdicts.append(Verdict(
                c.metric, c.must, c.at, False, 0, 0,
                note="selector matched no cells — a criterion checked over nothing "
                     "is not satisfied",
            ))
            continue

        use_abs, op, threshold = parse_must(c.must)
        checked = 0
        failed: list[tuple[str, float]] = []
        missing = 0
        for r in selected:
            if c.metric not in r.metrics:
                missing += 1
                continue
            checked += 1
            v = r.metrics[c.metric]
            if math.isnan(v) or not _holds(v, use_abs, op, threshold):
                failed.append((r.label, v))

        if checked == 0:
            verdicts.append(Verdict(
                c.metric, c.must, c.at, False, 0, 0,
                note=f"metric {c.metric!r} is missing from all {len(selected)} selected cell(s)",
            ))
            continue

        # Worst = furthest on the wrong side of the threshold.
        worst_label = worst_value = None
        if failed:
            scored = [(abs(v) if use_abs else v, lbl, v) for lbl, v in failed]
            pick = max if op in ("<", "<=") else min
            _, worst_label, worst_value = pick(scored)

        note = f"{missing} cell(s) lacked the metric" if missing else ""
        verdicts.append(Verdict(
            metric=c.metric, must=c.must, at=c.at,
            passed=not failed, n_checked=checked, n_failed=len(failed),
            worst_label=worst_label, worst_value=worst_value, note=note,
        ))
    return verdicts
