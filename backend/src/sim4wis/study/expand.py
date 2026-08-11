"""Sweep expansion — StudySpec grid → concrete batch variants.

The grid is a Cartesian product over the sweep axes. Each resulting cell knows
its **coordinates** (the research-vocabulary values that produced it), and those
coordinates travel with the run all the way to the result table, the grouping
and the criteria. Without them a study degrades into a list of runs with
labels, and the reader is back to parsing strings.

Expansion deliberately validates every `bind` path against the baseline
*before* anything runs. A typo in a dotted path would otherwise create the key
silently — `experiment/batch.py`'s setter fills in missing dicts as it walks —
and the study would run happily with the override having no effect at all.
That is the worst possible failure: a full grid of runs that are all secretly
identical, and a comparison table that looks fine.
"""

from __future__ import annotations

import copy
import itertools
from dataclasses import dataclass
from typing import Any

from sim4wis.experiment.schema import Experiment
from sim4wis.study.spec import StudySpec


@dataclass(frozen=True)
class Cell:
    """One point of the grid."""

    #: Research-vocabulary coordinates, e.g. {"rear_limit_deg": 4, "speed_kmh": 100}.
    coords: dict[str, Any]
    #: Human-readable label, also the batch variant label and the run label.
    label: str
    #: Dotted-path overrides handed to the batch runner.
    overrides: dict[str, Any]

    def to_variant(self) -> dict[str, Any]:
        return {"label": self.label, "overrides": dict(self.overrides)}


def _format_value(v: Any) -> str:
    if isinstance(v, float):
        s = f"{v:g}"
    else:
        s = str(v)
    return s.replace(" ", "").replace("/", "-")


def _label(coords: dict[str, Any]) -> str:
    if not coords:
        return "baseline"
    return " ".join(f"{k}={_format_value(v)}" for k, v in coords.items())


def _walk_exists(payload: Any, path: str) -> bool:
    """True if `path` resolves in `payload` without creating anything."""
    cur = payload
    for part in path.split("."):
        if isinstance(cur, list):
            try:
                idx = int(part)
            except ValueError:
                return False
            if not (0 <= idx < len(cur)):
                return False
            cur = cur[idx]
        elif isinstance(cur, dict):
            if part not in cur:
                return False
            cur = cur[part]
        else:
            return False
    return True


#: Paths under these prefixes are free-form dicts on the Experiment, so a key
#: that does not exist yet is a legitimate new entry rather than a typo.
_OPEN_PREFIXES = ("vehicle.overrides.", "mode_params.", "scene.")


def validate_binds(spec: StudySpec) -> list[str]:
    """Return a list of human-readable problems with the axes' bind paths."""
    payload = baseline_experiment(spec).model_dump(mode="json")
    problems: list[str] = []
    for name, axis in spec.sweep.items():
        path = axis.bind
        if path.startswith(_OPEN_PREFIXES):
            # Open dict — only the container has to exist.
            container = path.rsplit(".", 1)[0]
            if container and not _walk_exists(payload, container):
                problems.append(
                    f"axis {name!r}: bind {path!r} — container {container!r} does not exist"
                )
            continue
        if not _walk_exists(payload, path):
            problems.append(
                f"axis {name!r}: bind {path!r} does not resolve in the baseline experiment"
            )
    return problems


def baseline_experiment(spec: StudySpec) -> Experiment:
    """The baseline with the study's authoritative fields applied."""
    payload = spec.baseline.model_dump(mode="json")
    payload["model_type"] = spec.model
    if not payload.get("name"):
        payload["name"] = spec.study
    return Experiment.model_validate(payload)


def expand(spec: StudySpec) -> list[Cell]:
    """Expand the sweep into grid cells, in a stable order.

    Raises ValueError for solver axes (not implemented yet) and for bind paths
    that do not resolve — both are conditions a dry run should report rather
    than discover halfway through a few hundred runs.
    """
    solver = spec.solver_axes()
    if solver:
        raise ValueError(
            f"solver axes are not implemented yet: {', '.join(sorted(solver))}. "
            "They need the rising-branch constraint described in "
            "docs/agent_interface_design.md §4 — listing explicit `values` works today."
        )

    problems = validate_binds(spec)
    if problems:
        raise ValueError("; ".join(problems))

    if not spec.sweep:
        return [Cell(coords={}, label=spec.study, overrides={})]

    names = list(spec.sweep.keys())          # insertion order = declaration order
    value_lists = [list(spec.sweep[n].values or []) for n in names]

    cells: list[Cell] = []
    for combo in itertools.product(*value_lists):
        coords = dict(zip(names, combo, strict=True))
        overrides = {spec.sweep[n].bind: v for n, v in coords.items()}
        cells.append(Cell(coords=coords, label=_label(coords), overrides=overrides))
    return cells


def concrete_experiments(spec: StudySpec) -> list[tuple[Cell, Experiment]]:
    """Cells paired with the Experiment each one actually runs.

    Uses the same dotted-path setter as the batch runner so a study cannot
    drift from what `/api/batch` would have produced from the same variants.
    """
    from sim4wis.experiment.batch import _set_dotted

    base = baseline_experiment(spec).model_dump(mode="json")
    out: list[tuple[Cell, Experiment]] = []
    for cell in expand(spec):
        payload = copy.deepcopy(base)
        payload["name"] = f"{spec.study}"
        for path, value in cell.overrides.items():
            _set_dotted(payload, path, value)
        out.append((cell, Experiment.model_validate(payload)))
    return out


def grid_size(spec: StudySpec) -> int:
    n = 1
    for axis in spec.sweep.values():
        n *= len(axis.values or [1])
    return n
