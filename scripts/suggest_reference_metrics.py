#!/usr/bin/env python3
"""Suggest manifest.metrics entries from normalized reference data.

This helper computes candidate reference values for metrics the reference
checker already supports. It deliberately leaves tolerances and rationale as
TODO values, so the snippet cannot make a benchmark promotable until a reviewer
chooses and justifies acceptance bounds.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
CHECKER_SCRIPT = ROOT / "scripts" / "check_reference_benchmarks.py"
DEFAULT_METRICS = (
    "yaw_rate_peak_dps",
    "vy_peak_kmh",
    "speed_error_rms_kmh",
    "pose_y_peak_abs_m",
    "trajectory_error_rms_m",
    "trajectory_error_peak_m",
)
TRAJECTORY_METRICS = {"trajectory_error_rms_m", "trajectory_error_peak_m"}


@dataclass(frozen=True)
class SuggestedMetrics:
    metrics: dict[str, dict[str, Any]]
    skipped: tuple[str, ...]


def _load_reference_checker(root: Path = ROOT) -> Any:
    script = root / "scripts" / "check_reference_benchmarks.py"
    spec = importlib.util.spec_from_file_location("check_reference_benchmarks_for_metric_suggestions", script)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load {script}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def suggest_reference_metrics(
    reference_csv: Path,
    experiment_yaml: Path,
    metric_names: tuple[str, ...] = DEFAULT_METRICS,
) -> SuggestedMetrics:
    checker = _load_reference_checker()
    ref = checker._load_reference_csv(reference_csv)
    exp = checker._load_experiment(experiment_yaml)

    metrics: dict[str, dict[str, Any]] = {}
    skipped: list[str] = []
    for metric_name in metric_names:
        if metric_name in TRAJECTORY_METRICS:
            reference_value = 0.0
        else:
            reference_value = checker._reference_metric(metric_name, ref, exp)
        if reference_value is None:
            skipped.append(metric_name)
            continue
        metrics[metric_name] = {
            "reference_value": float(reference_value),
            "abs_tol": "TODO: reviewed absolute tolerance",
            "reason": "TODO: justify tolerance from source accuracy and acceptance criteria",
        }

    if not metrics:
        raise ValueError(f"no supported reference metrics could be computed from {reference_csv}")
    return SuggestedMetrics(metrics=metrics, skipped=tuple(skipped))


def _resolve_inputs(args: argparse.Namespace) -> tuple[Path, Path]:
    if args.benchmark_dir is not None:
        benchmark_dir = args.benchmark_dir
        reference_csv = args.reference_csv or benchmark_dir / "reference.csv"
        experiment_yaml = args.experiment_yaml or benchmark_dir / "sim4wis_experiment.yaml"
    else:
        if args.reference_csv is None or args.experiment_yaml is None:
            raise ValueError("provide benchmark_dir or both --reference-csv and --experiment-yaml")
        reference_csv = args.reference_csv
        experiment_yaml = args.experiment_yaml
    return reference_csv, experiment_yaml


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "benchmark_dir",
        nargs="?",
        type=Path,
        help="benchmark directory containing reference.csv and sim4wis_experiment.yaml",
    )
    parser.add_argument("--reference-csv", type=Path, help="override reference.csv path")
    parser.add_argument("--experiment-yaml", type=Path, help="override sim4wis_experiment.yaml path")
    parser.add_argument(
        "--metric",
        action="append",
        default=[],
        help="metric name to include; repeat to override the default candidate list",
    )
    args = parser.parse_args()

    try:
        reference_csv, experiment_yaml = _resolve_inputs(args)
        result = suggest_reference_metrics(
            reference_csv,
            experiment_yaml,
            tuple(args.metric) if args.metric else DEFAULT_METRICS,
        )
    except Exception as exc:
        print(f"metric suggestion failed: {exc}", file=sys.stderr)
        return 1
    for metric_name in result.skipped:
        print(f"warning: skipped unsupported or unavailable metric {metric_name!r}", file=sys.stderr)
    print(json.dumps({"metrics": result.metrics}, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
