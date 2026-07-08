#!/usr/bin/env python3
"""Validate external/reference benchmark datasets under validation_data/.

This gate is deliberately evidence-safe:

* no benchmark directories -> pass by default, with an explicit "no evidence"
  message (use --require-data to fail);
* malformed benchmark directories -> fail;
* valid benchmark directories -> run the paired Sim4WIS experiment and compare
  requested metrics against the reference CSV or manifest reference values.

It does not upgrade any validation-matrix level by itself. It only makes the
data reproducible enough for a human review to cite it.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "backend" / "src"))

from sim4wis.experiment.kpi import compute_kpis  # noqa: E402
from sim4wis.experiment.schema import Experiment  # noqa: E402
from sim4wis.experiment.session import run_experiment  # noqa: E402


DEFAULT_ROOT = ROOT / "validation_data"
REQUIRED_FILES = ("manifest.json", "reference.csv", "sim4wis_experiment.yaml")
REQUIRED_MANIFEST_FIELDS = (
    "benchmark_id",
    "source_type",
    "source_name",
    "source_version",
    "vehicle_mapping",
    "channels",
    "metrics",
    "limitations",
)
ALLOWED_SOURCE_TYPES = {"analytic", "external_tool", "bench", "scaled_vehicle", "full_vehicle"}
INDEPENDENT_SOURCE_TYPES = {"external_tool", "bench", "scaled_vehicle", "full_vehicle"}
REQUIRED_CHANNELS = ("t", "vx", "vy", "yaw_rate", "pose_x", "pose_y", "driver_steering")
UNRESOLVED_PLACEHOLDER_TOKENS = ("TODO", "TBD", "PLACEHOLDER", "FILL_ME", "待补", "待定")


@dataclass
class MetricComparison:
    name: str
    sim_value: float
    reference_value: float
    delta: float
    tolerance: float

    @property
    def ok(self) -> bool:
        return math.isfinite(self.delta) and self.delta <= self.tolerance


@dataclass
class BenchmarkResult:
    benchmark_id: str
    checked_metrics: int = 0
    source_type: str = ""
    source_name: str = ""
    source_version: str = ""
    limitations: list[str] = field(default_factory=list)
    reviewer_notes: str = ""
    metrics: list[MetricComparison] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    failures: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.failures

    @property
    def has_independent_source(self) -> bool:
        return self.source_type in INDEPENDENT_SOURCE_TYPES


def _load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def _load_experiment(path: Path) -> Experiment:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return Experiment.model_validate(raw)


def _load_text(path: Path) -> str:
    return path.read_text(encoding="utf-8").strip()


def _load_reference_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"{path} has no header row")
        cols: dict[str, list[float]] = {name: [] for name in reader.fieldnames}
        for row_idx, row in enumerate(reader, start=2):
            for name in cols:
                raw = (row.get(name) or "").strip()
                if raw == "":
                    cols[name].append(math.nan)
                    continue
                try:
                    cols[name].append(float(raw))
                except ValueError as exc:
                    raise ValueError(f"{path}:{row_idx}: column {name!r} is not numeric: {raw!r}") from exc
    return {name: np.asarray(values, dtype=np.float64) for name, values in cols.items()}


def _speed_error_rms_kmh(t: np.ndarray, vx: np.ndarray, exp: Experiment) -> float | None:
    target = np.full(t.size, np.nan)
    t_cursor = 0.0
    current: float | None = None
    for step in exp.maneuver.steps:
        if step.speed_kmh is not None:
            current = float(step.speed_kmh) / 3.6
            settle = 2.0 + float(step.speed_ramp_s)
            mask = (t >= t_cursor + settle) & (t < t_cursor + step.duration)
        else:
            mask = (t >= t_cursor) & (t < t_cursor + step.duration)
        if current is not None:
            target[mask] = current
        t_cursor += step.duration
    valid = np.isfinite(target)
    if not np.any(valid):
        return None
    return float(np.sqrt(np.mean((vx[valid] - target[valid]) ** 2)) * 3.6)


def _reference_metric(name: str, ref: dict[str, np.ndarray], exp: Experiment) -> float | None:
    if name == "yaw_rate_peak_dps":
        return float(np.nanmax(np.abs(ref["yaw_rate"])) * 180.0 / math.pi)
    if name == "vy_peak_kmh":
        return float(np.nanmax(np.abs(ref["vy"])) * 3.6)
    if name == "speed_error_rms_kmh":
        return _speed_error_rms_kmh(ref["t"], ref["vx"], exp)
    if name == "pose_y_peak_abs_m":
        return float(np.nanmax(np.abs(ref["pose_y"])))
    return None


def _sim_metric(name: str, sim_kpis: dict[str, Any], sim_channels: dict[str, np.ndarray]) -> float | None:
    if name in sim_kpis:
        return float(sim_kpis[name])
    if name == "pose_y_peak_abs_m":
        return float(np.nanmax(np.abs(sim_channels["pose_y"])))
    return None


def _trajectory_metric(name: str, ref: dict[str, np.ndarray], sim_channels: dict[str, np.ndarray]) -> float | None:
    if name not in {"trajectory_error_rms_m", "trajectory_error_peak_m"}:
        return None
    t_ref = ref["t"]
    t_sim = sim_channels["t"]
    sim_x = np.interp(t_ref, t_sim, sim_channels["pose_x"])
    sim_y = np.interp(t_ref, t_sim, sim_channels["pose_y"])
    err = np.hypot(sim_x - ref["pose_x"], sim_y - ref["pose_y"])
    if name == "trajectory_error_rms_m":
        return float(np.sqrt(np.nanmean(err**2)))
    return float(np.nanmax(err))


def _metric_reference_value(name: str, spec: dict[str, Any], ref: dict[str, np.ndarray], exp: Experiment) -> float | None:
    if "reference_value" in spec:
        return float(spec["reference_value"])
    return _reference_metric(name, ref, exp)


def _allowed_delta(want: float, spec: dict[str, Any]) -> float:
    abs_tol = float(spec.get("abs_tol", spec.get("tolerance", 0.0)))
    rel_tol = float(spec.get("rel_tol", 0.0))
    return max(abs_tol, abs(want) * rel_tol)


def _validate_manifest_shape(path: Path, manifest: dict[str, Any], result: BenchmarkResult) -> None:
    for field_name in REQUIRED_MANIFEST_FIELDS:
        if field_name not in manifest:
            result.failures.append(f"{path.name}: missing manifest field {field_name!r}")
    source_type = manifest.get("source_type")
    if source_type is not None and source_type not in ALLOWED_SOURCE_TYPES:
        result.failures.append(f"{path.name}: source_type {source_type!r} is not one of {sorted(ALLOWED_SOURCE_TYPES)}")
    metrics = manifest.get("metrics")
    if not isinstance(metrics, dict) or not metrics:
        result.failures.append(f"{path.name}: metrics must be a non-empty object")
    channels = manifest.get("channels")
    if not isinstance(channels, dict):
        result.failures.append(f"{path.name}: channels must be an object")
        return
    for ch in REQUIRED_CHANNELS:
        spec = channels.get(ch)
        if not isinstance(spec, dict):
            result.failures.append(f"{path.name}: channels.{ch} missing or not an object")
            continue
        if "unit" not in spec:
            result.failures.append(f"{path.name}: channels.{ch}.unit missing")
    for ch in ("yaw_rate", "pose_x", "pose_y", "driver_steering"):
        spec = channels.get(ch)
        if isinstance(spec, dict) and not any(k in spec for k in ("coordinate_frame", "frame", "convention")):
            result.warnings.append(f"{path.name}: channels.{ch} has no coordinate_frame/frame/convention note")


def _find_unresolved_placeholders(value: Any, prefix: str = "") -> list[str]:
    hits: list[str] = []
    if isinstance(value, dict):
        for key, item in value.items():
            child = f"{prefix}.{key}" if prefix else str(key)
            hits.extend(_find_unresolved_placeholders(item, child))
        return hits
    if isinstance(value, list):
        for idx, item in enumerate(value):
            child = f"{prefix}[{idx}]"
            hits.extend(_find_unresolved_placeholders(item, child))
        return hits
    if isinstance(value, str):
        upper = value.upper()
        if any(token in upper for token in UNRESOLVED_PLACEHOLDER_TOKENS[:4]) or any(
            token in value for token in UNRESOLVED_PLACEHOLDER_TOKENS[4:]
        ):
            hits.append(prefix or "<root>")
    return hits


def _validate_no_placeholders(path: Path, manifest: dict[str, Any], notes: str, result: BenchmarkResult) -> None:
    manifest_hits = _find_unresolved_placeholders(manifest)
    if manifest_hits:
        result.failures.append(
            f"{path.name}: manifest contains unresolved placeholder(s): {', '.join(manifest_hits[:8])}"
        )
    note_hits = _find_unresolved_placeholders(notes, "notes.md")
    if note_hits:
        result.failures.append(f"{path.name}: notes.md contains unresolved placeholder(s)")


def _validate_csv_channels(path: Path, ref: dict[str, np.ndarray], result: BenchmarkResult) -> None:
    for ch in REQUIRED_CHANNELS:
        if ch not in ref:
            result.failures.append(f"{path.name}: reference.csv missing required column {ch!r}")
    if result.failures:
        return
    n = len(ref["t"])
    if n < 2:
        result.failures.append(f"{path.name}: reference.csv must contain at least two samples")
    for ch, values in ref.items():
        if len(values) != n:
            result.failures.append(f"{path.name}: column {ch!r} length differs from t")
    t = ref["t"]
    if np.any(~np.isfinite(t)):
        result.failures.append(f"{path.name}: t contains non-finite values")
    if np.any(np.diff(t) <= 0):
        result.failures.append(f"{path.name}: t must be strictly increasing")


def check_benchmark(path: Path) -> BenchmarkResult:
    manifest_path = path / "manifest.json"
    benchmark_id = path.name
    result = BenchmarkResult(benchmark_id=benchmark_id)

    for filename in REQUIRED_FILES:
        if not (path / filename).is_file():
            result.failures.append(f"{path.name}: missing {filename}")
    if not (path / "notes.md").is_file():
        result.warnings.append(f"{path.name}: notes.md missing")
    if result.failures:
        return result

    try:
        manifest = _load_json(manifest_path)
    except Exception as exc:
        result.failures.append(f"{path.name}: failed to read manifest.json: {exc}")
        return result
    benchmark_id = str(manifest.get("benchmark_id", path.name))
    result.benchmark_id = benchmark_id
    result.source_type = str(manifest.get("source_type", ""))
    result.source_name = str(manifest.get("source_name", ""))
    result.source_version = str(manifest.get("source_version", ""))
    limitations = manifest.get("limitations", [])
    if isinstance(limitations, list):
        result.limitations = [str(item) for item in limitations]
    if benchmark_id != path.name:
        result.failures.append(f"{path.name}: manifest benchmark_id {benchmark_id!r} must match directory name")
    _validate_manifest_shape(manifest_path, manifest, result)

    notes_path = path / "notes.md"
    if notes_path.is_file():
        result.reviewer_notes = _load_text(notes_path)
    _validate_no_placeholders(path, manifest, result.reviewer_notes, result)

    try:
        ref = _load_reference_csv(path / "reference.csv")
    except Exception as exc:
        result.failures.append(f"{path.name}: failed to read reference.csv: {exc}")
        return result
    _validate_csv_channels(path, ref, result)

    try:
        exp = _load_experiment(path / "sim4wis_experiment.yaml")
    except Exception as exc:
        result.failures.append(f"{path.name}: failed to read sim4wis_experiment.yaml: {exc}")
        return result

    if result.failures:
        return result

    sim = run_experiment(exp)
    sim_kpis = compute_kpis(sim, exp)
    sim_channels = {
        "t": np.asarray(sim.t, dtype=np.float64),
        **{name: np.asarray(values, dtype=np.float64) for name, values in sim.channels.items()},
    }

    metrics = manifest.get("metrics", {})
    for metric_name, raw_spec in metrics.items():
        if not isinstance(raw_spec, dict):
            result.failures.append(f"{path.name}: metrics.{metric_name} must be an object")
            continue
        if "abs_tol" not in raw_spec and "rel_tol" not in raw_spec and "tolerance" not in raw_spec:
            result.failures.append(f"{path.name}: metrics.{metric_name} missing abs_tol/rel_tol/tolerance")
            continue
        if metric_name.startswith("trajectory_error_"):
            got = _trajectory_metric(metric_name, ref, sim_channels)
            want = float(raw_spec.get("reference_value", 0.0))
        else:
            want = _metric_reference_value(metric_name, raw_spec, ref, exp)
            got = _sim_metric(metric_name, sim_kpis, sim_channels)
        if got is None or want is None:
            result.failures.append(f"{path.name}: metric {metric_name!r} is not supported by the checker")
            continue
        allowed = _allowed_delta(want, raw_spec)
        delta = abs(got - want)
        comparison = MetricComparison(
            name=metric_name,
            sim_value=got,
            reference_value=want,
            delta=delta,
            tolerance=allowed,
        )
        result.metrics.append(comparison)
        if not math.isfinite(delta) or delta > allowed:
            result.failures.append(
                f"{path.name}.{metric_name}: sim {got:.6g}, reference {want:.6g}, "
                f"delta {delta:.3g} > tol {allowed:.3g}"
            )
        result.checked_metrics += 1
    return result


def discover_benchmarks(root: Path) -> list[Path]:
    if not root.exists():
        return []
    return sorted(p for p in root.iterdir() if p.is_dir() and not p.name.startswith("."))


def _fmt_float(value: float) -> str:
    if not math.isfinite(value):
        return str(value)
    return f"{value:.6g}"


def _first_lines(text: str, max_lines: int = 8) -> str:
    lines = [line.rstrip() for line in text.strip().splitlines()]
    return "\n".join(lines[:max_lines])


def _display_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(ROOT))
    except ValueError:
        return str(resolved)


def render_review_report(root: Path, results: list[BenchmarkResult]) -> str:
    """Render a reviewer-facing Markdown report from checked benchmark results."""
    ok_count = sum(1 for r in results if r.ok)
    independent_count = sum(1 for r in results if r.ok and r.has_independent_source)
    lines = [
        "# Reference Benchmark Review Report",
        "",
        f"- Data root: `{_display_path(root)}`",
        f"- Benchmarks checked: {len(results)}",
        f"- Passing benchmarks: {ok_count}/{len(results)}",
        f"- Passing independent external/measured benchmarks: {independent_count}",
        "- Evidence boundary: this report summarizes reproducibility checks only; it does not upgrade validation levels without human review.",
        "",
    ]
    if independent_count == 0:
        lines += [
            "> No passing independent external-tool, bench, scaled-vehicle, or full-vehicle benchmark is present.",
            "",
        ]
    if not results:
        lines += [
            "## No Benchmark Data",
            "",
            "No reference benchmark directories were found. External validation evidence remains absent.",
            "",
        ]
        return "\n".join(lines)

    for result in results:
        status = "PASS" if result.ok else "FAIL"
        lines += [
            f"## {result.benchmark_id} - {status}",
            "",
            f"- Source: {result.source_type or 'unknown'} / {result.source_name or 'unknown'} / {result.source_version or 'unknown'}",
            f"- Checked metrics: {result.checked_metrics}",
        ]
        if result.limitations:
            lines.append(f"- Limitations: {'; '.join(result.limitations)}")
        if result.warnings:
            lines.append(f"- Warnings: {'; '.join(result.warnings)}")
        if result.failures:
            lines.append(f"- Failures: {'; '.join(result.failures)}")
        lines += [
            "",
            "| Metric | Sim | Reference | Delta | Tolerance | Status |",
            "|---|---:|---:|---:|---:|---|",
        ]
        if result.metrics:
            for metric in result.metrics:
                lines.append(
                    f"| `{metric.name}` | {_fmt_float(metric.sim_value)} | "
                    f"{_fmt_float(metric.reference_value)} | {_fmt_float(metric.delta)} | "
                    f"{_fmt_float(metric.tolerance)} | {'PASS' if metric.ok else 'FAIL'} |"
                )
        else:
            lines.append("| _none_ |  |  |  |  |  |")
        lines += ["", "### Reviewer Notes", ""]
        if result.reviewer_notes:
            lines.append("```markdown")
            lines.append(_first_lines(result.reviewer_notes))
            lines.append("```")
        else:
            lines.append("_No notes.md content available._")
        lines.append("")
    return "\n".join(lines)


def write_review_report(path: Path, root: Path, results: list[BenchmarkResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render_review_report(root, results), encoding="utf-8")


def check_review_report_fresh(path: Path, root: Path, results: list[BenchmarkResult]) -> bool:
    expected = render_review_report(root, results)
    try:
        actual = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        print(
            f"{path}: reference benchmark review report is missing; "
            f"regenerate with --report {path}",
            file=sys.stderr,
        )
        return False
    if actual != expected:
        print(
            f"{path}: reference benchmark review report is stale; "
            f"regenerate with --report {path}",
            file=sys.stderr,
        )
        return False
    return True


def check_reference_benchmarks(
    root: Path = DEFAULT_ROOT,
    require_data: bool = False,
    require_independent_source: bool = False,
    report_path: Path | None = None,
    check_report_path: Path | None = None,
) -> tuple[int, list[BenchmarkResult]]:
    benches = discover_benchmarks(root)
    if not benches:
        report_fresh = True
        if report_path is not None:
            write_review_report(report_path, root, [])
        if check_report_path is not None:
            report_fresh = check_review_report_fresh(check_report_path, root, [])
        if require_data or require_independent_source:
            print(f"{root}: no reference benchmarks found", file=sys.stderr)
            return 1, []
        if not report_fresh:
            return 1, []
        print(f"{root}: no reference benchmarks found; external validation evidence remains absent")
        return 0, []

    results = [check_benchmark(path) for path in benches]
    if report_path is not None:
        write_review_report(report_path, root, results)
    report_fresh = True
    if check_report_path is not None:
        report_fresh = check_review_report_fresh(check_report_path, root, results)
    failures = sum(len(r.failures) for r in results)
    for r in results:
        status = "ok" if r.ok else "failed"
        print(f"{r.benchmark_id}: {status} ({r.checked_metrics} metrics)")
        for warning in r.warnings:
            print(f"  warning: {warning}")
        for failure in r.failures:
            print(f"  failure: {failure}", file=sys.stderr)
    independent_count = sum(1 for r in results if r.ok and r.has_independent_source)
    if require_independent_source and independent_count == 0:
        print(
            f"{root}: no passing independent external-tool, bench, scaled-vehicle, or full-vehicle benchmark found",
            file=sys.stderr,
        )
    return (
        1 if failures or not report_fresh or (require_independent_source and independent_count == 0) else 0
    ), results


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT, help="validation_data root")
    parser.add_argument("--require-data", action="store_true", help="fail when no benchmark directories exist")
    parser.add_argument(
        "--require-independent-source",
        action="store_true",
        help="fail unless at least one passing external_tool/bench/scaled_vehicle/full_vehicle benchmark exists",
    )
    report_group = parser.add_mutually_exclusive_group()
    report_group.add_argument("--report", type=Path, help="write a reviewer-facing Markdown report")
    report_group.add_argument(
        "--check-report",
        type=Path,
        help="fail if the reviewer-facing Markdown report is missing or stale",
    )
    args = parser.parse_args()
    code, _results = check_reference_benchmarks(
        args.root,
        require_data=args.require_data,
        require_independent_source=args.require_independent_source,
        report_path=args.report,
        check_report_path=args.check_report,
    )
    return code


if __name__ == "__main__":
    raise SystemExit(main())
