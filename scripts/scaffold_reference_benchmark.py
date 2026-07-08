#!/usr/bin/env python3
"""Create an intake scaffold for a real external or measured benchmark.

The scaffold is intentionally incomplete and defaults to validation_data/.incoming
so it cannot accidentally satisfy the independent-reference gate. After real
data, metrics, tolerances, and reviewer notes are filled in, move the benchmark
directory to validation_data/<benchmark_id>/ and run check_reference_benchmarks.py.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = ROOT / "validation_data" / ".incoming"
INDEPENDENT_SOURCE_TYPES = ("external_tool", "bench", "scaled_vehicle", "full_vehicle")
TEMPLATE_NAMES = ("steady_circle_30kmh", "step_steer_60kmh", "iso3888_dlc_60kmh", "single_wheel_failure_straight100")
CSV_COLUMNS = ("t", "vx", "vy", "yaw_rate", "pose_x", "pose_y", "driver_steering")


@dataclass(frozen=True)
class ScaffoldResult:
    path: Path
    files: tuple[Path, ...]


def _slug(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip()).strip("._-")
    if not slug:
        raise ValueError("benchmark id must contain at least one ASCII letter, digit, underscore, dot, or dash")
    if slug != value:
        raise ValueError(f"benchmark id must already be a filesystem-safe ASCII slug; got {value!r}, suggested {slug!r}")
    return slug


def experiment_template(name: str, benchmark_id: str) -> dict[str, Any]:
    if name == "steady_circle_30kmh":
        return {
            "name": benchmark_id,
            "description": "External/reference steady-circle benchmark at 30 km/h.",
            "vehicle": {"profile": None, "overrides": {}},
            "model_type": "kinematic",
            "strategy": "ideal_ackermann",
            "mode_params": {},
            "scene": None,
            "path": None,
            "maneuver": {
                "name": "steady_circle",
                "steps": [
                    {
                        "name": "30 km/h constant steer",
                        "duration": 8.0,
                        "steer": {"kind": "constant", "amplitude": 0.05},
                        "speed_kmh": 30.0,
                        "speed_ramp_s": 0.0,
                        "mode_params": {},
                    }
                ],
            },
            "dt": 0.005,
            "record_hz": 50.0,
            "kpis": [],
        }
    if name == "step_steer_60kmh":
        return {
            "name": benchmark_id,
            "description": "External/reference step-steer benchmark at 60 km/h.",
            "vehicle": {"profile": None, "overrides": {}},
            "model_type": "simplified_dynamic",
            "strategy": "ideal_ackermann",
            "mode_params": {},
            "scene": None,
            "path": None,
            "maneuver": {
                "name": "step_steer",
                "steps": [
                    {
                        "name": "60 km/h steering step",
                        "duration": 12.0,
                        "steer": {
                            "kind": "step",
                            "amplitude": 0.05,
                            "freq_hz": 0.5,
                            "f0_hz": 0.1,
                            "f1_hz": 2.0,
                            "t_step": 2.0,
                            "start": 0.0,
                        },
                        "speed_kmh": 60.0,
                        "speed_ramp_s": 1.5,
                        "mode_params": {},
                    }
                ],
            },
            "dt": 0.01,
            "record_hz": 50.0,
            "kpis": [],
        }
    if name == "iso3888_dlc_60kmh":
        return {
            "name": benchmark_id,
            "description": "External/reference ISO 3888-style double-lane-change benchmark at 60 km/h.",
            "vehicle": {"profile": None, "overrides": {}},
            "model_type": "simplified_dynamic",
            "strategy": "ideal_ackermann",
            "mode_params": {},
            "scene": None,
            "path": {"template": "double_lane_change", "params": {}, "waypoints": None, "closed": False},
            "maneuver": {
                "name": "iso3888_dlc",
                "steps": [
                    {
                        "name": "60 km/h double lane change",
                        "duration": 16.0,
                        "speed_kmh": 60.0,
                        "speed_ramp_s": 1.5,
                        "mode_params": {},
                    }
                ],
            },
            "dt": 0.01,
            "record_hz": 50.0,
            "kpis": [],
        }
    if name == "single_wheel_failure_straight100":
        return {
            "name": benchmark_id,
            "description": "External/reference straight-line single-wheel steering-failure benchmark.",
            "vehicle": {"profile": None, "overrides": {}},
            "model_type": "simplified_dynamic",
            "strategy": "ideal_ackermann",
            "mode_params": {},
            "scene": None,
            "path": None,
            "maneuver": {
                "name": "straight100_single_wheel_failure",
                "steps": [
                    {
                        "name": "100 km/h straight-line fault window",
                        "duration": 16.0,
                        "steer": {"kind": "constant", "amplitude": 0.0},
                        "speed_kmh": 100.0,
                        "speed_ramp_s": 2.0,
                        "mode_params": {},
                    }
                ],
            },
            "dt": 0.005,
            "record_hz": 100.0,
            "kpis": [],
        }
    raise ValueError(f"unknown template {name!r}")


def manifest_template(
    benchmark_id: str,
    source_type: str,
    source_name: str,
    source_version: str,
    template: str,
) -> dict[str, Any]:
    if source_type == "external_tool":
        provenance: dict[str, Any] = {
            "solver_step_s": "TODO",
            "tire_model": "TODO",
            "vehicle_parameter_source": "TODO",
            "export_pipeline": "TODO",
        }
    else:
        provenance = {
            "sensor_suite": "TODO",
            "sampling_rate_hz": "TODO",
            "filtering": "TODO",
            "time_sync": "TODO",
            "crop_window_s": "TODO",
            "calibration": "TODO",
        }
    return {
        "benchmark_id": benchmark_id,
        "source_type": source_type,
        "source_name": source_name,
        "source_version": source_version,
        "provenance": provenance,
        "vehicle_mapping": {
            "status": "TODO",
            "template": template,
            "required_review": [
                "Map mass, wheelbase, track, steering ratio, tyre model, inertia, CG, and wheel coordinates to VehicleParams.",
                "Record any unavailable source parameters and justify the Sim4WIS substitute.",
            ],
        },
        "source_artifacts": [
            {
                "path": "raw_source_export.csv",
                "role": "TODO: raw external-tool export, bench log, measurement log, or source report",
                "sha256": "TODO: run shasum -a 256 raw_source_export.csv and paste the digest",
            }
        ],
        "channels": {
            "t": {"unit": "s", "coordinate_frame": "experiment time", "sampling": "TODO"},
            "vx": {"unit": "m/s", "coordinate_frame": "body x", "sensor": "TODO"},
            "vy": {"unit": "m/s", "coordinate_frame": "body y", "sensor": "TODO"},
            "yaw_rate": {"unit": "rad/s", "coordinate_frame": "body yaw about world z, CCW positive", "sensor": "TODO"},
            "pose_x": {"unit": "m", "coordinate_frame": "world x", "sensor": "TODO"},
            "pose_y": {"unit": "m", "coordinate_frame": "world y", "sensor": "TODO"},
            "driver_steering": {"unit": "TODO", "convention": "TODO"},
        },
        "metrics": {},
        "limitations": [
            "TODO: describe source limitations, preprocessing, synchronisation, filtering, and non-representative assumptions.",
            "This scaffold is not validation evidence until real reference.csv samples and metric tolerances are filled in.",
        ],
    }


def notes_template(benchmark_id: str, source_type: str, template: str) -> str:
    return f"""# {benchmark_id}

Source type: `{source_type}`
Template: `{template}`

## Intake Checklist

- [ ] Replace `reference.csv` with real exported or measured samples.
- [ ] Save the raw/source export or measurement/report file in this directory and fill `manifest.json.source_artifacts` with its SHA-256.
- [ ] Fill `manifest.json.provenance` with tool solver/export details or measured sensor/filter/sync details.
- [ ] Fill `manifest.json.vehicle_mapping` with the parameter mapping used for Sim4WIS.
- [ ] Fill `manifest.json.metrics` with reviewed target metrics and tolerances.
- [ ] Document sampling rate, filtering, time synchronisation, coordinate frames, and any data crop.
- [ ] Run `backend/.venv/bin/python scripts/check_reference_benchmarks.py --root validation_data/.incoming`.
- [ ] Promote this directory with `backend/.venv/bin/python scripts/promote_reference_benchmark.py {benchmark_id} --dry-run`, then rerun without `--dry-run`.
- [ ] Regenerate `docs/reports/reference_benchmark_review.md` and `docs/reports/v1_readiness.md`.

## Reviewer Notes

TODO: Summarize source provenance, preprocessing, anomalies, and whether this should count as L3 external-tool evidence or L4 measured evidence.
"""


def scaffold_reference_benchmark(
    benchmark_id: str,
    source_type: str,
    source_name: str,
    source_version: str,
    template: str,
    output_root: Path = DEFAULT_OUTPUT_ROOT,
    force: bool = False,
) -> ScaffoldResult:
    benchmark_id = _slug(benchmark_id)
    if source_type not in INDEPENDENT_SOURCE_TYPES:
        raise ValueError(f"source_type must be one of {', '.join(INDEPENDENT_SOURCE_TYPES)}")
    if template not in TEMPLATE_NAMES:
        raise ValueError(f"template must be one of {', '.join(TEMPLATE_NAMES)}")
    path = output_root / benchmark_id
    if path.exists() and not force:
        raise FileExistsError(f"{path} already exists; pass --force to overwrite scaffold files")
    path.mkdir(parents=True, exist_ok=True)

    manifest_path = path / "manifest.json"
    reference_path = path / "reference.csv"
    experiment_path = path / "sim4wis_experiment.yaml"
    notes_path = path / "notes.md"

    manifest_path.write_text(
        json.dumps(
            manifest_template(benchmark_id, source_type, source_name, source_version, template),
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    with reference_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_COLUMNS)
    experiment_path.write_text(
        yaml.safe_dump(experiment_template(template, benchmark_id), allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    notes_path.write_text(notes_template(benchmark_id, source_type, template), encoding="utf-8")
    return ScaffoldResult(path=path, files=(manifest_path, reference_path, experiment_path, notes_path))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("benchmark_id", help="filesystem-safe benchmark directory name")
    parser.add_argument("--source-type", required=True, choices=INDEPENDENT_SOURCE_TYPES)
    parser.add_argument("--source-name", required=True, help="external tool, rig, platform, or vehicle name")
    parser.add_argument("--source-version", required=True)
    parser.add_argument("--template", required=True, choices=TEMPLATE_NAMES)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--force", action="store_true", help="overwrite scaffold files in an existing directory")
    args = parser.parse_args()

    result = scaffold_reference_benchmark(
        args.benchmark_id,
        source_type=args.source_type,
        source_name=args.source_name,
        source_version=args.source_version,
        template=args.template,
        output_root=args.output_root,
        force=args.force,
    )
    try:
        display_path = result.path.relative_to(ROOT)
    except ValueError:
        display_path = result.path
    print(f"created scaffold: {display_path}")
    for file_path in result.files:
        try:
            display_file = file_path.relative_to(ROOT)
        except ValueError:
            display_file = file_path
        print(f"  {display_file}")
    print("fill real data before moving this directory into validation_data/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
