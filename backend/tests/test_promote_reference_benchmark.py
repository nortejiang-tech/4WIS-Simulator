from __future__ import annotations

import csv
import importlib.util
import json
import sys
from pathlib import Path

import yaml

from sim4wis.experiment.schema import Experiment
from sim4wis.experiment.session import run_experiment


ROOT = Path(__file__).resolve().parents[2]
PROMOTE_SCRIPT = ROOT / "scripts" / "promote_reference_benchmark.py"
SCAFFOLD_SCRIPT = ROOT / "scripts" / "scaffold_reference_benchmark.py"
CHECK_SCRIPT = ROOT / "scripts" / "check_reference_benchmarks.py"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def write_valid_external_benchmark(root: Path, benchmark_id: str = "external_step_40kmh") -> Path:
    bench = root / benchmark_id
    bench.mkdir(parents=True)
    exp_raw = {
        "name": benchmark_id,
        "model_type": "simplified_dynamic",
        "strategy": "ideal_ackermann",
        "maneuver": {
            "name": "step",
            "steps": [
                {"name": "accel", "duration": 4.0, "speed_kmh": 40.0, "speed_ramp_s": 2.0},
                {
                    "name": "step",
                    "duration": 4.0,
                    "steer": {"kind": "step", "amplitude": 0.03, "t_step": 0.5},
                },
            ],
        },
        "dt": 0.01,
        "record_hz": 20.0,
    }
    (bench / "sim4wis_experiment.yaml").write_text(
        yaml.safe_dump(exp_raw, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    result = run_experiment(Experiment.model_validate(exp_raw))
    with (bench / "reference.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=("t", "vx", "vy", "yaw_rate", "pose_x", "pose_y", "driver_steering"),
        )
        writer.writeheader()
        for i, t in enumerate(result.t):
            writer.writerow(
                {
                    "t": t,
                    "vx": result.channels["vx"][i],
                    "vy": result.channels["vy"][i],
                    "yaw_rate": result.channels["yaw_rate"][i],
                    "pose_x": result.channels["pose_x"][i],
                    "pose_y": result.channels["pose_y"][i],
                    "driver_steering": result.channels["driver_steering"][i],
                }
            )
    channels = {
        "t": {"unit": "s"},
        "vx": {"unit": "m/s"},
        "vy": {"unit": "m/s"},
        "yaw_rate": {"unit": "rad/s", "coordinate_frame": "body z"},
        "pose_x": {"unit": "m", "coordinate_frame": "world x"},
        "pose_y": {"unit": "m", "coordinate_frame": "world y"},
        "driver_steering": {"unit": "normalized", "convention": "Sim4WIS driver input"},
    }
    manifest = {
        "benchmark_id": benchmark_id,
        "source_type": "external_tool",
        "source_name": "independent fixture",
        "source_version": "test",
        "vehicle_mapping": {"note": "same defaults for checker fixture"},
        "channels": channels,
        "metrics": {
            "yaw_rate_peak_dps": {"abs_tol": 1e-9, "reason": "deterministic fixture"},
            "pose_y_peak_abs_m": {"abs_tol": 1e-9, "reason": "deterministic fixture"},
            "trajectory_error_rms_m": {"abs_tol": 1e-9, "reason": "deterministic fixture"},
        },
        "limitations": ["test fixture"],
    }
    (bench / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (bench / "notes.md").write_text("# external_step_40kmh\n\nPromotable fixture.\n", encoding="utf-8")
    return bench


def test_promote_checked_independent_benchmark(tmp_path: Path) -> None:
    promote = load_module(PROMOTE_SCRIPT, "promote_reference_benchmark_test")
    checker = load_module(CHECK_SCRIPT, "check_reference_benchmarks_for_promote_test")
    incoming = tmp_path / "validation_data" / ".incoming"
    validation = tmp_path / "validation_data"
    write_valid_external_benchmark(incoming)

    dry_dest = promote.promote_reference_benchmark(
        "external_step_40kmh",
        incoming_root=incoming,
        validation_root=validation,
        dry_run=True,
    )

    assert dry_dest == validation / "external_step_40kmh"
    assert (incoming / "external_step_40kmh").is_dir()

    dest = promote.promote_reference_benchmark(
        "external_step_40kmh",
        incoming_root=incoming,
        validation_root=validation,
    )

    assert dest == validation / "external_step_40kmh"
    assert dest.is_dir()
    assert not (incoming / "external_step_40kmh").exists()
    code, results = checker.check_reference_benchmarks(validation, require_independent_source=True)
    assert code == 0
    assert len(results) == 1
    assert results[0].has_independent_source


def test_promote_refuses_incomplete_scaffold(tmp_path: Path) -> None:
    scaffold = load_module(SCAFFOLD_SCRIPT, "scaffold_reference_benchmark_for_promote_test")
    promote = load_module(PROMOTE_SCRIPT, "promote_reference_benchmark_incomplete_test")
    incoming = tmp_path / "validation_data" / ".incoming"
    validation = tmp_path / "validation_data"
    scaffold.scaffold_reference_benchmark(
        "carmaker_iso3888_dlc_60kmh",
        source_type="external_tool",
        source_name="CarMaker",
        source_version="14.0",
        template="iso3888_dlc_60kmh",
        output_root=incoming,
    )

    try:
        promote.promote_reference_benchmark(
            "carmaker_iso3888_dlc_60kmh",
            incoming_root=incoming,
            validation_root=validation,
        )
    except RuntimeError as exc:
        assert "metrics must be a non-empty object" in str(exc)
    else:
        raise AssertionError("expected incomplete scaffold promotion to fail")
    assert (incoming / "carmaker_iso3888_dlc_60kmh").is_dir()
    assert not (validation / "carmaker_iso3888_dlc_60kmh").exists()


def test_promote_refuses_existing_destination(tmp_path: Path) -> None:
    promote = load_module(PROMOTE_SCRIPT, "promote_reference_benchmark_existing_test")
    incoming = tmp_path / "validation_data" / ".incoming"
    validation = tmp_path / "validation_data"
    write_valid_external_benchmark(incoming, "external_step_40kmh")
    (validation / "external_step_40kmh").mkdir(parents=True)

    try:
        promote.promote_reference_benchmark(
            "external_step_40kmh",
            incoming_root=incoming,
            validation_root=validation,
        )
    except FileExistsError as exc:
        assert "refusing to overwrite" in str(exc)
    else:
        raise AssertionError("expected existing destination to fail")
