from __future__ import annotations

import csv
import importlib.util
import json
import math
import subprocess
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "suggest_reference_metrics.py"


def load_module():
    spec = importlib.util.spec_from_file_location("suggest_reference_metrics", SCRIPT)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def write_benchmark_fixture(path: Path) -> None:
    path.mkdir()
    with (path / "reference.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=("t", "vx", "vy", "yaw_rate", "pose_x", "pose_y", "driver_steering"),
        )
        writer.writeheader()
        writer.writerows(
            [
                {
                    "t": "0.0",
                    "vx": "10.0",
                    "vy": "0.0",
                    "yaw_rate": "0.0",
                    "pose_x": "0.0",
                    "pose_y": "0.0",
                    "driver_steering": "0.0",
                },
                {
                    "t": "2.5",
                    "vx": "10.0",
                    "vy": "1.0",
                    "yaw_rate": str(math.pi / 2.0),
                    "pose_x": "10.0",
                    "pose_y": "-2.0",
                    "driver_steering": "0.1",
                },
            ]
        )
    experiment = {
        "name": "metric_fixture",
        "model_type": "simplified_dynamic",
        "strategy": "ideal_ackermann",
        "maneuver": {
            "name": "steady",
            "steps": [
                {"name": "steady", "duration": 3.0, "speed_kmh": 36.0, "speed_ramp_s": 0.0},
            ],
        },
        "dt": 0.01,
        "record_hz": 10.0,
    }
    (path / "sim4wis_experiment.yaml").write_text(
        yaml.safe_dump(experiment, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )


def test_suggest_reference_metrics_computes_candidate_values(tmp_path: Path) -> None:
    suggester = load_module()
    bench = tmp_path / "bench"
    write_benchmark_fixture(bench)

    result = suggester.suggest_reference_metrics(
        bench / "reference.csv",
        bench / "sim4wis_experiment.yaml",
        (
            "yaw_rate_peak_dps",
            "vy_peak_kmh",
            "speed_error_rms_kmh",
            "pose_y_peak_abs_m",
            "trajectory_error_rms_m",
        ),
    )

    assert result.skipped == ()
    assert math.isclose(result.metrics["yaw_rate_peak_dps"]["reference_value"], 90.0)
    assert math.isclose(result.metrics["vy_peak_kmh"]["reference_value"], 3.6)
    assert math.isclose(result.metrics["speed_error_rms_kmh"]["reference_value"], 0.0)
    assert result.metrics["pose_y_peak_abs_m"]["reference_value"] == 2.0
    assert result.metrics["trajectory_error_rms_m"]["reference_value"] == 0.0
    assert result.metrics["yaw_rate_peak_dps"]["abs_tol"].startswith("TODO")
    assert result.metrics["yaw_rate_peak_dps"]["reason"].startswith("TODO")


def test_suggest_reference_metrics_cli_outputs_manifest_snippet(tmp_path: Path) -> None:
    bench = tmp_path / "bench"
    write_benchmark_fixture(bench)

    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            str(bench),
            "--metric",
            "yaw_rate_peak_dps",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 0
    assert completed.stderr == ""
    payload = json.loads(completed.stdout)
    assert math.isclose(payload["metrics"]["yaw_rate_peak_dps"]["reference_value"], 90.0)
    assert payload["metrics"]["yaw_rate_peak_dps"]["abs_tol"].startswith("TODO")


def test_suggest_reference_metrics_refuses_when_no_metric_can_be_computed(tmp_path: Path) -> None:
    suggester = load_module()
    bench = tmp_path / "bench"
    write_benchmark_fixture(bench)

    try:
        suggester.suggest_reference_metrics(
            bench / "reference.csv",
            bench / "sim4wis_experiment.yaml",
            ("unsupported_metric",),
        )
    except ValueError as exc:
        assert "no supported reference metrics" in str(exc)
    else:
        raise AssertionError("expected unsupported-only metric set to fail")
