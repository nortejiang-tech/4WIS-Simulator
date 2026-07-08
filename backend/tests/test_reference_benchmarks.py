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
SCRIPT = ROOT / "scripts" / "check_reference_benchmarks.py"


def load_checker():
    spec = importlib.util.spec_from_file_location("check_reference_benchmarks", SCRIPT)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_reference_checker_allows_empty_root_by_default(tmp_path: Path) -> None:
    checker = load_checker()
    default_report = checker.render_review_report(checker.DEFAULT_ROOT, [])
    assert "- Data root: `validation_data`" in default_report

    code, results = checker.check_reference_benchmarks(tmp_path)
    assert code == 0
    assert results == []

    code, results = checker.check_reference_benchmarks(tmp_path, require_data=True)
    assert code == 1
    assert results == []

    code, results = checker.check_reference_benchmarks(tmp_path, require_independent_source=True)
    assert code == 1
    assert results == []

    report = tmp_path / "review.md"
    code, results = checker.check_reference_benchmarks(tmp_path, report_path=report)
    assert code == 0
    assert results == []
    text = report.read_text(encoding="utf-8")
    assert "No reference benchmark directories were found" in text
    assert "External validation evidence remains absent" in text

    code, results = checker.check_reference_benchmarks(tmp_path, check_report_path=report)
    assert code == 0
    assert results == []

    code, results = checker.check_reference_benchmarks(tmp_path, check_report_path=tmp_path / "missing.md")
    assert code == 1
    assert results == []

    report.write_text(text + "\nStale manual edit.\n", encoding="utf-8")
    code, results = checker.check_reference_benchmarks(tmp_path, check_report_path=report)
    assert code == 1
    assert results == []


def test_reference_checker_reports_malformed_benchmark(tmp_path: Path) -> None:
    checker = load_checker()
    (tmp_path / "bad_case").mkdir()

    code, results = checker.check_reference_benchmarks(tmp_path)

    assert code == 1
    assert len(results) == 1
    assert any("missing manifest.json" in failure for failure in results[0].failures)


def test_reference_checker_compares_valid_benchmark(tmp_path: Path) -> None:
    checker = load_checker()
    bench = tmp_path / "analytic_step_40kmh"
    bench.mkdir()

    exp_raw = {
        "name": "analytic_step_40kmh",
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

    exp = Experiment.model_validate(exp_raw)
    result = run_experiment(exp)
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

    channel_specs = {
        "t": {"unit": "s"},
        "vx": {"unit": "m/s"},
        "vy": {"unit": "m/s"},
        "yaw_rate": {"unit": "rad/s", "coordinate_frame": "body z"},
        "pose_x": {"unit": "m", "coordinate_frame": "world x"},
        "pose_y": {"unit": "m", "coordinate_frame": "world y"},
        "driver_steering": {"unit": "normalized", "convention": "Sim4WIS driver input"},
    }
    manifest = {
        "benchmark_id": "analytic_step_40kmh",
        "source_type": "analytic",
        "source_name": "deterministic Sim4WIS fixture for checker test",
        "source_version": "test",
        "vehicle_mapping": {"note": "same Sim4WIS defaults for checker fixture"},
        "channels": channel_specs,
        "metrics": {
            "yaw_rate_peak_dps": {"abs_tol": 1e-9, "reason": "deterministic fixture"},
            "pose_y_peak_abs_m": {"abs_tol": 1e-9, "reason": "deterministic fixture"},
            "trajectory_error_rms_m": {"abs_tol": 1e-9, "reason": "deterministic fixture"},
        },
        "limitations": ["test fixture, not external validation evidence"],
    }
    (bench / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (bench / "notes.md").write_text(
        "# analytic_step_40kmh\n\nReviewer note: deterministic fixture, not external evidence.\n",
        encoding="utf-8",
    )
    report = tmp_path / "reference_review.md"

    code, results = checker.check_reference_benchmarks(tmp_path, require_data=True, report_path=report)

    assert code == 0
    assert len(results) == 1
    assert results[0].checked_metrics == 3
    assert results[0].failures == []
    assert len(results[0].metrics) == 3
    assert all(metric.ok for metric in results[0].metrics)
    assert "Reviewer note: deterministic fixture" in results[0].reviewer_notes

    code, results = checker.check_reference_benchmarks(tmp_path, require_independent_source=True)
    assert code == 1
    assert len(results) == 1
    assert results[0].source_type == "analytic"

    manifest["source_type"] = "external_tool"
    manifest["source_name"] = "independent checker fixture"
    (bench / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    code, results = checker.check_reference_benchmarks(tmp_path, check_report_path=report)
    assert code == 1
    assert len(results) == 1

    code, results = checker.check_reference_benchmarks(tmp_path, require_independent_source=True)
    assert code == 0
    assert len(results) == 1
    assert results[0].source_type == "external_tool"
    assert results[0].has_independent_source

    code, results = checker.check_reference_benchmarks(tmp_path, report_path=report)
    assert code == 0
    assert len(results) == 1
    code, results = checker.check_reference_benchmarks(tmp_path, check_report_path=report)
    assert code == 0
    assert len(results) == 1

    text = report.read_text(encoding="utf-8")
    assert "## analytic_step_40kmh - PASS" in text
    assert "Passing independent external/measured benchmarks: 1" in text
    assert "No passing independent external-tool" not in text
    assert "| `yaw_rate_peak_dps` |" in text
    assert "Reviewer note: deterministic fixture, not external evidence." in text
    assert "does not upgrade validation levels without human review" in text
