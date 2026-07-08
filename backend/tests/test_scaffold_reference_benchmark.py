from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCAFFOLD_SCRIPT = ROOT / "scripts" / "scaffold_reference_benchmark.py"
CHECK_SCRIPT = ROOT / "scripts" / "check_reference_benchmarks.py"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_scaffold_creates_incoming_template_without_active_evidence(tmp_path: Path) -> None:
    scaffold = load_module(SCAFFOLD_SCRIPT, "scaffold_reference_benchmark_test")
    checker = load_module(CHECK_SCRIPT, "check_reference_benchmarks_for_scaffold_test")
    validation_root = tmp_path / "validation_data"
    incoming = validation_root / ".incoming"

    result = scaffold.scaffold_reference_benchmark(
        "carmaker_iso3888_dlc_60kmh",
        source_type="external_tool",
        source_name="CarMaker",
        source_version="14.0",
        template="iso3888_dlc_60kmh",
        output_root=incoming,
    )

    assert result.path == incoming / "carmaker_iso3888_dlc_60kmh"
    assert {path.name for path in result.files} == {
        "manifest.json",
        "reference.csv",
        "sim4wis_experiment.yaml",
        "notes.md",
        "intake_checklist.json",
    }
    manifest = json.loads((result.path / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["source_type"] == "external_tool"
    assert manifest["provenance"]["solver_step_s"] == "TODO"
    assert manifest["provenance"]["tire_model"] == "TODO"
    assert manifest["metrics"] == {}
    assert (result.path / "reference.csv").read_text(encoding="utf-8").splitlines()[0] == (
        "t,vx,vy,yaw_rate,pose_x,pose_y,driver_steering"
    )
    checklist = json.loads(
        (result.path / "intake_checklist.json").read_text(encoding="utf-8")
    )
    assert checklist["benchmark_id"] == "carmaker_iso3888_dlc_60kmh"
    assert checklist["source_type"] == "external_tool"
    assert len(checklist["items"]) >= 8
    assert checklist["items"][0]["id"] == "reference_csv"

    code, results = checker.check_reference_benchmarks(validation_root, require_independent_source=True)
    assert code == 1
    assert results == []

    code, results = checker.check_reference_benchmarks(incoming, require_independent_source=True)
    assert code == 1
    assert len(results) == 1
    assert results[0].source_type == "external_tool"
    assert any("metrics must be a non-empty object" in failure for failure in results[0].failures)


def test_scaffold_refuses_unsafe_benchmark_id(tmp_path: Path) -> None:
    scaffold = load_module(SCAFFOLD_SCRIPT, "scaffold_reference_benchmark_unsafe_test")

    try:
        scaffold.scaffold_reference_benchmark(
            "bad benchmark id",
            source_type="bench",
            source_name="K&C rig",
            source_version="v1",
            template="steady_circle_30kmh",
            output_root=tmp_path,
        )
    except ValueError as exc:
        assert "filesystem-safe ASCII slug" in str(exc)
    else:
        raise AssertionError("expected unsafe benchmark id to fail")


def test_scaffold_refuses_existing_directory_without_force(tmp_path: Path) -> None:
    scaffold = load_module(SCAFFOLD_SCRIPT, "scaffold_reference_benchmark_force_test")

    scaffold.scaffold_reference_benchmark(
        "rig_step_steer",
        source_type="bench",
        source_name="K&C rig",
        source_version="v1",
        template="step_steer_60kmh",
        output_root=tmp_path,
    )

    try:
        scaffold.scaffold_reference_benchmark(
            "rig_step_steer",
            source_type="bench",
            source_name="K&C rig",
            source_version="v1",
            template="step_steer_60kmh",
            output_root=tmp_path,
        )
    except FileExistsError as exc:
        assert "already exists" in str(exc)
    else:
        raise AssertionError("expected existing scaffold to fail without force")

    result = scaffold.scaffold_reference_benchmark(
        "rig_step_steer",
        source_type="bench",
        source_name="K&C rig",
        source_version="v2",
        template="step_steer_60kmh",
        output_root=tmp_path,
        force=True,
    )
    manifest = json.loads((result.path / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["source_version"] == "v2"
