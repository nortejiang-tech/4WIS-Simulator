from __future__ import annotations

import hashlib
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "suggest_reference_artifacts.py"


def load_module():
    spec = importlib.util.spec_from_file_location("suggest_reference_artifacts", SCRIPT)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def sha256_text(content: str) -> str:
    return hashlib.sha256(content.encode("utf-8")).hexdigest()


def write_artifact(path: Path, rel_path: str, content: str) -> None:
    file_path = path / rel_path
    file_path.parent.mkdir(parents=True, exist_ok=True)
    file_path.write_text(content, encoding="utf-8")


def test_suggest_reference_artifacts_builds_snippets(tmp_path: Path) -> None:
    suggester = load_module()
    bench = tmp_path / "iso_test"
    bench.mkdir()

    write_artifact(bench, "raw/export.csv", "t,x\na,b")
    write_artifact(bench, "raw/report.json", '{"ok":true}')

    result = suggester.suggest_reference_artifacts(
        bench,
        ("raw/export.csv", "raw/report.json"),
        role="raw source bundle for validation",
    )

    assert len(result.source_artifacts) == 2
    assert result.source_artifacts[0]["path"] == "raw/export.csv"
    assert result.source_artifacts[0]["role"] == "raw source bundle for validation"
    assert result.source_artifacts[0]["sha256"] == sha256_text("t,x\na,b")
    assert result.source_artifacts[1]["path"] == "raw/report.json"
    assert result.source_artifacts[1]["sha256"] == sha256_text('{"ok":true}')


def test_suggest_reference_artifacts_cli_outputs_json(tmp_path: Path) -> None:
    suggester = load_module()
    bench = tmp_path / "iso_cli"
    bench.mkdir()
    write_artifact(bench, "raw/export.csv", "raw data")

    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            str(bench),
            "raw/export.csv",
            "--role",
            "raw export",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 0
    assert completed.stderr == ""
    payload = json.loads(completed.stdout)
    assert payload["source_artifacts"][0]["path"] == "raw/export.csv"
    assert payload["source_artifacts"][0]["role"] == "raw export"
    cli_count = len(suggester.suggest_reference_artifacts(bench, ("raw/export.csv",)).source_artifacts)
    assert cli_count == 1


def test_suggest_reference_artifacts_rejects_path_issues(tmp_path: Path) -> None:
    suggester = load_module()
    bench = tmp_path / "iso_reject"
    bench.mkdir()
    write_artifact(bench, "reference.csv", "t,x\na,b")
    write_artifact(bench, "raw/export.csv", "x")
    (tmp_path / "outside.csv").write_text("x", encoding="utf-8")

    with pytest.raises(ValueError, match="generated benchmark files"):
        suggester.suggest_reference_artifacts(bench, ("reference.csv",))

    with pytest.raises(ValueError, match="valid in-directory"):
        suggester.suggest_reference_artifacts(bench, (str((tmp_path / "outside.csv").resolve()),))

    with pytest.raises(ValueError, match="valid in-directory"):
        suggester.suggest_reference_artifacts(bench, ("../outside.csv",))

    with pytest.raises(ValueError, match="existing file"):
        suggester.suggest_reference_artifacts(bench, ("missing.csv",))
