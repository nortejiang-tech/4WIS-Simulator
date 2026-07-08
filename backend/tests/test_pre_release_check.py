from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "pre_release_check.py"


def load_checker():
    spec = importlib.util.spec_from_file_location("pre_release_check", SCRIPT)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_strict_v1_runs_even_when_tests_are_skipped(monkeypatch) -> None:
    checker = load_checker()
    commands: list[list[str]] = []

    monkeypatch.setattr(sys, "argv", ["pre_release_check.py", "--skip-tests", "--skip-build", "--skip-e2e", "--strict-v1"])
    monkeypatch.setattr(checker, "check_versions", lambda: 0)
    monkeypatch.setattr(checker, "backend_python", lambda: "py")
    monkeypatch.setattr(checker, "git_dirty", lambda: False)

    def fake_command(cmd: list[str], cwd: Path) -> int:
        commands.append(cmd)
        if cmd == ["py", "scripts/check_v1_readiness.py", "--strict-v1"]:
            return 1
        return 0

    monkeypatch.setattr(checker, "command", fake_command)

    assert checker.main() == 1
    assert ["py", "scripts/check_v1_readiness.py", "--strict-v1"] in commands
    assert [cmd for cmd in commands if cmd[:3] == ["py", "-m", "pytest"]] == []


def test_strict_v1_forwards_portable_zip_requirement(monkeypatch) -> None:
    checker = load_checker()
    commands: list[list[str]] = []

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "pre_release_check.py",
            "--skip-tests",
            "--skip-build",
            "--skip-e2e",
            "--strict-v1",
            "--require-portable-zips",
        ],
    )
    monkeypatch.setattr(checker, "check_versions", lambda: 0)
    monkeypatch.setattr(checker, "backend_python", lambda: "py")
    monkeypatch.setattr(checker, "git_dirty", lambda: False)

    def fake_command(cmd: list[str], cwd: Path) -> int:
        commands.append(cmd)
        return 0

    monkeypatch.setattr(checker, "command", fake_command)

    assert checker.main() == 0
    assert ["py", "scripts/check_v1_readiness.py", "--strict-v1", "--require-portable-zips"] in commands
