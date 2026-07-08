#!/usr/bin/env python3
"""Run the 4WIS Simulator pre-release verification gate.

The script is intentionally mechanical: it checks version consistency first,
then runs the same local commands used before packaging and publishing a
release. It does not build portable zips or upload anything.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "backend"
FRONTEND = ROOT / "frontend"


def log(msg: str) -> None:
    print(f"[pre-release] {msg}", flush=True)


def read_backend_version() -> str:
    init_py = ROOT / "backend" / "src" / "sim4wis" / "__init__.py"
    match = re.search(r'^__version__\s*=\s*["\']([^"\']+)["\']', init_py.read_text(), re.M)
    if not match:
        raise RuntimeError(f"could not find __version__ in {init_py}")
    return match.group(1)


def read_pyproject_version() -> str:
    pyproject = BACKEND / "pyproject.toml"
    match = re.search(r'^version\s*=\s*["\']([^"\']+)["\']', pyproject.read_text(), re.M)
    if not match:
        raise RuntimeError(f"could not find project version in {pyproject}")
    return match.group(1)


def read_versions() -> dict[str, str]:
    package = json.loads((FRONTEND / "package.json").read_text())
    lock = json.loads((FRONTEND / "package-lock.json").read_text())
    versions = {
        "backend/src/sim4wis/__init__.py": read_backend_version(),
        "backend/pyproject.toml": read_pyproject_version(),
        "frontend/package.json": package["version"],
        "frontend/package-lock.json": lock["version"],
        "frontend/package-lock.json packages['']": lock["packages"][""]["version"],
    }
    return versions


def check_versions() -> int:
    versions = read_versions()
    width = max(len(k) for k in versions)
    for source, version in versions.items():
        log(f"version {source:<{width}} = {version}")
    unique = set(versions.values())
    if len(unique) == 1:
        return 0
    print("\nVersion mismatch:", file=sys.stderr)
    for source, version in versions.items():
        print(f"  {source}: {version}", file=sys.stderr)
    return 1


def command(cmd: list[str], cwd: Path) -> int:
    log(f"$ {' '.join(cmd)}  (cwd={cwd.relative_to(ROOT)})")
    proc = subprocess.run(cmd, cwd=cwd)
    return proc.returncode


def backend_python() -> str:
    venv_python = BACKEND / ".venv" / "bin" / "python"
    if venv_python.exists():
        return str(venv_python)
    return sys.executable


def git_dirty() -> bool:
    proc = subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if proc.returncode != 0:
        log("git status unavailable; skipping dirty-tree check")
        return False
    return bool(proc.stdout.strip())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--skip-tests",
        action="store_true",
        help="skip pytest, smoke_test, and golden experiment regression",
    )
    parser.add_argument("--skip-build", action="store_true", help="skip frontend production build")
    parser.add_argument("--skip-e2e", action="store_true", help="skip Playwright browser smoke")
    parser.add_argument(
        "--require-reference-data",
        action="store_true",
        help="fail if validation_data has no external/reference benchmarks",
    )
    parser.add_argument(
        "--require-independent-reference",
        action="store_true",
        help="fail unless validation_data has at least one passing external_tool/bench/scaled_vehicle/full_vehicle benchmark",
    )
    parser.add_argument(
        "--strict-git",
        action="store_true",
        help="fail if the working tree is dirty",
    )
    parser.add_argument(
        "--require-portable-zips",
        action="store_true",
        help="fail unless current-version macOS and Windows portable zip files exist",
    )
    args = parser.parse_args()

    failures = 0
    failures += check_versions()
    py = backend_python()

    if git_dirty():
        msg = "working tree has uncommitted changes"
        if args.strict_git:
            print(msg, file=sys.stderr)
            failures += 1
        else:
            log(f"warning: {msg}")

    npm = "npm.cmd" if sys.platform.startswith("win") else "npm"
    failures += command([npm, "ci", "--dry-run", "--ignore-scripts"], FRONTEND)
    release_cmd = [py, "scripts/check_release_assets.py"]
    if args.require_portable_zips:
        release_cmd.append("--require-portable-zips")
    failures += command(release_cmd, ROOT)

    if not args.skip_tests:
        failures += command([py, "-m", "pytest", "tests/"], BACKEND)
        failures += command([py, "scripts/smoke_test.py"], ROOT)
        failures += command([py, "scripts/check_golden_experiments.py"], ROOT)
        ref_cmd = [py, "scripts/check_reference_benchmarks.py"]
        if args.require_reference_data:
            ref_cmd.append("--require-data")
        if args.require_independent_reference:
            ref_cmd.append("--require-independent-source")
        failures += command(ref_cmd, ROOT)
        failures += command(
            [
                py,
                "scripts/check_reference_benchmarks.py",
                "--check-report",
                "docs/reports/reference_benchmark_review.md",
            ],
            ROOT,
        )
        failures += command(
            [
                py,
                "scripts/check_v1_readiness.py",
                "--check-report",
                "docs/reports/v1_readiness.md",
            ],
            ROOT,
        )

    failures += command([npm, "run", "type-check"], FRONTEND)

    if not args.skip_build:
        failures += command([npm, "run", "build"], FRONTEND)

    if not args.skip_e2e:
        failures += command([npm, "run", "e2e:prod"], FRONTEND)

    if failures:
        log(f"failed with {failures} failing step(s)")
        return 1
    log("all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
