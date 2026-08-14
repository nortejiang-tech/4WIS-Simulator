#!/usr/bin/env python3
"""Run the 4WIS Simulator pre-release verification gate.

The script is intentionally mechanical: it checks version consistency first,
then runs the same local commands used before packaging and publishing a
release. It does not build portable zips or upload anything.

The one thing it *writes* is the pair of generated reports under docs/reports
(v1 readiness, reference-benchmark review). Those are derived artifacts, so a
stale one says nothing about the release — it only means the last release
forgot to rerun the generator. The gate regenerates them and tells you to
commit the result; `--no-refresh-reports` restores the old verify-only
behaviour for CI, where the gate must not touch the tree.
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


def refresh_report(py: str, script: str, report: str, refresh: bool) -> tuple[int, bool]:
    """Bring a tracked, generated report up to date — or verify it, if asked.

    `docs/reports/v1_readiness.md` and `docs/reports/reference_benchmark_review.md`
    are pure generated artifacts: their content is a function of the repository,
    so "stale" is never a finding about the release. It only means the previous
    release forgot to rerun the generator. Failing the gate on that stopped a
    release to demand a command the gate could run itself — which is exactly how
    v0.101.0 shipped with a readiness report still pinned to v0.100.0, reporting
    its own `release_delivery` check as FAIL.

    So by default the gate writes them. It still says loudly when a file moved,
    because the new content has to be committed for the release to carry it, and
    `--strict-git` (the "everything is already committed" mode) treats a moved
    report as a failure.

    Returns (exit code, whether the file changed).
    """
    path = ROOT / report
    if not refresh:
        return command([py, script, "--check-report", report], ROOT), False

    before = path.read_text(encoding="utf-8") if path.exists() else None
    code = command([py, script, "--report", report], ROOT)
    if code != 0:
        return code, False
    after = path.read_text(encoding="utf-8") if path.exists() else None
    if after == before:
        log(f"{report}: already up to date")
        return 0, False
    log(f"{report}: REGENERATED (was {'missing' if before is None else 'stale'}) — commit it before tagging")
    return 0, True


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
        "--check-incoming-audit",
        action="store_true",
        help="fail if any benchmark under incoming is not ready for promotion",
    )
    parser.add_argument(
        "--incoming-root",
        type=Path,
        default=(ROOT / "validation_data" / ".incoming"),
        help="incoming benchmark root for promotion-readiness audit",
    )
    parser.add_argument(
        "--strict-git",
        action="store_true",
        help="fail if the working tree is dirty, or if a generated report had to be rewritten",
    )
    parser.add_argument(
        "--no-refresh-reports",
        action="store_true",
        help=(
            "verify the generated readiness/benchmark reports instead of rewriting them "
            "(for CI, where the gate must not modify the tree)"
        ),
    )
    parser.add_argument(
        "--require-portable-zips",
        action="store_true",
        help="fail unless current-version macOS and Windows portable zip files exist",
    )
    parser.add_argument(
        "--strict-v1",
        action="store_true",
        help="require strict-v1 readiness checks to pass",
    )
    args = parser.parse_args()

    failures = 0
    regenerated = 0
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
        # -n auto: the suite is worker-safe by construction (env dirs through
        # monkeypatch+tmp_path, no fixed ports) and parallel is the difference
        # between 8 and 2 minutes of gate time (S5). xdist is a dev extra.
        failures += command([py, "-m", "pytest", "tests/", "-q", "-n", "auto"], BACKEND)
        failures += command([py, "scripts/smoke_test.py"], ROOT)
        failures += command([py, "scripts/check_golden_experiments.py"], ROOT)
        ref_cmd = [py, "scripts/check_reference_benchmarks.py"]
        if args.require_reference_data:
            ref_cmd.append("--require-data")
        if args.require_independent_reference:
            ref_cmd.append("--require-independent-source")
        failures += command(ref_cmd, ROOT)
        code, moved = refresh_report(
            py,
            "scripts/check_reference_benchmarks.py",
            "docs/reports/reference_benchmark_review.md",
            refresh=not args.no_refresh_reports,
        )
        failures += code
        regenerated += moved
        if args.check_incoming_audit:
            failures += command(
                [
                    py,
                    "scripts/check_reference_benchmarks.py",
                    "--incoming-audit",
                    "--incoming-root",
                    str(args.incoming_root),
                ],
                ROOT,
            )
        code, moved = refresh_report(
            py,
            "scripts/check_v1_readiness.py",
            "docs/reports/v1_readiness.md",
            refresh=not args.no_refresh_reports,
        )
        failures += code
        regenerated += moved

    if args.strict_v1:
        readiness_cmd = [py, "scripts/check_v1_readiness.py", "--strict-v1"]
        if args.require_portable_zips:
            readiness_cmd.append("--require-portable-zips")
        failures += command(readiness_cmd, ROOT)

    failures += command([npm, "run", "type-check"], FRONTEND)
    # Front-end unit tests (vitest). The e2e suite exercises the app shell; this
    # covers the pure logic underneath it — the input state machine and display
    # maths, which had no coverage at all until three blocking defects were
    # found there by inspection.
    failures += command([npm, "run", "test"], FRONTEND)

    if not args.skip_build:
        failures += command([npm, "run", "build"], FRONTEND)

    if not args.skip_e2e:
        failures += command([npm, "run", "e2e:prod"], FRONTEND)

    if regenerated:
        # The dirty-tree check above ran before these were written, so say it
        # again at the end where it will not scroll past unread.
        log(f"{regenerated} generated report(s) rewritten — commit them before tagging")
        if args.strict_git:
            print(
                "generated reports were out of date; commit the rewritten files and rerun",
                file=sys.stderr,
            )
            failures += 1

    if failures:
        log(f"failed with {failures} failing step(s)")
        return 1
    log("all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
