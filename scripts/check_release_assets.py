#!/usr/bin/env python3
"""Verify release-facing docs and package artifacts are present.

This checker is intentionally read-only. By default it validates tracked
delivery materials that should stay current before every release check. Portable
zip files are only required when --require-portable-zips is passed, because
building and uploading those assets remains a manual release step.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TARGETS = ("macos-arm64", "windows-x64")
REQUIRED_MANUAL_FIGS = (
    "00_quickstart.png",
    "01_run_overview.png",
    "02_view3d.png",
    "03_model_panel.png",
    "04_strategy_panel.png",
    "05_drive_panel.png",
    "06_crab_drive.png",
    "07_gp_panel.png",
    "08_gp_frontrear.png",
    "09_gp_perwheel.png",
    "10_gp_holonomic.png",
    "11_scene_page.png",
    "12_disturbance.png",
    "13_fault_panel.png",
    "14_vehicle_page.png",
    "14a_vehicle_diagram.png",
    "14b_kingpin_diagram.png",
    "14c_rack_diagram.png",
    "15_experiment_page.png",
    "16_analysis_kpi.png",
    "17_analysis_charts.png",
    "18_replay.png",
    "19_load_page.png",
    "20_theory_page.png",
    "21_cmdk.png",
    "gif_crab.gif",
    "gif_drive.gif",
    "gif_frontrear.gif",
    "gif_holonomic.gif",
    "gif_perwheel.gif",
)
REQUIRED_REPORTS = (
    "docs/reports/reference_benchmark_review.md",
    "docs/reports/single_wheel_failure_metrics.json",
    "docs/reports/single_wheel_failure_safety_analysis.html",
)
REQUIRED_SCRIPTS = (
    "scripts/build_manual.py",
    "scripts/build_portable.py",
)


@dataclass(frozen=True)
class AssetCheck:
    path: Path
    ok: bool
    detail: str


def read_version(root: Path = ROOT) -> str:
    init_py = root / "backend" / "src" / "sim4wis" / "__init__.py"
    match = re.search(r'^__version__\s*=\s*["\']([^"\']+)["\']', init_py.read_text(), re.M)
    if not match:
        raise RuntimeError(f"could not find __version__ in {init_py}")
    return match.group(1)


def _rel(path: Path, root: Path) -> str:
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def _file_check(path: Path, root: Path, min_bytes: int = 1) -> AssetCheck:
    if not path.is_file():
        return AssetCheck(path, False, "missing")
    size = path.stat().st_size
    if size < min_bytes:
        return AssetCheck(path, False, f"too small ({size} bytes)")
    return AssetCheck(path, True, f"{size} bytes")


def check_release_assets(
    root: Path = ROOT,
    require_portable_zips: bool = False,
    targets: tuple[str, ...] = DEFAULT_TARGETS,
) -> tuple[int, list[AssetCheck]]:
    version = read_version(root)
    checks: list[AssetCheck] = []

    readme = root / "README.md"
    readme_check = _file_check(readme, root, min_bytes=1024)
    if readme_check.ok:
        text = readme.read_text(encoding="utf-8")
        if f"`v{version}`" not in text and f"v{version}" not in text:
            readme_check = AssetCheck(readme, False, f"does not mention v{version}")
    checks.append(readme_check)

    manual = root / "docs" / "user_manual.html"
    manual_check = _file_check(manual, root, min_bytes=100_000)
    if manual_check.ok:
        text = manual.read_text(encoding="utf-8", errors="replace")
        missing = [
            marker
            for marker in (
                f"v{version}",
                "4WIS Simulator 使用说明书",
                "scripts/build_manual.py",
            )
            if marker not in text
        ]
        if missing:
            manual_check = AssetCheck(manual, False, f"missing marker(s): {', '.join(missing)}")
    checks.append(manual_check)

    fig_root = root / "docs" / "manual_figs"
    for name in REQUIRED_MANUAL_FIGS:
        checks.append(_file_check(fig_root / name, root, min_bytes=1024))

    for path in REQUIRED_REPORTS:
        checks.append(_file_check(root / path, root, min_bytes=1024))

    for path in REQUIRED_SCRIPTS:
        checks.append(_file_check(root / path, root, min_bytes=1024))

    if require_portable_zips:
        for target in targets:
            zip_path = root / "dist_portable" / f"4WIS_Simulator_v{version}_{target}.zip"
            checks.append(_file_check(zip_path, root, min_bytes=1_000_000))

    for check in checks:
        status = "ok" if check.ok else "failed"
        print(f"{_rel(check.path, root)}: {status} ({check.detail})")

    failed = [check for check in checks if not check.ok]
    if failed:
        print(f"{len(failed)} release asset check(s) failed", file=sys.stderr)
        return 1, checks
    return 0, checks


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument(
        "--require-portable-zips",
        action="store_true",
        help="also require current-version macOS and Windows portable zip files",
    )
    parser.add_argument("--targets", nargs="+", default=list(DEFAULT_TARGETS), choices=list(DEFAULT_TARGETS))
    args = parser.parse_args()

    code, _checks = check_release_assets(
        args.root,
        require_portable_zips=args.require_portable_zips,
        targets=tuple(args.targets),
    )
    return code


if __name__ == "__main__":
    raise SystemExit(main())
