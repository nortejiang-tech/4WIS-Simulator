#!/usr/bin/env python3
"""Summarize whether the project is ready to claim the v1.0 definition.

This checker is evidence-oriented. The default mode is suitable for regular
pre-release runs: it reports known v1 gaps but exits successfully unless a
checked artifact is malformed or a checked report is stale. Use --strict-v1
when deciding whether the current tree is allowed to become a v1.0 release.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
READINESS_REPORT = ROOT / "docs" / "reports" / "v1_readiness.md"
REQUIRED_GOLDENS = {
    "step_steer_60kmh",
    "iso3888_dlc_60kmh",
    "sw_straight100_rl_stuck_value_baseline",
    "sw_straight100_rl_stuck_value_mitigated",
    "sw_curve60_fl_free_caster_baseline",
}


@dataclass(frozen=True)
class ReadinessCheck:
    check_id: str
    title: str
    status: str
    detail: str
    evidence: tuple[str, ...] = field(default_factory=tuple)
    strict_required: bool = True

    @property
    def ok(self) -> bool:
        return self.status == "pass"

    @property
    def strict_blocker(self) -> bool:
        return self.strict_required and not self.ok


def _load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _rel(path: Path, root: Path) -> str:
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def _read_version(root: Path) -> str:
    init_py = root / "backend" / "src" / "sim4wis" / "__init__.py"
    match = re.search(r'^__version__\s*=\s*["\']([^"\']+)["\']', init_py.read_text(), re.M)
    if not match:
        raise RuntimeError(f"could not find __version__ in {init_py}")
    return match.group(1)


def _file_exists(root: Path, rel_path: str, min_bytes: int = 1) -> bool:
    path = root / rel_path
    return path.is_file() and path.stat().st_size >= min_bytes


def check_golden_regression(root: Path) -> ReadinessCheck:
    path = root / "docs" / "golden_experiments.json"
    script = root / "scripts" / "check_golden_experiments.py"
    evidence = (_rel(path, root), _rel(script, root))
    if not path.is_file():
        return ReadinessCheck(
            "golden_regression",
            "Golden experiment regression",
            "fail",
            "docs/golden_experiments.json is missing.",
            evidence,
        )
    if not script.is_file():
        return ReadinessCheck(
            "golden_regression",
            "Golden experiment regression",
            "fail",
            "scripts/check_golden_experiments.py is missing.",
            evidence,
        )
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return ReadinessCheck(
            "golden_regression",
            "Golden experiment regression",
            "fail",
            f"golden baseline is not valid JSON: {exc}",
            evidence,
        )
    experiments = data.get("experiments", {})
    tolerances = data.get("tolerances", {})
    missing = sorted(REQUIRED_GOLDENS - set(experiments))
    missing_tol = sorted(name for name in REQUIRED_GOLDENS if name not in tolerances)
    if data.get("schema") != 1:
        return ReadinessCheck(
            "golden_regression",
            "Golden experiment regression",
            "fail",
            "golden baseline schema must be 1.",
            evidence,
        )
    if missing or missing_tol:
        return ReadinessCheck(
            "golden_regression",
            "Golden experiment regression",
            "fail",
            f"missing experiments={missing}; missing tolerances={missing_tol}.",
            evidence,
        )
    return ReadinessCheck(
        "golden_regression",
        "Golden experiment regression",
        "pass",
        f"{len(experiments)} golden cases are recorded, including step steer, ISO 3888 DLC, and single-wheel-failure quick samples.",
        evidence,
    )


def check_validation_matrix(root: Path) -> ReadinessCheck:
    path = root / "docs" / "validation_matrix.md"
    evidence = (_rel(path, root),)
    if not path.is_file():
        return ReadinessCheck(
            "validation_matrix",
            "Validation matrix boundaries",
            "fail",
            "docs/validation_matrix.md is missing.",
            evidence,
        )
    text = path.read_text(encoding="utf-8")
    required_terms = (
        "L0 概念",
        "L1 单元",
        "L2 集成",
        "L3 参考对照",
        "L4 实测对照",
        "参考/外部/实测对照接入",
        "--require-independent-source",
        "当前最高风险",
    )
    missing = [term for term in required_terms if term not in text]
    rows = [line for line in text.splitlines() if line.startswith("| ") and "`" in line]
    if missing:
        return ReadinessCheck(
            "validation_matrix",
            "Validation matrix boundaries",
            "fail",
            f"validation matrix is missing required marker(s): {', '.join(missing)}.",
            evidence,
        )
    if len(rows) < 10:
        return ReadinessCheck(
            "validation_matrix",
            "Validation matrix boundaries",
            "fail",
            f"validation matrix has too few evidence rows ({len(rows)}).",
            evidence,
        )
    return ReadinessCheck(
        "validation_matrix",
        "Validation matrix boundaries",
        "pass",
        f"{len(rows)} capability rows declare levels, evidence, and boundaries.",
        evidence,
    )


def check_reference_evidence(root: Path) -> list[ReadinessCheck]:
    checker = _load_module(root / "scripts" / "check_reference_benchmarks.py", "check_reference_benchmarks_for_v1")
    code, results = checker.check_reference_benchmarks(
        root / "validation_data",
        require_data=True,
        check_report_path=root / "docs" / "reports" / "reference_benchmark_review.md",
    )
    ok_count = sum(1 for result in results if result.ok)
    independent_count = sum(1 for result in results if result.ok and result.has_independent_source)
    metric_count = sum(result.checked_metrics for result in results if result.ok)
    evidence = (
        "validation_data",
        "docs/reports/reference_benchmark_review.md",
        "scripts/check_reference_benchmarks.py",
    )
    if code != 0:
        first_failure = "reference benchmarks are missing or invalid"
        for result in results:
            if result.failures:
                first_failure = result.failures[0]
                break
        return [
            ReadinessCheck(
                "reference_reproducibility",
                "Reference benchmark reproducibility",
                "fail",
                first_failure,
                evidence,
            ),
            ReadinessCheck(
                "independent_reference",
                "Independent external/measured reference",
                "gap",
                "cannot be evaluated until reference reproducibility passes.",
                evidence,
            ),
        ]
    checks = [
        ReadinessCheck(
            "reference_reproducibility",
            "Reference benchmark reproducibility",
            "pass",
            f"{ok_count}/{len(results)} reference benchmarks pass with {metric_count} checked metrics.",
            evidence,
        )
    ]
    if independent_count:
        checks.append(
            ReadinessCheck(
                "independent_reference",
                "Independent external/measured reference",
                "pass",
                f"{independent_count} passing independent external/measured benchmark(s) are present.",
                evidence,
            )
        )
    else:
        checks.append(
            ReadinessCheck(
                "independent_reference",
                "Independent external/measured reference",
                "gap",
                "no passing external_tool, bench, scaled_vehicle, or full_vehicle benchmark is present.",
                evidence,
            )
        )
    return checks


def check_incoming_reference_pipeline(root: Path) -> ReadinessCheck:
    checker = _load_module(root / "scripts" / "check_reference_benchmarks.py", "check_reference_benchmarks_for_v1_incoming")
    code, statuses = checker.audit_incoming_benchmarks(root / "validation_data" / ".incoming")
    evidence = ("validation_data/.incoming", "scripts/check_reference_benchmarks.py")

    if code == 0 and not statuses:
        return ReadinessCheck(
            "incoming_reference_pipeline",
            "Incoming reference pipeline",
            "pass",
            "No incoming benchmark directories found; intake gate is clear.",
            evidence,
            strict_required=False,
        )

    if code != 0:
        blocked = [status.benchmark_id for status in statuses if not status.ready_for_promotion]
        detail = "; ".join(
            f"{status.benchmark_id}: {', '.join(status.blockers)}"
            for status in statuses
            if not status.ready_for_promotion
        )
        if detail:
            detail = detail[:300]
        return ReadinessCheck(
            "incoming_reference_pipeline",
            "Incoming reference pipeline",
            "gap",
            f"{len(blocked)} incoming benchmark(s) blocked for promotion ({detail}).",
            evidence,
            strict_required=False,
        )

    ready = [status for status in statuses if status.ready_for_promotion]
    return ReadinessCheck(
        "incoming_reference_pipeline",
        "Incoming reference pipeline",
        "pass",
        f"{len(ready)}/{len(statuses)} incoming benchmark(s) are ready for promotion.",
        evidence,
        strict_required=False,
    )


def check_frontend_smoke(root: Path) -> ReadinessCheck:
    spec = root / "frontend" / "tests" / "e2e" / "workflow-smoke.spec.ts"
    package = root / "frontend" / "package.json"
    evidence = (_rel(spec, root), _rel(package, root))
    if not spec.is_file():
        return ReadinessCheck(
            "browser_smoke",
            "Browser workflow smoke",
            "fail",
            "Playwright workflow smoke spec is missing.",
            evidence,
        )
    text = spec.read_text(encoding="utf-8")
    test_count = len(re.findall(r"^test\(", text, flags=re.M))
    pkg = json.loads(package.read_text(encoding="utf-8"))
    scripts = pkg.get("scripts", {})
    if "e2e:prod" not in scripts:
        return ReadinessCheck(
            "browser_smoke",
            "Browser workflow smoke",
            "fail",
            "frontend/package.json is missing the e2e:prod script.",
            evidence,
        )
    if test_count < 45:
        return ReadinessCheck(
            "browser_smoke",
            "Browser workflow smoke",
            "fail",
            f"workflow smoke has only {test_count} test cases; expected at least 45 for the current v1 surface.",
            evidence,
        )
    return ReadinessCheck(
        "browser_smoke",
        "Browser workflow smoke",
        "pass",
        f"workflow-smoke.spec.ts contains {test_count} browser smoke cases and package.json exposes e2e:prod.",
        evidence,
    )


def check_report_pipeline(root: Path) -> ReadinessCheck:
    required = (
        "scripts/reporting.py",
        "scripts/study_single_wheel_failure.py",
        "docs/reports/single_wheel_failure_metrics.json",
        "docs/reports/single_wheel_failure_safety_analysis.html",
    )
    missing = [path for path in required if not _file_exists(root, path, min_bytes=1024)]
    if missing:
        return ReadinessCheck(
            "report_pipeline",
            "Research report pipeline",
            "fail",
            f"missing or too small artifact(s): {', '.join(missing)}.",
            required,
        )
    return ReadinessCheck(
        "report_pipeline",
        "Research report pipeline",
        "pass",
        "single-wheel-failure study has reusable report helpers, metrics JSON, and self-contained HTML output.",
        required,
    )


def check_release_delivery(root: Path, require_portable_zips: bool) -> ReadinessCheck:
    checker = _load_module(root / "scripts" / "check_release_assets.py", "check_release_assets_for_v1")
    code, checks = checker.check_release_assets(root, require_portable_zips=require_portable_zips)
    failed = [check for check in checks if not check.ok]
    evidence = (
        "scripts/check_release_assets.py",
        "docs/user_manual.html",
        "docs/manual_figs",
        "dist_portable",
    )
    if code != 0:
        detail = "; ".join(f"{_rel(check.path, root)}: {check.detail}" for check in failed[:3])
        return ReadinessCheck(
            "release_delivery",
            "Release delivery materials",
            "fail",
            detail or "release asset checker failed.",
            evidence,
        )
    suffix = " including portable zips" if require_portable_zips else ""
    return ReadinessCheck(
        "release_delivery",
        "Release delivery materials",
        "pass",
        f"{len(checks)} delivery material checks pass{suffix}.",
        evidence,
    )


def check_physical_validation_interface(root: Path) -> ReadinessCheck:
    protocol = root / "docs" / "reference_benchmark_protocol.md"
    data_readme = root / "validation_data" / "README.md"
    evidence = (_rel(protocol, root), _rel(data_readme, root))
    if not protocol.is_file() or not data_readme.is_file():
        return ReadinessCheck(
            "physical_validation_interface",
            "Physical validation data interface",
            "fail",
            "reference benchmark protocol or validation_data README is missing.",
            evidence,
            strict_required=False,
        )
    text = protocol.read_text(encoding="utf-8") + "\n" + data_readme.read_text(encoding="utf-8")
    required_terms = (
        ("scaled_vehicle",),
        ("full_vehicle",),
        ("sampling", "采样率"),
        ("channels", "通道"),
        ("sensor", "传感器"),
    )
    missing = ["/".join(group) for group in required_terms if not any(term in text for term in group)]
    if missing:
        return ReadinessCheck(
            "physical_validation_interface",
            "Physical validation data interface",
            "gap",
            f"physical-data protocol is missing marker(s): {', '.join(missing)}.",
            evidence,
            strict_required=False,
        )
    return ReadinessCheck(
        "physical_validation_interface",
        "Physical validation data interface",
        "pass",
        "validation_data protocol reserves source types and channel metadata for scaled/full vehicle evidence.",
        evidence,
        strict_required=False,
    )


def evaluate_v1_readiness(
    root: Path = ROOT,
    strict_v1: bool = False,
    require_portable_zips: bool = False,
) -> tuple[int, list[ReadinessCheck]]:
    checks: list[ReadinessCheck] = []
    checks.append(check_golden_regression(root))
    checks.append(check_validation_matrix(root))
    checks.extend(check_reference_evidence(root))
    checks.append(check_incoming_reference_pipeline(root))
    checks.append(check_frontend_smoke(root))
    checks.append(check_report_pipeline(root))
    checks.append(check_release_delivery(root, require_portable_zips=require_portable_zips))
    checks.append(check_physical_validation_interface(root))

    hard_failures = [check for check in checks if check.status == "fail"]
    strict_blockers = [check for check in checks if check.strict_blocker]
    if hard_failures or (strict_v1 and strict_blockers):
        return 1, checks
    return 0, checks


def render_readiness_report(root: Path, checks: list[ReadinessCheck], strict_v1: bool = False) -> str:
    version = _read_version(root)
    strict_blockers = [check for check in checks if check.strict_blocker]
    advisory_gaps = [check for check in checks if not check.ok and not check.strict_required]
    status = "READY" if not strict_blockers else "NOT READY"
    lines = [
        "# 4WIS v1 Readiness Report",
        "",
        f"- Version inspected: `v{version}`",
        f"- Status: **{status}**",
        f"- Strict v1 blockers: {len(strict_blockers)}",
        f"- Advisory gaps: {len(advisory_gaps)}",
        "- Generated by: `scripts/check_v1_readiness.py --report docs/reports/v1_readiness.md`",
        "- Evidence boundary: this report summarizes repository artifacts and checker results; it does not replace running `scripts/pre_release_check.py`.",
        "",
    ]
    if strict_blockers:
        lines += [
            "## Release Blockers",
            "",
        ]
        for check in strict_blockers:
            lines.append(f"- `{check.check_id}`: {check.detail}")
        lines.append("")
    lines += [
        "## Checks",
        "",
        "| Check | Status | Strict v1 | Evidence | Detail |",
        "|---|---|---|---|---|",
    ]
    for check in checks:
        strict_label = "yes" if check.strict_required else "no"
        evidence = "<br>".join(f"`{item}`" for item in check.evidence) or "_none_"
        detail = check.detail.replace("|", "\\|")
        lines.append(f"| `{check.check_id}` | {check.status.upper()} | {strict_label} | {evidence} | {detail} |")
    if not strict_v1:
        lines += [
            "",
            "## Strict Mode",
            "",
            "Run `backend/.venv/bin/python scripts/check_v1_readiness.py --strict-v1 --require-portable-zips` before claiming a v1.0 release.",
        ]
    return "\n".join(lines) + "\n"


def write_readiness_report(path: Path, root: Path, checks: list[ReadinessCheck], strict_v1: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render_readiness_report(root, checks, strict_v1=strict_v1), encoding="utf-8")


def check_readiness_report_fresh(path: Path, root: Path, checks: list[ReadinessCheck], strict_v1: bool = False) -> bool:
    expected = render_readiness_report(root, checks, strict_v1=strict_v1)
    try:
        actual = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        print(f"{path}: v1 readiness report is missing; regenerate with --report {path}", file=sys.stderr)
        return False
    if actual != expected:
        print(f"{path}: v1 readiness report is stale; regenerate with --report {path}", file=sys.stderr)
        return False
    return True


def print_summary(checks: list[ReadinessCheck]) -> None:
    strict_blockers = [check for check in checks if check.strict_blocker]
    status = "READY" if not strict_blockers else "NOT READY"
    print(f"v1 readiness: {status} ({len(strict_blockers)} strict blocker(s))")
    for check in checks:
        marker = check.status.upper()
        required = "strict" if check.strict_required else "advisory"
        print(f"{check.check_id}: {marker} [{required}] - {check.detail}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--strict-v1", action="store_true", help="fail for any strict v1 blocker")
    parser.add_argument("--require-portable-zips", action="store_true", help="include current-version portable zips")
    report_group = parser.add_mutually_exclusive_group()
    report_group.add_argument("--report", type=Path, help="write a Markdown readiness report")
    report_group.add_argument("--check-report", type=Path, help="fail if the readiness report is missing or stale")
    args = parser.parse_args()

    code, checks = evaluate_v1_readiness(
        args.root,
        strict_v1=args.strict_v1,
        require_portable_zips=args.require_portable_zips,
    )
    if args.report is not None:
        write_readiness_report(args.report, args.root, checks, strict_v1=args.strict_v1)
    if args.check_report is not None:
        if not check_readiness_report_fresh(args.check_report, args.root, checks, strict_v1=args.strict_v1):
            code = 1
    print_summary(checks)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
