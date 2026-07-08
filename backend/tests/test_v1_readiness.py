from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "check_v1_readiness.py"


def load_checker():
    spec = importlib.util.spec_from_file_location("check_v1_readiness", SCRIPT)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_v1_readiness_current_repo_reports_known_state() -> None:
    checker = load_checker()

    code, checks = checker.evaluate_v1_readiness(ROOT)

    by_id = {check.check_id: check for check in checks}
    assert code == 0
    assert by_id["golden_regression"].status == "pass"
    assert by_id["reference_reproducibility"].status == "pass"
    assert by_id["browser_smoke"].status == "pass"
    assert by_id["release_delivery"].status == "pass"
    strict_blockers = [check for check in checks if check.strict_blocker]
    strict_code, _checks = checker.evaluate_v1_readiness(ROOT, strict_v1=True)
    assert strict_code == (1 if strict_blockers else 0)

    report = checker.render_readiness_report(ROOT, checks)
    assert "# 4WIS v1 Readiness Report" in report
    assert "Strict v1 blockers:" in report
    assert "`independent_reference`" in report


def test_v1_readiness_report_round_trip(tmp_path: Path) -> None:
    checker = load_checker()
    code, checks = checker.evaluate_v1_readiness(ROOT)
    assert code == 0

    report = tmp_path / "v1_readiness.md"
    checker.write_readiness_report(report, ROOT, checks)

    assert checker.check_readiness_report_fresh(report, ROOT, checks)
    report.write_text(report.read_text(encoding="utf-8") + "\nmanual stale edit\n", encoding="utf-8")
    assert not checker.check_readiness_report_fresh(report, ROOT, checks)


def test_golden_regression_fails_when_required_case_missing(tmp_path: Path) -> None:
    checker = load_checker()
    golden = tmp_path / "docs" / "golden_experiments.json"
    golden.parent.mkdir(parents=True)
    (tmp_path / "scripts").mkdir()
    (tmp_path / "scripts" / "check_golden_experiments.py").write_text("# placeholder\n", encoding="utf-8")
    golden.write_text(
        json.dumps(
            {
                "schema": 1,
                "experiments": {"step_steer_60kmh": {"kpis": {}}},
                "tolerances": {"step_steer_60kmh": {}},
            }
        ),
        encoding="utf-8",
    )

    result = checker.check_golden_regression(tmp_path)

    assert result.status == "fail"
    assert "missing experiments" in result.detail
