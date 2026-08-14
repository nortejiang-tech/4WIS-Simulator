"""Tests — the README status block generator (S4 · D6).

The counts in the README used to be hand-written and always stale. The
contract now: every number in the block was parsed from gate output the
moment the gate ran, anything unparsed is omitted rather than guessed, a
failing run never rewrites the block, and the rewrite is a pure replace
between markers that leaves the surrounding prose alone.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "scripts"


def _load(name: str, path: Path):
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


prc = _load("prc_status", SCRIPTS / "pre_release_check.py")
rrs = _load("rrs_status", SCRIPTS / "refresh_readme_status.py")

GATE_OUTPUTS = {
    "pytest": "804 passed, 2 warnings in 116.88s (0:01:56)",
    "vitest": "      Tests  37 passed (37)",
    "smoke": "  ✓  扰动 CRUD\n结果：32/32 通过",
    "golden": "step_steer_60kmh: ok (600 samples)\nsw_x: ok\nsw_y: failed",
    "benchmarks": "analytic_steady_circle_30kmh: ok (6 metrics)\n"
                  "analytic_step_steer_30kmh: ok (9 metrics)",
    "e2e": "  53 passed (41.2s)",
}


def test_every_fact_comes_from_gate_output():
    facts = prc.parse_status_facts(GATE_OUTPUTS)
    assert facts["pytest_passed"] == 804
    assert facts["vitest_passed"] == 37
    assert facts["smoke"] == "32/32 通过"
    assert facts["e2e_passed"] == 53
    # golden: the ok count and the total (one failed) are both reported —
    # the block must not round 2/3 up to a pass.
    assert (facts["golden_ok"], facts["golden_total"]) == (2, 3)
    assert (facts["benchmark_ok"], facts["benchmark_total"]) == (2, 2)


def test_an_unparsed_fact_is_omitted_not_guessed():
    facts = prc.parse_status_facts({"pytest": "collection error"})
    assert facts == {}


def test_rewrite_replaces_only_between_the_markers(tmp_path):
    readme = tmp_path / "README.md"
    before = "prose above\n"
    after = "\nprose below\n"
    block = rrs.render_block(prc.parse_status_facts(GATE_OUTPUTS))
    readme.write_text(
        before + rrs.BEGIN + "\nold stale block\n" + rrs.END + after,
        encoding="utf-8")
    assert rrs.rewrite(prc.parse_status_facts(GATE_OUTPUTS), readme) is True
    text = readme.read_text(encoding="utf-8")
    assert text.startswith(before) and text.endswith(after)
    assert block in text and "old stale block" not in text
    # Rewriting with the same facts is a no-op.
    assert rrs.rewrite(prc.parse_status_facts(GATE_OUTPUTS), readme) is False


def test_the_block_pins_the_current_version(tmp_path):
    version = rrs.read_version()
    block = rrs.render_block(prc.parse_status_facts(GATE_OUTPUTS))
    assert f"`v{version}`" in block


def test_check_refuses_a_missing_block_and_a_stale_version(tmp_path):
    good = tmp_path / "good.md"
    good.write_text(
        "x\n" + rrs.render_block(prc.parse_status_facts(GATE_OUTPUTS)) + "\ny",
        encoding="utf-8")
    assert rrs.check(good) == 0

    missing = tmp_path / "missing.md"
    missing.write_text("no markers here", encoding="utf-8")
    assert rrs.check(missing) == 1

    stale = tmp_path / "stale.md"
    block = rrs.render_block(prc.parse_status_facts(GATE_OUTPUTS))
    stale_block = block.replace(f"`v{rrs.read_version()}`", "`v0.0.1`")
    stale.write_text(stale_block, encoding="utf-8")
    assert rrs.check(stale) == 1
