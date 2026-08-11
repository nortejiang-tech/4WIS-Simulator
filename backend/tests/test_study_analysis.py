"""Tests — criteria verdicts and cross-run aggregation.

The criteria half is mostly about refusing to pass. A criterion that silently
succeeds over an empty selection is how a broken study reports success.
"""

from __future__ import annotations

import pytest

from sim4wis.study import compare
from sim4wis.study.criteria import SelectorError, evaluate, parse_selector
from sim4wis.study.result import Row
from sim4wis.study.spec import Criterion


def _rows():
    return [
        Row(coords={"limit": 2, "v": 60}, label="limit=2 v=60", metrics={"m": 10.0}),
        Row(coords={"limit": 2, "v": 120}, label="limit=2 v=120", metrics={"m": 20.0}),
        Row(coords={"limit": 8, "v": 60}, label="limit=8 v=60", metrics={"m": 11.0}),
        Row(coords={"limit": 8, "v": 120}, label="limit=8 v=120", metrics={"m": 26.0}),
    ]


class TestSelectors:
    @pytest.mark.parametrize("at,expected", [
        ("all", []), ("*", []), ("v == 120", [("v", 120)]),
        ("limit == 8 and v == 60", [("limit", 8), ("v", 60)]),
    ])
    def test_parsing(self, at, expected):
        assert parse_selector(at) == expected

    def test_garbage_is_refused(self):
        with pytest.raises(SelectorError):
            parse_selector("where v is big")


class TestVerdicts:
    def test_all_cells_satisfying_the_clause_passes(self):
        v = evaluate([Criterion(metric="m", must="< 30")], _rows(), ["limit", "v"])[0]
        assert v.passed and v.n_checked == 4 and v.n_failed == 0

    def test_a_failure_names_the_worst_cell(self):
        v = evaluate([Criterion(metric="m", must="< 15")], _rows(), ["limit", "v"])[0]
        assert not v.passed and v.n_failed == 2
        assert v.worst_label == "limit=8 v=120" and v.worst_value == 26.0

    def test_selector_narrows_the_check(self):
        v = evaluate([Criterion(metric="m", must="< 15", at="v == 60")],
                     _rows(), ["limit", "v"])[0]
        assert v.passed and v.n_checked == 2

    def test_a_selector_matching_nothing_fails(self):
        v = evaluate([Criterion(metric="m", must="< 15", at="v == 999")],
                     _rows(), ["limit", "v"])[0]
        assert not v.passed and "matched no cells" in v.note

    def test_a_selector_naming_an_unknown_axis_fails(self):
        v = evaluate([Criterion(metric="m", must="< 15", at="speed == 60")],
                     _rows(), ["limit", "v"])[0]
        assert not v.passed and "unknown axis" in v.note

    def test_a_metric_missing_everywhere_fails(self):
        rows = [Row(coords={"limit": 2}, label="a", metrics={})]
        v = evaluate([Criterion(metric="m", must="< 1")], rows, ["limit"])[0]
        assert not v.passed and "missing from all" in v.note

    def test_abs_clauses_bound_magnitude(self):
        rows = [Row(coords={"limit": 1}, label="a", metrics={"m": -3.0})]
        assert not evaluate([Criterion(metric="m", must="abs < 2")], rows, ["limit"])[0].passed
        assert evaluate([Criterion(metric="m", must="< 2")], rows, ["limit"])[0].passed

    def test_nan_never_satisfies_a_criterion(self):
        rows = [Row(coords={"limit": 1}, label="a", metrics={"m": float("nan")})]
        assert not evaluate([Criterion(metric="m", must="< 1")], rows, ["limit"])[0].passed


class TestCompare:
    def test_group_reports_spread_across_the_other_axes(self):
        groups = compare.group(_rows(), "limit", ["m"])
        assert [g["limit"] for g in groups] == [2, 8]
        assert groups[0]["metrics"]["m"] == pytest.approx(
            {"mean": 15.0, "min": 10.0, "max": 20.0, "spread": 10.0, "n": 2}
        )

    def test_deltas_compare_like_with_like(self):
        # against={"limit": 2} pins one axis; each row is compared with the
        # reference sharing its speed, not with a single global cell.
        out = compare.build(_rows(), ["m"], ["limit", "v"], None, {"limit": 2})
        by_label = {d["label"]: d for d in out["deltas"]}
        assert by_label["limit=8 v=60"]["reference_label"] == "limit=2 v=60"
        assert by_label["limit=8 v=60"]["metrics"]["m"]["delta"] == pytest.approx(1.0)
        assert by_label["limit=8 v=120"]["metrics"]["m"]["delta"] == pytest.approx(6.0)
        assert by_label["limit=8 v=120"]["metrics"]["m"]["delta_pct"] == pytest.approx(30.0)

    def test_a_zero_reference_yields_no_percentage(self):
        rows = [Row(coords={"k": 0}, label="a", metrics={"m": 0.0}),
                Row(coords={"k": 1}, label="b", metrics={"m": 5.0})]
        out = compare.build(rows, ["m"], ["k"], None, {"k": 0})
        assert out["deltas"][1]["metrics"]["m"]["delta_pct"] is None

    def test_an_unmatched_reference_warns_instead_of_failing(self):
        out = compare.build(_rows(), ["m"], ["limit", "v"], None, {"limit": 99})
        assert "deltas" not in out and out["warnings"]
