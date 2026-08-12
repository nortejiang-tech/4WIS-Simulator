"""Tests — the metric registry and the expression tier.

The rejection cases matter more than the arithmetic. This is the tier an agent
writes without review, so anything that escapes the expression sandbox escapes
into the backend process.
"""

from __future__ import annotations

import math

import pytest

from sim4wis.study.metrics import (
    BUILTIN,
    PROCEDURE,
    ExpressionError,
    collect,
    describe_metrics,
    evaluate_expression,
    validate_expression,
)
from sim4wis.study.spec import ExprMetric

CH = {"vx": [20.0] * 10, "vy": [0.5] * 10, "ay": [0.0, 1.0, -3.0] + [2.0] * 7}
T = [i * 0.1 for i in range(10)]


class TestSandbox:
    @pytest.mark.parametrize("expr", [
        "__import__('os').system('echo pwned')",
        "vx.mean()",                       # attribute access
        "(lambda: 1)()",                   # lambda
        "[x for x in vx][0]",              # comprehension
        "open('/etc/passwd')",             # non-whitelisted call
        "nope + 1",                        # unknown name
        "vx if __import__ else vy",
    ])
    def test_unsafe_or_unknown_expressions_are_refused(self, expr):
        with pytest.raises(ExpressionError):
            validate_expression(expr, list(CH))
            evaluate_expression(expr, T, CH)

    def test_an_expression_returning_an_array_is_refused(self):
        # Silently reducing it would invent a metric the author did not write.
        with pytest.raises(ExpressionError, match="not a single number"):
            evaluate_expression("vx", T, CH)

    def test_the_error_names_the_available_functions(self):
        with pytest.raises(ExpressionError, match="steady"):
            validate_expression("wat(vx)", list(CH))


class TestEvaluation:
    def test_steady_uses_the_tail_of_the_record(self):
        rising = {"x": list(range(100))}
        # Last 20 % of 0..99 → mean of 80..99.
        assert evaluate_expression("steady(x)", list(range(100)), rising) == pytest.approx(89.5)

    def test_peak_is_the_largest_magnitude_either_way(self):
        assert evaluate_expression("peak(ay)", T, CH) == pytest.approx(3.0)

    def test_units_and_trig_compose(self):
        got = evaluate_expression("degrees(atan2(steady(vy), steady(vx)))", T, CH)
        assert got == pytest.approx(math.degrees(math.atan2(0.5, 20.0)))

    def test_nan_samples_do_not_poison_the_result(self):
        ch = {"x": [1.0, None, 3.0]}                  # None is how the CSV stores non-finite
        assert evaluate_expression("mean(x)", [0, 1, 2], ch) == pytest.approx(2.0)


class TestCollection:
    def test_builtins_come_straight_from_the_stored_kpis(self):
        mv = collect(["yaw_gain_dps"], [], {"yaw_gain_dps": 8.25})
        assert mv.values == {"yaw_gain_dps": pytest.approx(8.25)} and not mv.errors

    def test_a_missing_builtin_is_an_error_not_a_nan(self):
        # A NaN in a comparison table cannot be told apart from a measurement
        # that genuinely came out undefined.
        mv = collect(["yaw_gain_dps"], [], {})
        assert "yaw_gain_dps" not in mv.values
        assert "step-kind" in mv.errors["yaw_gain_dps"]

    def test_unknown_metric_names_are_reported(self):
        assert "unknown metric" in collect(["nope"], [], {}).errors["nope"]

    def test_expression_failure_is_reported_per_metric(self):
        mv = collect(
            ["yaw_gain_dps"],
            [ExprMetric(name="bad", expr="steady(nope)")],
            {"yaw_gain_dps": 1.0}, T, CH,
        )
        assert mv.values["yaw_gain_dps"] == 1.0        # one bad metric does not sink the rest
        assert "bad" in mv.errors

    def test_expressions_without_channels_are_reported(self):
        mv = collect([], [ExprMetric(name="b", expr="steady(vx)")], {})
        assert "channels" in mv.errors["b"]


def test_every_metric_is_described_with_a_capability():
    """The catalogue an agent reads must cover every tier, and label it.

    `describe_metrics` is the only place a caller finds out a metric exists,
    so a tier that is registered but not described is a metric nobody can
    discover — and one that is described without `requires` is a metric an
    envelope guard cannot reason about.
    """
    described = {m["name"]: m for m in describe_metrics()}
    assert set(described) == set(BUILTIN) | set(PROCEDURE)
    assert all(m["requires"] for m in described.values())
    assert all(described[n]["source"] == "builtin" for n in BUILTIN)
    assert all(described[n]["source"] == "procedure" for n in PROCEDURE)
