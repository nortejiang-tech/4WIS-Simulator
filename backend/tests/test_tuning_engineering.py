"""Tests — tuning workbench engineering (direction 4).

Multi-condition weighted cost, the explicit grid channel, parameter-space
margins on the tuned gains (the C5 bisection pattern), record persistence,
and the CLI face.
"""

from __future__ import annotations

import json

import pytest

from sim4wis.steering.tracking.tuning import (
    CostCondition,
    analytic_pid,
    grid_search,
    save_record,
    step_cost,
    tune,
    tune_margins,
)

PID_ANALYTIC = analytic_pid(0.6, 4.0)


class TestMultiConditionCost:
    def test_the_legacy_path_is_one_condition(self):
        legacy = step_cost("pid_single", PID_ANALYTIC)
        explicit = step_cost(
            "pid_single", PID_ANALYTIC,
            conditions=[{"target": 0.01745, "load_torque": 3.0,
                         "t_end": 2.0, "weight": 1.0}])
        assert legacy == pytest.approx(explicit)

    def test_the_aggregate_is_weight_weighted(self):
        one = step_cost("pid_single", PID_ANALYTIC,
                        conditions=[{"target": 0.01745, "load_torque": 3.0}])
        half = step_cost(
            "pid_single", PID_ANALYTIC,
            conditions=[{"target": 0.01745, "load_torque": 3.0, "weight": 0.5},
                        {"target": 0.01745, "load_torque": 3.0, "weight": 0.5}])
        assert one == pytest.approx(half)

    def test_empty_conditions_are_refused(self):
        with pytest.raises(ValueError, match="must not be empty"):
            step_cost("pid_single", PID_ANALYTIC, conditions=[])

    def test_condition_mapping_and_roundtrip(self):
        c = CostCondition.from_mapping({"target": 0.03, "weight": 2.0})
        assert c.target == pytest.approx(0.03)
        assert c.to_dict()["weight"] == pytest.approx(2.0)

    def test_tune_records_the_conditions(self):
        conds = [{"target": 0.01745, "load_torque": 8.0, "weight": 1.0}]
        result = tune("pid_single", max_evals=30, conditions=conds)
        # Recorded normalised: defaults made explicit.
        assert result.conditions == [{"target": 0.01745, "load_torque": 8.0,
                                      "t_end": 2.0, "weight": 1.0}]
        assert result.accepted


class TestGridChannel:
    def test_the_grid_is_deterministic_and_exhaustive(self):
        a = grid_search("pid_single", points=3)
        b = grid_search("pid_single", points=3)
        assert a.tuned_params == b.tuned_params
        assert a.cost_tuned == b.cost_tuned
        assert a.n_evals == 27  # 3 axes, 3 points each
        assert "grid search" in a.note

    def test_an_oversized_grid_is_refused(self):
        with pytest.raises(ValueError, match="max_cells"):
            grid_search("pid_single", points=12)

    def test_grid_and_nelder_mead_agree_on_the_basin(self):
        # Two independent channels, one neighbourhood: the grid best and the
        # NM best must land close in cost — a cross-check, not a coincidence.
        g = grid_search("pid_single", points=3)
        n = tune("pid_single", max_evals=60)
        assert abs(g.cost_tuned - n.cost_tuned) < 0.5 * n.cost_tuned


class TestMargins:
    def test_every_axis_gets_a_margin_with_a_cost_bar(self):
        result = tune("pid_single", max_evals=40)
        margins = tune_margins("pid_single", result.tuned_params, ratio=1.5)
        assert [m.name for m in margins] == list(result.tuned_params)
        for m in margins:
            assert m.cost_base == pytest.approx(result.cost_tuned)
            if m.flip_low is not None:
                assert m.lo <= m.flip_low < m.nominal
                assert m.cost_at_flip_low > 1.5 * m.cost_base
            if m.flip_high is not None:
                assert m.nominal < m.flip_high <= m.hi
                assert m.cost_at_flip_high > 1.5 * m.cost_base

    def test_missing_gains_are_refused(self):
        with pytest.raises(ValueError, match="missing axis entries"):
            tune_margins("pid_single", {"kp": 1.0})


class TestRecord:
    def test_a_record_roundtrips_with_provenance(self, tmp_path):
        result = tune("pid_single", max_evals=20)
        path = save_record(result, tmp_path / "tune.json")
        record = json.loads(path.read_text(encoding="utf-8"))
        assert record["controller"] == "pid_single"
        assert record["accepted"] is True
        assert set(record["tuned_params"]) == {"kp", "ki", "kd"}
        assert record["git"]


class TestCli:
    def test_tune_command_prints_a_record(self, capsys):
        from sim4wis.cli import main

        rc = main(["--json", "tune", "pid_single", "--evals", "20"])
        out = capsys.readouterr().out
        record = json.loads(out)
        assert rc == 0
        assert record["controller"] == "pid_single"
        assert record["cost_tuned"] <= record["cost_analytic"]

    def test_tune_grid_and_margins_and_record_file(self, capsys, tmp_path):
        from sim4wis.cli import main

        out_path = tmp_path / "record.json"
        rc = main(["tune", "pid_single", "--grid", "3", "--margins",
                   "--out", str(out_path)])
        out = capsys.readouterr().out
        assert rc == 0
        record = json.loads(out_path.read_text(encoding="utf-8"))
        assert "grid search" in record["note"]
        assert {m["name"] for m in record["margins"]} == set(
            record["tuned_params"])
        assert "record:" in out

    def test_unknown_controller_is_an_error(self, capsys):
        from sim4wis.cli import main

        rc = main(["tune", "not_a_controller"])
        capsys.readouterr()
        assert rc == 1
