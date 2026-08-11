"""Tests — StudySpec validation and sweep expansion.

Most of these pin *refusals*. The study layer's job is to be strict at the
declaration, because everything it lets through costs minutes of simulation
and, worse, can produce a table that looks right.
"""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from sim4wis.study.expand import concrete_experiments, expand, grid_size, validate_binds
from sim4wis.study.spec import StudySpec, parse_must


def _spec(**over):
    base = {
        "study": "s",
        "question": "q",
        "model": "simplified_dynamic",
        "baseline": {
            "name": "b",
            "maneuver": {"steps": [{"duration": 4, "speed_kmh": 60,
                                    "steer": {"kind": "step", "amplitude": 0.05}}]},
        },
        "sweep": {"speed_kmh": {"values": [60, 100], "bind": "maneuver.steps.0.speed_kmh"}},
        "metrics": ["yaw_gain_dps"],
    }
    base.update(over)
    return StudySpec.model_validate(base)


class TestValidation:
    def test_unknown_model_is_refused(self):
        with pytest.raises(ValidationError, match="unknown model"):
            _spec(model="not_a_model")

    def test_model_conflicting_with_the_baseline_is_refused(self):
        # Silently overriding it would make the spec lie about what it ran.
        with pytest.raises(ValidationError, match="remove one of them"):
            _spec(baseline={"name": "b", "model_type": "kinematic",
                            "maneuver": {"steps": [{"duration": 4}]}})

    def test_a_study_with_no_metrics_is_refused(self):
        with pytest.raises(ValidationError, match="measures nothing"):
            _spec(metrics=[])

    def test_duplicate_metric_names_are_refused(self):
        with pytest.raises(ValidationError, match="duplicate metric"):
            _spec(metrics=["yaw_gain_dps",
                           {"name": "yaw_gain_dps", "expr": "steady(vx)"}])

    def test_criterion_on_an_uncomputed_metric_is_refused(self):
        with pytest.raises(ValidationError, match="does not compute"):
            _spec(criteria=[{"metric": "nope", "must": "< 1"}])

    def test_group_by_must_name_a_sweep_axis(self):
        with pytest.raises(ValidationError, match="not a sweep axis"):
            _spec(compare={"group_by": "nope"})

    def test_unparseable_must_clause_is_refused(self):
        with pytest.raises(ValidationError, match="unparseable"):
            _spec(criteria=[{"metric": "yaw_gain_dps", "must": "kind of small"}])

    def test_axis_needs_exactly_one_of_values_or_solve_for(self):
        with pytest.raises(ValidationError, match="exactly one"):
            _spec(sweep={"v": {"bind": "maneuver.steps.0.speed_kmh"}})

    @pytest.mark.parametrize("clause,expected", [
        ("< 20", (False, "<", 20.0)),
        ("abs < 1.5", (True, "<", 1.5)),
        (">= -0.5", (False, ">=", -0.5)),
        ("== 1e-3", (False, "==", 1e-3)),
    ])
    def test_must_clauses_parse(self, clause, expected):
        assert parse_must(clause) == expected


class TestDigest:
    def test_is_stable_across_key_order(self):
        a = _spec()
        b = StudySpec.model_validate(
            {k: v for k, v in reversed(list(a.model_dump(mode="json").items()))}
        )
        assert a.digest() == b.digest()

    def test_changes_when_the_grid_changes(self):
        assert _spec().digest() != _spec(
            sweep={"speed_kmh": {"values": [60, 120], "bind": "maneuver.steps.0.speed_kmh"}}
        ).digest()


class TestExpansion:
    def test_cartesian_product_in_declaration_order(self):
        s = _spec(sweep={
            "speed_kmh": {"values": [60, 100], "bind": "maneuver.steps.0.speed_kmh"},
            "amp": {"values": [0.05, 0.1], "bind": "maneuver.steps.0.steer.amplitude"},
        })
        assert grid_size(s) == 4
        assert [c.label for c in expand(s)] == [
            "speed_kmh=60 amp=0.05", "speed_kmh=60 amp=0.1",
            "speed_kmh=100 amp=0.05", "speed_kmh=100 amp=0.1",
        ]

    def test_coordinates_survive_into_the_experiments(self):
        s = _spec()
        pairs = concrete_experiments(s)
        assert [c.coords["speed_kmh"] for c, _ in pairs] == [60, 100]
        assert [e.maneuver.steps[0].speed_kmh for _, e in pairs] == [60.0, 100.0]

    def test_the_study_model_reaches_the_experiment(self):
        s = _spec(model="multibody")
        assert all(e.model_type == "multibody" for _, e in concrete_experiments(s))

    def test_no_sweep_is_a_single_cell(self):
        s = _spec(sweep={})
        cells = expand(s)
        assert len(cells) == 1 and cells[0].overrides == {}

    def test_a_typo_in_bind_is_refused_rather_than_created(self):
        # The batch setter fills in missing keys as it walks, so an unvalidated
        # typo yields a full grid of secretly identical runs — the worst
        # possible failure, since the comparison table still looks fine.
        s = _spec(sweep={"v": {"values": [1], "bind": "maneuver.stps.0.speed_kmh"}})
        assert validate_binds(s)
        with pytest.raises(ValueError, match="does not resolve"):
            expand(s)

    def test_open_dicts_accept_new_keys(self):
        s = _spec(sweep={"mass": {"values": [2000, 2400],
                                  "bind": "vehicle.overrides.mass"}})
        assert validate_binds(s) == []
        assert [e.vehicle.overrides["mass"] for _, e in concrete_experiments(s)] == [2000, 2400]

    def test_solver_axes_are_refused_with_a_pointer(self):
        s = _spec(sweep={"steer": {
            "solve_for": {"metric": "yaw_gain_dps", "target": 4.0},
            "bracket": [0.2, 8.0], "bind": "maneuver.steps.0.steer.amplitude",
        }})
        with pytest.raises(ValueError, match="not implemented yet"):
            expand(s)
