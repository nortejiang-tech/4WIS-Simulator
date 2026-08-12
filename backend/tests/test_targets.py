"""Tests — target sets and compliance.

The interesting ones are all about the same thing: **not measuring something
must never read as passing it.** A compliance table that quietly scores the
untested part as green is worse than no table, because it converts an absence
of evidence into a signed-off requirement. Half of this file exists to hold
that line, and the other half holds the margin arithmetic that makes the table
worth reading at all.
"""

from __future__ import annotations

import dataclasses
import math

import pytest

from sim4wis.core.state import VehicleParams
from sim4wis.steering.sizing import DEFAULT_SCENARIOS, run_scenario, size_actuator
from sim4wis.study.result import Row, StudyResult
from sim4wis.targets import compliance, library, measure
from sim4wis.targets import report as target_report
from sim4wis.targets.spec import Band, TargetEntry, TargetError, TargetSet


def _entry(**kw) -> TargetEntry:
    base = {"id": "t1", "metric": "m", "limit": "<= 10", "source": "test"}
    return TargetEntry(**{**base, **kw})


def _set(*entries: TargetEntry) -> TargetSet:
    return TargetSet(name="s", applies_to="test", entries=list(entries or [_entry()]))


def _m(metric: str, value: float, **kw) -> measure.Measurement:
    return measure.Measurement(metric=metric, value=value, label=kw.pop("label", "cell"), **kw)


# ---------------------------------------------------------------------------
# Band — every requirement reduces to one
# ---------------------------------------------------------------------------


class TestBand:
    @pytest.mark.parametrize(("clause", "inside", "outside"), [
        ("<= 5", 5.0, 5.1),
        ("< 5", 4.9, 5.0),
        (">= 2", 2.0, 1.9),
        ("> 2", 2.1, 2.0),
        ("abs <= 1", -1.0, -1.1),
        ([0.4, 1.2], 0.8, 1.3),
    ])
    def test_it_reads_both_clause_and_pair_forms(self, clause, inside, outside):
        b = Band.parse(clause)
        assert b.contains(inside) and not b.contains(outside)

    def test_strictness_survives_parsing(self):
        # A requirements document that says "<" and is checked as "<=" is a
        # document that does not mean what it says.
        assert not Band.parse("< 5").contains(5.0)
        assert Band.parse("<= 5").contains(5.0)

    def test_not_equal_is_refused_as_a_requirement(self):
        with pytest.raises(TargetError, match="measure zero"):
            Band.parse("!= 0")

    def test_an_inverted_pair_is_refused(self):
        with pytest.raises(TargetError, match="inverted"):
            Band.parse([2.0, 1.0])

    def test_margin_on_a_one_sided_limit_is_the_unused_fraction(self):
        assert Band.parse("<= 10").margin(8.0) == pytest.approx(0.2)
        assert Band.parse(">= 10").margin(12.0) == pytest.approx(0.2)
        assert Band.parse("<= 10").margin(12.0) == pytest.approx(-0.2)

    def test_margin_in_a_two_sided_band_is_measured_from_the_centre(self):
        b = Band.parse([0.2, 0.6])
        assert b.margin(0.4) == pytest.approx(1.0)      # centre
        assert b.margin(0.6) == pytest.approx(0.0)      # edge
        assert b.margin(0.7) < 0                        # outside

    def test_an_undefined_margin_is_none_rather_than_a_made_up_number(self):
        # Dividing by a zero bound would report inf or nan; the report already
        # leaves undefined ratios blank and this keeps that rule.
        assert Band.parse(">= 0").margin(5.0) is None
        assert Band.parse("== 3").margin(3.0) is None
        assert Band(lo=None, hi=None).margin(1.0) is None


# ---------------------------------------------------------------------------
# Schema
# ---------------------------------------------------------------------------


class TestSchema:
    def test_a_target_must_say_where_its_number_came_from(self):
        # The first question in a requirements review, refused at the schema.
        with pytest.raises(ValueError, match="source"):
            TargetEntry(id="t", metric="m", limit="<= 1")

    def test_a_goal_looser_than_the_requirement_is_refused(self):
        with pytest.raises(ValueError, match="inside the limit"):
            _entry(limit="<= 10", target="<= 20")

    def test_an_unbounded_limit_is_refused(self):
        with pytest.raises(ValueError, match="requires nothing"):
            _entry(limit=Band())

    def test_duplicate_ids_are_refused(self):
        with pytest.raises(ValueError, match="duplicate"):
            _set(_entry(id="a"), _entry(id="a"))

    def test_a_bad_file_names_the_file_and_the_reason(self, tmp_path, monkeypatch):
        """Pydantic's own error is not what a mistyped YAML key deserves.

        A user editing a requirements document gets the filename and the
        reason, not a stack of validation frames.
        """
        monkeypatch.setenv("SIM4WIS_TARGETS_DIR", str(tmp_path))
        (tmp_path / "mine.yaml").write_text(
            "name: mine\napplies_to: x\nentries:\n"
            "  - {id: a, metric: m, limit: '<= 1'}\n",
            encoding="utf-8",
        )
        with pytest.raises(TargetError, match="mine.yaml"):
            library.load_all()

    def test_a_file_may_not_shadow_a_shipped_version(self, tmp_path, monkeypatch):
        # A quoted requirement version must not mean two different documents.
        monkeypatch.setenv("SIM4WIS_TARGETS_DIR", str(tmp_path))
        (tmp_path / "clash.yaml").write_text(
            "name: eps_actuator\nversion: 1\napplies_to: x\nentries:\n"
            "  - {id: a, metric: m, limit: '<= 1', source: mine}\n",
            encoding="utf-8",
        )
        with pytest.raises(TargetError, match="Bump `version`"):
            library.load_all()

    def test_the_ref_pins_a_version(self):
        assert TargetSet(name="s", version=3, applies_to="x",
                         entries=[_entry()]).ref == "s@3"

    def test_the_digest_ignores_key_order(self):
        a = TargetSet(name="s", applies_to="x", entries=[_entry()])
        b = TargetSet(applies_to="x", entries=[_entry()], name="s")
        assert a.digest() == b.digest()


# ---------------------------------------------------------------------------
# The line: absence of evidence is not compliance
# ---------------------------------------------------------------------------


class TestNotEvaluatedIsNotPassing:
    def test_a_metric_nobody_measured_is_not_evaluated(self):
        r = compliance.evaluate(_set(_entry(metric="never_measured")),
                                [_m("something_else", 1.0)])
        assert r.rows[0].status == compliance.NOT_EVALUATED
        assert r.verdict == "incomplete"
        assert r.coverage == 0.0

    def test_a_selector_naming_a_missing_axis_says_which_one(self):
        # Different mistake from "the filter excluded everything", and a
        # different fix — so the note has to distinguish them.
        r = compliance.evaluate(
            _set(_entry(at="speed_kmh == 100")),
            [_m("m", 1.0, at={"scenario": "parking"})],
        )
        assert r.rows[0].status == compliance.NOT_EVALUATED
        assert "speed_kmh" in r.rows[0].note and "scenario" in r.rows[0].note

    def test_a_filter_that_excludes_everything_says_so_differently(self):
        r = compliance.evaluate(
            _set(_entry(at="scenario == nope")),
            [_m("m", 1.0, at={"scenario": "parking"})],
        )
        assert r.rows[0].status == compliance.NOT_EVALUATED
        assert "未匹配" in r.rows[0].note

    def test_an_untrusted_measurement_is_refused_rather_than_judged(self):
        """The cross-module integrity link.

        Sizing states plainly that past its trust limit its numbers describe a
        manoeuvre that did not happen. A compliance table that turned such a
        run green would launder an unusable result into a signed-off
        requirement — worse than having no table.
        """
        r = compliance.evaluate(
            _set(_entry(limit="<= 10")),
            [_m("m", 1.0, trusted=False, untrusted_reason="过了能力边界")],
        )
        assert r.rows[0].status == compliance.NOT_EVALUATED
        assert "过了能力边界" in r.rows[0].note

    def test_a_violation_outranks_a_gap(self):
        # If something failed, that is the headline, whatever else is missing.
        r = compliance.evaluate(
            _set(_entry(id="a", metric="m", limit="<= 1"),
                 _entry(id="b", metric="unmeasured", limit="<= 1")),
            [_m("m", 99.0)],
        )
        assert r.verdict == "non_compliant"

    def test_should_severity_does_not_block_the_verdict(self):
        r = compliance.evaluate(
            _set(_entry(id="a", metric="m", limit="<= 10"),
                 _entry(id="b", metric="unmeasured", limit="<= 1", severity="should")),
            [_m("m", 1.0)],
        )
        assert r.verdict == "compliant"
        assert r.coverage == 1.0


# ---------------------------------------------------------------------------
# Status ladder
# ---------------------------------------------------------------------------


class TestStatus:
    def test_inside_the_target_is_met(self):
        r = compliance.evaluate(_set(_entry(limit="<= 10", target="<= 8")), [_m("m", 5.0)])
        assert r.rows[0].status == compliance.MET
        assert r.verdict == "compliant"

    def test_between_target_and_limit_is_marginal_not_a_pass(self):
        """"Passed" and "passed with 2% left" are different situations.

        Only one of them survives a tolerance stack, so the table has to keep
        them apart — while still counting the configuration as compliant,
        because the limit is what was agreed.
        """
        r = compliance.evaluate(_set(_entry(limit="<= 10", target="<= 8")), [_m("m", 9.0)])
        assert r.rows[0].status == compliance.MARGINAL
        assert r.verdict == "compliant"

    def test_past_the_limit_is_violated(self):
        r = compliance.evaluate(_set(_entry(limit="<= 10")), [_m("m", 11.0)])
        assert r.rows[0].status == compliance.VIOLATED
        assert r.verdict == "non_compliant"

    def test_a_failure_names_the_tightest_cell(self):
        # "over the limit" is not actionable; "over the limit at 140 km/h" is.
        r = compliance.evaluate(
            _set(_entry(limit="<= 10")),
            [_m("m", 11.0, label="120 km/h"), _m("m", 30.0, label="140 km/h")],
        )
        row = r.rows[0]
        assert row.worst_label == "140 km/h"
        assert row.n_violated == 2 and row.n_checked == 2

    def test_untrusted_cells_are_dropped_but_counted_in_the_note(self):
        r = compliance.evaluate(
            _set(_entry(limit="<= 10")),
            [_m("m", 1.0), _m("m", 999.0, trusted=False, untrusted_reason="x")],
        )
        assert r.rows[0].status == compliance.MET
        assert r.rows[0].n_checked == 1
        assert "1 个测量点" in r.rows[0].note


# ---------------------------------------------------------------------------
# Measurement adapters
# ---------------------------------------------------------------------------


class TestMeasurementAdapters:
    def test_study_rows_carry_their_sweep_coordinates(self):
        result = StudyResult(study="s", question="q", model="kinematic", spec_digest="d",
                             rows=[Row(coords={"speed_kmh": 100}, label="100",
                                       metrics={"yaw_gain_dps": 12.0})])
        [m] = measure.from_study(result)
        assert m.at == {"speed_kmh": 100} and m.value == 12.0
        assert m.source == "study:s"

    def test_a_nan_metric_is_marked_untrusted(self):
        result = StudyResult(study="s", question="q", model="kinematic", spec_digest="d",
                             rows=[Row(coords={}, label="c", metrics={"x": float("nan")})])
        assert not measure.from_study(result)[0].trusted

    def test_sizing_taints_only_what_the_bad_scenario_drove(self):
        """Trust propagates per metric, not in bulk.

        A maximum is only as good as the scenario that produced it — which
        `driven_by` names — while the duty-cycle RMS mixes every scenario, so
        any bad run taints it. Blanket-tainting would throw away good numbers;
        blanket-trusting would keep bad ones.
        """
        params = _sized_params(peak_torque=1.0)     # undersized on purpose
        req = size_actuator(params)
        assert any(s.beyond_capability for s in req.scenarios)
        ms = {m.metric: m for m in measure.from_sizing(req) if not m.at}
        assert not ms["required_rms_torque_nm"].trusted
        driver = req.driven_by["peak_torque"]
        expected = driver in {s.scenario.id for s in req.scenarios if s.beyond_capability}
        assert ms["required_peak_torque_nm"].trusted is (not expected)

    def test_rack_force_counts_both_shares_of_the_tyre_load(self):
        # At standstill the whole load is scrub, and a rack force that read
        # zero exactly where it is largest would be a lie.
        parking = next(s for s in DEFAULT_SCENARIOS if s.id == "parking_full_lock")
        r = run_scenario(_sized_params(), parking)
        assert r.peak_rack_force > 15000


def _sized_params(**motor) -> VehicleParams:
    p = VehicleParams()
    st = dataclasses.replace(p.steering_system, enabled=True)
    if motor:
        st = dataclasses.replace(st, motor=dataclasses.replace(st.motor, **motor))
    return dataclasses.replace(p, steering_system=st)


# ---------------------------------------------------------------------------
# The shipped library
# ---------------------------------------------------------------------------


class TestLibrary:
    def test_every_shipped_target_says_where_its_number_came_from(self):
        for ts in library.BUILTIN.values():
            for e in ts.entries:
                assert e.source.strip(), f"{ts.ref}/{e.id}"

    def test_a_bare_name_resolves_to_the_latest_version(self):
        assert library.get("eps_actuator").ref == "eps_actuator@1"
        assert library.get("eps_actuator@1").version == 1

    def test_an_unknown_set_lists_what_there_is(self):
        with pytest.raises(TargetError, match="eps_actuator"):
            library.get("no_such_set")

    def test_the_actuator_set_is_fully_evaluable_from_one_sizing_pass(self):
        """The shipped set has to actually work on the shipped vehicle.

        A requirements document nobody can evaluate is a wish list, and this is
        the one place the tool can prove otherwise without external data.
        """
        req = size_actuator(_sized_params())
        r = compliance.evaluate(library.get("eps_actuator"), measure.from_sizing(req))
        assert r.coverage == 1.0
        assert r.verdict in ("compliant", "non_compliant")

    def test_the_feel_set_reports_incomplete_and_names_what_is_missing(self):
        """Shipped deliberately unevaluable — and honest about it.

        A requirements document exists before the test capability does. The gap
        between them is the work list, so the table has to name the missing
        measurement rather than quietly scoring it either way.
        """
        req = size_actuator(_sized_params())
        r = compliance.evaluate(library.get("steering_feel"), measure.from_sizing(req))
        assert r.verdict == "incomplete"
        assert r.coverage == 0.0
        assert all("onc_" in row.note or "return_" in row.note for row in r.rows)


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------


class TestReport:
    def test_the_dict_and_the_object_render_identically(self):
        # A stored study renders the verdict it was given, not one re-derived
        # from a library that may have been edited since.
        r = compliance.evaluate(_set(_entry(limit="<= 10")), [_m("m", 5.0)])
        assert target_report.render_html(r) == target_report.render_html(r.to_dict())

    def test_markup_in_cells_is_not_escaped_into_view(self):
        r = compliance.evaluate(_set(_entry(limit="<= 10")), [_m("m", 5.0)])
        html = target_report.render_html(r)
        assert "&lt;b&gt;" not in html and "<b>t1</b>" in html

    def test_worst_rows_come_first(self):
        r = compliance.evaluate(
            _set(_entry(id="ok", metric="a", limit="<= 10"),
                 _entry(id="bad", metric="b", limit="<= 1")),
            [_m("a", 1.0), _m("b", 99.0)],
        )
        html = target_report.render_html(r)
        assert html.index(">bad<") < html.index(">ok<")

    def test_the_validity_boundary_travels_with_the_table(self):
        r = compliance.evaluate(_set(_entry(limit="<= 10")), [_m("m", 5.0)])
        assert "未经实车" in target_report.render_html(r) or \
               "尚未与实车" in target_report.render_html(r)


# ---------------------------------------------------------------------------
# Integration
# ---------------------------------------------------------------------------


class TestSizingCompliance:
    def test_a_bigger_motor_moves_rows_toward_met(self):
        small = compliance.evaluate(
            library.get("eps_actuator"), measure.from_sizing(size_actuator(_sized_params())))
        big = compliance.evaluate(
            library.get("eps_actuator"),
            measure.from_sizing(size_actuator(_sized_params(peak_torque=16.0,
                                                            continuous_torque=9.0))))
        rank = {compliance.VIOLATED: 0, compliance.NOT_EVALUATED: 1,
                compliance.MARGINAL: 2, compliance.MET: 3}
        by_id = {r.entry.id: r.status for r in small.rows}
        for row in big.rows:
            # `motor_peak_torque` is checked against the *fitted* rating, which
            # grew too, so it is the one row that can legitimately not improve.
            if row.entry.id in ("motor_peak_torque", "motor_continuous_torque"):
                continue
            assert rank[row.status] >= rank[by_id[row.entry.id]], row.entry.id

    def test_the_margin_is_reported_not_just_the_verdict(self):
        r = compliance.evaluate(
            library.get("eps_actuator"), measure.from_sizing(size_actuator(_sized_params())))
        evaluated = [row for row in r.rows if row.status != compliance.NOT_EVALUATED]
        assert evaluated
        assert all(row.worst_margin is not None and math.isfinite(row.worst_margin)
                   for row in evaluated)


# ---------------------------------------------------------------------------
# Study integration
# ---------------------------------------------------------------------------


_USER_SET = """
name: vehicle_response
version: 2
title: 车辆响应要求（示例）
applies_to: 测试用
owner: 系统工程
entries:
  - id: lateral_accel
    metric: ay
    at: all
    limit: "abs <= 6.0"
    target: "abs <= 4.0"
    unit: m/s²
    source: 测试夹具
    rationale: 线性区判定
  - id: sideslip
    metric: beta_deg
    at: "speed_kmh == 60"
    limit: [-3.0, 3.0]
    unit: deg
    source: 测试夹具
    rationale: 两侧都是要求
  - id: never_measured
    metric: onc_torque_deadband_deg
    limit: "<= 1.0"
    source: 测试夹具
    rationale: 覆盖率必须看得见
"""


class TestStudyIntegration:
    @pytest.fixture(autouse=True)
    def _dirs(self, tmp_path, monkeypatch):
        monkeypatch.setenv("SIM4WIS_RUNS_DIR", str(tmp_path / "runs"))
        monkeypatch.setenv("SIM4WIS_STUDIES_DIR", str(tmp_path / "studies"))
        targets = tmp_path / "targets"
        targets.mkdir()
        (targets / "vehicle_response.yaml").write_text(_USER_SET, encoding="utf-8")
        monkeypatch.setenv("SIM4WIS_TARGETS_DIR", str(targets))

    def _spec(self, **kw):
        from sim4wis.study.spec import StudySpec

        return StudySpec.model_validate({
            "study": "targets_demo",
            "question": "示例",
            "model": "simplified_dynamic",
            "baseline": {
                "name": "step", "strategy": "ideal_ackermann",
                "maneuver": {"steps": [
                    {"duration": 1.0, "speed_kmh": 60,
                     "steer": {"kind": "constant", "amplitude": 0.0}},
                    {"duration": 5.0, "speed_kmh": 60,
                     "steer": {"kind": "step", "amplitude": 1.0,
                               "unit": "front_deg", "t_step": 0.2}},
                ]},
            },
            "sweep": {"speed_kmh": {"values": [40, 60],
                                    "bind": "maneuver.steps.1.speed_kmh"}},
            "metrics": [
                {"name": "ay", "expr": "steady(ay)", "unit": "m/s²"},
                {"name": "beta_deg", "expr": "degrees(atan2(steady(vy), steady(vx)))"},
            ],
            "report": {"template": "sweep"},
            **kw,
        })

    def test_a_user_set_extends_the_shipped_library(self):
        assert library.get("vehicle_response").ref == "vehicle_response@2"
        assert {c["ref"] for c in library.catalogue()} >= {
            "eps_actuator@1", "vehicle_response@2"}

    def test_dry_run_says_what_will_not_be_covered_before_anything_runs(self):
        """Coverage is knowable without executing a single run.

        Finding out after a few hundred runs that a third of the requirements
        were never going to be measured is finding out too late.
        """
        from sim4wis.study.runner import dry_run

        check = dry_run(self._spec(targets="vehicle_response"))
        assert check["ok"] and check["targets"] == "vehicle_response"
        assert any("onc_torque_deadband_deg" in w for w in check["warnings"])

    def test_an_unknown_target_set_is_a_problem_not_a_warning(self):
        from sim4wis.study.runner import dry_run

        check = dry_run(self._spec(targets="no_such_thing"))
        assert not check["ok"]
        assert any("no_such_thing" in p for p in check["problems"])

    def test_a_study_carries_its_compliance_table_and_its_gaps(self):
        from sim4wis.study.runner import run_sync

        _, result = run_sync(self._spec(targets="vehicle_response@2"))
        c = result.compliance
        assert c and c["ref"] == "vehicle_response@2"
        by_id = {r["id"]: r for r in c["rows"]}
        assert by_id["lateral_accel"]["status"] in ("met", "marginal")
        # The two-sided band was checked only where the selector pointed.
        assert by_id["sideslip"]["checked"] == 1
        # And the requirement nothing measures is visible as a gap, not a pass.
        assert by_id["never_measured"]["status"] == "not_evaluated"
        assert c["verdict"] == "incomplete" and c["coverage"] < 1.0

    def test_the_table_reaches_the_rendered_report(self):
        from sim4wis.study.runner import run_sync

        _, result = run_sync(self._spec(targets="vehicle_response"))
        html = open(result.report_path, encoding="utf-8").read()
        assert "目标符合性" in html and "vehicle_response@2" in html
        assert "lateral_accel" in html

    def test_a_study_without_targets_is_unchanged(self):
        from sim4wis.study.runner import run_sync

        _, result = run_sync(self._spec())
        assert result.compliance is None
        assert result.to_summary()["compliant"] is None


# ---------------------------------------------------------------------------
# API
# ---------------------------------------------------------------------------


class TestApi:
    @pytest.fixture()
    def client(self):
        from fastapi.testclient import TestClient

        from sim4wis.main import app

        return TestClient(app)

    def test_the_library_is_listed(self, client):
        body = client.get("/api/targets").json()
        assert {s["ref"] for s in body["sets"]} >= {"eps_actuator@1", "steering_feel@1"}

    def test_a_set_comes_back_with_its_sources(self, client):
        body = client.get("/api/targets/eps_actuator@1").json()
        assert body["ref"] == "eps_actuator@1"
        assert all(e["source"] for e in body["entries"])

    def test_an_unknown_ref_is_404_and_lists_what_there_is(self, client):
        r = client.get("/api/targets/nope")
        assert r.status_code == 404 and "eps_actuator" in r.json()["detail"]

    def test_check_runs_a_sizing_pass_and_returns_both_halves(self, client):
        body = client.post("/api/targets/eps_actuator/check", json={}).json()
        assert body["compliance"]["counts"]["must"] == 6
        # The sizing result travels with the verdict: a compliance row that
        # says "7.29 N·m" is only auditable next to the run that produced it.
        assert body["sizing"]["scenarios"]

    def test_check_can_render_the_table(self, client):
        r = client.post("/api/targets/steering_feel/check", json={"html": True})
        assert r.status_code == 200
        assert "text/html" in r.headers["content-type"]
        assert "覆盖不全" in r.text

    def test_capabilities_advertises_the_library(self, client):
        caps = client.get("/api/study/capabilities").json()
        assert {t["ref"] for t in caps["target_sets"]} >= {"eps_actuator@1"}
        assert any("targets belong to the product" in n for n in caps["notes"])
