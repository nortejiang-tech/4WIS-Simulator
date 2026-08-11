"""Acceptance — a study, end to end, checked against an independent reference.

The reference is the platform's own closed-form steady-state solve
(`solve_steady_state_body`), which shares no code with the time-domain path a
study actually runs. Agreeing with it in the linear range exercises the whole
chain — expansion, batch execution, channel storage, the expression tier's
`steady()`, and the result table — against something that was not produced by
that chain.

Getting them to agree turned up a real defect; see
`test_analytic_path_omits_the_axle_cornering_split` at the bottom.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from sim4wis.experiment.session import resolve_vehicle_params
from sim4wis.study import store as study_store
from sim4wis.study.runner import dry_run, run_sync
from sim4wis.study.spec import StudySpec
from sim4wis.vehicle.load_transfer import vertical_loads
from sim4wis.vehicle.model_core import (
    axle_cornering_scale,
    quasi_static_wheel_loads,
    solve_steady_state_body,
)

SPEEDS = [30.0, 40.0, 50.0, 60.0]


@pytest.fixture(autouse=True)
def _isolated_dirs(tmp_path, monkeypatch):
    monkeypatch.setenv("SIM4WIS_RUNS_DIR", str(tmp_path / "runs"))
    monkeypatch.setenv("SIM4WIS_STUDIES_DIR", str(tmp_path / "studies"))


def _spec() -> StudySpec:
    """Small front-angle step, held long enough to settle, swept over speed.

    `unit: front_deg` bypasses the steering-feel layer so the experiment
    measures the vehicle rather than the driver-input mapping — which is what
    makes a comparison against a closed form meaningful at all.
    """
    return StudySpec.model_validate({
        "study": "steady_yaw_vs_analytic",
        "question": "阶跃转向的稳态横摆角速度，与解析稳态解是否一致？",
        "model": "simplified_dynamic",
        "baseline": {
            "name": "step_1deg",
            "strategy": "ideal_ackermann",
            "maneuver": {"steps": [
                {"duration": 2.0, "speed_kmh": 60,
                 "steer": {"kind": "constant", "amplitude": 0.0}},
                {"duration": 14.0, "speed_kmh": 60,
                 "steer": {"kind": "step", "amplitude": 1.0,
                           "unit": "front_deg", "t_step": 0.5}},
            ]},
        },
        "sweep": {"speed_kmh": {"values": SPEEDS, "bind": "maneuver.steps.1.speed_kmh"}},
        "metrics": [
            {"name": "r_dps", "expr": "degrees(steady(yaw_rate))", "unit": "°/s"},
            {"name": "beta_deg", "expr": "degrees(atan2(steady(vy), steady(vx)))", "unit": "°"},
            {"name": "ay", "expr": "steady(ay)", "unit": "m/s²"},
            {"name": "vx_ss", "expr": "steady(vx)", "unit": "m/s"},
            {"name": "d_fl", "expr": "degrees(steady(delta_fl))", "unit": "°"},
            {"name": "d_fr", "expr": "degrees(steady(delta_fr))", "unit": "°"},
            {"name": "d_rl", "expr": "degrees(steady(delta_rl))", "unit": "°"},
            {"name": "d_rr", "expr": "degrees(steady(delta_rr))", "unit": "°"},
        ],
        "compare": {"group_by": "speed_kmh"},
        "criteria": [{"metric": "ay", "must": "abs < 4.0", "at": "all"}],
    })


def _analytic_yaw_rate_dps(params, row, *, with_axle_split: bool) -> float:
    """Closed-form steady yaw rate for the steer angles the run actually held."""
    fz_static = vertical_loads(params, ax=0.0, ay=0.0)
    loads = quasi_static_wheel_loads(params, fz_static, row.metrics["vx_ss"])
    c_alpha = loads.c_alpha * (axle_cornering_scale(params) if with_axle_split else 1.0)
    delta = np.radians([row.metrics[f"d_{w}"] for w in ("fl", "fr", "rl", "rr")])
    _, yaw = solve_steady_state_body(
        speed=row.metrics["vx_ss"], delta=delta, c_alpha=c_alpha,
        wheel_positions_body=params.wheel_positions_body(), mass=float(params.mass),
    )
    return math.degrees(yaw)


class TestAcceptance:
    def test_dry_run_reports_the_grid_without_running_it(self):
        d = dry_run(_spec())
        assert d["ok"] and d["problems"] == []
        assert d["grid"] == len(SPEEDS) and d["runs"] == len(SPEEDS)
        assert d["sim_seconds"] == pytest.approx(len(SPEEDS) * 16.0)
        assert d["cells"] == [f"speed_kmh={int(v)}" for v in SPEEDS]

    def test_study_matches_the_closed_form_in_the_linear_range(self):
        spec = _spec()
        _, result = run_sync(spec, write_report=False)
        params = resolve_vehicle_params(spec.baseline)

        assert len(result.rows) == len(SPEEDS)
        assert not any(r.errors for r in result.rows), [r.errors for r in result.rows]

        for row in result.rows:
            analytic = _analytic_yaw_rate_dps(params, row, with_axle_split=True)
            measured = row.metrics["r_dps"]
            rel = abs(measured - analytic) / abs(analytic)
            assert rel < 0.02, (
                f"{row.label}: measured {measured:.4f} °/s vs analytic {analytic:.4f} °/s "
                f"({rel * 100:.2f}%)"
            )
            # Sanity on the manoeuvre itself: still comfortably inside the
            # linear range, or the comparison would not mean anything.
            assert abs(row.metrics["ay"]) < 4.0

    def test_yaw_gain_flattens_with_speed_as_understeer_predicts(self):
        _, result = run_sync(_spec(), write_report=False)
        by_speed = {r.coords["speed_kmh"]: r.metrics for r in result.rows}
        gains = [by_speed[v]["r_dps"] / by_speed[v]["vx_ss"] for v in SPEEDS]
        # r/(δ·V) falls monotonically with speed for an understeering car.
        assert all(a > b for a, b in zip(gains, gains[1:], strict=False)), gains

    def test_result_is_persisted_with_provenance(self):
        study_id, result = run_sync(_spec(), write_report=False)
        stored = study_store.load(study_id)
        assert stored["spec_digest"] == result.spec_digest
        prov = stored["provenance"]
        assert prov["sim4wis_version"] and prov["git_sha"] and prov["params_hash"]
        assert prov["model"] == "simplified_dynamic"
        assert study_store.load_spec(study_id).digest() == result.spec_digest

    def test_the_summary_stays_small_enough_to_read(self):
        import json

        _, result = run_sync(_spec(), write_report=False)
        blob = json.dumps(result.to_summary(), ensure_ascii=False)
        # A single run holds ~670 samples across ~35 channels; the whole point
        # of the summary is that it is nothing like that size.
        assert len(blob) < 20_000, len(blob)
        assert "channels" not in blob

    def test_repeat_runs_are_deterministic(self):
        _, a = run_sync(_spec(), write_report=False)
        _, b = run_sync(_spec(), write_report=False)
        for ra, rb in zip(a.rows, b.rows, strict=True):
            assert ra.metrics == rb.metrics


def test_analytic_path_omits_the_axle_cornering_split():
    """Documents a defect the acceptance test above ran into.

    `dynamic.py` scales tyre cornering stiffness per axle by
    `axle_cornering_scale` (0.80 front / 1.20 rear on the default vehicle), but
    `quasi_static_wheel_loads` → `load_sensitive_cornering_stiffness` does not.
    So every consumer of the quasi-static path — the bicycle-gain demo, the
    load-analysis page — is solving for a *different car* than the one the time
    domain simulates.

    This is the same defect class the decoupling study recorded in
    `axle_cornering_stiffness()`; this is a second instance of it.

    The effect is amplitude-independent and grows with v² (it moves the
    understeer gradient K, and yaw gain is V/(L + K·V²)): about −5 % of steady
    yaw rate at 30 km/h and −17 % at 60 km/h.

    **When the analytic path is fixed, delete this test** — and the
    `with_axle_split` switch above with it.
    """
    spec = _spec()
    _, result = run_sync(spec, write_report=False)
    params = resolve_vehicle_params(spec.baseline)

    errors = {}
    for row in result.rows:
        analytic = _analytic_yaw_rate_dps(params, row, with_axle_split=False)
        errors[row.coords["speed_kmh"]] = (row.metrics["r_dps"] - analytic) / analytic

    assert errors[30.0] < -0.02, errors           # already visible at 30 km/h
    assert errors[60.0] < -0.10, errors           # and much worse at 60
    # Monotone in speed — the signature of a term that moves K, not of noise.
    ordered = [errors[v] for v in SPEEDS]
    assert all(a > b for a, b in zip(ordered, ordered[1:], strict=False)), ordered
