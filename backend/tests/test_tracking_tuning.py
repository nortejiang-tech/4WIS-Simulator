"""Tests — tuning workbench (M3) and the external-controller framework (M4).

Tuning: the 110 % acceptance gate, determinism, the relay identification's
hysteresis guard, and the provenance shape. External: the file-backed
reference adapter exercises the contract (init → invoke → shutdown,
missing-table refusal, param provenance).
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from sim4wis.steering.tracking.external import FileGainController
from sim4wis.steering.tracking.tuning import (
    ACCEPTANCE_RATIO,
    analytic_cascade,
    analytic_pid,
    relay_identify,
    step_cost,
    tune,
    zn_pid,
)


def test_tune_accepts_within_the_110_percent_bar():
    for name in ("pid_single", "pid_cascade", "lqr"):
        result = tune(name, max_evals=40)
        assert result.accepted
        assert result.cost_tuned <= ACCEPTANCE_RATIO * result.cost_analytic + 1e-9
        assert result.n_evals > 0
        assert set(result.analytic_params) == set(result.tuned_params)


def test_tune_is_deterministic():
    a = tune("pid_single", max_evals=20)
    b = tune("pid_single", max_evals=20)
    assert a.tuned_params == b.tuned_params
    assert a.cost_tuned == b.cost_tuned


def test_step_cost_ranks_a_better_controller_better():
    good = step_cost("pid_single", analytic_pid(0.6, 4.0))
    bad = step_cost("pid_single", {"kp": 10.0, "ki": 1.0, "kd": 0.1})
    assert good < bad


def test_relay_identification_produces_a_physical_limit_cycle():
    ku, tu = relay_identify()
    assert 100.0 < ku < 2000.0  # the hysteresis band keeps Ku physical
    assert 0.05 < tu < 1.0      # and the period off the sample rate


def test_zn_table_derives_from_ku_tu():
    g = zn_pid(500.0, 0.1)
    assert g["kp"] == pytest.approx(0.6 * 500.0)
    assert g["ki"] == pytest.approx(1.2 * 500.0 / 0.1)
    assert g["kd"] == pytest.approx(0.075 * 500.0 * 0.1)


def test_unknown_controller_has_no_tuning_space():
    with pytest.raises(ValueError, match="no tuning space"):
        tune("open_loop", max_evals=5)


def test_analytic_gains_match_the_formulas():
    pid = analytic_pid(0.6, 4.0, wn=25.0, zeta=0.9)
    from sim4wis.steering.tracking.controllers import pole_place_pid
    np.testing.assert_allclose(
        [pid["kp"], pid["ki"], pid["kd"]],
        pole_place_pid(0.6, 4.0, 25.0, 0.9), rtol=1e-12)
    cas = analytic_cascade(0.6, 4.0, wc_vel=60.0, wn_pos=18.0)
    assert cas["kp_vel"] == pytest.approx(0.6 * 60.0)
    assert cas["kp_pos"] == pytest.approx(2.0 * 0.9 * 18.0)


# ---- external framework -----------------------------------------------------


def test_file_gain_controller_runs_the_adapter_contract(tmp_path):
    table = tmp_path / "gains.json"
    table.write_text(json.dumps({"gain": 120.0}), encoding="utf-8")
    c = FileGainController(table)
    assert "gain" not in c.param_table  # provenance only after init
    c.init_backend(tmp_path)
    assert c.param_table["gain"].value == 120.0
    assert "file:" in c.param_table["gain"].provenance
    out = c.step(5e-4, target_angle=0.2, target_rate=0.0,
                 feedback_angle=0.1, plant_angle=0.1,
                 load_torque=0.0, speed_ms=0.0)
    assert out.torque_cmd == pytest.approx(120.0 * 0.1)
    c.shutdown()
    with pytest.raises(RuntimeError, match="init_backend"):
        c.step(5e-4, target_angle=0.2, target_rate=0.0, feedback_angle=0.1,
               plant_angle=0.1, load_torque=0.0, speed_ms=0.0)


def test_file_gain_refuses_a_missing_table(tmp_path):
    c = FileGainController(tmp_path / "missing.json")
    with pytest.raises(FileNotFoundError, match="honest refusal"):
        c.init_backend(tmp_path)


def test_file_gain_port_contract_is_declared():
    c = FileGainController("any.json")
    ports = c.describe_ports()
    assert {p["name"] for p in ports} == {
        "target_angle", "target_rate", "feedback_angle", "feedback_rate",
        "load_torque", "speed_ms", "actuator_torque",
    }
