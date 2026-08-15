"""Tests — tuning workbench (M3) and the external-controller framework (M4).

Tuning: the 110 % acceptance gate, determinism, the relay identification's
hysteresis guard, and the provenance shape. External: the file-backed
reference adapter exercises the contract (init → invoke → shutdown,
missing-table refusal, param provenance).
"""

from __future__ import annotations

import json
import math

import numpy as np
import pytest

from sim4wis.steering.tracking.controller import make_controller
from sim4wis.steering.tracking.external import FileGainController
from sim4wis.steering.tracking.feedback import AngleSensor
from sim4wis.steering.tracking.plant import CornerActuatorPlant
from sim4wis.steering.tracking.tuning import (
    ACCEPTANCE_RATIO,
    analytic_cascade,
    analytic_cascade_compliant,
    analytic_pid,
    analytic_pid_compliant,
    relay_identify,
    step_cost,
    transmission_resonance_hz,
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


# ---- compliance channel (direction 3) ---------------------------------------


COMPLIANT_PLANT = {"transmission_stiffness_nms_per_rad": 1200.0,
                   "peak_torque_nm": 120.0, "motor_inertia_fraction": 0.2}


def _corner_step(controller: str, kwargs: dict,
                 t_end: float = 3.0, dt: float = 5.0e-4):
    """One 1° step on the compliant corner plant; returns (overshoot %, ss %)."""
    c = make_controller(controller, **kwargs)
    plant = CornerActuatorPlant(
        inertia_kgm2=0.6, damping_nms_per_rad=4.0, coulomb_friction_nm=0.5,
        peak_torque_nm=120.0, transmission_stiffness_nms_per_rad=1200.0,
        motor_inertia_fraction=0.2)
    sensor = AngleSensor(quant_rad=0.00017, delay_steps=1)
    target = 0.01745
    hist = []
    for _ in range(int(t_end / dt)):
        out = c.step(dt, target_angle=target, target_rate=0.0,
                     feedback_angle=sensor.measure(plant.angle),
                     plant_angle=plant.angle, load_torque=3.0, speed_ms=0.0)
        plant.step(dt, out.torque_cmd, 3.0)
        hist.append(plant.angle)
    h = np.asarray(hist)
    overshoot = ((h.max() - target) / target * 100 if h.max() > target else 0.0)
    ss_err = abs(h[-1] - target) / target * 100
    return float(overshoot), float(ss_err)


def test_transmission_resonance_matches_the_closed_form():
    k, j, frac = 1200.0, 0.6, 0.2
    j_m, j_w = j * frac, j * (1.0 - frac)
    omega = math.sqrt(k * (j_m + j_w) / (j_m * j_w))
    assert transmission_resonance_hz(k, j, frac) == pytest.approx(
        omega / (2.0 * math.pi))


def test_compliant_analytic_seeds_are_stable():
    """The resonance-safe pole placements must land inside the stable basin:
    bounded overshoot, and the wheel actually reaches the command."""
    for controller, seed in (
        ("pid_single", analytic_pid_compliant(0.6, 4.0, 1200.0)),
        ("pid_cascade", analytic_cascade_compliant(0.6, 4.0, 1200.0)),
    ):
        overshoot, ss_err = _corner_step(controller, seed)
        assert overshoot < 25.0, (controller, overshoot)
        assert ss_err < 2.0, (controller, ss_err)


def test_step_cost_measures_the_compliance_channel():
    """The rigid vehicle-tuned defaults run away on the compliant plant; the
    resonance-safe design does not — the channel must rank them accordingly."""
    cost_bad = step_cost("pid_single", {}, **COMPLIANT_PLANT)
    seed = analytic_pid_compliant(0.6, 4.0, 1200.0)
    cost_good = step_cost("pid_single", seed, **COMPLIANT_PLANT)
    assert cost_good < cost_bad
    assert math.isfinite(cost_bad)


def test_tune_compliant_accepts_with_the_rate_axis():
    """On the compliant plant the black-box runs with the rate-channel
    low-pass in the axis and must still clear the 110 % bar."""
    result = tune("pid_single", max_evals=60, plant_kwargs=COMPLIANT_PLANT)
    assert result.accepted
    assert result.cost_tuned <= ACCEPTANCE_RATIO * result.cost_analytic + 1e-9
    assert "deriv_tau_s" in result.tuned_params
    assert set(result.analytic_params) == set(result.tuned_params)


def test_rigid_tune_keeps_the_three_gain_axis():
    """Without a transmission stiffness the tuning space is the frozen rigid
    one — no rate axis sneaks in."""
    result = tune("pid_single", max_evals=20)
    assert "deriv_tau_s" not in result.tuned_params
    assert set(result.tuned_params) == {"kp", "ki", "kd"}


def test_lqr_rate_filter_default_and_reset():
    """The LQR rate low-pass is a parameter now; the default is the frozen
    5 ms and reset() must keep a configured value."""
    c = make_controller("lqr")
    assert c._rate.tau == pytest.approx(0.005)
    d = make_controller("lqr", deriv_tau_s=0.05)
    d.reset()
    assert d._rate.tau == pytest.approx(0.05)


def test_relay_identify_accepts_the_compliant_plant():
    """The relay channel accepts the transmission parameters and either
    finds a physical limit cycle or refuses honestly."""
    try:
        ku, tu = relay_identify(transmission_stiffness_nms_per_rad=1200.0)
    except RuntimeError:
        return  # honest refusal is a valid outcome on the two-mass plant
    assert math.isfinite(ku) and math.isfinite(tu) and ku > 0.0 and tu > 0.0


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
