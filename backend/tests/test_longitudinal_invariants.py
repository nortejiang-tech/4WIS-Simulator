"""Tests — invariants across models × longitudinal modes.

These came out of an adversarial sweep rather than from the design: each one
below is a defect the hand-written unit tests missed, because those tested what
the code was built to do rather than what it does at the edges.

  1. torque mode ignored `v_max_reverse` entirely (62 m/s in reverse against
     5 m/s in speed-servo mode)
  2. torque mode ignored an experiment's whole speed profile — a run asking
     for 60 km/h executed at 94, silently invalidating anything measured
  3. the kinematic model reported 5560 m/s² (≈570 g) of specific force on a
     standing start, which would have gone into the recorded channels

A fourth "failure" the sweep flagged turned out to be correct behaviour: ΣFz
falls 8.5% below m·g at 200 km/h because of aero lift. That check is written
aero-aware below so it stays meaningful.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import replace

import numpy as np
import pytest

from sim4wis.controller.longitudinal import apply_brake_command, apply_drive_command
from sim4wis.controller.registry import available_strategies, make_strategy
from sim4wis.core.derived import update_derived_outputs
from sim4wis.core.simulator import Simulator
from sim4wis.core.state import EnvironmentState, VehicleParams
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel
from sim4wis.vehicle.kinematic import KinematicModel
from sim4wis.vehicle.multibody import MultiBodyModel

DT = 0.005
G = 9.81
MODELS = {"kinematic": KinematicModel,
          "dynamic": SimplifiedDynamicModel,
          "multibody": MultiBodyModel}


def _sim(model: str = "dynamic", **over) -> Simulator:
    p = replace(VehicleParams(), **over)
    s = Simulator(dt_sim=DT, dt_push=0.05)
    s.reset()
    s.params = p
    s.model = MODELS[model](p)
    s.model.reset()
    s.strategy = make_strategy("ideal_ackermann", p)
    return s


def _tick(s: Simulator, env: EnvironmentState) -> None:
    cmd = s.strategy.compute(s.driver, s.model.state, DT)
    apply_brake_command(cmd, s.driver, s.params)
    apply_drive_command(cmd, s.driver, s.params, s.model.state)
    s.model.step(DT, cmd, env)
    update_derived_outputs(s.model.state, s.params)


def _assert_finite(s: Simulator, tag: str, k: int) -> None:
    st = s.model.state
    for name in ("vx", "vy", "yaw_rate", "x", "y", "psi", "ax", "ay", "mu_avg"):
        assert math.isfinite(getattr(st, name)), f"{tag}: {name} not finite at step {k}"
    for name in ("fz", "wheel_omega", "grip_util", "grip_capacity",
                 "grip_margin_lat", "grip_margin_long", "delta"):
        arr = np.asarray(getattr(st, name), dtype=float)
        assert np.all(np.isfinite(arr)), f"{tag}: {name} not finite at step {k}"


# ── 1. nothing diverges anywhere in the matrix ───────────────────────────────

@pytest.mark.parametrize("model", sorted(MODELS))
@pytest.mark.parametrize("mode", ["speed_servo", "torque"])
@pytest.mark.parametrize("mu", [0.05, 0.85])
@pytest.mark.parametrize("lam", [0.0, 0.5, 1.0])
def test_no_divergence_across_the_matrix(model, mode, mu, lam):
    """Drive hard through accelerate → turn → brake → reverse-turn in every
    combination of model, longitudinal mode, surface and drive split."""
    s = _sim(model, longitudinal_mode=mode, drive_front_ratio=lam)
    env = EnvironmentState(mu=mu)
    tag = f"{model}/{mode}/mu={mu}/lam={lam}"
    for k in range(1200):
        if k == 200:
            s.set_driver(throttle=1.0, gear=1)
        elif k == 500:
            s.set_driver(steering=0.9)
        elif k == 800:
            s.set_driver(throttle=0.0, brake=1.0)
        elif k == 1000:
            s.set_driver(brake=0.0, throttle=0.4, steering=-0.9)
        _tick(s, env)
        _assert_finite(s, tag, k)
    assert abs(s.model.state.vx) < 200, f"{tag}: runaway speed"


# ── 2. reverse ceiling holds in BOTH modes ───────────────────────────────────

@pytest.mark.parametrize("mode", ["speed_servo", "torque"])
def test_reverse_speed_cap_holds_in_both_modes(mode):
    """Regression: the drivetrain had no reverse ceiling at all, so torque mode
    reversed at 62 m/s while the same car was held to 5 m/s on the speed path."""
    s = _sim("dynamic", longitudinal_mode=mode)
    env = EnvironmentState(mu=0.85)
    s.set_driver(throttle=1.0, gear=-1)
    for _ in range(4000):
        _tick(s, env)
    assert abs(s.model.state.vx) <= s.params.v_max_reverse * 1.05


# ── 3. an experiment's speed profile is followed in BOTH modes ───────────────

@pytest.mark.parametrize("mode", ["speed_servo", "torque"])
def test_experiment_speed_profile_is_followed_in_both_modes(mode):
    """Regression: torque mode drove on the pedal echo and ignored the
    experiment's speed profile entirely — a run asking for 60 km/h executed at
    94, so anything measured from it was quietly wrong.

    A commanded speed must override the pedal whatever the mode is.
    """
    import yaml
    from pathlib import Path
    from sim4wis.experiment.schema import Experiment
    from sim4wis.experiment.session import SimSession

    root = Path(__file__).resolve().parents[2]
    raw = yaml.safe_load((root / "experiments" / "step_steer_60kmh.yaml").read_text())
    raw["vehicle"] = {**raw.get("vehicle", {}),
                      "overrides": {"longitudinal_mode": mode}}
    res = SimSession(Experiment.model_validate(raw)).run()
    assert res.channels["vx"][-1] * 3.6 == pytest.approx(60.0, abs=3.0)


# ── 4. specific force is never a numerical artefact ──────────────────────────

@pytest.mark.parametrize("model", sorted(MODELS))
def test_no_specific_force_spike_on_standing_start(model):
    """Regression: the kinematic model solves velocity algebraically, so a
    finite difference of it read 5560 m/s² (≈570 g) on the first step. It now
    reports zero rather than an artefact — see KinematicModel.step."""
    s = _sim(model)
    env = EnvironmentState(mu=0.85)
    s.set_driver(throttle=0.5, gear=1)
    peak = 0.0
    for _ in range(30):
        _tick(s, env)
        peak = max(peak, abs(s.model.state.ax), abs(s.model.state.ay))
    assert peak < 2.0 * G, f"{model}: |a| = {peak:.0f} m/s² ≈ {peak / G:.0f} g"


def test_kinematic_model_declares_it_has_no_force_data():
    """It reports neither a friction budget nor a specific force, so nothing
    downstream can mistake its kinematics for a force balance."""
    s = _sim("kinematic")
    env = EnvironmentState(mu=0.85)
    s.set_driver(throttle=0.6, steering=0.4, gear=1)
    for _ in range(400):
        _tick(s, env)
    assert s.model.state.grip_valid is False
    assert s.model.state.ax == 0.0
    assert s.model.state.ay == 0.0


# ── 5. physical invariants under hard use ────────────────────────────────────

def test_physical_invariants_hold_through_a_hard_run():
    s = _sim("dynamic")
    p = s.params
    env = EnvironmentState(mu=0.85)
    for k in range(4000):
        if k == 100:
            s.set_driver(throttle=1.0, gear=1)
        elif k == 1200:
            s.set_driver(steering=1.0)
        elif k == 2500:
            s.set_driver(throttle=0.0, brake=1.0)
        _tick(s, env)
        st = s.model.state
        # the friction budget can be spent but not overspent
        assert np.max(st.grip_util) <= 1.001
        # vertical equilibrium — aero-aware. ΣFz legitimately falls below m·g
        # at speed (Cl_f + Cl_r lift), which an unqualified ΣFz == m·g check
        # flags as an 8.5% error at 200 km/h.
        lift = (0.5 * p.air_density * p.frontal_area * st.vx ** 2
                * (p.aero_lift_coeff_front + p.aero_lift_coeff_rear))
        assert float(np.sum(st.fz)) == pytest.approx(p.mass * G - lift, rel=0.02)
        # and the body never outruns the surface
        assert math.hypot(st.ax, st.ay) <= 0.85 * G * 1.10


# ── 6. switching modes mid-run is smooth ─────────────────────────────────────

def test_mode_switch_mid_run_does_not_jolt():
    """Servo integrators are cleared on entry to torque mode; without that a
    stale integral lands in the drivetrain as a torque step."""
    s = _sim("dynamic", longitudinal_mode="speed_servo")
    env = EnvironmentState(mu=0.85)
    s.set_driver(throttle=0.4, gear=1)
    for _ in range(1500):
        _tick(s, env)
    s.params = replace(s.params, longitudinal_mode="torque")
    s.model.params = s.params
    peak = 0.0
    for _ in range(200):
        _tick(s, env)
        peak = max(peak, abs(s.model.state.ax))
    assert peak < 2.0 * G


# ── 7. every registered strategy survives torque mode ────────────────────────

@pytest.mark.parametrize("name", sorted(available_strategies()))
def test_every_strategy_runs_in_torque_mode(name):
    p = replace(VehicleParams(), longitudinal_mode="torque")
    s = Simulator(dt_sim=DT, dt_push=0.05)
    s.reset()
    s.params = p
    s.model = SimplifiedDynamicModel(p)
    s.model.reset()
    s.strategy = make_strategy(name, p)
    s.set_driver(throttle=0.6, steering=0.3, gear=1)
    for k in range(600):
        _tick(s, EnvironmentState(mu=0.85))
        _assert_finite(s, f"strategy/{name}", k)


# ── 8. degenerate configurations stay finite ─────────────────────────────────

@pytest.mark.parametrize("tag,over,mu", [
    ("speed limit below the taper band", dict(driver_speed_limit=0.3), 0.85),
    ("power cap disabled", dict(motor_power_max=0.0), 0.85),
    ("no motor torque at all", dict(motor_torque_max=0.0), 0.85),
    ("open diff on ice", dict(diff_type="open"), 0.05),
    ("split out of range", dict(drive_front_ratio=5.0), 0.85),
    ("split negative", dict(drive_front_ratio=-2.0), 0.85),
])
def test_degenerate_params_stay_finite(tag, over, mu):
    s = _sim("dynamic", longitudinal_mode="torque", **over)
    env = EnvironmentState(mu=mu)
    s.set_driver(throttle=1.0, gear=1)
    for k in range(600):
        _tick(s, env)
        _assert_finite(s, tag, k)


def test_zero_motor_torque_means_the_car_does_not_move():
    s = _sim("dynamic", longitudinal_mode="torque", motor_torque_max=0.0)
    env = EnvironmentState(mu=0.85)
    s.set_driver(throttle=1.0, gear=1)
    for _ in range(600):
        _tick(s, env)
    assert abs(s.model.state.vx) < 0.1
