"""Tests — tyre relaxation length and load-dependent friction.

Two mechanisms that were missing and that both bear on conclusions the
platform already draws:

  * **Relaxation length.** A tyre builds side force over a rolling distance,
    not instantly. The `rear_wheel_steer` transient modes exist to exploit the
    phase between front and rear force build-up, so their measured advantage
    depends on this being modelled.

  * **μ(Fz) falloff.** Friction drops as a tyre is loaded, so a pair of tyres
    with load transferred between them does less than the same pair evenly
    loaded. It bears on the LIMIT only: the linear-range understeer gradient
    is a cornering-stiffness quantity and μ never enters it. See
    `test_mu_falloff_is_a_limit_mechanism_not_a_linear_range_one`, which pins
    that down — it was written expecting the opposite and the measurement said
    no.
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest

from sim4wis.controller.longitudinal import apply_brake_command, apply_drive_command
from sim4wis.controller.registry import make_strategy
from sim4wis.core.derived import update_derived_outputs
from sim4wis.core.state import DriverInput, EnvironmentState, VehicleParams
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel
from sim4wis.vehicle.model_core import load_sensitive_mu, relax_slip
from sim4wis.vehicle.multibody import MultiBodyModel

DT = 0.002
MU = 0.90


def _step_steer(sigma, v_kmh=60, delta_deg=1.5, secs=3.0, model="multibody", **over):
    p = replace(VehicleParams(), tire_relax_length=sigma,
                longitudinal_mode="torque", **over)
    m = (MultiBodyModel if model == "multibody" else SimplifiedDynamicModel)(p)
    m.reset()
    strat = make_strategy("rear_wheel_steer", p)
    d = DriverInput(gear=1, mode_params={
        "rws_mode": "fixed_ratio", "rear_ratio": 0.0,
        "speed_target_ms": v_kmh / 3.6})
    env = EnvironmentState(mu=MU)

    def run(n):
        for _ in range(n):
            c = strat.compute(d, m.state, DT)
            apply_brake_command(c, d, p)
            apply_drive_command(c, d, p, m.state)
            m.step(DT, c, env)
            update_derived_outputs(m.state, p)

    run(int(12.0 / DT))
    d.mode_params["steer_raw_rad"] = math.radians(delta_deg)
    ts, yaw = [], []
    t = 0.0
    for _ in range(int(secs / DT)):
        run(1)
        t += DT
        ts.append(t)
        yaw.append(abs(m.state.yaw_rate))
    yaw = np.array(yaw)
    ts = np.array(ts)
    steady = float(np.mean(yaw[-int(0.5 / DT):]))
    t90 = float(ts[np.argmax(yaw >= 0.9 * steady)]) if steady > 1e-6 else float("nan")
    return dict(t90=t90, steady=steady,
                overshoot=(float(np.max(yaw)) / max(steady, 1e-9) - 1) * 100)


# ─── relaxation: the maths ──────────────────────────────────────────────────

def test_relaxation_off_is_a_passthrough():
    target = np.array([0.1, -0.2, 0.05, 0.0])
    out = relax_slip(np.zeros(4), target, np.full(4, 20.0), 0.0, DT)
    assert np.allclose(out, target)


def test_relaxation_converges_toward_the_target():
    lag = np.zeros(4)
    target = np.full(4, 0.1)
    v = np.full(4, 20.0)
    for _ in range(400):
        lag = relax_slip(lag, target, v, 0.5, DT)
    assert np.allclose(lag, target, rtol=1e-6)


def test_relaxation_time_constant_scales_with_speed():
    """The tyre winds up over a rolling DISTANCE, so the same manoeuvre lags
    longer at low speed — that is the defining property of the model."""
    target = np.full(4, 0.1)
    sigma = 0.5
    # Pure maths, so integrate finely: at the model's own 2 ms the 12.5 ms
    # constant lands between steps and the answer quantises to 14 ms.
    dt = 1e-4
    for v, expect_ms in ((10.0, 50.0), (40.0, 12.5)):
        lag = np.zeros(4)
        t = 0.0
        while lag[0] < target[0] * (1 - 1 / math.e) and t < 5.0:
            lag = relax_slip(lag, target, np.full(4, v), sigma, dt)
            t += dt
        assert t * 1000 == pytest.approx(expect_ms, rel=0.02)


def test_relaxation_cannot_overshoot_at_large_step():
    """Integrated exactly over the step rather than by forward Euler: with
    dt=5 ms, v=50 m/s and sigma=0.3 the Euler factor would be 0.83, close
    enough to the stability edge to matter."""
    lag = np.zeros(4)
    target = np.full(4, 1.0)
    for _ in range(50):
        lag = relax_slip(lag, target, np.full(4, 80.0), 0.2, 0.01)
        assert np.all(lag <= target + 1e-12)


# ─── relaxation: on the vehicle ─────────────────────────────────────────────

@pytest.mark.parametrize("model", ["multibody", "dynamic"])
def test_relaxation_changes_the_transient_response(model):
    """It must reach the body dynamics, not just the diagnostics. (First cut of
    this feature was applied only in the wheel-spin pass, which is downstream
    of the force that moves the car — every sigma gave identical results.)"""
    off = _step_steer(0.0, model=model)
    on = _step_steer(0.8, model=model)
    assert not math.isclose(off["t90"], on["t90"], rel_tol=1e-6) or \
        not math.isclose(off["overshoot"], on["overshoot"], rel_tol=1e-6)


def test_relaxation_matters_more_at_low_speed():
    """Because the lag is sigma/v, the same relaxation length is a larger
    fraction of the response at low speed. At 30 km/h it is ~26% of T90;
    at 100 km/h ~6%."""
    slow_off = _step_steer(0.0, v_kmh=30)
    slow_on = _step_steer(0.5, v_kmh=30)
    fast_off = _step_steer(0.0, v_kmh=100)
    fast_on = _step_steer(0.5, v_kmh=100)
    slow_shift = abs(slow_on["t90"] / slow_off["t90"] - 1)
    fast_shift = abs(fast_on["t90"] / fast_off["t90"] - 1)
    assert slow_shift > fast_shift


def test_relaxation_does_not_move_the_steady_state():
    """It is a transient mechanism only — once the slip has settled the tyre
    works at exactly the geometric angle."""
    off = _step_steer(0.0)
    on = _step_steer(0.8)
    assert on["steady"] == pytest.approx(off["steady"], rel=0.01)


# ─── μ(Fz) ──────────────────────────────────────────────────────────────────

def test_mu_falloff_off_is_a_passthrough():
    p = VehicleParams()
    mu = np.full(4, 0.9)
    assert np.allclose(load_sensitive_mu(p, mu, np.full(4, 20000.0)), mu)


def test_mu_falls_with_load_by_the_stated_coefficient():
    p = replace(VehicleParams(), tire_mu_load_sensitivity=0.10)
    fz_nom = p.mass * 9.80665 / 4
    mu0 = np.full(4, 0.9)
    # at nominal load, unchanged
    assert load_sensitive_mu(p, mu0, np.full(4, fz_nom))[0] == pytest.approx(0.9)
    # at double load, down by k
    assert load_sensitive_mu(p, mu0, np.full(4, 2 * fz_nom))[0] == pytest.approx(0.9 * 0.9)
    # and unloading gains grip
    assert load_sensitive_mu(p, mu0, np.full(4, 0.5 * fz_nom))[0] > 0.9


def test_mu_stays_positive_under_extreme_load():
    p = replace(VehicleParams(), tire_mu_load_sensitivity=0.5)
    out = load_sensitive_mu(p, np.full(4, 0.9), np.full(4, 1e6))
    assert np.all(out > 0.0)


def _corner(k_mu, eps_f, delta_deg, v_kmh=80, secs=14.0):
    """Settled steady-state cornering at a given steer angle."""
    base = VehicleParams()
    p = replace(base, tire_mu_load_sensitivity=k_mu, longitudinal_mode="torque",
                suspension=replace(base.suspension,
                                   roll_stiffness_front_frac=eps_f))
    m = MultiBodyModel(p)
    m.reset()
    strat = make_strategy("rear_wheel_steer", p)
    d = DriverInput(gear=1, mode_params={
        "rws_mode": "fixed_ratio", "rear_ratio": 0.0,
        "steer_raw_rad": math.radians(delta_deg),
        "speed_target_ms": v_kmh / 3.6})
    env = EnvironmentState(mu=MU)
    for _ in range(int(secs / DT)):
        c = strat.compute(d, m.state, DT)
        apply_brake_command(c, d, p)
        apply_drive_command(c, d, p, m.state)
        m.step(DT, c, env)
        update_derived_outputs(m.state, p)
    s = m.state
    util = np.asarray(s.grip_util, dtype=float)
    return dict(ay=abs(s.ay), vx=s.vx, delta_f=abs(float(np.mean(s.delta[:2]))),
                balance=float(np.mean(util[:2]) - np.mean(util[2:])),
                util_max=float(np.max(util)))


def test_mu_falloff_is_a_limit_mechanism_not_a_linear_range_one():
    """Written expecting the opposite; the measurement said no, so it is now
    a test of the negative result.

    The linear-range understeer gradient K = W_f/C_f − W_r/C_r is a cornering-
    STIFFNESS quantity. mu caps force but does not change the slope below the
    cap, so no amount of mu(Fz) gives roll-couple distribution authority over
    K — measured span 0.0137 -> 0.0131 across roll splits 0.35..0.75, i.e. it
    very slightly *reduces* it. The axle stiffness split cannot be retired in
    favour of this term.
    """
    lo, hi = 0.35, 0.75
    span_off = abs(_corner(0.0, hi, 3.0)["balance"] - _corner(0.0, lo, 3.0)["balance"])
    span_on = abs(_corner(0.15, hi, 3.0)["balance"] - _corner(0.15, lo, 3.0)["balance"])
    assert span_on < span_off * 1.5, (
        f"sub-limit, mu(Fz) must not buy the roll split authority: "
        f"{span_off:.5f} -> {span_on:.5f}"
    )


def test_mu_falloff_gives_roll_couple_distribution_authority_at_the_limit():
    """What it IS for. Once the front axle is saturated, the split between the
    axles decides how much the transferred load costs, and mu(Fz) is what makes
    transferred load cost anything at all. Measured at 6 deg / 80 km/h:
    front-vs-rear utilisation span across the roll split goes 0.0051 -> 0.0211,
    a factor of four."""
    lo, hi = 0.35, 0.75
    span_off = abs(_corner(0.0, hi, 6.0)["balance"] - _corner(0.0, lo, 6.0)["balance"])
    span_on = abs(_corner(0.15, hi, 6.0)["balance"] - _corner(0.15, lo, 6.0)["balance"])
    assert span_on > span_off * 2.0, (
        f"at the limit mu(Fz) must give the roll split authority: "
        f"{span_off:.5f} -> {span_on:.5f}"
    )


def test_mu_falloff_reaches_the_body_force_not_just_the_wheel_spin():
    """Regression for the bug this shipped with: `load_sensitive_mu` was applied
    only in the wheel-spin/diagnostic pass, while the RK4 `_derivatives` that
    produce the force moving the car still used nominal mu. The symptom was a
    friction circle reporting utilisation of 1.10 — the tyre drawn outside its
    own circle — and limit grip that barely moved."""
    off = _corner(0.0, 0.60, 6.0)
    on = _corner(0.15, 0.60, 6.0)
    assert on["ay"] < off["ay"] * 0.99, (
        f"load transfer must cost limit grip: {off['ay']:.3f} -> {on['ay']:.3f} m/s²")
    assert on["util_max"] <= 1.0 + 1e-6, (
        f"utilisation above 1 means the circle disagrees with the forces: "
        f"{on['util_max']:.4f}")
