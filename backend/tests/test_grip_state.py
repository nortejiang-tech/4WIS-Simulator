"""Tests — friction-budget readout (grip state).

The readouts these guard are the instrument the drive-form work will be judged
with, so they have to be right before that lands. Three things matter:

  * the numbers agree with the forces that actually moved the vehicle,
  * the directional margins reflect the *coupling* between Fx and Fy, not just
    a scalar "how much grip is left",
  * beyond-peak is reported where it is meaningful and suppressed where the
    underlying slip is a numerical artefact rather than a physical slip.
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest

from sim4wis.controller.longitudinal import apply_brake_command, apply_drive_command
from sim4wis.controller.registry import make_strategy
from sim4wis.core.derived import update_derived_outputs
from sim4wis.core.simulator import Simulator
from sim4wis.core.state import EnvironmentState, VehicleParams
from sim4wis.vehicle.model_core import VMIN_SLIP, wheel_grip_state
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel
from sim4wis.vehicle.tire import LinearTireModel, PacejkaTireModel

DT = 0.005
G = 9.81


def _sim(**over) -> Simulator:
    p = VehicleParams(**over)
    s = Simulator(dt_sim=DT, dt_push=0.05)
    s.reset()
    s.params = p
    s.model = SimplifiedDynamicModel(p)
    s.model.reset()
    s.strategy = make_strategy("ideal_ackermann", p)
    return s


def _tick(s: Simulator, env: EnvironmentState) -> None:
    cmd = s.strategy.compute(s.driver, s.model.state, DT)
    apply_brake_command(cmd, s.driver, s.params)
    apply_drive_command(cmd, s.driver, s.params)
    s.model.step(DT, cmd, env)
    update_derived_outputs(s.model.state, s.params)


def _grip(**over):
    """wheel_grip_state on a single wheel, other three parked and idle."""
    base = dict(fx=np.zeros(4), fy=np.zeros(4), fz=np.full(4, 7000.0),
                mu=np.full(4, 0.85), alpha=np.zeros(4), kappa=np.zeros(4),
                vx_wheel=np.full(4, 20.0), tire=LinearTireModel())
    base.update(over)
    return wheel_grip_state(**base)


# ── 1. utilisation and capacity ──────────────────────────────────────────────

def test_capacity_is_mu_times_fz():
    g = _grip(fz=np.array([7000.0, 3500.0, 7000.0, 7000.0]))
    assert g.capacity[0] == pytest.approx(0.85 * 7000.0)
    assert g.capacity[1] == pytest.approx(0.85 * 3500.0)


def test_utilisation_is_the_resultant_over_capacity():
    cap = 0.85 * 7000.0
    g = _grip(fx=np.full(4, 0.6 * cap), fy=np.full(4, 0.8 * cap))
    # 3-4-5: the resultant is exactly the capacity
    assert g.utilization[0] == pytest.approx(1.0)
    assert g.margin[0] == pytest.approx(0.0, abs=1e-12)


def test_zero_capacity_does_not_divide_by_zero():
    g = _grip(fz=np.zeros(4), fx=np.full(4, 10.0))
    assert np.all(np.isfinite(g.utilization))
    assert g.utilization[0] == 0.0


# ── 2. directional margins couple through the circle ─────────────────────────

def test_lateral_margin_collapses_as_longitudinal_force_grows():
    """The point of reporting ΔFy separately: at 90% of the circle spent on Fx
    the scalar margin still reads 0.1, but the remaining lateral force is only
    √(1−0.81) ≈ 0.44 of capacity — and that is what decides whether a driven
    axle can still hold a line."""
    cap = 0.85 * 7000.0
    g = _grip(fx=np.full(4, 0.9 * cap))
    assert g.margin[0] == pytest.approx(0.1, abs=1e-9)
    assert g.margin_lat[0] == pytest.approx(math.sqrt(1 - 0.81) * cap, rel=1e-9)


def test_margins_are_exact_for_the_pure_cases():
    cap = 0.85 * 7000.0
    g = _grip()                                   # nothing used
    assert g.margin_lat[0] == pytest.approx(cap)
    assert g.margin_long[0] == pytest.approx(cap)

    g = _grip(fx=np.full(4, cap))                 # all of it longitudinal
    assert g.margin_lat[0] == pytest.approx(0.0, abs=1e-6)


def test_margins_never_go_negative():
    cap = 0.85 * 7000.0
    g = _grip(fx=np.full(4, 2 * cap), fy=np.full(4, 2 * cap))
    assert np.all(g.margin_lat >= 0.0)
    assert np.all(g.margin_long >= 0.0)


# ── 3. beyond-peak ───────────────────────────────────────────────────────────

@pytest.mark.parametrize("tire", [LinearTireModel(), PacejkaTireModel()])
def test_peak_slip_marks_the_top_of_the_curve(tire):
    """Below the peak the force still rises; above it never does. This is the
    property that makes the flag mean something."""
    fz, mu = 7000.0, 0.85
    a_pk, k_pk = tire.peak_slips(fz, mu)
    below = abs(tire.forces(a_pk * 0.7, 0.0, fz, mu)[1])
    at = abs(tire.forces(a_pk, 0.0, fz, mu)[1])
    above = abs(tire.forces(a_pk * 1.6, 0.0, fz, mu)[1])
    assert below < at
    assert above <= at + 1e-6
    assert abs(tire.forces(0.0, k_pk * 0.7, fz, mu)[0]) < abs(tire.forces(0.0, k_pk, fz, mu)[0])


def test_pacejka_actually_falls_past_the_peak():
    """Only the Magic Formula has a falling side; the linear tyre plateaus.
    Both are 'no more force this way', which is what the flag claims."""
    fz, mu = 7000.0, 0.85
    pac, lin = PacejkaTireModel(), LinearTireModel()
    a_p, _ = pac.peak_slips(fz, mu)
    a_l, _ = lin.peak_slips(fz, mu)
    assert abs(pac.forces(a_p * 2.5, 0.0, fz, mu)[1]) < abs(pac.forces(a_p, 0.0, fz, mu)[1])
    assert abs(lin.forces(a_l * 2.5, 0.0, fz, mu)[1]) == pytest.approx(
        abs(lin.forces(a_l, 0.0, fz, mu)[1]), rel=1e-9)


def test_peak_slip_scales_with_capacity():
    """α_peak ∝ μ·Fz — a lightly loaded wheel saturates at a smaller slip
    angle, which is why the inner wheel lets go first in a hard corner."""
    for tire in (LinearTireModel(), PacejkaTireModel()):
        a_hi, _ = tire.peak_slips(7000.0, 0.85)
        a_lo, _ = tire.peak_slips(3500.0, 0.85)
        assert a_lo == pytest.approx(a_hi / 2, rel=1e-9)
        a_ice, _ = tire.peak_slips(7000.0, 0.20)
        assert a_ice < a_hi


def test_beyond_peak_flags_track_the_slip():
    tire = LinearTireModel()
    a_pk, k_pk = tire.peak_slips(7000.0, 0.85)
    g = _grip(alpha=np.full(4, a_pk * 1.2), kappa=np.full(4, k_pk * 0.5), tire=tire)
    assert bool(g.beyond_peak_lat[0]) and not bool(g.beyond_peak_long[0])
    g = _grip(alpha=np.full(4, a_pk * 0.5), kappa=np.full(4, k_pk * 1.2), tire=tire)
    assert not bool(g.beyond_peak_lat[0]) and bool(g.beyond_peak_long[0])


def test_beyond_peak_is_suppressed_at_a_standstill():
    """Both slips divide by max(|vx_wheel|, VMIN_SLIP), so below that floor
    they are placeholders, not physical slips. A parked car reported all four
    tyres past the peak before this guard — visibly wrong the moment the HUD
    showed it."""
    tire = LinearTireModel()
    a_pk, _ = tire.peak_slips(7000.0, 0.85)
    huge = np.full(4, a_pk * 50)
    parked = _grip(alpha=huge, vx_wheel=np.full(4, VMIN_SLIP * 0.5), tire=tire)
    rolling = _grip(alpha=huge, vx_wheel=np.full(4, VMIN_SLIP * 4.0), tire=tire)
    assert not np.any(parked.beyond_peak_lat)
    assert np.all(rolling.beyond_peak_lat)


def test_forces_still_reported_at_a_standstill():
    """Suppressing the slip flags must not blank the force budget — a car held
    on the brake is using real grip."""
    cap = 0.85 * 7000.0
    g = _grip(fx=np.full(4, 0.5 * cap), vx_wheel=np.zeros(4))
    assert g.utilization[0] == pytest.approx(0.5)
    assert g.margin_lat[0] > 0.0


# ── 4. end-to-end against the running model ──────────────────────────────────

def test_utilisation_never_exceeds_one_in_a_real_run():
    """The tyre model clips to the circle, so the budget can be spent but not
    overspent. A utilisation above 1 would mean the readout and the dynamics
    disagree."""
    s = _sim()
    env = EnvironmentState(mu=0.85)
    s.set_driver(throttle=0.5, gear=1)
    for k in range(3000):
        if k == 1500:
            s.set_driver(steering=1.0)
        _tick(s, env)
        assert np.all(s.model.state.grip_util <= 1.0 + 1e-9)


def test_load_transfer_moves_capacity_between_wheels():
    """The circle radius IS μ·Fz, so this is what makes load transfer legible
    in the display rather than needing a separate readout."""
    s = _sim()
    env = EnvironmentState(mu=0.85)
    s.set_driver(throttle=0.35, gear=1)
    for _ in range(2000):
        _tick(s, env)
    flat = s.model.state.grip_capacity.copy()
    assert flat[0] == pytest.approx(flat[1], rel=0.05), "straight line: left≈right"

    s.set_driver(steering=0.8)
    for _ in range(1200):
        _tick(s, env)
    turning = s.model.state.grip_capacity
    # left turn → load moves to the right-hand wheels
    assert turning[1] > turning[0] * 1.5
    assert turning[3] > turning[2] * 1.5
    # and the total is roughly conserved (it is the same car)
    assert turning.sum() == pytest.approx(flat.sum(), rel=0.15)


def test_braking_moves_capacity_forward():
    s = _sim()
    env = EnvironmentState(mu=0.85)
    s.set_driver(throttle=0.5, gear=1)
    for _ in range(2500):
        _tick(s, env)
    s.set_driver(throttle=0.0, brake=1.0)
    for _ in range(100):
        _tick(s, env)
    cap = s.model.state.grip_capacity
    assert cap[0] > cap[2] * 1.4, "front axle must gain under braking"


def test_gg_envelope_bounds_the_body_acceleration():
    """Steady cornering at the limit sits just inside μ·g — if |a| ran past the
    envelope the g-g plot would be lying about the surface."""
    for mu in (0.85, 0.35):
        s = _sim()
        env = EnvironmentState(mu=mu)
        s.set_driver(throttle=0.4, steering=1.0, gear=1)
        for _ in range(3000):
            _tick(s, env)
        st = s.model.state
        assert math.hypot(st.ax, st.ay) <= mu * G * 1.10


def test_kinematic_model_reports_grip_invalid():
    """No tyre, no friction budget — say so rather than drawing a full circle
    that reads as 'no grip used'."""
    from sim4wis.vehicle.kinematic import KinematicModel
    p = VehicleParams()
    s = Simulator(dt_sim=DT, dt_push=0.05)
    s.reset()
    s.params = p
    s.model = KinematicModel(p)
    s.model.reset()
    s.strategy = make_strategy("ideal_ackermann", p)
    s.set_driver(throttle=0.4, gear=1)
    for _ in range(200):
        _tick(s, EnvironmentState(mu=0.85))
    assert s.model.state.grip_valid is False
