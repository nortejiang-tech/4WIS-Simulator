"""Tests — tracking control library v1: PID single/cascade, LQR, feedforward.

Every controller gets two kinds of pins: an **analytic** one (the pole
placement formula, the Riccati residual) and a **behavioural** one (closed
loop on the corner plant converges, is deterministic, and each controller's
fingerprint — overshoot, steady error — is stable enough to regression
against).
"""

from __future__ import annotations

import numpy as np
import pytest

from sim4wis.steering.tracking.controller import (
    CONTROLLERS,
    AngleTrackingController,
    make_controller,
)
from sim4wis.steering.tracking.controllers import (
    pole_place_pid,
    solve_dare,
)
from sim4wis.steering.tracking.coupling import CornerTracker
from sim4wis.steering.tracking.feedback import AngleSensor
from sim4wis.steering.tracking.feedforward import (
    FeedforwardContext,
    FeedforwardStack,
    make_feedforward,
)
from sim4wis.steering.tracking.plant import CornerActuatorPlant


def _plant() -> CornerActuatorPlant:
    return CornerActuatorPlant(inertia_kgm2=0.6, damping_nms_per_rad=4.0,
                               coulomb_friction_nm=0.5, peak_torque_nm=40.0)


def _run(name: str, kwargs: dict, *, load: float = 3.0, steps: int = 4000,
         target: float = 0.2, quant: float = 0.00017, delay: int = 1,
         seed_noise: float = 0.0) -> tuple[np.ndarray, AngleTrackingController]:
    c = make_controller(name, **kwargs)
    plant = _plant()
    tr = CornerTracker(corner=0, controller=c, plant=plant,
                       sensor=AngleSensor(quant_rad=quant, delay_steps=delay,
                                          noise_std_rad=seed_noise))
    tr.reset()
    dt = 5e-4
    angles = []
    for _ in range(steps):
        out = c.step(dt, target_angle=target, target_rate=0.0,
                     feedback_angle=tr.sensor.measure(tr.angle),
                     plant_angle=tr.angle, load_torque=load, speed_ms=0.0)
        tr.apply(out, dt=dt, load_torque=load)
        angles.append(tr.angle)
    return np.array(angles), c


# ---- analytic pins ----------------------------------------------------------


def test_pole_place_pid_places_the_poles():
    j, b, wn, zeta = 0.6, 4.0, 25.0, 0.9
    kp, ki, kd = pole_place_pid(j, b, wn, zeta)
    p = 2.0 * zeta * wn
    # Characteristic polynomial of J·a + (b+kd)·ω = kp·e + ki·∫e:
    # J·s³ + (b+kd)·s² + kp·s + ki ≡ (s+p)(s²+2ζωₙs+ωₙ²).
    expected = j * np.asarray(np.polynomial.polynomial.polymul(
        [p, 1.0], [wn ** 2, 2.0 * zeta * wn, 1.0]))
    got = np.array([ki, kp, b + kd, j])
    np.testing.assert_allclose(got, expected, rtol=1e-12)


def test_lqr_gains_satisfy_the_discrete_riccati_equation():
    c = make_controller("lqr")
    a, b, r = c._a_d, c._b_d, c._r
    # Rebuild the design from the constructor defaults, then pin the gain
    # against a fresh Riccati solution — the shipped numbers must be *the*
    # Riccati solution, not an approximation of it.
    q = np.diag([50.0, 1.97e4, 0.1])
    kp, ki, kd = pole_place_pid(0.6, 4.0, wn=25.0, zeta=0.9)
    p = solve_dare(a, b, q, np.array([[r]]), np.array([[-ki, -kp, kd]]))
    k = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)[0]
    residual = (q + a.T @ p @ a
                - a.T @ p @ b @ np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
                - p)
    # The residual is zero up to double-precision rounding of the
    # Riccati terms, whose magnitude follows the weights (~1e4).
    np.testing.assert_allclose(residual, 0.0, atol=1e-6 * float(np.max(q)))
    np.testing.assert_allclose(c._k, k, rtol=1e-9)


# ---- behavioural pins -------------------------------------------------------


@pytest.mark.parametrize("name", ["pid_single", "pid_cascade", "lqr"])
def test_v1_controllers_converge_under_load(name):
    angles, _ = _run(name, {})
    ss = angles[-200:].mean()
    assert abs(ss - 0.2) < 0.006, f"{name}: steady error {abs(ss - 0.2):.2e}"
    assert angles.max() < 0.2 * 2.0  # bounded transient


@pytest.mark.parametrize("name", ["pid_single", "pid_cascade", "lqr"])
def test_v1_controllers_are_deterministic(name):
    a1, _ = _run(name, {}, steps=800)
    a2, _ = _run(name, {}, steps=800)
    np.testing.assert_array_equal(a1, a2)


def test_feedforward_stack_is_sum_of_enabled_blocks():
    stack = make_feedforward({"blocks": [
        {"type": "rack_force", "gain": 2.0},
        {"type": "velocity", "damping": 4.0, "inertia": 0.6},
        {"type": "friction", "friction_nm": 0.5},
        {"type": "friction", "friction_nm": 0.5, "enabled": False},
    ]})
    ctx = FeedforwardContext(target_rate=2.0, load_torque=3.0, dt=5e-4)
    expected = 2.0 * 3.0 + 4.0 * 2.0 + 0.5 * np.tanh(2.0 / 0.01)
    assert stack.compute(ctx) == pytest.approx(expected, rel=1e-12)
    assert "friction (off)" in stack.describe()


def test_unknown_feedforward_block_is_refused():
    with pytest.raises(ValueError, match="unknown feedforward block"):
        make_feedforward({"blocks": [{"type": "magic"}]})


def test_gain_schedule_interpolates():
    from sim4wis.steering.tracking.controllers import GainSchedule
    sched = GainSchedule({0.0: {"kp": 100.0}, 20.0: {"kp": 300.0}})
    assert sched.gains(0.0)["kp"] == pytest.approx(100.0)
    assert sched.gains(10.0)["kp"] == pytest.approx(200.0)
    assert sched.gains(100.0)["kp"] == pytest.approx(300.0)  # clamps at the edge


def test_registry_contains_the_v1_library():
    assert {"open_loop", "pid_single", "pid_cascade", "lqr"} <= set(CONTROLLERS)


def test_velocity_feedforward_compensates_the_rate():
    stack = FeedforwardStack([make_feedforward({"blocks": [
        {"type": "velocity", "damping": 4.0, "inertia": 0.6, "accel_term": False},
    ]}).blocks[0]])
    ctx = FeedforwardContext(target_rate=2.0, dt=5e-4)
    assert stack.compute(ctx) == pytest.approx(8.0)
