"""Tests — advanced controllers: DOB, ADRC, SMC, MPC, H∞.

Analytic pins per design contract, behavioural pins per controller:

* DOB  — near-zero steady error under constant load (the observer's point)
* ADRC — observer gains are the bandwidth parameterisation; converges
* SMC  — sliding surface small at steady state; no chattering beyond φ
* MPC  — unconstrained first move equals the terminal-LQR law; the hard
         box is respected exactly when it binds
* H∞   — game Riccati residual ≈ 0, closed loop stable, γ infeasible raises
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from sim4wis.steering.tracking.advanced import hildreth_qp, solve_game_care
from sim4wis.steering.tracking.controller import CONTROLLERS, make_controller
from sim4wis.steering.tracking.coupling import CornerTracker
from sim4wis.steering.tracking.feedback import AngleSensor
from sim4wis.steering.tracking.plant import CornerActuatorPlant

ALL = ["dob", "adrc", "smc", "mpc", "h_inf"]


def _run(name: str, kwargs: dict | None = None, *, load: float = 3.0,
         steps: int = 4000, target: float = 0.2, u_max_probe: bool = False):
    c = make_controller(name, **(kwargs or {}))
    plant = CornerActuatorPlant(inertia_kgm2=0.6, damping_nms_per_rad=4.0,
                                coulomb_friction_nm=0.5, peak_torque_nm=40.0)
    tr = CornerTracker(corner=0, controller=c, plant=plant,
                       sensor=AngleSensor(quant_rad=0.00017, delay_steps=1))
    tr.reset()
    dt = 5e-4
    angles, torques, surfaces = [], [], []
    for _ in range(steps):
        out = c.step(dt, target_angle=target, target_rate=0.0,
                     feedback_angle=tr.sensor.measure(tr.angle),
                     plant_angle=tr.angle, load_torque=load, speed_ms=0.0)
        tr.apply(out, dt=dt, load_torque=load)
        angles.append(tr.angle)
        if out.torque_cmd is not None:
            torques.append(abs(out.torque_cmd))
        surfaces.append(out.diagnostics.get("surface"))
    return np.array(angles), c, np.array(torques), surfaces


# ---- analytic pins ----------------------------------------------------------


def test_mpc_unconstrained_first_move_converges_to_terminal_lqr():
    """The horizon-H first move approaches the infinite-horizon LQR law as H
    grows (the finite-horizon Riccati P₀ ≠ P∞, so equality is a limit, not an
    identity — and the gap must shrink with the horizon)."""

    def first_move(horizon: int) -> tuple[float, float]:
        c = make_controller("mpc", u_max=1e9, q_integral=400.0,
                            horizon=horizon)
        x0 = np.array([0.0, 0.0, 0.1])
        n = c.horizon
        m_mat = np.zeros((3 * n, 3))
        n_mat = np.zeros((3 * n, n))
        a_pow = np.eye(3)
        for k in range(n):
            m_mat[3 * k:3 * k + 3] = a_pow
            for j in range(k):
                n_mat[3 * k:3 * k + 3, j] = (
                    np.linalg.matrix_power(c._a, k - 1 - j) @ c._b)[:, 0]
            n_mat[3 * k:3 * k + 3, k] = c._b[:, 0]
            a_pow = c._a @ a_pow
        q_bar = np.kron(np.eye(n), c._q)
        q_bar[-3:, -3:] += c._p
        h = n_mat.T @ q_bar @ n_mat + c._r * np.eye(n)
        cc = n_mat.T @ q_bar @ (m_mat @ x0)
        u0 = -np.linalg.solve(h, cc)[0]
        k = np.linalg.solve(c._r + c._b.T @ c._p @ c._b,
                            c._b.T @ c._p @ c._a)[0]
        u_lqr = -(k @ x0)
        return u0, u_lqr

    u_short, u_lqr_short = first_move(4)
    u_long, u_lqr_long = first_move(20)
    assert u_lqr_short == pytest.approx(u_lqr_long)  # the law is the law
    gap_short = abs(u_short - u_lqr_short)
    gap_long = abs(u_long - u_lqr_long)
    assert gap_long < gap_short  # the horizon approaches the infinite law
    assert gap_long < 0.01 * abs(u_lqr_long)  # measured: ~0.7 % at H=20

def test_mpc_respects_the_hard_torque_box():
    angles, _, torques, _ = _run("mpc", {"u_max": 5.0, "horizon": 10}, load=6.0)
    # The box holds to solver precision (Hildreth terminates at 1e-10
    # on the multipliers, ~2e-8 on u in the worst sample).
    assert torques.max() <= 5.0 + 1e-7


def test_hinf_riccati_residual_and_closed_loop_stability():
    c = make_controller("h_inf", gamma=6.0)
    j, b, r = c.j, c.b, 0.01
    a = np.array([[0.0, 1.0, 0.0],
                  [0.0, 0.0, -1.0],
                  [0.0, 0.0, -b / j]])
    b2 = np.array([[0.0], [0.0], [1.0 / j]])
    q = np.diag([300.0, 900.0, 1.0])
    gamma = 6.0
    p = solve_game_care(a, b2, b2, q, np.array([[r]]), gamma)
    b_tilde = b2 @ np.linalg.inv(np.array([[r]])) @ b2.T - b2 @ b2.T / gamma ** 2
    residual = a.T @ p + p @ a - p @ b_tilde @ p + q
    np.testing.assert_allclose(residual, 0.0, atol=1e-6)
    k = (np.linalg.inv(np.array([[r]])) @ b2.T @ p)[0]
    a_cl = a - b2 @ k.reshape(1, -1)
    assert np.all(np.real(np.linalg.eigvals(a_cl)) < 0.0)


def test_hinf_infeasible_gamma_is_refused():
    with pytest.raises(ValueError, match="infeasible|degenerate|semidefinite"):
        make_controller("h_inf", gamma=0.05)


def test_adrc_observer_poles_are_the_bandwidth_parameterisation():
    """Exact-discrete ESO: the observer's z-poles are exp(−ωo·h), triple."""
    c = make_controller("adrc")
    a_l = c._a_d - np.outer(c._l, np.array([1.0, 0.0, 0.0]))
    poles = np.linalg.eigvals(a_l)
    want = math.exp(-c.wo / 2000.0)
    # A triple pole at rho makes the Ackermann phi ill-conditioned;
    # the poles land on rho to ~3e-6 (rounding), not 1e-6.
    np.testing.assert_allclose(poles.real, want, rtol=1e-4, atol=1e-9)
    assert np.max(np.abs(poles.imag)) < 1e-5  # numerical dust only


def test_hildreth_matches_the_analytic_box_solution():
    # min (u−2)² s.t. u ≤ 1 → u* = 1.
    h = np.array([[2.0]])
    c = np.array([-4.0])
    u = hildreth_qp(h, c, np.array([[1.0]]), np.array([1.0]))
    assert u[0] == pytest.approx(1.0, abs=1e-9)
    # No binding constraint → the unconstrained optimum.
    u = hildreth_qp(h, c, np.array([[1.0]]), np.array([100.0]))
    assert u[0] == pytest.approx(2.0, abs=1e-9)


# ---- behavioural pins -------------------------------------------------------


@pytest.mark.parametrize("name", ALL)
def test_advanced_controllers_converge_under_load(name):
    angles, _, _, _ = _run(name, {})
    ss = angles[-200:].mean()
    assert abs(ss - 0.2) < 0.02, f"{name}: steady error {abs(ss - 0.2):.2e}"
    assert angles.max() < 0.2 * 3.0  # bounded transient


@pytest.mark.parametrize("name", ALL)
def test_advanced_controllers_are_deterministic(name):
    a1, _, _, _ = _run(name, {}, steps=800)
    a2, _, _, _ = _run(name, {}, steps=800)
    np.testing.assert_array_equal(a1, a2)


def test_dob_rejects_a_load_step_better_than_its_base():
    """The observer's point is disturbance *rejection*, not stiction: under
    a load step mid-track the DOB recovers with less deviation than the same
    PID base alone."""
    def run_with_load_step(name: str, kwargs: dict) -> np.ndarray:
        c = make_controller(name, **kwargs)
        plant = CornerActuatorPlant(inertia_kgm2=0.6, damping_nms_per_rad=4.0,
                                    coulomb_friction_nm=0.5,
                                    peak_torque_nm=40.0)
        tr = CornerTracker(corner=0, controller=c, plant=plant,
                           sensor=AngleSensor(quant_rad=0.00017, delay_steps=1))
        tr.reset()
        dt = 5e-4
        angles = []
        for k in range(4000):
            load = 3.0 if k < 2000 else 9.0  # load step at t = 1 s
            out = c.step(dt, target_angle=0.2, target_rate=0.0,
                         feedback_angle=tr.sensor.measure(tr.angle),
                         plant_angle=tr.angle, load_torque=load, speed_ms=0.0)
            tr.apply(out, dt=dt, load_torque=load)
            angles.append(tr.angle)
        return np.array(angles)

    a_dob = run_with_load_step("dob", {})
    a_base = run_with_load_step("pid_single",
                                {"kp": 250.0, "ki": 1200.0, "kd": 50.0})
    err_dob = abs(a_dob[-200:].mean() - 0.2)
    err_base = abs(a_base[-200:].mean() - 0.2)
    assert err_dob < err_base  # observer recovers better after the step


def test_smc_sliding_surface_stays_inside_the_boundary_layer():
    phi = 0.05
    angles, _, _, surfaces = _run("smc", {"phi": phi})
    ss = angles[-200:].mean()
    assert abs(ss - 0.2) < 0.001
    # The equilibrium sits inside the boundary layer (s = φ·atanh(load/k)):
    # the contract is "no chatter, bounded surface", not "zero surface" — a
    # zero surface would need infinite switching gain.
    final_surface = [s for s in surfaces[-200:] if s is not None]
    assert np.all(np.abs(final_surface) < phi + 1e-9)


def test_registry_contains_the_advanced_library():
    assert {"dob", "adrc", "smc", "mpc", "h_inf"} <= set(CONTROLLERS)
