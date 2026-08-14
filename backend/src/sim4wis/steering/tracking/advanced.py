"""Advanced controllers — DOB, ADRC, SMC, MPC, H∞ (M4 · FR-3).

The v1 library (PID, LQR) trades between the two classical poles: cascade
PID is the production shape, LQR is the optimal-state-feedback shape. These
five are the answers to the question the evaluation exists to ask — what
happens when the plant is not the nominal one:

    dob   disturbance observer: estimate the load + modelling error from the
          difference between what the commanded torque *should* produce and
          what the rate actually did, feed it back through a Q filter
    adrc  linear ADRC: an extended state observer treats everything that is
          not the nominal double integrator as one state and cancels it —
          the least model-dependent of the five
    smc   sliding mode: drive the error onto s = ė + λ·e and hold it there
          with a bounded switching term (tanh boundary layer, no chatter)
    mpc   constrained linear MPC: finite-horizon quadratic cost with a hard
          |u| ≤ u_max box, solved by a small primal active-set QP — the
          constraint handling is the entire point of having it
    h_inf suboptimal H∞ state feedback: the game Riccati solution for the
          augmented tracking plant at attenuation γ; it guarantees the L2
          gain from load disturbance to [e, ω] is below γ — the only member
          of the library with a disturbance-attenuation *bound* instead of a
          hope

All run at the inner rate and share the v1 feedforward stack. Each one's
analytic contract is pinned in tests (observer pole placement, sliding
surface, MPC unconstrained == LQR first move, Riccati residual and the
closed-loop Hamiltonian stability for H∞).
"""

from __future__ import annotations

import math
from typing import Any, ClassVar

import numpy as np

from sim4wis.steering.tracking.controller import (
    INNER_RATE_HZ,
    AngleTrackingController,
    TrackingOutput,
    register,
)
from sim4wis.steering.tracking.controllers import (
    PidSingleController,
    _FilteredDerivative,
    solve_dare,
)
from sim4wis.steering.tracking.feedforward import (
    FeedforwardContext,
    FeedforwardStack,
    make_feedforward,
)

_DT = 1.0 / INNER_RATE_HZ


def _ctx(dt: float, target_rate: float, load: float, speed_ms: float,
         accel: float = 0.0) -> FeedforwardContext:
    return FeedforwardContext(target_rate=target_rate, target_accel=accel,
                              load_torque=load, speed_ms=speed_ms, dt=dt)


class DobController(AngleTrackingController):
    """PID base + disturbance observer.

    The observer reconstructs the total disturbance (load + modelling
    error) as the difference between the torque actually commanded and the
    torque the nominal plant (J, b) would have needed for the measured
    acceleration, low-passed by the Q filter. Feeding that estimate forward
    lets the PID loop act on a plant that behaves nominal.
    """

    name: ClassVar[str] = "dob"
    inner_rate_hz: ClassVar[float | None] = INNER_RATE_HZ

    def __init__(self, *, inertia: float = 0.6, damping: float = 4.0,
                 q_hz: float = 6.0, base: dict[str, Any] | None = None,
                 torque_limit_nm: float | None = None) -> None:
        self.j, self.b = float(inertia), float(damping)
        self.q_hz = float(q_hz)
        self.torque_limit = torque_limit_nm
        # Softer than the bare PID defaults: the observer feeds the estimate
        # back on top, and a full-strength base double-counts it in the
        # transient (measured: 60 %+ overshoot). The trade — a larger
        # initial overshoot for near-zero steady error under load — is the
        # DOB's characteristic, and the comparison table shows it.
        self.base = PidSingleController(**({"kp": 250.0, "ki": 1200.0,
                                            "kd": 50.0} | (base or {})))
        self._d_hat = 0.0
        self._last_u = 0.0
        self._rate = _FilteredDerivative(0.005)
        self._acc = _FilteredDerivative(0.005)

    def reset(self) -> None:
        self.base.reset()
        self._d_hat = 0.0
        self._last_u = 0.0
        self._rate = _FilteredDerivative(0.005)
        self._acc = _FilteredDerivative(0.005)

    def step(self, dt, *, target_angle, target_rate, feedback_angle,
             plant_angle, load_torque, speed_ms):
        dt = max(float(dt), 1e-9)
        u_base = self.base.step(
            dt, target_angle=target_angle, target_rate=target_rate,
            feedback_angle=feedback_angle, plant_angle=plant_angle,
            load_torque=load_torque, speed_ms=speed_ms).torque_cmd or 0.0
        # Two cascaded filtered differences: angle → rate → acceleration.
        w = self._rate.update(float(plant_angle), dt)
        a = self._acc.update(w, dt)
        raw = self._last_u - (self.j * a + self.b * w)
        alpha = 1.0 - math.exp(-2.0 * math.pi * self.q_hz * dt)
        self._d_hat += (raw - self._d_hat) * alpha
        u = u_base + self._d_hat
        if self.torque_limit is not None:
            u = max(-self.torque_limit, min(self.torque_limit, u))
        self._last_u = u
        return TrackingOutput(
            torque_cmd=u,
            diagnostics={"d_hat": self._d_hat, "torque_cmd": u},
        )


class AdrcController(AngleTrackingController):
    """Linear ADRC (LADRC) — ESO over the nominal double integrator.

    Normalised plant θ̈ = f + b0·u, b0 = 1/J; f collects damping, friction
    and load. A third-order linear ESO estimates [θ, ω, f] with poles at
    −ωo; the control law u = (ωc²(r − θ̂) − 2ωc·ω̂ − f̂)/b0 places the closed
    loop at −ωc when f̂ = f. The least model-dependent design in the
    library: it only needs b0.
    """

    name: ClassVar[str] = "adrc"
    inner_rate_hz: ClassVar[float | None] = INNER_RATE_HZ

    def __init__(self, *, inertia: float = 0.6, omega_c: float = 30.0,
                 omega_o: float = 120.0,
                 torque_limit_nm: float | None = None) -> None:
        self.j = float(inertia)
        self.b0 = 1.0 / self.j
        self.wc, self.wo = float(omega_c), float(omega_o)
        self.torque_limit = torque_limit_nm
        self._z = np.zeros(3)  # [θ̂, ω̂, f̂]
        # Gao's bandwidth parameterisation of the ESO gains.
        self._l = np.array([3.0 * self.wo, 3.0 * self.wo ** 2, self.wo ** 3])

    def reset(self) -> None:
        self._z = np.zeros(3)

    def step(self, dt, *, target_angle, target_rate, feedback_angle,
             plant_angle, load_torque, speed_ms):
        dt = max(float(dt), 1e-9)
        z = self._z
        y = float(plant_angle)
        e = y - z[0]
        u_prev = self._u_prev if hasattr(self, "_u_prev") else 0.0
        u0 = self.wc ** 2 * (float(target_angle) - z[0]) - 2.0 * self.wc * z[1]
        u = (u0 - z[2]) / self.b0
        if self.torque_limit is not None:
            u = max(-self.torque_limit, min(self.torque_limit, u))
        z[0] += (z[1] + self._l[0] * e) * dt
        z[1] += (z[2] + self._l[1] * e + self.b0 * u_prev) * dt
        z[2] += (self._l[2] * e) * dt
        self._u_prev = u
        return TrackingOutput(
            torque_cmd=u,
            diagnostics={"f_hat": float(z[2]), "theta_hat": float(z[0]),
                         "omega_hat": float(z[1])},
        )


class SmcController(AngleTrackingController):
    """Sliding mode on s = ė + λ·e, with a tanh boundary layer.

    The equivalent control (b·ω_ref term) plus a bounded switching term
    that dominates the disturbance; the boundary layer replaces the sign
    function so the command is smooth and the plant sees no chatter.
    """

    name: ClassVar[str] = "smc"
    inner_rate_hz: ClassVar[float | None] = INNER_RATE_HZ

    def __init__(self, *, damping: float = 4.0, lam: float = 25.0,
                 switching_gain: float = 12.0, phi: float = 0.05,
                 ff: FeedforwardStack | dict[str, Any] | None = None,
                 torque_limit_nm: float | None = None) -> None:
        self.b = float(damping)
        self.lam = float(lam)
        self.k = float(switching_gain)
        self.phi = float(phi)
        self.ff = ff if isinstance(ff, FeedforwardStack) else make_feedforward(ff)
        self.torque_limit = torque_limit_nm
        self._rate = _FilteredDerivative(0.005)

    def reset(self) -> None:
        self._rate = _FilteredDerivative(0.005)

    def step(self, dt, *, target_angle, target_rate, feedback_angle,
             plant_angle, load_torque, speed_ms):
        w = self._rate.update(float(plant_angle), dt)
        e = float(target_angle) - float(plant_angle)
        s = (float(target_rate) - w) + self.lam * e
        u = self.b * float(target_rate) + self.k * math.tanh(s / self.phi)
        u += self.ff.compute(_ctx(dt, float(target_rate), float(load_torque),
                                  float(speed_ms)))
        if self.torque_limit is not None:
            u = max(-self.torque_limit, min(self.torque_limit, u))
        return TrackingOutput(
            torque_cmd=u,
            diagnostics={"surface": s, "error_rad": e},
        )


def hildreth_qp(h: np.ndarray, c: np.ndarray, m: np.ndarray, q: np.ndarray,
                *, max_iter: int = 4000, tol: float = 1e-10) -> np.ndarray:
    """Box-constrained QP: min ½u'Hu + c'u s.t. Mu ≤ q (Hildreth's method).

    Gauss-Seidel over the KKT multipliers: making constraint i active needs
    Δλᵢ = (mᵢu − qᵢ)/(mᵢH⁻¹mᵢ) with u = −H⁻¹(c + M'λ) refreshed after
    every update; λᵢ is clamped at zero (the inactive case). The classic
    embedded-MPC solver for exactly this constraint shape; deterministic.
    """
    h_inv = np.linalg.inv(np.asarray(h, dtype=float))
    c = np.asarray(c, dtype=float)
    m, q = np.asarray(m, dtype=float), np.asarray(q, dtype=float)
    lam = np.zeros(m.shape[0])
    u = -h_inv @ c
    if np.all(m @ u - q <= tol):
        return u
    for _ in range(max_iter):
        lam_prev = lam.copy()
        for i in range(m.shape[0]):
            denom = float(m[i] @ h_inv @ m[i])
            if denom < 1e-12:
                continue
            w = (float(m[i] @ u) - float(q[i])) / denom
            lam[i] = max(0.0, lam[i] + w)
            u = -h_inv @ (c + m.T @ lam)
        if np.max(np.abs(lam - lam_prev)) < tol:
            return u
    raise RuntimeError("Hildreth QP did not converge")


class MpcController(AngleTrackingController):
    """Constrained linear MPC — finite horizon, hard |u| ≤ u_max box.

    States [∫e, θ, ω]: the integral state is what removes the constant-load
    steady-state error (an offset-free design for a constant disturbance).
    Unconstrained optimum from the batch LQR recursion (terminal cost from
    the discrete Riccati solution of the 2-state plant), then the box is
    enforced by Hildreth's active-set QP. The constraint handling is the
    whole point: PID and LQR clip *after* the fact, this one plans against
    the limit.
    """

    name: ClassVar[str] = "mpc"
    inner_rate_hz: ClassVar[float | None] = INNER_RATE_HZ

    def __init__(self, *, inertia: float = 0.6, damping: float = 4.0,
                 horizon: int = 10, q_integral: float = 400.0,
                 q_angle: float = 1500.0, q_rate: float = 1.0,
                 r: float = 0.01, u_max: float = 40.0,
                 ff: FeedforwardStack | dict[str, Any] | None = None,
                 dt: float = _DT) -> None:
        self.j, self.b = float(inertia), float(damping)
        self.horizon = max(int(horizon), 2)
        self.u_max = float(u_max)
        self.ff = ff if isinstance(ff, FeedforwardStack) else make_feedforward(ff)
        self.dt = float(dt)
        self._rate = _FilteredDerivative(0.005)
        self._integral = 0.0
        from sim4wis.steering.tracking.controllers import _expm_taylor
        # Augmented plant [∫e, θ, ω]: d∫e = e = target − θ, θ̇ = ω,
        # ω̇ = −b/J·ω + u/J, ZOH-discretised.
        a_c = np.array([[0.0, -1.0, 0.0],
                        [0.0, 0.0, 1.0],
                        [0.0, 0.0, -self.b / self.j]])
        b_c = np.array([[0.0], [0.0], [1.0 / self.j]])
        blk = np.zeros((4, 4))
        blk[:3, :3] = a_c
        blk[:3, 3] = b_c[:, 0]
        e = _expm_taylor(blk, self.dt)
        self._a, self._b = e[:3, :3], e[:3, 3].reshape(3, 1)
        # Terminal cost: DARE on the augmented plant, Newton from the
        # pole-placement PID design in state-feedback form. In the
        # [∫e, θ, ω] basis u = kp·e + ki·∫e − kd·ω = −K·x + kp·target with
        # K = [−ki, kp, kd].
        from sim4wis.steering.tracking.controllers import pole_place_pid
        kp, ki, kd = pole_place_pid(self.j, self.b, wn=30.0, zeta=0.9)
        k0 = np.array([[-ki, kp, kd]])
        q3 = np.diag([q_integral, q_angle, q_rate])
        self._p = solve_dare(self._a, self._b, q3, np.array([[r]]), k0)
        self._q = np.diag([q_integral, q_angle, q_rate])
        self._r = float(r)

    def reset(self) -> None:
        self._rate = _FilteredDerivative(0.005)
        self._integral = 0.0

    def step(self, dt, *, target_angle, target_rate, feedback_angle,
             plant_angle, load_torque, speed_ms):
        # Condensed QP over the horizon: x = M·x0 + N·u_seq.
        n = self.horizon
        n_state = 3
        m_mat = np.zeros((n_state * n, n_state))
        n_mat = np.zeros((n_state * n, n))
        a_pow = np.eye(n_state)
        for k in range(n):
            m_mat[n_state * k:n_state * k + n_state] = a_pow
            for j in range(k):
                n_mat[n_state * k:n_state * k + n_state, j] = (
                    np.linalg.matrix_power(self._a, k - 1 - j) @ self._b)[:, 0]
            n_mat[n_state * k:n_state * k + n_state, k] = self._b[:, 0]
            a_pow = self._a @ a_pow
        q_bar = np.kron(np.eye(n), self._q)
        q_bar[-n_state:, -n_state:] += self._p
        r_bar = self._r * np.eye(n)
        h_qp = n_mat.T @ q_bar @ n_mat + r_bar
        w_meas = self._rate.update(float(feedback_angle), dt)
        e_now = float(target_angle) - float(feedback_angle)
        self._integral += e_now * float(dt)
        x0 = np.array([self._integral, float(feedback_angle), w_meas])
        # Reference along the horizon: integral state 0, target angle, 0 rate.
        x_ref = np.tile([0.0, float(target_angle), float(target_rate)], n)
        c_qp = n_mat.T @ q_bar @ (m_mat @ x0 - x_ref)
        m_cons = np.vstack([np.eye(n), -np.eye(n)])
        q_cons = np.concatenate([np.full(n, self.u_max), np.full(n, self.u_max)])
        u_seq = hildreth_qp(h_qp, c_qp, m_cons, q_cons)
        u = float(u_seq[0])
        u += self.ff.compute(_ctx(dt, float(target_rate), float(load_torque),
                                  float(speed_ms)))
        return TrackingOutput(
            torque_cmd=u,
            diagnostics={"error_rad": e_now, "u0_mpc": u},
        )


def solve_game_care(a: np.ndarray, b1: np.ndarray, b2: np.ndarray,
                    q: np.ndarray, r: np.ndarray, gamma: float) -> np.ndarray:
    """Game Riccati (H∞ state feedback): A'P + PA − P(B2R⁻¹B2' − γ⁻²B1B1')P
    + Q = 0, solved through the Hamiltonian's stable eigenspace.

    Returns the stabilising P; raises ValueError when γ is infeasible (the
    Hamiltonian has imaginary-axis eigenvalues, or the eigenspace does not
    yield a positive-semidefinite solution).
    """
    a, q = np.asarray(a, dtype=float), np.asarray(q, dtype=float)
    b1, b2, r = (np.asarray(x, dtype=float) for x in (b1, b2, r))
    b_tilde = b2 @ np.linalg.inv(r) @ b2.T - (b1 @ b1.T) / (gamma * gamma)
    h = np.block([[a, -b_tilde], [-q, -a.T]])
    vals, vecs = np.linalg.eig(h)
    stable = np.abs(vals.real) > 1e-9
    if np.any(~stable):
        raise ValueError(
            f"γ = {gamma} is infeasible: the Hamiltonian has imaginary-axis "
            "eigenvalues (the attenuation bound cannot be met)")
    keep = vals.real < 0.0
    if int(np.count_nonzero(keep)) != a.shape[0]:
        raise ValueError("Hamiltonian eigenspace is degenerate")
    x1 = vecs[:a.shape[0], keep]
    x2 = vecs[a.shape[0]:, keep]
    p = (x2 @ np.linalg.inv(x1)).real
    p = 0.5 * (p + p.T)
    if np.linalg.eigvalsh(p)[0] < -1e-6:
        raise ValueError("game Riccati solution is not positive semidefinite")
    return p


class HInfController(AngleTrackingController):
    """Suboptimal H∞ state feedback on the augmented tracking plant.

    States [∫e, e, ω]; the disturbance enters as a torque disturbance on ω
    (B1 = the load input). The gain K = R⁻¹B2'P with P from the game Riccati
    guarantees the L2 gain from disturbance to the weighted [e, ω] output is
    below γ — the only controller in the library whose disturbance rejection
    is a bound, not a hope. Raises at construction when γ is infeasible.
    """

    name: ClassVar[str] = "h_inf"
    inner_rate_hz: ClassVar[float | None] = INNER_RATE_HZ

    def __init__(self, *, inertia: float = 0.6, damping: float = 4.0,
                 gamma: float = 6.0, q_integral: float = 300.0,
                 q_angle: float = 900.0, q_rate: float = 1.0,
                 r: float = 0.01,
                 torque_limit_nm: float | None = None) -> None:
        self.j, self.b = float(inertia), float(damping)
        self.torque_limit = torque_limit_nm
        a = np.array([[0.0, 1.0, 0.0],
                      [0.0, 0.0, -1.0],
                      [0.0, 0.0, -self.b / self.j]])
        b2 = np.array([[0.0], [0.0], [1.0 / self.j]])
        b1 = b2.copy()  # torque disturbance — the load's entry point
        q = np.diag([q_integral, q_angle, q_rate])
        p = solve_game_care(a, b1, b2, q, np.array([[r]]), float(gamma))
        self._k = (np.linalg.inv(np.array([[r]])) @ b2.T @ p)[0]
        self._integral = 0.0
        self._rate = _FilteredDerivative(0.005)
        self._gamma = float(gamma)

    def reset(self) -> None:
        self._integral = 0.0
        self._rate = _FilteredDerivative(0.005)

    def step(self, dt, *, target_angle, target_rate, feedback_angle,
             plant_angle, load_torque, speed_ms):
        e = float(target_angle) - float(feedback_angle)
        self._integral += e * float(dt)
        w = self._rate.update(float(feedback_angle), float(dt))
        u = -(self._k[0] * self._integral + self._k[1] * e + self._k[2] * w)
        if self.torque_limit is not None:
            u = max(-self.torque_limit, min(self.torque_limit, u))
        return TrackingOutput(
            torque_cmd=u,
            diagnostics={"error_rad": e, "integral": self._integral},
        )


register(DobController)
register(AdrcController)
register(SmcController)
register(MpcController)
register(HInfController)
