"""Feedback controllers v1 — PID single-loop, PID cascade, LQR.

All three command torque through the corner plant, run at the layer's inner
rate (2000 Hz), and accept an optional :class:`FeedforwardStack`. Defaults
are analytic: pole placement for the two PID structures on the nominal plant
(J·θ̈ + b·θ̇ = τ), and a discrete LQR solution for the state-feedback one.
The defaults are *engineering defaults*, not calibrated values — the tuning
workbench (sim4wis.steering.tracking.tuning) is the source of reviewed
numbers, and the evaluation protocol is what judges them.

Design notes that matter later:

* **Derivative on measurement, filtered.** A quantised angle (5e-4 rad steps)
  differentiated at 2000 Hz is mostly noise; every controller here uses a
  first-order filtered difference instead of a raw one. PID's D term acts on
  the measured rate, not the error rate — the classic anti-derivative-kick
  arrangement.
* **Integral clamping.** Unbounded integrators wind up while the actuator is
  saturated and unwind visibly afterwards. Each integral state is clamped.
* **Scheduling.** ``GainSchedule`` linearly interpolates gains by vehicle
  speed; steering load changes by an order of magnitude between parking and
  motorway, and no single gain covers both (FR-8).
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
from sim4wis.steering.tracking.feedforward import (
    FeedforwardContext,
    FeedforwardStack,
    make_feedforward,
)


class _FilteredDerivative:
    """First-order filtered difference of a signal, per-sample state."""

    def __init__(self, tau_s: float = 0.01) -> None:
        self.tau = float(tau_s)
        self.prev: float | None = None
        self.value = 0.0

    def update(self, v: float, dt: float) -> float:
        dt = max(float(dt), 1e-9)
        if self.prev is None:
            self.prev = float(v)
            return 0.0
        raw = (float(v) - self.prev) / dt
        self.prev = float(v)
        self.value += (raw - self.value) * dt / (self.tau + dt)
        return self.value


def pole_place_pid(j: float, b: float, wn: float, zeta: float,
                   p: float | None = None) -> tuple[float, float, float]:
    """(kp, ki, kd) placing (s+p)(s²+2ζωₙs+ωₙ²) on J·a + (b+kd)·ω = kp·e + ki·∫e.

    The damping term kd is the *rate* feedback, so it adds to the plant's
    viscous term; kp/ki are the position loop. p defaults to 2ζωₙ (the ITAE
    choice for a third-order system).
    """
    p = 2.0 * zeta * wn if p is None else float(p)
    kd = j * (p + 2.0 * zeta * wn) - b
    kp = j * (wn * wn + 2.0 * zeta * wn * p)
    ki = j * p * wn * wn
    return kp, ki, kd


class GainSchedule:
    """Linear interpolation of a gain vector by vehicle speed [m/s].

    ``table`` = {speed_ms: {"kp": ..., "ki": ..., ...}} — the controller
    resolves every named gain through it each step.
    """

    def __init__(self, table: dict[float, dict[str, float]]) -> None:
        if not table:
            raise ValueError("gain schedule needs at least one breakpoint")
        self._speeds = np.array(sorted(table), dtype=float)
        self._cols = sorted({k for v in table.values() for k in v})
        self._grid = np.array(
            [[table[s].get(c, math.nan) for c in self._cols]
             for s in self._speeds], dtype=float)

    def gains(self, speed_ms: float) -> dict[str, float]:
        v = float(speed_ms)
        idx = int(np.searchsorted(self._speeds, v, side="right") - 1)
        idx = max(0, min(idx, self._speeds.size - 2))
        w = 0.0 if self._speeds.size < 2 else (
            (v - self._speeds[idx])
            / max(self._speeds[idx + 1] - self._speeds[idx], 1e-12))
        w = max(0.0, min(1.0, w))
        row = self._grid[idx] * (1.0 - w) + self._grid[idx + 1] * w
        return {c: float(row[i]) for i, c in enumerate(self._cols)}


def _context(dt: float, target_rate: float, target_accel: float,
             load_torque: float, speed_ms: float) -> FeedforwardContext:
    return FeedforwardContext(target_rate=target_rate, target_accel=target_accel,
                              load_torque=load_torque, speed_ms=speed_ms, dt=dt)


class PidSingleController(AngleTrackingController):
    """Single-loop PID on the position error, torque output.

    Structure: τ = kp·e + ki·∫e − kd·ω_meas + τ_ff. The D term is rate
    feedback (filtered measured rate) rather than error-rate feedback.
    """

    name: ClassVar[str] = "pid_single"
    inner_rate_hz: ClassVar[float | None] = INNER_RATE_HZ

    def __init__(
        self,
        # Defaults tuned in the vehicle loop against the shipped step
        # procedure (scripts/tune_vehicle_defaults.py, 2026-08-15): the
        # pole-placement defaults left a slow tail once the one-step-old
        # rack force and the tire aligning spring were in the loop.
        kp: float = 2165.0, ki: float = 8170.0, kd: float = 47.91,
        *,
        ff: FeedforwardStack | dict[str, Any] | None = None,
        torque_limit_nm: float | None = None,
        integral_clamp: float = 2.0,
        deriv_tau_s: float = 0.01,
        schedule: dict[float, dict[str, float]] | None = None,
    ) -> None:
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
        self.ff = ff if isinstance(ff, FeedforwardStack) else make_feedforward(ff)
        self.torque_limit = torque_limit_nm
        self.integral_clamp = float(integral_clamp)
        self.schedule = GainSchedule(schedule) if schedule else None
        self._integral = 0.0
        self._rate = _FilteredDerivative(deriv_tau_s)
        self._prev_feedback: float | None = None
        self._prev_target_rate = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._rate = _FilteredDerivative(self._rate.tau)
        self._prev_feedback = None
        self._prev_target_rate = 0.0

    def _gains(self, speed_ms: float) -> tuple[float, float, float]:
        if self.schedule is None:
            return self.kp, self.ki, self.kd
        g = self.schedule.gains(speed_ms)
        return (g.get("kp", self.kp), g.get("ki", self.ki), g.get("kd", self.kd))

    def step(self, dt, *, target_angle, target_rate, feedback_angle,
             plant_angle, load_torque, speed_ms):
        dt = max(float(dt), 1e-9)
        kp, ki, kd = self._gains(float(speed_ms))
        e = float(target_angle) - float(feedback_angle)
        self._integral += e * dt
        self._integral = max(-self.integral_clamp,
                             min(self.integral_clamp, self._integral))
        w_meas = self._rate.update(float(feedback_angle), dt)
        target_accel = (float(target_rate) - self._prev_target_rate) / dt
        self._prev_target_rate = float(target_rate)
        self._prev_feedback = float(feedback_angle)
        u = (kp * e + ki * self._integral - kd * w_meas
             + self.ff.compute(_context(dt, float(target_rate), target_accel,
                                        float(load_torque), float(speed_ms))))
        if self.torque_limit is not None:
            u = max(-self.torque_limit, min(self.torque_limit, u))
        return TrackingOutput(
            torque_cmd=u,
            diagnostics={"error_rad": e, "integral": self._integral,
                         "rate_meas": w_meas, "torque_cmd": u},
        )


class PidCascadeController(AngleTrackingController):
    """Cascade: position PI (outer) → rate command → velocity PI (inner).

    The production EPS/SBW shape: the inner loop linearises the actuator, the
    outer loop then sees a velocity servo rather than an inertia. Integral
    clamps on both loops.
    """

    name: ClassVar[str] = "pid_cascade"
    inner_rate_hz: ClassVar[float | None] = INNER_RATE_HZ

    def __init__(
        self,
        # Defaults tuned in the vehicle loop (see pid_single).
        kp_pos: float = 45.17, ki_pos: float = 2.0,
        kp_vel: float = 44.86, ki_vel: float = 327.3,
        *,
        ff: FeedforwardStack | dict[str, Any] | None = None,
        torque_limit_nm: float | None = None,
        integral_clamp_pos: float = 2.0,
        integral_clamp_vel: float = 2.0,
        deriv_tau_s: float = 0.005,
        rate_ref_clamp: float = 6.0,
        schedule: dict[float, dict[str, float]] | None = None,
    ) -> None:
        self.kp_pos, self.ki_pos = float(kp_pos), float(ki_pos)
        self.kp_vel, self.ki_vel = float(kp_vel), float(ki_vel)
        self.ff = ff if isinstance(ff, FeedforwardStack) else make_feedforward(ff)
        self.torque_limit = torque_limit_nm
        self.clamp_pos, self.clamp_vel = float(integral_clamp_pos), float(integral_clamp_vel)
        self.rate_ref_clamp = float(rate_ref_clamp)
        self.schedule = GainSchedule(schedule) if schedule else None
        self._i_pos = 0.0
        self._i_vel = 0.0
        self._rate = _FilteredDerivative(deriv_tau_s)
        self._prev_target_rate = 0.0

    def reset(self) -> None:
        self._i_pos = 0.0
        self._i_vel = 0.0
        self._rate = _FilteredDerivative(self._rate.tau)
        self._prev_target_rate = 0.0

    def _gains(self, speed_ms: float) -> tuple[float, float, float, float]:
        if self.schedule is None:
            return self.kp_pos, self.ki_pos, self.kp_vel, self.ki_vel
        g = self.schedule.gains(speed_ms)
        return (g.get("kp_pos", self.kp_pos), g.get("ki_pos", self.ki_pos),
                g.get("kp_vel", self.kp_vel), g.get("ki_vel", self.ki_vel))

    def step(self, dt, *, target_angle, target_rate, feedback_angle,
             plant_angle, load_torque, speed_ms):
        dt = max(float(dt), 1e-9)
        kp_pos, ki_pos, kp_vel, ki_vel = self._gains(float(speed_ms))
        e = float(target_angle) - float(feedback_angle)
        self._i_pos += e * dt
        self._i_pos = max(-self.clamp_pos, min(self.clamp_pos, self._i_pos))
        rate_ref = kp_pos * e + ki_pos * self._i_pos
        rate_ref = max(-self.rate_ref_clamp, min(self.rate_ref_clamp, rate_ref))
        w_meas = self._rate.update(float(feedback_angle), dt)
        e_vel = rate_ref - w_meas
        self._i_vel += e_vel * dt
        self._i_vel = max(-self.clamp_vel, min(self.clamp_vel, self._i_vel))
        target_accel = (float(target_rate) - self._prev_target_rate) / dt
        self._prev_target_rate = float(target_rate)
        u = (kp_vel * e_vel + ki_vel * self._i_vel
             + self.ff.compute(_context(dt, float(target_rate), target_accel,
                                        float(load_torque), float(speed_ms))))
        if self.torque_limit is not None:
            u = max(-self.torque_limit, min(self.torque_limit, u))
        return TrackingOutput(
            torque_cmd=u,
            diagnostics={"error_rad": e, "rate_ref": rate_ref,
                         "rate_meas": w_meas, "torque_cmd": u},
        )


def solve_dare(a: np.ndarray, b: np.ndarray, q: np.ndarray,
               r: np.ndarray, k0: np.ndarray, *,
               tol: float = 1e-9, max_iter: int = 40) -> np.ndarray:
    """Discrete algebraic Riccati by Newton iteration from a stabilising gain.

    A tracking plant contains a pure integrator — its discrete A has an
    eigenvalue exactly on the unit circle — and the plain fixed-point Riccati
    iteration converges to the antistabilising solution for such systems.
    Newton's method does not care: starting from any stabilising K it
    converges quadratically to the optimal solution. The caller's initial K
    is the pole-placement PID design, so the LQR gain is literally the PID
    design refined to optimality.

    Each step: A_cl = A − B·K; P solves the discrete Lyapunov equation
    P = Q + K'RK + A_cl'PA_cl (by contractive iteration, A_cl stable);
    K_new = (R + B'PB)⁻¹B'PA.
    """
    a, b, q, r = (np.asarray(x, dtype=float) for x in (a, b, q, r))
    k = np.asarray(k0, dtype=float).reshape(1, -1)
    p = np.zeros_like(q)
    for _ in range(max_iter):
        a_cl = a - b @ k
        p[:] = 0.0
        for _ in range(200000):
            p_next = q + k.T @ r @ k + a_cl.T @ p @ a_cl
            # Relative tolerance: with large weights the solution's entries
            # reach ~1e7, and an absolute 1e-13 floor is unreachable in
            # double precision — the loop would exit unconverged.
            if np.max(np.abs(p_next - p)) < 1e-12 * max(
                    1.0, float(np.max(np.abs(p_next)))):
                p = p_next
                break
            p = p_next
        k_new = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
        if np.max(np.abs(k_new - k)) < tol:
            return p
        k = k_new
    raise RuntimeError("DARE Newton iteration did not converge")


class LqrController(AngleTrackingController):
    """LQR state feedback with integral action on the nominal plant.

    States: [∫e, e, ω] with the plant (J·θ̈ + b·θ̇ = τ) discretised at the
    inner rate. Gains from a discrete Riccati solution — given the model, the
    gains are given too, which is what the tuning workbench's "analytic"
    channel is built on.
    """

    name: ClassVar[str] = "lqr"
    inner_rate_hz: ClassVar[float | None] = INNER_RATE_HZ

    def __init__(
        self,
        *,
        inertia: float = 0.6, damping: float = 4.0,
        # Defaults tuned in the vehicle loop (see pid_single).
        q_integral: float = 50.0, q_angle: float = 1.97e4,
        q_rate: float = 0.1, r: float = 0.001,
        dt: float = 1.0 / INNER_RATE_HZ,
        ff: FeedforwardStack | dict[str, Any] | None = None,
        torque_limit_nm: float | None = None,
    ) -> None:
        self.j, self.b = float(inertia), float(damping)
        self.ff = ff if isinstance(ff, FeedforwardStack) else make_feedforward(ff)
        self.torque_limit = torque_limit_nm
        self.dt = float(dt)
        self._integral = 0.0
        self._rate = _FilteredDerivative(0.005)
        # Continuous plant on [e, ω] with ė = −ω, ω̇ = −b/J·ω + u/J.
        a_c = np.array([[0.0, -1.0], [0.0, -self.b / self.j]])
        b_c = np.array([[0.0], [1.0 / self.j]])
        # Augment with the integral state [∫e, e, ω]; ė_int = e.
        n = 3
        a_a = np.zeros((n, n))
        a_a[0, 1] = 1.0
        a_a[1:, 1:] = a_c
        b_a = np.zeros((n, 1))
        b_a[1:, 0] = b_c[:, 0]
        # Exact discretisation (matrix exponential of the block).
        m = np.zeros((n + 1, n + 1))
        m[:n, :n] = a_a
        m[:n, n] = b_a[:, 0]
        expm = _expm_taylor(m, self.dt)
        a_d = expm[:n, :n]
        b_d = expm[:n, n].reshape(n, 1)
        q_a = np.diag([q_integral, q_angle, q_rate])
        # Newton needs a stabilising start: the pole-placement PID design in
        # state-feedback form, u = kp·e + ki·∫e − kd·ω = −K·[∫e, e, ω].
        kp0, ki0, kd0 = pole_place_pid(self.j, self.b, wn=25.0, zeta=0.9)
        k0 = np.array([[-ki0, -kp0, kd0]])
        p = solve_dare(a_d, b_d, q_a, np.array([[r]]), k0)
        # K = (R + B'PB)⁻¹ B'PA
        self._k = np.linalg.solve(r + b_d.T @ p @ b_d, b_d.T @ p @ a_d)[0]
        self._a_d, self._b_d = a_d, b_d  # kept for the analytic test pin
        self._r = float(r)

    def reset(self) -> None:
        self._integral = 0.0
        self._rate = _FilteredDerivative(0.005)

    def step(self, dt, *, target_angle, target_rate, feedback_angle,
             plant_angle, load_torque, speed_ms):
        e = float(target_angle) - float(feedback_angle)
        self._integral += e * float(dt)
        w_meas = self._rate.update(float(feedback_angle), float(dt))
        u = -(self._k[0] * self._integral + self._k[1] * e + self._k[2] * w_meas)
        u += self.ff.compute(_context(float(dt), float(target_rate), 0.0,
                                      float(load_torque), float(speed_ms)))
        if self.torque_limit is not None:
            u = max(-self.torque_limit, min(self.torque_limit, u))
        return TrackingOutput(
            torque_cmd=u,
            diagnostics={"error_rad": e, "integral": self._integral,
                         "torque_cmd": u},
        )


def _expm_taylor(m: np.ndarray, t: float, order: int = 24) -> np.ndarray:
    """Matrix exponential via scaling-and-squaring on a Taylor series.

    Used to discretise the 3-state augmented plant exactly; scipy is not a
    dependency and a 4×4 matrix needs nothing heavier. After scaling the
    argument below norm 1, a 24-term Taylor series is accurate to machine
    precision.
    """
    n = m.shape[0]
    s = max(0, int(math.ceil(math.log2(max(float(t * np.linalg.norm(m, 1)), 1e-12)))))
    a = m * (t / (2.0 ** s))
    ident = np.eye(n)
    out = ident.copy()
    term = ident.copy()
    for k in range(1, order + 1):
        term = term @ a / k
        out = out + term
    for _ in range(s):
        out = out @ out
    return out


register(PidSingleController)
register(PidCascadeController)
register(LqrController)
