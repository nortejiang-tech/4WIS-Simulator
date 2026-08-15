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
    _expm_taylor,
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

    The observer reconstructs the total disturbance (load + friction +
    modelling error) as the difference between the torque actually commanded
    and the torque the nominal plant (J, b) would have needed for the
    measured motion, low-passed by the Q filter. Feeding that estimate
    forward lets the base loop act on a plant that behaves nominal.

    **The observer IS the load compensator.** A rack-force or friction
    feedforward block stacked on top would double-count the same
    disturbance — the observer cannot tell "torque that cancels the load"
    from "the load itself"; both appear in the commanded torque — and the
    doubled compensation drives the wheel off its command (measured in the
    vehicle loop: 0.26-0.6 rad of wander across scenarios, vs 0.005 with
    the roles kept separate). The constructor therefore refuses those
    blocks; the velocity feedforward is fine (it compensates the reference
    dynamics, not a disturbance).

    The base carries no integral (ki = 0): the DC action is the observer's
    job, and a base integrator on the same error forms the classic
    two-integrator fight.
    """

    name: ClassVar[str] = "dob"
    inner_rate_hz: ClassVar[float | None] = INNER_RATE_HZ

    #: Feedforward blocks that would double-count a disturbance the observer
    #: already cancels. Refused at construction, loudly.
    _FORBIDDEN_FF = ("rack_force", "friction")

    def __init__(self, *, inertia: float = 0.6, damping: float = 4.0,
                 # Defaults tuned in the vehicle loop against the step
                 # procedure (scripts/tune_vehicle_defaults.py, 2026-08-15).
                 q_hz: float = 8.09, d_hat_limit_nm: float = 60.0,
                 base: dict[str, Any] | None = None,
                 ff: FeedforwardStack | dict[str, Any] | None = None,
                 torque_limit_nm: float = 120.0) -> None:
        self.j, self.b = float(inertia), float(damping)
        self.q_hz = float(q_hz)
        #: The observer's estimate is clamped at a physical bound: a
        #: "disturbance" larger than anything the actuator could oppose is
        #: an artefact of the filtered acceleration lagging the command,
        #: not physics.
        self.d_hat_limit = float(d_hat_limit_nm)
        self.torque_limit = torque_limit_nm
        stack = ff if isinstance(ff, FeedforwardStack) else make_feedforward(ff)
        bad = [b.name for b in stack.blocks if b.name in self._FORBIDDEN_FF]
        if bad:
            raise ValueError(
                f"dob refuses feedforward block(s) {bad}: the disturbance "
                "observer already cancels the load and the friction, and "
                "stacking them again double-counts the same disturbance "
                "(measured: the wheel wanders off its command). Velocity "
                "feedforward is fine.")
        merged = {"kp": 1253.0, "ki": 0.0, "kd": 43.83} | (base or {})
        merged["ff"] = stack
        self.base = PidSingleController(**merged)
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
        self._d_hat = max(-self.d_hat_limit, min(self.d_hat_limit, self._d_hat))
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

    #: The ESO lumps load + friction + model error into f̂ and cancels it —
    #: the same mutual exclusion as the DOB: rack-force/friction feedforward
    #: on top double-counts the disturbance (measured: the vehicle-loop
    #: tuner against a feedforward-carrying spec read a 12 s cost where the
    #: bare controller settles in 0.3 s).
    _FORBIDDEN_FF = ("rack_force", "friction")

    def __init__(self, *, inertia: float = 0.6,
                 # Defaults probed at procedure conditions after the
                 # feedforward exclusion fix (30 km/h 1°: 0.10 s settle,
                 # 0.8 % overshoot); the ESO runs on the plant truth here —
                 # a real quantised sensor bounds omega_o in practice.
                 omega_c: float = 60.0, omega_o: float = 400.0,
                 ff: FeedforwardStack | dict[str, Any] | None = None,
                 torque_limit_nm: float = 120.0) -> None:
        # The limit must mirror the actuator's: the ESO integrates the
        # torque it believes was applied, and an unsaturated command against
        # a saturated plant is pure model mismatch (measured: the bench
        # diverged at large angles while small-angle vehicle runs passed).
        self.j = float(inertia)
        self.b0 = 1.0 / self.j
        self.wc, self.wo = float(omega_c), float(omega_o)
        stack = ff if isinstance(ff, FeedforwardStack) else make_feedforward(ff)
        bad = [b.name for b in stack.blocks if b.name in self._FORBIDDEN_FF]
        if bad:
            raise ValueError(
                f"adrc refuses feedforward block(s) {bad}: the extended state "
                "observer already cancels the load and the friction — "
                "stacking them again double-counts the disturbance.")
        self.ff = stack
        self.torque_limit = torque_limit_nm
        self._z = np.zeros(3)  # [θ̂, ω̂, f̂]
        self._u_prev = 0.0
        # Exact-discrete ESO (Gao's bandwidth parameterisation, ZOH form).
        # The continuous observer with poles at −ωo is discretised exactly
        # and the gain places the discrete poles at z = exp(−ωo·h): the
        # explicit-Euler ESO turns sample-rate fragile above ωo·h ≈ 0.2
        # (measured: divergence at ωo = 400 Hz on the plant bench while the
        # small-angle vehicle runs looked fine — amplitude masked it).
        h = 1.0 / INNER_RATE_HZ
        a_obs = np.array([[0.0, 1.0, 0.0],
                          [0.0, 0.0, 1.0],
                          [0.0, 0.0, 0.0]])
        b_obs = np.array([[0.0], [self.b0], [0.0]])
        blk = np.zeros((4, 4))
        blk[:3, :3] = a_obs
        blk[:3, 3] = b_obs[:, 0]
        e = _expm_taylor(blk, h)
        self._a_d = e[:3, :3]
        self._b_d = e[:3, 3]
        rho = math.exp(-self.wo * h)
        # Ackermann on the dual (controller) problem: K for (A', C') places
        # the poles, then L = K'. φ is of the DUAL matrix A' — φ(A) ≠ φ(A')
        # for a non-symmetric A, and using the wrong one hands back a gain
        # that diverges instead of observing (found the hard way).
        at, ct = self._a_d.T, np.array([[1.0, 0.0, 0.0]]).T
        ctrb = np.hstack([ct, at @ ct, at @ at @ ct])
        phi = np.linalg.matrix_power(at - rho * np.eye(3), 3)
        e3 = np.zeros((1, 3))
        e3[0, 2] = 1.0
        self._l = (e3 @ np.linalg.inv(ctrb) @ phi).ravel()

    def reset(self) -> None:
        self._z = np.zeros(3)
        self._u_prev = 0.0

    def step(self, dt, *, target_angle, target_rate, feedback_angle,
             plant_angle, load_torque, speed_ms):
        z = self._z
        y = float(plant_angle)
        u_prev = self._u_prev
        u0 = self.wc ** 2 * (float(target_angle) - z[0]) - 2.0 * self.wc * z[1]
        u = (u0 - z[2]) / self.b0
        u += self.ff.compute(_ctx(dt, float(target_rate), float(load_torque),
                                  float(speed_ms)))
        if self.torque_limit is not None:
            u = max(-self.torque_limit, min(self.torque_limit, u))
        self._z = self._a_d @ z + self._b_d * u_prev + self._l * (y - z[0])
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
                 switching_gain: float = 25.0, phi: float = 0.05,
                 ff: FeedforwardStack | dict[str, Any] | None = None,
                 torque_limit_nm: float | None = None) -> None:
        self.b = float(damping)
        self.lam = float(lam)
        # The switching gain must exceed the disturbance bound or the
        # sliding condition is unreachable: at 100 km/h the straight-line
        # aligning load alone is ~12 N·m (600 N rack × 0.02 m pinion), and
        # with k = 12 the saturated tanh could not push back — the wheel was
        # blown to the steer stop (measured, weave_100). 25 N·m covers the
        # aligning load up to ~1.25 kN of rack force.
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
                 horizon: int = 10,
                 # Defaults tuned in the vehicle loop at the actuator-aligned
                 # u_max (scripts/tune_vehicle_defaults.py, 2026-08-15; the
                 # first tuning round at u_max = 40 does not carry over —
                 # widening the box re-weights the whole QP).
                 q_integral: float = 103.5, q_angle: float = 2.0e4,
                 q_rate: float = 1.0, r: float = 0.001, u_max: float = 120.0,
                 ff: FeedforwardStack | dict[str, Any] | None = None,
                 dt: float = _DT) -> None:
        self.j, self.b = float(inertia), float(damping)
        self.horizon = max(int(horizon), 2)
        self.u_max = float(u_max)
        self.ff = ff if isinstance(ff, FeedforwardStack) else make_feedforward(ff)
        self.dt = float(dt)
        self._rate = _FilteredDerivative(0.005)
        self._integral = 0.0
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
        # The condensed QP's state-independent parts are constant: A, B, Q,
        # R and the terminal weight do not change per step. Precomputing
        # them moves ~n² matrix powers out of the 2000 Hz inner loop — the
        # per-step work drops to one matvec, one solve and the QP iteration.
        n = self.horizon
        m_mat = np.zeros((3 * n, 3))
        n_mat = np.zeros((3 * n, n))
        a_pow = np.eye(3)
        for k in range(n):
            m_mat[3 * k:3 * k + 3] = a_pow
            for j in range(k):
                n_mat[3 * k:3 * k + 3, j] = (
                    np.linalg.matrix_power(self._a, k - 1 - j) @ self._b)[:, 0]
            n_mat[3 * k:3 * k + 3, k] = self._b[:, 0]
            a_pow = self._a @ a_pow
        q_bar = np.kron(np.eye(n), self._q)
        q_bar[-3:, -3:] += self._p
        self._h_inv = np.linalg.inv(n_mat.T @ q_bar @ n_mat + self._r * np.eye(n))
        self._nq_m = n_mat.T @ q_bar @ m_mat      # multiplies x0
        self._nq = n_mat.T @ q_bar                 # multiplies (−x_ref)
        self._hqp = n_mat.T @ q_bar @ n_mat + self._r * np.eye(n)

    def reset(self) -> None:
        self._rate = _FilteredDerivative(0.005)
        self._integral = 0.0

    def step(self, dt, *, target_angle, target_rate, feedback_angle,
             plant_angle, load_torque, speed_ms):
        n = self.horizon
        w_meas = self._rate.update(float(feedback_angle), dt)
        e_now = float(target_angle) - float(feedback_angle)
        self._integral += e_now * float(dt)
        x0 = np.array([self._integral, float(feedback_angle), w_meas])
        x_ref = np.tile([0.0, float(target_angle), float(target_rate)], n)
        c_qp = self._nq_m @ x0 - self._nq @ x_ref
        m_cons = np.vstack([np.eye(n), -np.eye(n)])
        q_cons = np.concatenate([np.full(n, self.u_max), np.full(n, self.u_max)])
        u_seq = hildreth_qp(self._hqp, c_qp, m_cons, q_cons)
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
                 # Defaults tuned in the vehicle loop (see pid/dob notes);
                 # the tuner walked gamma down to 2 — a 3x tighter
                 # disturbance bound than the plant-level default.
                 gamma: float = 2.0, q_integral: float = 6324.0,
                 q_angle: float = 2.0e4, q_rate: float = 1.0,
                 r: float = 0.01,
                 ff: FeedforwardStack | dict[str, Any] | None = None,
                 torque_limit_nm: float = 120.0) -> None:
        self.j, self.b = float(inertia), float(damping)
        self.ff = ff if isinstance(ff, FeedforwardStack) else make_feedforward(ff)
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
        u += self.ff.compute(_ctx(dt, float(target_rate), float(load_torque),
                                  float(speed_ms)))
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
