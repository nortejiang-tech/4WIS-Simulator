"""Unified rear-wheel-steering (RWS) strategy with selectable sub-modes.

One strategy, five control laws picked via `mode_params["rws_mode"]`:

    fixed_ratio      δr = ratio · δf                       (constant, baseline)
    speed_schedule   δr = k(vx) · δf                       (low-speed counter / high-speed same)
    yaw_feedback     δr = g1·δf + g2·ŷaw                    (closed loop)
    transient        δr = k_ss(vx)·δf − c·d̂δf/dt           (steady + fast-steer counter)
    model_following  δr = k_ss(vx)·δf + g·(r_ref − ŷaw)    (zero-sideslip FF + yaw tracking)

ŷaw / d̂δf are first-order low-pass filtered (τ=FILT_TAU). This is essential on
the kinematic model, where the body realises the commanded yaw with no lag —
raw yaw feedback there is an algebraic loop that oscillates (±limit) once the
loop gain exceeds 1. Filtering breaks that loop and also removes the rear-wheel
jitter seen on the dynamic models. Defaults are tuned (scripts/tune_rws.py) to
be stable on kinematic / simplified_dynamic / multibody alike.
"""

from __future__ import annotations

import numpy as np

from sim4wis.controller.base import ControllerStrategy
from sim4wis.controller.longitudinal import speed_command
from sim4wis.controller.steering_feel import front_steer_angle
from sim4wis.controller.rws_common import (
    command_from_axle_angles,
    reference_yaw_rate,
    zero_sideslip_ratio,
)
from sim4wis.core.state import ControlCommand, DriverInput, VehicleState

MODES = ("fixed_ratio", "speed_schedule", "yaw_feedback", "transient", "model_following")
DEFAULT_MODE = "speed_schedule"

# Filter + tuned gains (see scripts/tune_rws.py). A light filter (τ≈0.08) plus
# modest gains keeps the kinematic algebraic loop stable WITHOUT adding the lag
# that a heavy filter would (heavy filtering rang/overshot on the dynamic models).
FILT_TAU = 0.08          # s — low-pass time constant for fed-back signals
DEFAULT_RATIO = -0.4     # fixed_ratio
DEFAULT_G1 = -0.10       # yaw_feedback feed-forward
DEFAULT_G2 = 0.12        # yaw_feedback yaw gain [s] (kinematic-stable)
DEFAULT_C = 0.10         # transient rate gain [s]
DEFAULT_G_YAW = 0.08     # model_following yaw-error gain [s]

# Default speed schedule (km/h, k) — sampled analytic zero-sideslip ratio (LS9).
DEFAULT_SPEEDS_KMH = (0.0, 20.0, 40.0, 60.0, 90.0, 130.0, 200.0)


class RearWheelSteerStrategy(ControllerStrategy):
    name = "rear_wheel_steer"

    def __init__(self, params) -> None:  # noqa: ANN001
        super().__init__(params)
        self._yaw_f = 0.0          # filtered yaw rate
        self._ddf_f = 0.0          # filtered steering-rate
        self._df_prev: float | None = None
        self._t_prev: float | None = None

    def _default_curve(self) -> list[tuple[float, float]]:
        return [(v, zero_sideslip_ratio(self.params, v / 3.6)) for v in DEFAULT_SPEEDS_KMH]

    def compute(self, driver: DriverInput, state: VehicleState, dt: float = 0.0) -> ControlCommand:
        p = self.params
        mp = driver.mode_params
        mode = mp.get("rws_mode", DEFAULT_MODE)
        if mode not in MODES:
            mode = DEFAULT_MODE
        # Front-axle angle via the shared feel layer (work-package B): the
        # same variable gear ratio + soft limit that ideal_ackermann uses, so
        # both hero strategies have consistent on-centre feel across speed.
        raw_rad = mp.get("steer_raw_rad") if mp else None
        if raw_rad is not None:
            df = max(-p.steer_limit, min(p.steer_limit, float(raw_rad)))
        elif mp and mp.get("steer_bypass_feel"):
            # Open-loop excitation bypass — see ideal_ackermann.compute.
            df = p.steer_limit * float(driver.steering)
        else:
            df = front_steer_angle(p, float(driver.steering), float(state.vx),
                                   float(state.mu_avg))
        v = abs(float(state.vx))
        v_cmd = speed_command(p, driver, float(state.vx), dt, float(state.mu_avg))

        # Timestep for the filters (robust to the fixed 200 Hz loop).
        t = float(state.t)
        dt = 0.005
        if self._t_prev is not None and t > self._t_prev:
            dt = min(0.05, t - self._t_prev)
        self._t_prev = t
        a = dt / (FILT_TAU + dt)

        # Filtered yaw rate.
        self._yaw_f += a * (float(state.yaw_rate) - self._yaw_f)
        # Filtered steering derivative.
        ddf_raw = 0.0
        if self._df_prev is not None and dt > 1e-6:
            ddf_raw = (df - self._df_prev) / dt
        self._df_prev = df
        self._ddf_f += a * (ddf_raw - self._ddf_f)

        if mode == "fixed_ratio":
            ratio = float(np.clip(mp.get("rear_ratio", DEFAULT_RATIO), -1.0, 1.0))
            dr = ratio * df
        elif mode == "speed_schedule":
            raw = mp.get("k_curve")
            if isinstance(raw, (list, tuple)) and len(raw) >= 2:
                pts = sorted(((float(x), float(y)) for x, y in raw), key=lambda q: q[0])
            else:
                pts = self._default_curve()
            xs = np.array([q[0] for q in pts])
            ys = np.array([q[1] for q in pts])
            dr = float(np.interp(v * 3.6, xs, ys)) * df
        elif mode == "yaw_feedback":
            g1 = float(mp.get("g1", DEFAULT_G1))
            g2 = float(mp.get("g2", DEFAULT_G2))
            dr = g1 * df + g2 * self._yaw_f
        elif mode == "transient":
            c = float(mp.get("c_transient", DEFAULT_C))
            dr = zero_sideslip_ratio(p, v) * df - c * self._ddf_f
        else:  # model_following
            g = float(mp.get("g_yaw", DEFAULT_G_YAW))
            r_ref = reference_yaw_rate(p, v, df)
            dr = zero_sideslip_ratio(p, v) * df + g * (r_ref - self._yaw_f)

        return command_from_axle_angles(p, df, dr, v_cmd)
