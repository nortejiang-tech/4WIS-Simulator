"""ControllerStrategy abstract base class + shared helpers.

A strategy maps `DriverInput` → `ControlCommand`. Conceptually:

    DriverInput (throttle, steering, mode_params)
        ↓  Strategy.compute(...)
    ControlCommand (δ_cmd[4], wheel_speed_cmd[4], icr_target)

The Phase-1 strategies are pure kinematic: they pick a target body motion
(vx, vy, ω) from (throttle, steering) and then derive per-wheel commands so
the wheels obey pure-rolling constraints exactly. The `compute_commands(...)`
helper does that derivation in one place so each strategy only has to
declare *what* body motion it intends and (optionally) which wheels are
locked to a fixed angle.

A `name` class attribute is used by the registry; subclasses must set it.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from sim4wis.core.state import (
    N_WHEELS,
    ControlCommand,
    DriverInput,
    VehicleParams,
    VehicleState,
    WheelIndex,
)
from sim4wis.vehicle.geometry import wrap_to_pmhalfpi


@dataclass
class BodyMotionTarget:
    """The (vx, vy, ω) the strategy wants the body origin to track.

    Plus an optional `icr_target_body` for visualisation. If `icr_target_body`
    is None, the strategy can compute it from (vx, vy, ω) via the geometry
    helper (or leave NaN for crab / straight-line cases).
    """

    vx: float
    vy: float
    omega: float
    icr_target_body: np.ndarray | None = None
    heading_at_rest: float = 0.0


def limit_target_to_grip(target: BodyMotionTarget, accel_limit: float | None,
                         cg_x: float = 0.0) -> BodyMotionTarget:
    """Limit a driver's steady-turn speed without changing its ICR/angles.

    Scaling the whole twist by q scales centripetal demand by q². Explicit
    experiment speed commands pass None so the requested test is not altered.
    This is a reference governor, not a stability controller or tyre model.
    """
    if accel_limit is None:
        return target
    demand = abs(target.omega) * np.hypot(target.vx, target.vy + cg_x * target.omega)
    if demand <= max(accel_limit, 0.0) or demand < 1e-12:
        return target
    scale = float(np.sqrt(max(accel_limit, 0.0) / demand))
    return BodyMotionTarget(target.vx * scale, target.vy * scale, target.omega * scale,
                            target.icr_target_body, target.heading_at_rest)


def compute_commands(
    wheels: np.ndarray,
    target: BodyMotionTarget,
    *,
    tire_radius: float,
    steer_limit: float,
    delta_lock: np.ndarray | None = None,
) -> ControlCommand:
    """Turn (vx, vy, ω, optional locked δ) into per-wheel (δ, ω_wheel) commands.

    Args:
        wheels: (4, 2) wheel positions in body frame.
        target: requested body motion at body origin.
        tire_radius: rolling radius [m].
        steer_limit: ±max wheel angle [rad].
        delta_lock: (4,) array, or None. NaN entries → strategy is free to
                    align the wheel with its velocity vector; finite entries
                    → that wheel is forced to that δ (e.g. rear wheels of
                    traditional Ackermann are locked to 0).

    Returns:
        ControlCommand with delta_cmd in [-steer_limit, +steer_limit] and
        wheel_speed_cmd in rad/s.
    """
    assert wheels.shape == (N_WHEELS, 2)
    vx, vy, omega = target.vx, target.vy, target.omega

    # Velocity at each wheel: v_origin + ω × r_wheel
    v_wheel_x = vx - omega * wheels[:, 1]
    v_wheel_y = vy + omega * wheels[:, 0]

    if delta_lock is None:
        delta_lock = np.full(N_WHEELS, np.nan)

    delta = np.zeros(N_WHEELS)
    for i in range(N_WHEELS):
        if np.isfinite(delta_lock[i]):
            delta[i] = float(delta_lock[i])
        else:
            if abs(v_wheel_x[i]) + abs(v_wheel_y[i]) < 1e-12:
                # Steering geometry exists at standstill too. Velocity atan2
                # alone loses it when throttle=0, preventing parking steer.
                icr = target.icr_target_body
                raw = (np.arctan2(wheels[i, 0] - icr[0], icr[1] - wheels[i, 1])
                       if icr is not None and np.all(np.isfinite(icr))
                       else target.heading_at_rest)
            else:
                raw = np.arctan2(v_wheel_y[i], v_wheel_x[i])
            delta[i] = float(wrap_to_pmhalfpi(raw))
    # Clip to physical steering limit.
    delta = np.clip(delta, -steer_limit, +steer_limit)

    # Wheel angular velocity = signed projection of v_wheel onto rolling dir.
    s = v_wheel_x * np.cos(delta) + v_wheel_y * np.sin(delta)
    wheel_omega = s / tire_radius

    icr = (
        target.icr_target_body
        if target.icr_target_body is not None
        else np.array([np.nan, np.nan])
    )

    return ControlCommand(
        delta_cmd=delta,
        wheel_speed_cmd=wheel_omega,
        icr_target_body=icr,
    )


class ControllerStrategy(ABC):
    """Abstract control strategy. Subclass + register via the strategy registry."""

    name: str = "base"

    def __init__(self, params: VehicleParams) -> None:
        self.params = params

    @abstractmethod
    def compute(self, driver: DriverInput, state: VehicleState, dt: float = 0.0) -> ControlCommand:
        ...


# Convenience aliases for readability in strategy code.
FL = int(WheelIndex.FL)
FR = int(WheelIndex.FR)
RL = int(WheelIndex.RL)
RR = int(WheelIndex.RR)
