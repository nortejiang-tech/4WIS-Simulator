"""Holonomic body-motion strategy — the manual gamepad "全向" mode.

A 4WIS chassis can independently command planar longitudinal, lateral and yaw
motion. This strategy exposes all three as direct driver intents (normalised
fractions in ``driver.mode_params``) and lets the unified ``compute_commands``
helper derive the four wheel angles + speeds that realise them:

    vx = vx_frac · v_max        (forward / reverse)
    vy = vy_frac · v_max        (crab / lateral)
    ω  = yaw_frac · ω_max       (spin about the body origin)

ω_max is the spin rate whose outermost wheel linear speed equals v_max (same
convention as the zero-radius strategy). Driving forward while crabbing and
yawing simultaneously is the signature holonomic move conventional cars cannot
perform — this mode makes it hand-drivable.
"""

from __future__ import annotations

import numpy as np

from sim4wis.controller.base import (
    BodyMotionTarget,
    ControllerStrategy,
    compute_commands,
)
from sim4wis.core.state import ControlCommand, DriverInput, VehicleParams, VehicleState


class ManualBodyStrategy(ControllerStrategy):
    name = "manual_body"

    def __init__(self, params: VehicleParams) -> None:
        super().__init__(params)
        wheels = params.wheel_positions_body()
        r_max = float(np.max(np.linalg.norm(wheels, axis=1)))
        self._omega_max = params.v_max / r_max if r_max > 1e-6 else 1.0

    @staticmethod
    def _frac(mp: dict, key: str) -> float:
        try:
            return float(np.clip(float(mp.get(key, 0.0)), -1.0, 1.0))
        except (TypeError, ValueError):
            return 0.0

    def compute(self, driver: DriverInput, state: VehicleState, dt: float = 0.0) -> ControlCommand:  # noqa: ARG002
        p = self.params
        mp = driver.mode_params or {}
        vx = self._frac(mp, "vx_frac") * p.v_max
        vy = self._frac(mp, "vy_frac") * p.v_max
        omega = self._frac(mp, "yaw_frac") * self._omega_max

        icr = (
            np.array([-vy / omega, vx / omega])
            if abs(omega) > 1e-6
            else np.array([np.nan, np.nan])
        )
        target = BodyMotionTarget(vx=vx, vy=vy, omega=omega, icr_target_body=icr)
        return compute_commands(
            wheels=p.wheel_positions_body(),
            target=target,
            tire_radius=p.tire_radius,
            steer_limit=p.steer_limit,
        )
