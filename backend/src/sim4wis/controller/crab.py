"""Crab (sideways translation) strategy.

All four wheels turn to the same angle δ_crab; the body does not rotate
(ω = 0). The body origin moves in direction (cos δ_crab, sin δ_crab) in
body frame, i.e. it translates without changing yaw.

This is the 4WIS signature move that conventional cars cannot do — useful
for parallel parking, lateral docking, etc.

Steering input maps to δ_crab; throttle maps to the magnitude of translation.
Negative throttle → reverse-crab.
"""

from __future__ import annotations

import numpy as np

from sim4wis.controller.base import (
    BodyMotionTarget,
    ControllerStrategy,
    compute_commands,
)
from sim4wis.core.state import ControlCommand, DriverInput, VehicleState


class CrabStrategy(ControllerStrategy):
    name = "crab"

    def compute(self, driver: DriverInput, state: VehicleState) -> ControlCommand:  # noqa: ARG002
        p = self.params
        delta_crab = p.steer_limit * float(driver.steering)
        v_cmd = p.v_max * float(driver.throttle)

        # ω = 0 — ICR at infinity, perpendicular to translation direction.
        target = BodyMotionTarget(
            vx=v_cmd * np.cos(delta_crab),
            vy=v_cmd * np.sin(delta_crab),
            omega=0.0,
            icr_target_body=np.array([np.nan, np.nan]),
        )

        # Lock all 4 wheels to the same angle — guarantees perfect parallel
        # alignment (the unified helper would also produce identical angles
        # for ω=0, but locking is explicit and robust to v_cmd=0).
        return compute_commands(
            wheels=p.wheel_positions_body(),
            target=target,
            tire_radius=p.tire_radius,
            steer_limit=p.steer_limit,
            delta_lock=np.full(4, delta_crab),
        )
