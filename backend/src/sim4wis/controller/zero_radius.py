"""Zero-radius (spin in place) strategy.

ICR is at the vehicle geometric centre (body origin). Each wheel's perpendicular
line is forced to pass through the origin, so the four wheels point
"tangentially around the centre".

Geometry (no clipping needed since wheels are at L/2 and track/2 from origin):
    For wheel at (x, y):
        δ_i = atan2(x, -y)  (raw, then wrapped to [-π/2, π/2])
    Forward-FL = (+L/2, +tf/2) →  atan2(L/2, -tf/2) ≈ +119°  → wrapped: ≈ -61°
        i.e. the wheel points "down-right" and rolls *backward* during a
        left spin.

Throttle controls the spin rate. Steering input controls the spin direction:
    steering > 0 → CCW (left spin, positive yaw)
    steering < 0 → CW (right spin, negative yaw)
"""

from __future__ import annotations

import numpy as np

from sim4wis.controller.base import (
    BodyMotionTarget,
    ControllerStrategy,
    compute_commands,
)
from sim4wis.core.state import ControlCommand, DriverInput, VehicleState


class ZeroRadiusStrategy(ControllerStrategy):
    name = "zero_radius"

    def compute(self, driver: DriverInput, state: VehicleState) -> ControlCommand:  # noqa: ARG002
        p = self.params

        # Yaw-rate magnitude: scale so the outer wheel's linear speed equals v_max
        # at full throttle. Outer wheel is at distance R_outer from origin.
        wheels = p.wheel_positions_body()
        R_max = float(np.max(np.linalg.norm(wheels, axis=1)))
        omega_max = p.v_max / R_max if R_max > 1e-6 else 1.0
        omega = omega_max * float(driver.throttle) * float(np.sign(driver.steering) or 1.0)

        target = BodyMotionTarget(
            vx=0.0,
            vy=0.0,
            omega=omega,
            icr_target_body=np.array([0.0, 0.0]),
        )

        return compute_commands(
            wheels=wheels,
            target=target,
            tire_radius=p.tire_radius,
            steer_limit=p.steer_limit,
            delta_lock=None,
        )
