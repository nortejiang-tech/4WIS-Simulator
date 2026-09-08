"""Zero-radius (spin in place) strategy.

ICR is at the vehicle geometric centre (body origin). Each wheel's perpendicular
line is forced to pass through the origin, so the four wheels point
"tangentially around the centre".

Geometry (pure rolling requires steer_limit >= atan(L/min(track))):
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
    limit_target_to_grip,
)
from sim4wis.controller.longitudinal import cornering_accel_limit
from sim4wis.core.state import ControlCommand, DriverInput, VehicleState


class ZeroRadiusStrategy(ControllerStrategy):
    name = "zero_radius"

    def compute(self, driver: DriverInput, state: VehicleState, dt: float = 0.0) -> ControlCommand:  # noqa: ARG002
        p = self.params

        # A manoeuvre rate is independent of the vehicle's highway top speed.
        # 30 deg/s is a configurable interactive command ceiling, not a tyre
        # capability prediction. Explicit tests may choose another ceiling.
        wheels = p.wheel_positions_body()
        R_max = float(np.max(np.linalg.norm(wheels, axis=1)))
        rate = float((driver.mode_params or {}).get("spin_rate_max_dps", 30.0))
        if not np.isfinite(rate):
            rate = 30.0
        omega_max = min(float(np.deg2rad(np.clip(rate, 0.0, 180.0))),
                        p.v_max / max(R_max, 1e-6))
        omega = omega_max * float(driver.throttle) * float(np.sign(driver.steering) or 1.0)

        target = BodyMotionTarget(
            vx=0.0,
            vy=0.0,
            omega=omega,
            icr_target_body=np.array([0.0, 0.0]),
        )

        target = limit_target_to_grip(target, cornering_accel_limit(p, driver, state),
                                      p.wheelbase / 2.0 - p.cg_to_front)
        return compute_commands(
            wheels=wheels,
            target=target,
            tire_radius=p.tire_radius,
            steer_limit=p.steer_limit,
            delta_lock=None,
        )
