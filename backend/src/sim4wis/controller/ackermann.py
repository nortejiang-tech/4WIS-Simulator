"""Traditional (front-axle-only) Ackermann strategy.

Steering input → bicycle-equivalent steer angle δ_bike at the front; rear
wheels are locked to δ=0. Per-wheel front angles fall out of the unified
`compute_commands(...)` helper, so the left/right differential is the true
Ackermann geometry (not a small-angle approximation).

ICR sits at (-L/2, L / tan δ_bike) — on the rear-axle extension — so the
body origin has a nonzero vy = ω · L / 2 in steady state. This is the
hallmark "rear-axle pivots" behaviour you also see in a real Ackermann car
(no slip at rear, mild side-slip at the geometric centre).
"""

from __future__ import annotations

import numpy as np

from sim4wis.controller.base import (
    N_WHEELS,
    BodyMotionTarget,
    ControllerStrategy,
    compute_commands,
    limit_target_to_grip,
)
from sim4wis.controller.longitudinal import cornering_accel_limit, speed_command
from sim4wis.core.state import ControlCommand, DriverInput, VehicleState


class AckermannStrategy(ControllerStrategy):
    name = "ackermann"

    def compute(self, driver: DriverInput, state: VehicleState, dt: float = 0.0) -> ControlCommand:  # noqa: ARG002
        p = self.params
        delta_bike = p.steer_limit * float(driver.steering)
        v_cmd = speed_command(p, driver, float(state.vx), dt, float(state.mu_avg))

        L = p.wheelbase
        if abs(delta_bike) < 1e-6:
            target = BodyMotionTarget(vx=v_cmd, vy=0.0, omega=0.0,
                                      icr_target_body=np.array([np.nan, np.nan]))
        else:
            y_R = L / np.tan(delta_bike)
            omega = v_cmd / y_R
            vy_origin = omega * L / 2.0
            target = BodyMotionTarget(
                vx=v_cmd,
                vy=vy_origin,
                omega=omega,
                icr_target_body=np.array([-L / 2.0, y_R]),
            )

        # Rear wheels are locked to 0 — this is what makes the strategy
        # "traditional" and creates a small slip residual if δ_bike is large.
        delta_lock = np.array([np.nan, np.nan, 0.0, 0.0])
        assert delta_lock.shape == (N_WHEELS,)

        target = limit_target_to_grip(target, cornering_accel_limit(p, driver, state),
                                      p.wheelbase / 2.0 - p.cg_to_front)
        return compute_commands(
            wheels=p.wheel_positions_body(),
            target=target,
            tire_radius=p.tire_radius,
            steer_limit=p.steer_limit,
            delta_lock=delta_lock,
        )
