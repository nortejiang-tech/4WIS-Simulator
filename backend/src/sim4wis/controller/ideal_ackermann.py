"""Ideal Ackermann strategy for 4WIS — all four wheels share the same ICR.

This is the 4WIS hero strategy. Given a desired curvature κ (signed,
κ > 0 = left turn), the ICR is placed on the body-Y axis at (0, 1/κ), so
the body origin has pure forward motion (vy_origin = 0) and every wheel's
rolling axis is perpendicular to its own radius to the ICR. Result:

    * Zero side-slip at every wheel (in kinematic terms).
    * No tyre-scrub during steady-state cornering.
    * The geometric "Ackermann condition" is satisfied exactly, not
      approximately, across all four wheels.

That last property is the testable promise the test_ideal_ackermann.py
checks via line intersection of the four wheel perpendiculars.

Steering input mapping:
    driver.steering ∈ [-1, +1] → κ ∈ [-κ_max, +κ_max], where κ_max
    corresponds to the inner wheel reaching `steer_limit` at the smallest
    achievable radius. Computed once at strategy init based on geometry.
"""

from __future__ import annotations

import numpy as np

from sim4wis.controller.base import (
    BodyMotionTarget,
    ControllerStrategy,
    compute_commands,
)
from sim4wis.core.state import ControlCommand, DriverInput, VehicleParams, VehicleState


def _max_curvature(params: VehicleParams) -> float:
    """Tightest κ before the inner wheel hits its steer limit.

    Inner front wheel position (left turn, so inner = FL at (+L/2, +tf/2))
    relative to ICR at (0, 1/κ):
        δ_inner = atan2(L/2, 1/κ - tf/2)
    Solve δ_inner = steer_limit for κ:
        tan(steer_limit) = (L/2) / (1/κ - tf/2)
        1/κ - tf/2 = (L/2) / tan(steer_limit)
        1/κ = (L/2) / tan(steer_limit) + tf/2
    """
    L = params.wheelbase
    tf = params.track_front
    inv_kappa_min = (L / 2.0) / np.tan(params.steer_limit) + tf / 2.0
    return 1.0 / inv_kappa_min


class IdealAckermannStrategy(ControllerStrategy):
    name = "ideal_ackermann"

    def __init__(self, params: VehicleParams) -> None:
        super().__init__(params)
        self._kappa_max = _max_curvature(params)

    def compute(self, driver: DriverInput, state: VehicleState) -> ControlCommand:  # noqa: ARG002
        p = self.params
        kappa = self._kappa_max * float(driver.steering)
        v_cmd = p.v_max * float(driver.throttle)

        if abs(kappa) < 1e-9:
            target = BodyMotionTarget(vx=v_cmd, vy=0.0, omega=0.0,
                                      icr_target_body=np.array([np.nan, np.nan]))
        else:
            y_R = 1.0 / kappa
            omega = v_cmd * kappa   # vx = v_cmd, vy = 0, ω = vx/y_R = vx·κ
            target = BodyMotionTarget(
                vx=v_cmd,
                vy=0.0,
                omega=omega,
                icr_target_body=np.array([0.0, y_R]),
            )

        return compute_commands(
            wheels=p.wheel_positions_body(),
            target=target,
            tire_radius=p.tire_radius,
            steer_limit=p.steer_limit,
            delta_lock=None,  # all 4 wheels free → all 4 align with ICR
        )
