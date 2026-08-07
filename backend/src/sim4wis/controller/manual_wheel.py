"""Direct per-wheel steering strategy — the manual gamepad "direct" mode.

The four wheel angles are commanded directly (normalised [-1, 1] → ±steer_limit)
via ``driver.mode_params["wheel_norm"]``; the frontend fills that array from
whatever axis→wheel *grouping* the user picked (front/rear, left/right,
per-wheel, or crab). Because the grouping lives entirely in the frontend
binding table, one backend strategy covers every direct-steer layout.

Throttle drives all four wheel spins open-loop (same target-speed contract as
the other kinematic strategies). Commanding angles that do not share one ICR is
allowed and physically meaningful — the dynamic models then show the tyre scrub
the mismatch produces, which is exactly what a hands-on 4WIS feel test wants.
"""

from __future__ import annotations

import numpy as np

from sim4wis.controller.base import ControlCommand, ControllerStrategy
from sim4wis.controller.longitudinal import speed_command
from sim4wis.core.state import DriverInput, N_WHEELS, VehicleParams, VehicleState
from sim4wis.vehicle.geometry import vehicle_icr_from_velocity


class ManualWheelStrategy(ControllerStrategy):
    """Pass the driver's four normalised wheel angles straight to the model."""

    name = "manual_wheel"

    def __init__(self, params: VehicleParams) -> None:
        super().__init__(params)

    def compute(self, driver: DriverInput, state: VehicleState, dt: float = 0.0) -> ControlCommand:  # noqa: ARG002
        p = self.params
        mp = driver.mode_params or {}
        raw = mp.get("wheel_norm", [0.0, 0.0, 0.0, 0.0])
        try:
            norm = np.asarray(raw, dtype=np.float64).reshape(N_WHEELS)
        except (ValueError, TypeError):
            norm = np.zeros(N_WHEELS)
        delta = np.clip(norm, -1.0, 1.0) * p.steer_limit

        v = speed_command(p, driver, float(state.vx), dt, float(state.mu_avg))
        omega_wheel = np.full(N_WHEELS, v / max(p.tire_radius, 1e-6))
        return ControlCommand(
            delta_cmd=delta,
            wheel_speed_cmd=omega_wheel,
            icr_target_body=np.array([np.nan, np.nan]),
        )
