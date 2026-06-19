"""Browser JS strategy pass-through.

The computation runs in the browser (inside JsStrategyPanel.tsx). The frontend
evaluates the user's JS function on every incoming state update, then sends the
result back via WebSocket as {type: "steer_cmd", fl, fr, rl, rr} [rad].

This strategy simply holds the last set of commanded angles received via that
message and feeds them to the model, acting as a transparent pass-through.
The wheel-speed command uses the same throttle-proportional approach as the
other kinematic strategies.
"""

from __future__ import annotations

import numpy as np

from sim4wis.controller.base import ControlCommand, ControllerStrategy
from sim4wis.core.state import DriverInput, N_WHEELS, VehicleParams, VehicleState


class UserJsStrategy(ControllerStrategy):
    """Pass-through strategy for browser-side JS sandbox."""

    name = "user_js"

    def __init__(self, params: VehicleParams) -> None:
        super().__init__(params)
        self._delta_cmd: np.ndarray = np.zeros(N_WHEELS)

    def set_steer_cmd(self, fl: float, fr: float, rl: float, rr: float) -> None:
        """Called from the WebSocket handler when a steer_cmd message arrives."""
        raw = np.array([fl, fr, rl, rr], dtype=np.float64)
        self._delta_cmd = np.clip(raw, -self.params.steer_limit, self.params.steer_limit)

    def compute(self, driver: DriverInput, state: VehicleState) -> ControlCommand:
        v = driver.throttle * self.params.v_max
        omega_wheel = np.full(N_WHEELS, v / max(self.params.tire_radius, 1e-6))
        return ControlCommand(
            delta_cmd=self._delta_cmd.copy(),
            wheel_speed_cmd=omega_wheel,
            icr_target_body=np.array([np.nan, np.nan]),
        )
