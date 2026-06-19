"""VehicleModel abstract base class.

Phase 1: KinematicModel.
Phase 2: SimplifiedDynamicModel (tyre side-slip, vertical load transfer, body roll).
Phase 3: MultiBodyModel — extension point only, no implementation yet.

All concrete models must:
    * Hold the static parameters (VehicleParams) immutably.
    * Implement step(dt, cmd, env) which mutates `state` in place and
      returns it.
    * Implement reset(init_state) which puts the model back to a known state.

This is intentionally minimal — Phase 1's only real consumer is the
simulation loop in `core.loop`, which calls step() at a fixed dt.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from sim4wis.core.state import (
    ControlCommand,
    EnvironmentState,
    VehicleParams,
    VehicleState,
)


class VehicleModel(ABC):
    """Abstract vehicle dynamics/kinematics model."""

    def __init__(self, params: VehicleParams) -> None:
        self.params = params
        self.state = VehicleState()
        self.state.wheel_pos_body = params.wheel_positions_body()

    @abstractmethod
    def step(
        self,
        dt: float,
        cmd: ControlCommand,
        env: EnvironmentState,
    ) -> VehicleState:
        """Advance the model by one time step.

        Args:
            dt:  integration step in seconds.
            cmd: control command computed by the active strategy this tick.
            env: environment context (friction, road profile, …).

        Returns:
            The new (mutated) VehicleState — same object as self.state.
        """

    def reset(self, init: VehicleState | None = None) -> None:
        """Reset model state. If `init` is None, returns to zero-pose, zero-velocity."""
        if init is None:
            init = VehicleState()
        init.wheel_pos_body = self.params.wheel_positions_body()
        self.state = init
