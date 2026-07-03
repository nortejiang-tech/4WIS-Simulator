"""Strategy registry — single source of truth for available controllers.

The Phase-1 strategies are pre-registered. Phase 2+ will add a plugin
loader that scans `plugins/` for `ControllerStrategy` subclasses (including
FMU/MATLAB-Engine adapters) and registers them here.
"""

from __future__ import annotations

from typing import Callable

from sim4wis.controller.ackermann import AckermannStrategy
from sim4wis.controller.base import ControllerStrategy
from sim4wis.controller.crab import CrabStrategy
from sim4wis.controller.fault_reconfig import FaultReconfigStrategy
from sim4wis.controller.follow_trajectory import FollowTrajectoryStrategy
from sim4wis.controller.ideal_ackermann import IdealAckermannStrategy
from sim4wis.controller.manual_body import ManualBodyStrategy
from sim4wis.controller.manual_wheel import ManualWheelStrategy
from sim4wis.controller.rear_wheel_steer import RearWheelSteerStrategy
from sim4wis.controller.user_js import UserJsStrategy
from sim4wis.controller.user_python import HotReloadStrategy
from sim4wis.controller.zero_radius import ZeroRadiusStrategy
from sim4wis.core.state import VehicleParams

# Map name → factory(params) → ControllerStrategy
_BUILTIN: dict[str, Callable[[VehicleParams], ControllerStrategy]] = {
    AckermannStrategy.name: AckermannStrategy,
    IdealAckermannStrategy.name: IdealAckermannStrategy,
    RearWheelSteerStrategy.name: RearWheelSteerStrategy,
    CrabStrategy.name: CrabStrategy,
    ZeroRadiusStrategy.name: ZeroRadiusStrategy,
    FollowTrajectoryStrategy.name: FollowTrajectoryStrategy,
    FaultReconfigStrategy.name: FaultReconfigStrategy,
    ManualWheelStrategy.name: ManualWheelStrategy,
    ManualBodyStrategy.name: ManualBodyStrategy,
    HotReloadStrategy.name: HotReloadStrategy,
    UserJsStrategy.name: UserJsStrategy,
}


def available_strategies() -> list[str]:
    return list(_BUILTIN.keys())


def make_strategy(name: str, params: VehicleParams) -> ControllerStrategy:
    if name not in _BUILTIN:
        raise KeyError(
            f"Unknown strategy {name!r}. Available: {', '.join(_BUILTIN.keys())}"
        )
    return _BUILTIN[name](params)
