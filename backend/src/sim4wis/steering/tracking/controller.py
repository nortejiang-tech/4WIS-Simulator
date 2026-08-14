"""The angle-tracking controller protocol and the open-loop reference.

Every control strategy — PID variants, LQR, MPC, SMC, ADRC, H∞ — implements
:class:`AngleTrackingController`. The protocol has one deliberate asymmetry:
a controller may either **set the angle directly** (``angle_set``) or
**command a torque** (``torque_cmd``), never both. Direct-angle controllers
bypass the actuator plant; that is how the open-loop reference reproduces
the legacy by-wire update bit-for-bit. Torque controllers drive
:class:`~sim4wis.steering.tracking.plant.CornerActuatorPlant`.

The controller also declares its own update rate. The legacy update runs
once per 5 ms outer frame (``inner_rate_hz = None``); feedback controllers
run at the layer's inner rate (default 2000 Hz) with the target interpolated
across the frame — the D5 lesson applied to the control layer: a command
staircase under-resolves the quantities the controller acts on.

Feedforward blocks (load/rack-force, velocity, friction compensation — the
"积木" from the requirement) are plain components a feedback controller
composes, not part of the protocol: they live in the controller that uses
them, or in ``sim4wis.steering.tracking.feedforward`` once they are shared.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, ClassVar

#: The layer's inner rate [Hz] for feedback controllers. Same 0.5 ms inner
#: step as the mechanical-column path (STEERING_INNER_DT): 10x the 5 ms outer
#: frame, small enough that a ~10 Hz actuator mode is resolved, and free —
#: the corner plant is a handful of scalars.
INNER_RATE_HZ = 2000.0


@dataclass
class TrackingOutput:
    """One controller step's decision. Exactly one branch must be set."""

    #: Direct angle command — the controller bypasses the plant (open_loop).
    angle_set: float | None = None
    #: Torque command in the wheel domain [N·m] — drives the plant.
    torque_cmd: float | None = None
    #: Scalar diagnostics for recording (deviation, rate_limited, …).
    diagnostics: dict[str, float] = field(default_factory=dict)


class AngleTrackingController(ABC):
    """One per-corner angle-tracking strategy.

    All quantities are in the **road-wheel domain**: angles and rates in
    rad / rad·s⁻¹, torques in N·m at the wheel. ``feedback_angle`` is the
    *sensor* reading (quantised, delayed); ``plant_angle`` is the actuator
    truth — controllers that must match legacy behaviour use the truth,
    feedback controllers use the sensor.
    """

    #: Registry name, also the ``controller:`` value in a spec.
    name: ClassVar[str] = "abstract"
    #: Own update rate [Hz]. None = once per outer frame (legacy semantics);
    #: a finite rate = stepped at that rate with the target interpolated.
    inner_rate_hz: ClassVar[float | None] = INNER_RATE_HZ

    def reset(self) -> None:
        """Clear integrators, filters, internal state. Optional override."""
        return None

    @abstractmethod
    def step(
        self,
        dt: float,
        *,
        target_angle: float,
        target_rate: float,
        feedback_angle: float,
        plant_angle: float,
        load_torque: float,
        speed_ms: float,
    ) -> TrackingOutput:
        """One step. See the class docstring for units and semantics."""


class OpenLoopController(AngleTrackingController):
    """The reference: the legacy by-wire update, reproduced bit-exactly.

    Legacy ``ByWirePlant`` reached the commanded angle through a first-order
    bandwidth limit (10 Hz) plus a rate limit (6 rad/s), once per 5 ms outer
    frame, using the plant truth rather than a sensor. This controller
    implements exactly that update — same formula, same order of operations —
    so a run with the layer enabled and the default controller is
    indistinguishable from one without it. It ignores the sensor reading and
    the load by construction: there is no feedback and no disturbance term in
    the behaviour it reproduces.
    """

    name: ClassVar[str] = "open_loop"
    inner_rate_hz: ClassVar[float | None] = None  # once per outer frame

    def __init__(self, bandwidth_hz: float = 10.0, rate_limit: float = 6.0,
                 **_: Any) -> None:
        self.bandwidth_hz = float(bandwidth_hz)
        self.rate_limit = float(rate_limit)

    def step(
        self,
        dt: float,
        *,
        target_angle: float,
        target_rate: float,
        feedback_angle: float,
        plant_angle: float,
        load_torque: float,
        speed_ms: float,
    ) -> TrackingOutput:
        tau = 1.0 / max(2.0 * math.pi * self.bandwidth_hz, 1e-6)
        alpha = 1.0 - math.exp(-dt / tau)
        target = plant_angle + (float(target_angle) - plant_angle) * alpha
        max_step = self.rate_limit * dt
        step_taken = target - plant_angle
        rate_limited = abs(step_taken) > max_step
        if rate_limited:
            step_taken = math.copysign(max_step, step_taken)
        angle = plant_angle + step_taken
        return TrackingOutput(
            angle_set=angle,
            diagnostics={
                "rate_limited": 1.0 if rate_limited else 0.0,
                "deviation": float(target_angle) - angle,
            },
        )


CONTROLLERS: dict[str, type[AngleTrackingController]] = {}


def register(cls: type[AngleTrackingController]) -> type[AngleTrackingController]:
    if cls.name in CONTROLLERS and CONTROLLERS[cls.name] is not cls:
        raise ValueError(f"controller name already registered: {cls.name}")
    CONTROLLERS[cls.name] = cls
    return cls


def make_controller(name: str, **kwargs: Any) -> AngleTrackingController:
    """Build a controller from the registry, passing the spec's kwargs."""
    cls = CONTROLLERS.get(name)
    if cls is None:
        raise ValueError(
            f"unknown angle-tracking controller {name!r}; known: {sorted(CONTROLLERS)}"
        )
    return cls(**kwargs)


register(OpenLoopController)
