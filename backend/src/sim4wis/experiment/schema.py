"""Experiment schema — the reproducible unit of simulation work.

Design notes:
    * The maneuver is driven by **simulation time**, not wall clock, so a run
      is bit-reproducible and can execute as fast as the CPU allows.
    * Steering values are in the normalised driver range [-1, 1] (the active
      strategy maps them to wheel angles), matching the interactive workbench
      so an experiment reproduces exactly what a driver input would do.
    * Speed targets are km/h because the models interpret throttle as a
      target-speed fraction (throttle = speed / v_max).
    * `vehicle.profile` names a YAML in vehicle_profiles/; `vehicle.overrides`
      is a params_codec dict fragment applied on top (either may be omitted).
    * **Unknown fields are refused.** They used to be dropped in silence, which
      meant a misspelling ran successfully and produced a result that answered
      a different question: `frequency: 0.2` on a sine profile is not
      `freq_hz`, so the run executed at the 0.5 Hz default and the weave
      analysis downstream was measuring something nobody had asked for. The
      study layer already refuses a typo'd `bind` path for exactly this reason;
      the schema underneath it should not be the soft spot.
"""

from __future__ import annotations

import math
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from sim4wis.fault.schema import FaultSpec, validate_timeline

SteerKind = Literal["constant", "step", "ramp", "sine", "sweep", "dlc"]
# Unit of the steer `amplitude`:
#   "normalized" — driver-style [-1, 1] passed through the strategy's feel layer
#   "front_deg"  — front-wheel angle in degrees, BYPASSING the feel layer so a
#                  validation experiment measures the *vehicle*, not the driver
#                  input mapping. Work-package B3 decoupling.
SteerUnit = Literal["normalized", "front_deg"]


class SteerProfile(BaseModel):
    """Steering input as a function of time within one step.

    `unit` selects how `amplitude` is interpreted:
      * "normalized" (default, legacy): [-1, 1], goes through the strategy's
        steering-feel mapping (variable gear ratio, soft limit).
      * "front_deg": front-wheel angle in degrees. The experiment applies this
        directly to delta_cmd, bypassing the feel layer — so a regression
        baseline survives future tuning of the driver-input mapping.
    """
    model_config = ConfigDict(extra="forbid")

    kind: SteerKind = "constant"
    # The bound depends on the unit — see `_check_amplitude_unit`. The field
    # bound here is only the widest of the two; a single ±360 bound would let a
    # "normalized" profile carry 50 and silently command permanent full lock.
    amplitude: float = Field(0.0, ge=-90.0, le=90.0)
    unit: SteerUnit = "normalized"
    freq_hz: float = Field(0.5, gt=0.0, le=10.0)     # sine
    f0_hz: float = Field(0.1, gt=0.0, le=10.0)       # sweep start frequency
    f1_hz: float = Field(2.0, gt=0.0, le=10.0)       # sweep end frequency
    t_step: float = Field(0.5, ge=0.0)               # step lead-in [s]
    start: float = Field(0.0, ge=-90.0, le=90.0)     # ramp start value

    @model_validator(mode="after")
    def _check_amplitude_unit(self) -> SteerProfile:
        """Bound `amplitude`/`start` against the declared unit."""
        limit = 1.0 if self.unit == "normalized" else 90.0
        for name in ("amplitude", "start"):
            v = getattr(self, name)
            if abs(v) > limit:
                raise ValueError(
                    f"{name}={v} is out of range for unit '{self.unit}' "
                    f"(|{name}| <= {limit})"
                )
        return self

    def value(self, t: float, duration: float) -> float:
        """Evaluate the profile at sim-time `t` seconds into the step."""
        if self.kind == "constant":
            return self.amplitude
        if self.kind == "step":
            return 0.0 if t < self.t_step else self.amplitude
        if self.kind == "ramp":
            u = min(max(t / max(duration, 1e-9), 0.0), 1.0)
            return self.start + (self.amplitude - self.start) * u
        if self.kind == "sine":
            return self.amplitude * math.sin(2.0 * math.pi * self.freq_hz * t)
        if self.kind == "sweep":
            # Linear chirp f0→f1 over the step duration.
            k = (self.f1_hz - self.f0_hz) / max(duration, 1e-9)
            phase = 2.0 * math.pi * (self.f0_hz * t + 0.5 * k * t * t)
            return self.amplitude * math.sin(phase)
        if self.kind == "dlc":
            # ISO-3888-style open-loop double bump: left then right.
            u = t / max(duration, 1e-9)
            return self.amplitude * (_bump(u, 0.12, 0.42) - _bump(u, 0.5, 0.8))
        return 0.0


def _bump(u: float, lo: float, hi: float) -> float:
    """Raised-sine window in [lo, hi] → 0..1."""
    if u <= lo or u >= hi:
        return 0.0
    x = (u - lo) / (hi - lo)
    return math.sin(math.pi * x) ** 2


class ManeuverStep(BaseModel):
    """One segment of a maneuver, ending after `duration` sim-seconds."""
    model_config = ConfigDict(extra="forbid")

    name: str = ""
    duration: float = Field(..., gt=0.0, le=600.0)
    steer: SteerProfile = Field(default_factory=SteerProfile)
    speed_kmh: float | None = Field(None, ge=0.0)   # None = keep previous target
    # Ramp the speed target from the previous value over this many seconds
    # (0 = instant). A step in target speed on a heavy EV is a drag-strip
    # launch — the wheel servos torque-saturate and wind up; real test
    # procedures always ramp in. This is the minimal IPGDriver-style shaping.
    speed_ramp_s: float = Field(0.0, ge=0.0, le=60.0)
    mode_params: dict[str, Any] = Field(default_factory=dict)


class Maneuver(BaseModel):
    model_config = ConfigDict(extra="forbid")
    name: str = "untitled"
    steps: list[ManeuverStep] = Field(default_factory=list)

    @property
    def total_duration(self) -> float:
        return sum(s.duration for s in self.steps)


class ExperimentVehicle(BaseModel):
    model_config = ConfigDict(extra="forbid")
    profile: str | None = None                       # vehicle_profiles/<name>.yaml
    overrides: dict[str, Any] = Field(default_factory=dict)


class PathSpec(BaseModel):
    """Reference path for follow_trajectory: template or explicit waypoints."""
    model_config = ConfigDict(extra="forbid")

    template: str | None = None                      # controller.path template name
    params: dict[str, Any] = Field(default_factory=dict)
    waypoints: list[list[float]] | None = None       # world-frame [x, y]
    closed: bool = False


class Experiment(BaseModel):
    model_config = ConfigDict(extra="forbid")
    name: str = Field(..., min_length=1, max_length=64)
    description: str = ""
    vehicle: ExperimentVehicle = Field(default_factory=ExperimentVehicle)
    model_type: str = "simplified_dynamic"
    strategy: str = "ideal_ackermann"
    mode_params: dict[str, Any] = Field(default_factory=dict)
    scene: dict[str, Any] | None = None              # SceneSection payload
    path: PathSpec | None = None
    faults: list[FaultSpec] = Field(default_factory=list, max_length=64)
    fault_seed: int = Field(0, ge=0, le=2**32 - 1, strict=True)

    @model_validator(mode="after")
    def check_faults(self):
        validate_timeline(self.faults)
        return self
    maneuver: Maneuver = Field(default_factory=Maneuver)
    dt: float = Field(0.005, gt=0.0005, le=0.05)
    record_hz: float = Field(50.0, gt=1.0, le=200.0)
    kpis: list[str] = Field(default_factory=list)    # empty = full default set
