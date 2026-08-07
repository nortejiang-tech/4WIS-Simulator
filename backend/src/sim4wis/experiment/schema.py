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
"""

from __future__ import annotations

import math
from typing import Any, Literal

from pydantic import BaseModel, Field, model_validator

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
    name: str = "untitled"
    steps: list[ManeuverStep] = Field(default_factory=list)

    @property
    def total_duration(self) -> float:
        return sum(s.duration for s in self.steps)


class ExperimentVehicle(BaseModel):
    profile: str | None = None                       # vehicle_profiles/<name>.yaml
    overrides: dict[str, Any] = Field(default_factory=dict)


class FaultSpec(BaseModel):
    """Time-triggered actuator fault for headless runs (ISO 26262 studies).

    Applied to the steer command of one wheel from ``t_start`` onward; the
    steering-actuator model (first-order lag + rate limit) still governs how
    fast the physical wheel reaches the faulted angle — i.e. this models the
    *command/mechanism* failure, not a teleporting wheel.

        stuck_zero   δ_cmd[i] ≡ 0            actuator returns to centre and jams
        stuck_hold   δ_cmd[i] ≡ δ(t_start)   self-locking mechanism jams in place
        stuck_value  δ_cmd[i] ≡ value        runaway-then-jam at a given angle
        limited      |δ_cmd[i]| ≤ value      restricted authority (degraded)
        free_caster  actuator de-energised on a NON-self-locking mechanism —
                     the wheel becomes a dynamic castering DOF driven by the
                     tyre kingpin torque through the back-drive path:

                         J·δ̈ = −η_rev·τ_kingpin − c·δ̇ − τ_c·sign(δ̇)

                     (τ_kingpin is the platform's per-wheel Reimpell moment;
                     the session integrates this ODE and feeds δ through the
                     actuator lag, which adds mechanism-lag damping.)

    Mechanism parameters (free_caster only): ``eta_rev`` back-drive
    efficiency, ``j_steer`` steering-system inertia about the kingpin
    [kg·m²] (wheel assembly + reflected rack/motor), ``c_damp`` viscous
    damping [N·m·s/rad], ``tau_coulomb`` breakaway friction [N·m].
    """

    fault_type: Literal["stuck_zero", "stuck_hold", "stuck_value", "limited", "free_caster"]
    wheel: int = Field(..., ge=0, le=3)              # 0=FL 1=FR 2=RL 3=RR
    value: float = 0.0                               # [rad] meaning per type
    t_start: float = Field(0.0, ge=0.0)              # sim time [s]
    eta_rev: float = Field(0.6, ge=0.0, le=1.0)      # back-drive efficiency
    j_steer: float = Field(3.0, gt=0.0)              # [kg·m²]
    c_damp: float = Field(80.0, ge=0.0)              # [N·m·s/rad]
    tau_coulomb: float = Field(5.0, ge=0.0)          # [N·m]


class PathSpec(BaseModel):
    """Reference path for follow_trajectory: template or explicit waypoints."""

    template: str | None = None                      # controller.path template name
    params: dict[str, Any] = Field(default_factory=dict)
    waypoints: list[list[float]] | None = None       # world-frame [x, y]
    closed: bool = False


class Experiment(BaseModel):
    name: str = Field(..., min_length=1, max_length=64)
    description: str = ""
    vehicle: ExperimentVehicle = Field(default_factory=ExperimentVehicle)
    model_type: str = "simplified_dynamic"
    strategy: str = "ideal_ackermann"
    mode_params: dict[str, Any] = Field(default_factory=dict)
    scene: dict[str, Any] | None = None              # SceneSection payload
    path: PathSpec | None = None
    faults: list[FaultSpec] = Field(default_factory=list)
    maneuver: Maneuver = Field(default_factory=Maneuver)
    dt: float = Field(0.005, gt=0.0005, le=0.05)
    record_hz: float = Field(50.0, gt=1.0, le=200.0)
    kpis: list[str] = Field(default_factory=list)    # empty = full default set
