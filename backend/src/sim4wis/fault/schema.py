"""Shared finite fault timeline: simulation seconds, half-open active intervals."""

from __future__ import annotations

import math
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

WHEELS = ("fl", "fr", "rl", "rr")
ALIASES = {
    "motor_stuck_zero": "stuck_zero",
    "motor_stuck_angle": "stuck_value",
    "motor_limited_range": "limited",
    "motor_stuck_hold": "stuck_hold",
    "motor_free_caster": "free_caster",
}
ACTUATOR_TYPES = frozenset({"stuck_zero", "stuck_value", "limited", "stuck_hold", "free_caster"})
PHYSICAL_TYPES = frozenset({"stuck_hold", "free_caster"})


class FaultSpec(BaseModel):
    """Target faults retain actuator dynamics; jam/free act on the mechanism.

    stuck_hold locks the actuator coordinate; alignment/K&C still apply.
    Sensor faults affect observations only, not controller feedback.
    """

    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    fault_type: Literal[
        "stuck_zero",
        "stuck_hold",
        "stuck_value",
        "limited",
        "free_caster",
        "sensor_bias",
        "sensor_noise",
        "sensor_dropout",
    ]
    wheel: int = Field(ge=0, le=3, strict=True)
    value: float = 0.0
    t_start: float = Field(0.0, ge=0.0)
    t_end: float | None = Field(None, ge=0.0)
    active: bool = True
    id: str | None = Field(None, min_length=1, max_length=64, pattern=r"^[A-Za-z0-9_.:-]+$")
    eta_rev: float = Field(0.6, ge=0.0, le=1.0)
    j_steer: float = Field(3.0, gt=0.0)
    c_damp: float = Field(80.0, ge=0.0)
    tau_coulomb: float = Field(5.0, ge=0.0)

    @field_validator("fault_type", mode="before")
    @classmethod
    def legacy_names(cls, value):
        return ALIASES.get(value, value)

    @field_validator("wheel", mode="before")
    @classmethod
    def wheel_names(cls, value):
        return (
            WHEELS.index(value.lower())
            if isinstance(value, str) and value.lower() in WHEELS
            else value
        )

    @model_validator(mode="after")
    def check_window_and_value(self):
        if self.t_end is not None and self.t_end <= self.t_start:
            raise ValueError("t_end must be greater than t_start")
        if self.fault_type in {"limited", "sensor_noise"} and self.value < 0:
            raise ValueError("range limit and noise standard deviation must be nonnegative")
        if self.fault_type in {"stuck_value", "limited"} and abs(self.value) > math.pi / 2:
            raise ValueError("actuator target/limit must be within pi/2 radians")
        return self


def validate_timeline(faults: list[FaultSpec]) -> None:
    if len(faults) > 64:
        raise ValueError("at most 64 faults per timeline")
    ids = [f.id for f in faults if f.id is not None]
    if len(set(ids)) != len(ids):
        raise ValueError("fault ids must be unique")
    for i, a in enumerate(faults):
        for b in faults[i + 1 :]:
            if a.wheel != b.wheel:
                continue
            overlap = max(a.t_start, b.t_start) < min(
                a.t_end if a.t_end is not None else math.inf,
                b.t_end if b.t_end is not None else math.inf,
            )
            conflict = (
                a.fault_type in ACTUATOR_TYPES
                and b.fault_type in ACTUATOR_TYPES
                or a.fault_type == b.fault_type
            )
            if overlap and conflict:
                raise ValueError(f"overlapping {a.fault_type}/{b.fault_type} on {WHEELS[a.wheel]}")


class FaultSchedule(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    faults: list[FaultSpec] = Field(default_factory=list, max_length=64)
    seed: int = Field(0, ge=0, le=2**32 - 1, strict=True)

    @model_validator(mode="after")
    def check(self):
        validate_timeline(self.faults)
        return self
