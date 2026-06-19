"""Fault injection framework for ISO 26262 / functional safety analysis.

Faults are applied at two points in the simulation loop:
  * apply_to_cmd   — actuator faults: modify delta_cmd *before* the steering
                     actuator model, so the physical wheel deviates from intent.
  * apply_to_reported — sensor faults: modify the delta values reported in the
                        WS state message, simulating what a faulty sensor would
                        tell an on-board diagnostic system. The actual wheel
                        angle (s.delta) is always physical truth.

Supported fault types:
  motor_stuck_zero    — wheel frozen at 0°
  motor_stuck_angle   — wheel frozen at a fixed angle [rad]
  motor_limited_range — commanded angle clamped to ±value [rad]
  sensor_bias         — constant offset added to reported angle [rad]
  sensor_noise        — zero-mean Gaussian noise (std = value [rad])
  sensor_dropout      — reported angle frozen at last known value
"""

from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Literal

import numpy as np


class FaultType(str, Enum):
    MOTOR_STUCK_ZERO    = "motor_stuck_zero"
    MOTOR_STUCK_ANGLE   = "motor_stuck_angle"
    MOTOR_LIMITED_RANGE = "motor_limited_range"
    SENSOR_BIAS         = "sensor_bias"
    SENSOR_NOISE        = "sensor_noise"
    SENSOR_DROPOUT      = "sensor_dropout"


WheelName = Literal["fl", "fr", "rl", "rr"]
_WHEEL_IDX: dict[str, int] = {"fl": 0, "fr": 1, "rl": 2, "rr": 3}


@dataclass
class FaultConfig:
    fault_type: FaultType
    wheel: WheelName
    value: float = 0.0   # meaning depends on type (angle [rad], std [rad], etc.)
    active: bool = True
    id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "fault_type": self.fault_type.value,
            "wheel": self.wheel,
            "value": self.value,
            "active": self.active,
        }


class FaultInjector:
    """Stateful fault injector — shared with the Simulator instance."""

    def __init__(self) -> None:
        self._faults: dict[str, FaultConfig] = {}
        self._last_reported: np.ndarray = np.zeros(4)

    # ---- management ---------------------------------------------------------

    def add(self, cfg: FaultConfig) -> FaultConfig:
        self._faults[cfg.id] = cfg
        return cfg

    def remove(self, fault_id: str) -> bool:
        return self._faults.pop(fault_id, None) is not None

    def clear(self) -> None:
        self._faults.clear()

    def list_faults(self) -> list[FaultConfig]:
        return list(self._faults.values())

    def set_active(self, fault_id: str, active: bool) -> bool:
        if fault_id not in self._faults:
            return False
        self._faults[fault_id].active = active
        return True

    # ---- injection hooks -----------------------------------------------------

    def apply_to_cmd(self, delta_cmd: np.ndarray) -> np.ndarray:
        """Return a (possibly modified) copy of delta_cmd for actuator faults."""
        result = np.array(delta_cmd, dtype=np.float64)
        for f in self._faults.values():
            if not f.active:
                continue
            i = _WHEEL_IDX[f.wheel]
            if f.fault_type == FaultType.MOTOR_STUCK_ZERO:
                result[i] = 0.0
            elif f.fault_type == FaultType.MOTOR_STUCK_ANGLE:
                result[i] = float(f.value)
            elif f.fault_type == FaultType.MOTOR_LIMITED_RANGE:
                result[i] = float(np.clip(result[i], -abs(f.value), abs(f.value)))
        return result

    def apply_to_reported(self, delta_actual: np.ndarray) -> np.ndarray:
        """Return the angle values a (possibly faulty) sensor would report."""
        result = np.array(delta_actual, dtype=np.float64)
        for f in self._faults.values():
            if not f.active:
                continue
            i = _WHEEL_IDX[f.wheel]
            if f.fault_type == FaultType.SENSOR_BIAS:
                result[i] = delta_actual[i] + float(f.value)
            elif f.fault_type == FaultType.SENSOR_NOISE:
                result[i] = delta_actual[i] + float(np.random.normal(0.0, abs(f.value)))
            elif f.fault_type == FaultType.SENSOR_DROPOUT:
                result[i] = float(self._last_reported[i])
        self._last_reported = result.copy()
        return result

    @property
    def has_active(self) -> bool:
        return any(f.active for f in self._faults.values())
