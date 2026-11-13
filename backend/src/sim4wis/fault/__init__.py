"""Shared simulation-time fault engine. Status reads never sample or advance."""

from __future__ import annotations

import hashlib
import math
import uuid
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Literal

import numpy as np

from sim4wis.fault.schema import (
    ACTUATOR_TYPES,
    PHYSICAL_TYPES,
    WHEELS,
    FaultSpec,
    validate_timeline,
)


class FaultType(str, Enum):
    MOTOR_STUCK_ZERO = "motor_stuck_zero"
    MOTOR_STUCK_ANGLE = "motor_stuck_angle"
    MOTOR_LIMITED_RANGE = "motor_limited_range"
    MOTOR_STUCK_HOLD = "motor_stuck_hold"
    MOTOR_FREE_CASTER = "motor_free_caster"
    SENSOR_BIAS = "sensor_bias"
    SENSOR_NOISE = "sensor_noise"
    SENSOR_DROPOUT = "sensor_dropout"


WheelName = Literal["fl", "fr", "rl", "rr"]
_LEGACY = {
    "stuck_zero": FaultType.MOTOR_STUCK_ZERO,
    "stuck_value": FaultType.MOTOR_STUCK_ANGLE,
    "limited": FaultType.MOTOR_LIMITED_RANGE,
    "stuck_hold": FaultType.MOTOR_STUCK_HOLD,
    "free_caster": FaultType.MOTOR_FREE_CASTER,
}


@dataclass
class FaultConfig:
    """Compatibility adapter for the pre-v0.105 live API."""

    fault_type: FaultType
    wheel: WheelName
    value: float = 0.0
    active: bool = True
    id: str = field(default_factory=lambda: uuid.uuid4().hex[:12])
    t_start: float = 0.0
    t_end: float | None = None
    eta_rev: float = 0.6
    j_steer: float = 3.0
    c_damp: float = 80.0
    tau_coulomb: float = 5.0

    def to_dict(self):
        return {**self.__dict__, "fault_type": self.fault_type.value}

    def spec(self):
        return FaultSpec(**self.to_dict())

    @classmethod
    def from_spec(cls, spec, index):
        data = spec.model_dump()
        data.update(
            fault_type=_LEGACY[spec.fault_type]
            if spec.fault_type in _LEGACY
            else FaultType(spec.fault_type),
            wheel=WHEELS[spec.wheel],
            id=spec.id or f"fault-{index + 1:03d}",
        )
        return cls(**data)


def passive_steering_step(angle, rate, torque, dt, *, inertia, damping, friction, limit):
    """J*(w1-w0)/dt = torque-c*w1-tau_c*Sign(w1); inelastic end stops.

    Sign(0) is set-valued: static friction holds below breakaway, with no creep.
    Backward Euler dissipates energy for positive damping and Coulomb friction.
    """
    momentum = inertia * rate + dt * torque
    rate = math.copysign(max(abs(momentum) - dt * friction, 0.0), momentum) / (
        inertia + dt * damping
    )
    angle += dt * rate
    if angle >= limit:
        angle, rate = limit, min(rate, 0.0)
    elif angle <= -limit:
        angle, rate = -limit, max(rate, 0.0)
    return float(angle), float(rate)


class FaultInjector:
    def __init__(self, faults=None, seed=0):
        self._faults = {}
        self.seed = seed
        self.reset_runtime()
        if faults:
            self.replace_schedule(faults, seed=seed)

    def reset_runtime(self, actual=None):
        self._time = 0.0
        self._enabled = set()
        self._hold = {}
        self._free = {}
        self._rng = {}
        self._last_reported = np.zeros(4) if actual is None else np.array(actual, dtype=float)
        self.events = []

    def specs(self):
        return [f.spec() for f in self._faults.values()]

    @staticmethod
    def validate_model(faults, model, model_type=None):
        from sim4wis.steering.architecture import CAP_PER_WHEEL, get

        p = model.params
        kinematic = model_type == "kinematic" or model.__class__.__name__ == "KinematicModel"
        for f in faults:
            if f.fault_type in PHYSICAL_TYPES and kinematic:
                raise ValueError(f"{f.fault_type} requires a force-based dynamic model")
            if f.fault_type in {"stuck_value", "limited"} and abs(f.value) > p.steer_limit:
                raise ValueError("fault angle/limit exceeds vehicle steer_limit")
            if f.fault_type in ACTUATOR_TYPES and p.steering_system.enabled:
                if not get(p.steering_system.architecture).can(CAP_PER_WHEEL):
                    raise ValueError(
                        "per-wheel actuator faults require independent 4wis steering; coupled racks are unsupported"
                    )
                if not p.steering_system.angle_control.enabled:
                    raise ValueError(
                        "enabled 4wis steering requires angle_control.enabled for independent corner faults"
                    )

    def replace_schedule(self, faults, *, seed=0, model=None, now=0.0):
        configs = [FaultConfig.from_spec(f, i) for i, f in enumerate(faults)]
        specs = [f.spec() for f in configs]
        validate_timeline(specs)
        if model is not None:
            self.validate_model(specs, model)
        history = list(self.events)
        for fid in sorted(self._enabled):
            history.append({"id": fid, "event": "removed", "time": now})
        self._faults = {f.id: f for f in configs}
        self.seed = seed
        self.reset_runtime(model.state.delta if model is not None else None)
        self.events = history
        self._time = now

    def add(self, cfg):
        if cfg.id in self._faults:
            raise ValueError("fault id already exists")
        validate_timeline([*self.specs(), cfg.spec()])
        self._faults[cfg.id] = cfg
        return cfg

    def remove(self, fault_id):
        found = self._faults.pop(fault_id, None)
        if fault_id in self._enabled:
            self.events.append({"id": fault_id, "event": "removed", "time": self._time})
        self._enabled.discard(fault_id)
        self._hold.pop(fault_id, None)
        self._free.pop(fault_id, None)
        return found is not None

    def clear(self):
        for fid in list(self._faults):
            self.remove(fid)

    def list_faults(self):
        return list(self._faults.values())

    def set_active(self, fault_id, active):
        if fault_id not in self._faults:
            return False
        self._faults[fault_id].active = active
        return True

    def _active(self, f, t=None):
        t = self._time if t is None else t
        return f.active and t >= f.t_start - 1e-10 and (f.t_end is None or t < f.t_end - 1e-10)

    def sync(self, t, model=None):
        self._time = float(t)
        current = {f.id for f in self._faults.values() if self._active(f)}
        for fid in sorted(self._enabled - current):
            self.events.append({"id": fid, "event": "deactivated", "time": self._time})
            self._hold.pop(fid, None)
            self._free.pop(fid, None)
        for fid in sorted(current - self._enabled):
            f = self._faults[fid].spec()
            self.events.append(
                {
                    "id": fid,
                    "event": "activated",
                    "time": self._time,
                    "scheduled_time": f.t_start,
                    "wheel": f.wheel,
                    "fault_type": f.fault_type,
                    "config": f.model_dump(mode="json"),
                }
            )
            if model is not None:
                angle = float(getattr(model, "_delta_act", model.state.delta)[f.wheel])
                rate = float(getattr(model, "_steer_rate", np.zeros(4))[f.wheel])
                self._hold[fid] = angle
                self._free[fid] = (angle, rate)
        self._enabled = current

    def next_boundary(self, start, end):
        boundaries = [
            t
            for f in self._faults.values()
            if f.active
            for t in (f.t_start, f.t_end)
            if t is not None and start + 1e-10 < t < end - 1e-10
        ]
        return min(boundaries, default=end)

    def apply_to_cmd(self, delta_cmd):
        result = np.array(delta_cmd, dtype=float)
        for cfg in self._faults.values():
            if not self._active(cfg):
                continue
            f = cfg.spec()
            if f.fault_type == "stuck_zero":
                result[f.wheel] = 0.0
            elif f.fault_type == "stuck_value":
                result[f.wheel] = f.value
            elif f.fault_type == "limited":
                result[f.wheel] = np.clip(result[f.wheel], -f.value, f.value)
        return result

    def command(self, cmd, model, dt):
        self.sync(model.state.t, model)
        out = replace(
            cmd,
            delta_cmd=self.apply_to_cmd(cmd.delta_cmd),
            steer_override=np.full(4, np.nan),
            steer_override_rate=np.zeros(4),
        )
        for fid in sorted(self._enabled):
            f = self._faults[fid].spec()
            w = f.wheel
            if f.fault_type == "stuck_hold":
                out.steer_override[w] = self._hold[fid]
            elif f.fault_type == "free_caster":
                angle, rate = self._free[fid]
                angle, rate = passive_steering_step(
                    angle,
                    rate,
                    -f.eta_rev * float(model.state.torque_steer[w]),
                    dt,
                    inertia=f.j_steer,
                    damping=f.c_damp,
                    friction=f.tau_coulomb,
                    limit=float(model.params.steer_limit),
                )
                self._free[fid] = (angle, rate)
                out.steer_override[w], out.steer_override_rate[w] = angle, rate
            if f.fault_type in PHYSICAL_TYPES:
                out.delta_cmd[w] = out.steer_override[w]
        return out

    def apply_to_reported(self, delta_actual):
        """Advance a sensor sample. Never call on a status/serialization path."""
        result = np.array(delta_actual, dtype=float)
        active = [cfg.spec() for cfg in self._faults.values() if self._active(cfg)]
        for f in sorted(active, key=lambda f: (f.wheel, f.fault_type, f.t_start)):
            if f.fault_type == "sensor_bias":
                result[f.wheel] += f.value
            elif f.fault_type == "sensor_noise":
                key = f"{f.wheel}:{f.fault_type}:{f.t_start}"
                if key not in self._rng:
                    digest = hashlib.sha256(f"{self.seed}:{key}".encode()).digest()
                    self._rng[key] = np.random.default_rng(int.from_bytes(digest[:8], "little"))
                result[f.wheel] += self._rng[key].normal(0.0, f.value)
        for f in active:
            if f.fault_type == "sensor_dropout":
                result[f.wheel] = self._last_reported[f.wheel]
        self._last_reported = result.copy()
        return result

    def sample(self, model):
        self.sync(model.state.t, model)
        model.state.delta_reported[:] = self.apply_to_reported(model.state.delta)
        model.state.fault_mode[:] = 0
        model.state.sensor_fault_active[:] = 0
        modes = {"stuck_zero": 1, "stuck_value": 1, "limited": 2, "stuck_hold": 3, "free_caster": 4}
        for fid in self._enabled:
            f = self._faults[fid].spec()
            if f.fault_type in modes:
                model.state.fault_mode[f.wheel] = modes[f.fault_type]
            else:
                model.state.sensor_fault_active[f.wheel] = 1

    def status(self):
        return {
            "clock": "simulation",
            "time": self._time,
            "seed": self.seed,
            "faults": [
                {
                    **f.to_dict(),
                    "effective_active": self._active(f),
                    "phase": "disabled"
                    if not f.active
                    else "scheduled"
                    if self._time < f.t_start - 1e-10
                    else "active"
                    if self._active(f)
                    else "completed",
                }
                for f in self._faults.values()
            ],
            "events": list(self.events),
        }

    @property
    def has_active(self):
        return any(self._active(f) for f in self._faults.values())
