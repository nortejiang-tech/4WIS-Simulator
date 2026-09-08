"""Shared numeric telemetry for manual recordings and isolated Agent sessions.

Names, wheel order and units are stable across execution hosts. Signals that
need a tyre/steering model remain NaN when absent; exporters encode them as
empty CSV fields or JSON null. This layer never computes surrogate forces.
"""
from __future__ import annotations

import math
from collections.abc import Callable

from sim4wis.core.state import ControlCommand, VehicleState
from sim4wis.vehicle.steering_link import STEERING_CHANNELS, idle_channels

# Type for a per-sample value extractor.
Extractor = Callable[[VehicleState, ControlCommand], float]


def _wheel_extractor(field_name: str, idx: int) -> Extractor:
    def get(s: VehicleState, _c: ControlCommand) -> float:
        return float(getattr(s, field_name)[idx])
    return get


def _cmd_wheel_extractor(field_name: str, idx: int) -> Extractor:
    def get(_s: VehicleState, c: ControlCommand) -> float:
        return float(getattr(c, field_name)[idx])
    return get


def _icr_extractor(source: str, axis: int) -> Extractor:
    def get(s: VehicleState, c: ControlCommand) -> float:
        v = s.vehicle_icr_body[axis] if source == "vehicle" else c.icr_target_body[axis]
        return float(v) if math.isfinite(v) else math.nan
    return get


_WHEEL_NAMES = ("fl", "fr", "rl", "rr")


AVAILABLE_CHANNELS: dict[str, Extractor] = {
    "pose_x":    lambda s, c: s.x,
    "pose_y":    lambda s, c: s.y,
    "pose_psi":  lambda s, c: s.psi,
    "vx":        lambda s, c: s.vx,
    "vy":        lambda s, c: s.vy,
    "yaw_rate":  lambda s, c: s.yaw_rate,
    "ax": lambda s, c: s.ax,
    "ay": lambda s, c: s.ay,
    "pose_z": lambda s, c: s.z,
    "roll": lambda s, c: s.roll,
    "pitch": lambda s, c: s.pitch,
    "grip_valid": lambda s, c: float(s.grip_valid),
}
DRIVER_CHANNELS = ("throttle", "brake", "steering", "gear", "handbrake")
for _name in DRIVER_CHANNELS:
    AVAILABLE_CHANNELS[f"driver_{_name}"] = lambda s, c: math.nan
for _i, _w in enumerate(_WHEEL_NAMES):
    AVAILABLE_CHANNELS[f"delta_{_w}"] = _wheel_extractor("delta", _i)
    AVAILABLE_CHANNELS[f"deltacmd_{_w}"] = _cmd_wheel_extractor("delta_cmd", _i)
    AVAILABLE_CHANNELS[f"delta_cmd_{_w}"] = _cmd_wheel_extractor("delta_cmd", _i)
    AVAILABLE_CHANNELS[f"omega_{_w}"] = _wheel_extractor("wheel_omega", _i)
    AVAILABLE_CHANNELS[f"fz_{_w}"] = _wheel_extractor("fz", _i)
    AVAILABLE_CHANNELS[f"torque_steer_{_w}"] = _wheel_extractor("torque_steer", _i)
    # Per-wheel steering centre: signed deviation from vehicle ICR [m] and
    # the projected point itself (body frame) — NaN when driving straight.
    AVAILABLE_CHANNELS[f"icr_dev_{_w}"] = _wheel_extractor("wheel_icr_dev", _i)
for _i, _w in enumerate(_WHEEL_NAMES):
    def _wheel_icr_axis(idx: int, axis: int) -> Extractor:
        def get(s: VehicleState, _c: ControlCommand) -> float:
            v = s.wheel_icr_body[idx, axis]
            return float(v) if math.isfinite(v) else math.nan
        return get
    AVAILABLE_CHANNELS[f"icr_x_{_w}"] = _wheel_icr_axis(_i, 0)
    AVAILABLE_CHANNELS[f"icr_y_{_w}"] = _wheel_icr_axis(_i, 1)
AVAILABLE_CHANNELS["icr_vehicle_body_x"] = _icr_extractor("vehicle", 0)
AVAILABLE_CHANNELS["icr_vehicle_body_y"] = _icr_extractor("vehicle", 1)
AVAILABLE_CHANNELS["icr_target_body_x"]  = _icr_extractor("target", 0)
AVAILABLE_CHANNELS["icr_target_body_y"]  = _icr_extractor("target", 1)
for _i, _w in enumerate(_WHEEL_NAMES):
    AVAILABLE_CHANNELS[f"rack_force_{_w}"]   = _wheel_extractor("rack_force", _i)
    AVAILABLE_CHANNELS[f"motor_torque_{_w}"] = _wheel_extractor("motor_torque_demand", _i)


# Additional published body/wheel state, including validity flags.
AVAILABLE_CHANNELS["mu_avg"] = lambda s, c: float(s.mu_avg)
_STATE_WHEEL_FIELDS = (
    "susp_defl", "wheel_locked", "grip_capacity", "grip_util", "grip_margin_lat",
    "grip_margin_long", "grip_alpha_peak", "grip_kappa_peak", "grip_beyond_peak_lat",
    "grip_beyond_peak_long", "linkage_arm_tie_angle", "linkage_tie_rack_angle", "linkage_efficiency",
)
for _i, _w in enumerate(_WHEEL_NAMES):
    for _field in _STATE_WHEEL_FIELDS:
        AVAILABLE_CHANNELS[f"{_field}_{_w}"] = _wheel_extractor(_field, _i)
    AVAILABLE_CHANNELS[f"drive_torque_cmd_{_w}"] = _cmd_wheel_extractor("drive_torque_cmd", _i)
    AVAILABLE_CHANNELS[f"brake_cmd_{_w}"] = _cmd_wheel_extractor("brake_cmd", _i)
    AVAILABLE_CHANNELS[f"wheel_speed_cmd_{_w}"] = _cmd_wheel_extractor("wheel_speed_cmd", _i)

_MODEL_FIELDS = ("slip_alpha", "slip_kappa", "tire_fx", "tire_fy")
MODEL_CHANNELS = {f"{field}_{wheel}": (field, i)
                  for field in _MODEL_FIELDS for i, wheel in enumerate(_WHEEL_NAMES)}
for _name in (*MODEL_CHANNELS, *STEERING_CHANNELS):
    AVAILABLE_CHANNELS[_name] = lambda s, c: math.nan
_IDLE_STEERING = idle_channels()


def sample_channels(state, cmd, driver=None, model=None, names=None) -> dict[str, float]:
    """Capture one accepted integration boundary, without decimation."""
    out = {}
    steering = (getattr(model, "steering_channels", None) or _IDLE_STEERING)
    for name in AVAILABLE_CHANNELS if names is None else names:
        if name.startswith("driver_"):
            value = getattr(driver, name.removeprefix("driver_"), math.nan)
        elif name in MODEL_CHANNELS:
            field, i = MODEL_CHANNELS[name]
            values = getattr(model, field, None)
            value = values[i] if values is not None else math.nan
        elif name in STEERING_CHANNELS:
            value = steering.get(name, math.nan)
        else:
            value = AVAILABLE_CHANNELS[name](state, cmd)
        out[name] = float(value)
    return out
