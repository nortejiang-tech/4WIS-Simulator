"""In-memory ring buffer for recording simulator state.

Designed so the simulator can call `recorder.write(state, cmd, strategy)`
from its hot loop without blocking — the buffer stores up to
`buffer_seconds × rate_hz` samples then drops oldest.

Channels are addressed by snake_case names with a unit suffix encoded in the
name where relevant. The list of *available* channels (`AVAILABLE_CHANNELS`)
is the single source of truth; the user (or project YAML) selects which
subset to record.

Phase 1 channels:
    pose_x, pose_y, pose_psi        位姿 [m, m, rad]
    vx, vy, yaw_rate                速度 [m/s, m/s, rad/s]
    delta_<wheel>                   实际转角 [rad]
    deltacmd_<wheel>                指令转角 [rad]
    omega_<wheel>                   车轮转速 [rad/s]
    fz_<wheel>                      垂直载荷 [N]
    torque_steer_<wheel>            转向阻力矩 [N·m]
    icr_vehicle_body_{x,y}          实际整车 ICR (车体系) [m]
    icr_target_body_{x,y}           策略目标 ICR (车体系) [m]

`<wheel>` ∈ {fl, fr, rl, rr}. NaN/Inf values are emitted as empty fields in
CSV (pandas reads "" as NaN by default).
"""

from __future__ import annotations

import math
from bisect import bisect_left, bisect_right
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from sim4wis.core.state import ControlCommand, VehicleState

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
}
for _i, _w in enumerate(_WHEEL_NAMES):
    AVAILABLE_CHANNELS[f"delta_{_w}"] = _wheel_extractor("delta", _i)
    AVAILABLE_CHANNELS[f"deltacmd_{_w}"] = _cmd_wheel_extractor("delta_cmd", _i)
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


# Default recording set — the channels we believe are useful for almost
# every analysis. Project YAML can override.
DEFAULT_CHANNELS: list[str] = [
    "pose_x", "pose_y", "pose_psi",
    "vx", "vy", "yaw_rate",
    "delta_fl", "delta_fr", "delta_rl", "delta_rr",
    "torque_steer_fl", "torque_steer_fr", "torque_steer_rl", "torque_steer_rr",
    "icr_dev_fl", "icr_dev_fr", "icr_dev_rl", "icr_dev_rr",
    "icr_vehicle_body_x", "icr_vehicle_body_y",
    "icr_target_body_x", "icr_target_body_y",
    "rack_force_fl", "rack_force_fr", "rack_force_rl", "rack_force_rr",
    "motor_torque_fl", "motor_torque_fr", "motor_torque_rl", "motor_torque_rr",
]


@dataclass
class RecorderConfig:
    buffer_seconds: float = 1800.0  # 30 min default
    rate_hz: float = 60.0
    channels: list[str] = field(default_factory=lambda: list(DEFAULT_CHANNELS))


class Recorder:
    """Append-only ring buffer with selectable channel set.

    Not thread-safe — assumes single-writer (the sim loop). Reader-side
    (CSV export) is fine to call from a different coroutine because we
    only read snapshots of the buffer length.
    """

    def __init__(self, cfg: RecorderConfig | None = None) -> None:
        self.cfg = cfg or RecorderConfig()
        self.max_samples = int(self.cfg.buffer_seconds * self.cfg.rate_hz) + 60
        self._recording = False
        # Filter requested channels down to those we know how to extract.
        self._channels: list[str] = [c for c in self.cfg.channels if c in AVAILABLE_CHANNELS]
        self._t: list[float] = []
        self._strategy: list[str] = []
        self._data: dict[str, list[float]] = {c: [] for c in self._channels}

    # ---- mutators ----------------------------------------------------------

    def start(self) -> None:
        self._t.clear()
        self._strategy.clear()
        for c in self._channels:
            self._data[c].clear()
        self._recording = True

    def stop(self) -> None:
        self._recording = False

    def set_channels(self, channels: list[str]) -> None:
        """Reset the buffer with a new channel selection."""
        was_recording = self._recording
        self._recording = False
        self._channels = [c for c in channels if c in AVAILABLE_CHANNELS]
        self._t.clear()
        self._strategy.clear()
        self._data = {c: [] for c in self._channels}
        if was_recording:
            self._recording = True

    def write(self, state: VehicleState, cmd: ControlCommand, strategy: str) -> None:
        if not self._recording:
            return
        self._t.append(float(state.t))
        self._strategy.append(strategy)
        for c in self._channels:
            self._data[c].append(AVAILABLE_CHANNELS[c](state, cmd))
        # Trim from the front if we exceeded buffer capacity.
        excess = len(self._t) - self.max_samples
        if excess > 0:
            del self._t[:excess]
            del self._strategy[:excess]
            for c in self._channels:
                del self._data[c][:excess]

    # ---- readers ------------------------------------------------------------

    @property
    def is_recording(self) -> bool:
        return self._recording

    def status(self) -> dict[str, Any]:
        return {
            "recording": self._recording,
            "samples": len(self._t),
            "from_t": (self._t[0] if self._t else None),
            "to_t": (self._t[-1] if self._t else None),
            "channels": list(self._channels),
            "buffer_seconds": self.cfg.buffer_seconds,
            "available_channels": list(AVAILABLE_CHANNELS.keys()),
        }

    def to_csv(
        self,
        channels: list[str] | None = None,
        from_t: float | None = None,
        to_t: float | None = None,
        include_strategy: bool = True,
    ) -> Iterator[str]:
        """Stream CSV rows. Yields lines including trailing newline."""
        if channels is None:
            chans = list(self._channels)
        else:
            chans = [c for c in channels if c in self._data]
        # Index range
        i0, i1 = 0, len(self._t)
        if from_t is not None:
            i0 = bisect_left(self._t, from_t)
        if to_t is not None:
            i1 = bisect_right(self._t, to_t)
        # Header
        cols = ["t", *chans] + (["strategy"] if include_strategy else [])
        yield ",".join(cols) + "\n"
        # Rows
        for i in range(i0, i1):
            parts = [_fmt(self._t[i])]
            for c in chans:
                parts.append(_fmt(self._data[c][i]))
            if include_strategy:
                parts.append(self._strategy[i])
            yield ",".join(parts) + "\n"


def _fmt(v: float) -> str:
    """Format a float for CSV — NaN/Inf become empty (pandas reads as NaN)."""
    if isinstance(v, str):
        return v
    try:
        f = float(v)
    except (TypeError, ValueError):
        return ""
    if not math.isfinite(f):
        return ""
    # 6 significant digits is enough for engineering analysis and keeps CSVs small.
    return f"{f:.6g}"
