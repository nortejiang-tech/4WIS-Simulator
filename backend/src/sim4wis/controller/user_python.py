"""Hot-reload Python strategy plugin.

Watches `plugins/strategies/user_strategy.py` (via mtime polling). On save,
reloads the module and grabs the `compute(driver, state) -> dict` function.
Falls back to all-zero steering on file-not-found or import error; the error
message is surfaced via `/api/user_python/status`.

The `state` dict passed to user code includes:
  t, x, y, psi, vx, vy, yaw_rate,
  delta [4], fz [4], torque_steer [4],
  steer_limit, wheelbase, track_front, track_rear
"""

from __future__ import annotations

import asyncio
import importlib.util
import logging
import sys
import time
from pathlib import Path
from typing import Any, Callable

import numpy as np

from sim4wis.controller.base import ControlCommand, ControllerStrategy
from sim4wis.core.state import DriverInput, N_WHEELS, VehicleState, VehicleParams
from sim4wis.paths import plugins_dir

logger = logging.getLogger(__name__)

_PLUGIN_FILE = "user_strategy.py"
_POLL_INTERVAL = 1.0   # seconds between mtime checks


class HotReloadStrategy(ControllerStrategy):
    """Delegates compute() to user-supplied Python; reloads on file change."""

    name = "user_python"

    def __init__(self, params: VehicleParams) -> None:
        super().__init__(params)
        self._path: Path = plugins_dir() / _PLUGIN_FILE
        self._mtime: float = 0.0
        self._compute_fn: Callable | None = None
        self._status: str = "no_file"   # "ok" | "error" | "no_file"
        self._error: str | None = None
        self._last_reload: float | None = None
        self._poll_task: asyncio.Task | None = None
        self._load()
        self._start_watcher()

    # ---- lifecycle -----------------------------------------------------------

    def _start_watcher(self) -> None:
        """Attach a polling coroutine to the running event loop (if any)."""
        try:
            loop = asyncio.get_running_loop()
            self._poll_task = loop.create_task(self._poll_loop(), name="user_python_watcher")
        except RuntimeError:
            pass   # no running loop (e.g. during unit tests); skip background task

    async def _poll_loop(self) -> None:
        while True:
            await asyncio.sleep(_POLL_INTERVAL)
            try:
                mtime = self._path.stat().st_mtime
            except FileNotFoundError:
                if self._status != "no_file":
                    self._status = "no_file"
                    self._compute_fn = None
                    logger.warning("user_strategy.py removed or not found")
                continue
            if mtime != self._mtime:
                self._load()

    def _load(self) -> None:
        """Attempt to (re-)load the user strategy module."""
        try:
            spec = importlib.util.spec_from_file_location("user_strategy", self._path)
            if spec is None or spec.loader is None:
                raise ImportError(f"cannot load spec from {self._path}")
            mod = importlib.util.module_from_spec(spec)
            sys.modules["user_strategy"] = mod
            spec.loader.exec_module(mod)  # type: ignore[union-attr]
            fn = getattr(mod, "compute", None)
            if not callable(fn):
                raise AttributeError("user_strategy.py must define compute(driver, state)")
            self._compute_fn = fn
            self._mtime = self._path.stat().st_mtime
            self._status = "ok"
            self._error = None
            self._last_reload = time.time()
            logger.info("user_strategy.py loaded OK")
        except FileNotFoundError:
            self._status = "no_file"
            self._compute_fn = None
            self._error = None
        except Exception as exc:
            self._status = "error"
            self._error = str(exc)
            logger.error("user_strategy.py load error: %s", exc)

    # ---- public status -------------------------------------------------------

    def status_dict(self) -> dict[str, Any]:
        return {
            "status": self._status,
            "error": self._error,
            "last_reload": self._last_reload,
            "file_path": str(self._path),
        }

    def force_reload(self) -> dict[str, Any]:
        self._load()
        return self.status_dict()

    # ---- ControllerStrategy --------------------------------------------------

    def compute(self, driver: DriverInput, state: VehicleState) -> ControlCommand:
        if self._compute_fn is None:
            return ControlCommand.zero()
        driver_d = {
            "throttle": driver.throttle,
            "steering": driver.steering,
            "handbrake": driver.handbrake,
            "mode_params": dict(driver.mode_params),
        }
        state_d = {
            "t": state.t,
            "x": state.x, "y": state.y, "psi": state.psi,
            "vx": state.vx, "vy": state.vy, "yaw_rate": state.yaw_rate,
            "delta": state.delta.tolist(),
            "fz": state.fz.tolist(),
            "torque_steer": state.torque_steer.tolist(),
            "steer_limit": self.params.steer_limit,
            "wheelbase": self.params.wheelbase,
            "track_front": self.params.track_front,
            "track_rear": self.params.track_rear,
        }
        try:
            result = self._compute_fn(driver_d, state_d)
        except Exception as exc:
            self._status = "error"
            self._error = f"runtime error: {exc}"
            return ControlCommand.zero()

        delta_cmd = np.asarray(result.get("delta_cmd", [0.0] * N_WHEELS), dtype=np.float64)
        delta_cmd = np.clip(delta_cmd, -self.params.steer_limit, self.params.steer_limit)

        # Derive wheel speeds from delta_cmd + throttle (same as IdealAckermann does).
        wheels = self.params.wheel_positions_body()
        v = driver.throttle * self.params.v_max
        # Approximate: assume forward-dominant motion.
        omega_wheel = np.full(N_WHEELS, v / self.params.tire_radius)

        return ControlCommand(
            delta_cmd=delta_cmd,
            wheel_speed_cmd=omega_wheel,
            icr_target_body=np.array([np.nan, np.nan]),
        )
