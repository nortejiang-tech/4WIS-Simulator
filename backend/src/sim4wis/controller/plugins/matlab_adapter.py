"""MATLAB Engine controller adapter.

For algorithm devs who want to iterate on a Simulink model without exporting
an FMU each time. The adapter calls into a live MATLAB session via the
official `matlabengine` Python package.

Workflow:
    1. Install MATLAB R2020a+ on the same machine
    2. Install the engine bridge:  pip install matlabengine
    3. Place an `<name>.slx` model under `plugins/strategies/`
       with a `<name>.slx.yaml` sidecar (same shape as FMU sidecar, but
       fields map to base workspace variables instead of FMU value refs):

           name: my_simulink_strategy
           inputs:
             vx:        state.vx
             yaw_rate:  state.yaw_rate
             throttle:  driver.throttle
             steering:  driver.steering
           outputs:
             delta_cmd[0]: delta_fl   # name of an "out" workspace variable
             ...
           step_size_ms: 10

    4. Restart simulator OR `POST /api/plugins/reload`

Performance caveat: starting MATLAB takes 5–15 seconds and each step incurs
JVM-bridge overhead. Don't expect real-time at 200 Hz; this adapter is for
offline / slow-replay analysis. The frontend marks the strategy as "slow"
when active so users aren't surprised.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import numpy as np

from sim4wis.controller.base import ControllerStrategy
from sim4wis.controller.plugins.fmu_adapter import _read_state_path, _write_command_path
from sim4wis.core.state import (
    ControlCommand,
    DriverInput,
    N_WHEELS,
    VehicleParams,
    VehicleState,
)

logger = logging.getLogger(__name__)


class MatlabEngineControllerStrategy(ControllerStrategy):
    """Wrap a Simulink .slx model via the MATLAB Engine API."""

    def __init__(
        self,
        params: VehicleParams,
        *,
        name: str,
        slx_path: Path,
        inputs: dict[str, str],
        outputs: dict[str, str],
        step_size_ms: float = 10.0,
    ) -> None:
        super().__init__(params)
        self.name = name
        self.slx_path = Path(slx_path)
        self.inputs = dict(inputs)
        self.outputs = dict(outputs)
        self.step_size = step_size_ms / 1000.0
        self._eng: Any = None
        self._model_name = self.slx_path.stem
        self._loaded = False

    def _ensure_loaded(self) -> None:
        if self._loaded:
            return
        try:
            import matlab.engine
        except ImportError as e:
            raise RuntimeError(
                "matlabengine not installed — install with: "
                "pip install matlabengine  (requires MATLAB R2020a+)"
            ) from e
        logger.info("Starting MATLAB engine (this may take 5-15 s)…")
        self._eng = matlab.engine.start_matlab()
        # Add the plugin dir to MATLAB path
        self._eng.addpath(str(self.slx_path.parent), nargout=0)
        self._eng.load_system(str(self.slx_path), nargout=0)
        self._loaded = True
        logger.info("MATLAB engine ready; model %s loaded", self._model_name)

    def compute(self, driver: DriverInput, state: VehicleState) -> ControlCommand:
        self._ensure_loaded()
        import matlab

        # 1) Write inputs into MATLAB base workspace
        for ws_var, expr in self.inputs.items():
            try:
                val = _read_state_path(expr, state, driver, self.params)
            except Exception:
                logger.exception("input mapping failed for %s = %s", ws_var, expr)
                continue
            self._eng.workspace[ws_var] = matlab.double([val])

        # 2) Run one step of the model
        self._eng.set_param(
            self._model_name, "SimulationCommand", "step", nargout=0,
        )

        # 3) Read outputs back
        delta_cmd = np.zeros(N_WHEELS)
        wheel_speed_cmd = np.zeros(N_WHEELS)
        cmd = ControlCommand(
            delta_cmd=delta_cmd,
            wheel_speed_cmd=wheel_speed_cmd,
            icr_target_body=np.array([np.nan, np.nan]),
        )
        for cmd_path, ws_var in self.outputs.items():
            try:
                v = self._eng.workspace[ws_var]
                # MATLAB returns scalars wrapped in matlab.double; convert
                val = float(v[0][0]) if hasattr(v, "__getitem__") else float(v)
                _write_command_path(cmd_path, val, cmd)
            except Exception:
                logger.exception("output mapping failed for %s ← %s", cmd_path, ws_var)
        return cmd

    def shutdown(self) -> None:
        if self._eng is not None:
            try:
                self._eng.quit()
            except Exception:
                pass
            self._eng = None
            self._loaded = False
