"""FMUControllerStrategy — wrap a Simulink-exported FMU as a sim4wis strategy.

Workflow for algorithm devs (Simulink users):
    1. Build the strategy in Simulink (.slx) with explicit Inport / Outport blocks
    2. Use Simulink Coder + FMI Kit to export an FMU (.fmu file)
    3. Copy the .fmu into <repo>/plugins/strategies/
    4. Add a sidecar `<name>.fmu.yaml` describing input/output mapping
    5. Restart simulator OR POST /api/plugins/reload — the FMU appears in the
       strategy list with the name from the sidecar

The sidecar YAML is consumed by `loader.py`; it's plain YAML so this module
itself doesn't depend on pydantic. Schema (informal):

    name:        my_strategy           # registered strategy name
    description: ...
    inputs:      # FMU variable name → expression evaluated from sim4wis state
      vx:        state.vx
      yaw_rate:  state.yaw_rate
      throttle:  driver.throttle
      steering:  driver.steering
      delta_fl:  state.delta[0]        # ... etc
    outputs:     # ControlCommand field expression ← FMU variable
      delta_cmd[0]: delta_fl
      ...
      wheel_speed_cmd[0]: omega_fl
      ...
    step_size_ms: 10                   # FMU internal sample period [ms]

Expressions in `inputs` use a tiny sandboxed evaluator — only attribute /
index access on `state`, `driver`, `params` are allowed. No arithmetic, no
calls — keeps the surface predictable.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import numpy as np

from sim4wis.controller.base import ControllerStrategy
from sim4wis.core.state import (
    ControlCommand,
    DriverInput,
    N_WHEELS,
    VehicleParams,
    VehicleState,
)

logger = logging.getLogger(__name__)


def _read_state_path(path: str, state: VehicleState, driver: DriverInput, params: VehicleParams) -> float:
    """Read a dot/bracket path like 'state.delta[0]' or 'driver.throttle'."""
    parts = path.replace("]", "").split(".")
    if parts[0] == "state":
        obj: Any = state
    elif parts[0] == "driver":
        obj = driver
    elif parts[0] == "params":
        obj = params
    else:
        raise ValueError(f"unsupported root in path: {path!r}")
    for p in parts[1:]:
        if "[" in p:
            name, idx = p.split("[", 1)
            obj = getattr(obj, name)[int(idx)]
        else:
            obj = getattr(obj, p)
    return float(obj)


def _write_command_path(path: str, value: float, cmd: ControlCommand) -> None:
    """Write `value` into a path like 'delta_cmd[0]' on ControlCommand."""
    if "[" not in path:
        raise ValueError(f"output path must include [i]: {path!r}")
    name, idx = path.replace("]", "").split("[", 1)
    getattr(cmd, name)[int(idx)] = float(value)


class FMUControllerStrategy(ControllerStrategy):
    """Wrap a single FMU as a 4WIS controller strategy.

    The actual FMU loading is deferred until first compute() so backends
    without `fmpy` installed can still start (the strategy will just throw
    on use).
    """

    def __init__(
        self,
        params: VehicleParams,
        *,
        name: str,
        fmu_path: Path,
        inputs: dict[str, str],
        outputs: dict[str, str],
        step_size_ms: float = 10.0,
    ) -> None:
        super().__init__(params)
        self.name = name      # overrides class default for registry purposes
        self.fmu_path = Path(fmu_path)
        self.inputs = dict(inputs)
        self.outputs = dict(outputs)
        self.step_size = step_size_ms / 1000.0
        self._fmu = None
        self._t = 0.0

    def _ensure_loaded(self) -> None:
        if self._fmu is not None:
            return
        try:
            import fmpy
            from fmpy.fmi2 import FMU2Slave
        except ImportError as e:
            raise RuntimeError(
                "fmpy not installed — install with: pip install fmpy"
            ) from e
        model_desc = fmpy.read_model_description(str(self.fmu_path))
        unzipped = fmpy.extract(str(self.fmu_path))
        self._unzip_dir = unzipped
        self._model_desc = model_desc

        # Map variable names → value references
        self._vrefs: dict[str, int] = {
            v.name: v.valueReference for v in model_desc.modelVariables
        }

        self._fmu = FMU2Slave(
            guid=model_desc.guid,
            unzipDirectory=unzipped,
            modelIdentifier=model_desc.coSimulation.modelIdentifier,
            instanceName=f"sim4wis_{self.name}",
        )
        self._fmu.instantiate()
        self._fmu.setupExperiment(startTime=0.0)
        self._fmu.enterInitializationMode()
        self._fmu.exitInitializationMode()
        self._t = 0.0
        logger.info("FMU loaded: %s (%d vars)", self.fmu_path.name, len(self._vrefs))

    def compute(self, driver: DriverInput, state: VehicleState) -> ControlCommand:
        self._ensure_loaded()
        # 1) Set FMU inputs from sim4wis state via expression mapping
        for fmu_var, expr in self.inputs.items():
            if fmu_var not in self._vrefs:
                logger.warning("FMU input '%s' not found in FMU", fmu_var)
                continue
            try:
                val = _read_state_path(expr, state, driver, self.params)
            except Exception:
                logger.exception("input mapping failed for %s = %s", fmu_var, expr)
                continue
            self._fmu.setReal([self._vrefs[fmu_var]], [val])
        # 2) Step the FMU one sample period
        self._fmu.doStep(currentCommunicationPoint=self._t, communicationStepSize=self.step_size)
        self._t += self.step_size
        # 3) Read outputs and build ControlCommand
        delta_cmd = np.zeros(N_WHEELS)
        wheel_speed_cmd = np.zeros(N_WHEELS)
        cmd = ControlCommand(
            delta_cmd=delta_cmd,
            wheel_speed_cmd=wheel_speed_cmd,
            icr_target_body=np.array([np.nan, np.nan]),
        )
        for cmd_path, fmu_var in self.outputs.items():
            if fmu_var not in self._vrefs:
                logger.warning("FMU output '%s' not found in FMU", fmu_var)
                continue
            (val,) = self._fmu.getReal([self._vrefs[fmu_var]])
            try:
                _write_command_path(cmd_path, val, cmd)
            except Exception:
                logger.exception("output mapping failed for %s ← %s", cmd_path, fmu_var)
        return cmd

    def shutdown(self) -> None:
        if self._fmu is not None:
            try:
                self._fmu.terminate()
                self._fmu.freeInstance()
            except Exception:
                pass
            self._fmu = None
