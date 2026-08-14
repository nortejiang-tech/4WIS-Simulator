"""The external-controller interface — the Simulink door, framed but not hung.

FR-11: real control laws will arrive from Simulink models (or embedded code
generated from them) in the production environment. What ships now is the
**calling contract** a real adapter must implement, plus one reference
implementation of it: a file-backed gain table. The FMI co-simulation
loader and the code-generation wrapper are production work — the framework
is here so the interface they plug into is already fixed, tested, and
spec-complete, and so a study can name an external controller today and get
an honest "adapter missing" refusal instead of a silent fallback.

The contract (see docs/steering_control_guide.md §Simulink):

* The adapter implements ``AngleTrackingController``; its ``step`` maps the
  protocol inputs onto the external ports, invokes the backend, and maps
  the torque output back. Ports: target_angle, target_rate, feedback_angle,
  feedback_rate (in), speed_ms, load_torque (in), actuator_torque (out) —
  all in the road-wheel domain, per the layer's convention.
* ``param_table`` documents every external parameter with its provenance
  (who set it, from where) — the same "no number without a source" rule the
  targets layer enforces.
* ``init_backend`` / ``shutdown`` bound the backend's lifetime; the coupling
  calls ``step`` exactly once per control period, which the adapter must
  state it supports via ``inner_rate_hz``.
"""

from __future__ import annotations

import json
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar

from sim4wis.steering.tracking.controller import (
    INNER_RATE_HZ,
    AngleTrackingController,
    TrackingOutput,
    register,
)

#: The port contract every external controller exposes — the ABI a future
#: FMI/codegen adapter must satisfy. Direction is from the sim's view.
EXTERNAL_PORTS: tuple[tuple[str, str, str], ...] = (
    ("target_angle", "rad", "in"),
    ("target_rate", "rad/s", "in"),
    ("feedback_angle", "rad", "in"),
    ("feedback_rate", "rad/s", "in"),
    ("load_torque", "N·m", "in"),
    ("speed_ms", "m/s", "in"),
    ("actuator_torque", "N·m", "out"),
)


@dataclass
class ExternalParam:
    value: Any
    provenance: str


class ExternalController(AngleTrackingController, ABC):
    """A tracking controller whose law lives outside sim4wis."""

    inner_rate_hz: ClassVar[float | None] = INNER_RATE_HZ

    #: Parameters the external law needs, each with where it came from.
    #: Filled by init_backend — before that it is empty, and an adapter that
    #: steps without initialising its parameters is refusing to run, which
    #: is the honest state for a controller with no provenance.
    param_table: dict[str, ExternalParam]

    def describe_ports(self) -> list[dict[str, str]]:
        return [{"name": n, "unit": u, "direction": d} for n, u, d in EXTERNAL_PORTS]

    @abstractmethod
    def init_backend(self, workspace: str | Path) -> None:
        """Open the external backend (load FMU, dlopen the codegen .so, …)."""

    @abstractmethod
    def shutdown(self) -> None:
        """Release the backend. The coupling never calls step afterwards."""

    def _invoke(self, ports: dict[str, float]) -> float:
        """One backend call: ports dict → actuator torque. Adapters override."""
        raise NotImplementedError


class FileGainController(ExternalController):
    """Reference adapter: a static P gain read from a JSON table.

    The trivialest honest external law — torque = gain·(target − feedback).
    It exists to exercise the adapter contract end to end (init → per-step
    invoke → shutdown, param provenance, refusal when the table is missing);
    a real FMU/codegen adapter replaces ``_invoke`` and keeps everything
    else. This is the stand-in the spec calls a framework, not a fake FMU.
    """

    name: ClassVar[str] = "file_gain"

    def __init__(self, table_path: str | Path) -> None:
        self.table_path = Path(table_path)
        self._gain: float | None = None
        self.param_table = {}

    def init_backend(self, workspace: str | Path) -> None:
        path = self.table_path
        if not path.is_absolute():
            path = Path(workspace) / path
        if not path.exists():
            raise FileNotFoundError(
                f"external controller {self.name}: gain table {path} missing — "
                "this is the framework's honest refusal, not a silent fallback"
            )
        data = json.loads(path.read_text(encoding="utf-8"))
        if "gain" not in data:
            raise ValueError(f"{path}: external gain table needs a 'gain' key")
        self._gain = float(data["gain"])
        self.param_table = {
            "gain": ExternalParam(self._gain, f"file: {path}"),
        }

    def shutdown(self) -> None:
        self._gain = None

    def _invoke(self, ports: dict[str, float]) -> float:
        if self._gain is None:
            raise RuntimeError(f"{self.name}: init_backend was never called")
        return self._gain * (float(ports["target_angle"])
                             - float(ports["feedback_angle"]))

    def step(self, dt, *, target_angle, target_rate, feedback_angle,
             plant_angle, load_torque, speed_ms):
        return TrackingOutput(torque_cmd=self._invoke({
            "target_angle": float(target_angle),
            "target_rate": float(target_rate),
            "feedback_angle": float(feedback_angle),
            "feedback_rate": 0.0,
            "load_torque": float(load_torque),
            "speed_ms": float(speed_ms),
        }))


register(FileGainController)
