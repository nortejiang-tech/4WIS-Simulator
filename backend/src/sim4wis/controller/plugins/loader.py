"""Plugin discovery + registration.

Scans `<repo>/plugins/strategies/*.fmu` and, for each FMU with a matching
`<name>.fmu.yaml` sidecar, registers an `FMUControllerStrategy` into the
in-process registry. Failures are logged per-plugin and never abort the
process startup.

The discovery report (succeeded / failed) is exposed via
`/api/plugins`. Frontend can show a "plugins loaded" badge.
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from sim4wis.controller.base import ControllerStrategy
from sim4wis.controller.registry import _BUILTIN  # we mutate the registry dict
from sim4wis.controller.plugins.fmu_adapter import FMUControllerStrategy
from sim4wis.controller.plugins.matlab_adapter import MatlabEngineControllerStrategy
from sim4wis.core.state import VehicleParams

logger = logging.getLogger(__name__)


@dataclass
class PluginResult:
    name: str
    path: str
    ok: bool
    error: str | None = None


_LAST_REPORT: list[PluginResult] = []


def plugin_root() -> Path:
    env = os.environ.get("SIM4WIS_PLUGINS_DIR")
    if env:
        return Path(env)
    from sim4wis.paths import plugins_dir
    return plugins_dir()


def discover_and_register() -> list[PluginResult]:
    """Scan plugin dir; register every successfully loaded plugin into the
    global controller registry. Returns the report so callers can surface it."""
    global _LAST_REPORT
    report: list[PluginResult] = []
    root = plugin_root()
    if not root.exists():
        logger.info("Plugins dir %s does not exist — skipping", root)
        _LAST_REPORT = report
        return report

    for path in sorted(list(root.glob("*.fmu")) + list(root.glob("*.slx"))):
        ext = path.suffix.lower()
        kind = "fmu" if ext == ".fmu" else "slx"
        sidecar = path.with_suffix(f".{kind}.yaml")
        if not sidecar.exists():
            report.append(PluginResult(
                name=path.stem, path=str(path),
                ok=False, error=f"missing sidecar {sidecar.name}",
            ))
            continue
        try:
            cfg = yaml.safe_load(sidecar.read_text(encoding="utf-8")) or {}
            name = str(cfg.get("name", path.stem))
            inputs = dict(cfg.get("inputs", {}))
            outputs = dict(cfg.get("outputs", {}))
            step_ms = float(cfg.get("step_size_ms", 10.0))
        except Exception as e:
            report.append(PluginResult(
                name=path.stem, path=str(path),
                ok=False, error=f"sidecar parse: {e}",
            ))
            continue

        try:
            if kind == "fmu":
                def _factory(params: VehicleParams, _fp=path, _name=name,
                             _inputs=inputs, _outputs=outputs, _step=step_ms) -> ControllerStrategy:
                    return FMUControllerStrategy(
                        params=params, name=_name, fmu_path=_fp,
                        inputs=_inputs, outputs=_outputs, step_size_ms=_step,
                    )
            else:  # slx
                def _factory(params: VehicleParams, _fp=path, _name=name,
                             _inputs=inputs, _outputs=outputs, _step=step_ms) -> ControllerStrategy:
                    return MatlabEngineControllerStrategy(
                        params=params, name=_name, slx_path=_fp,
                        inputs=_inputs, outputs=_outputs, step_size_ms=_step,
                    )
            _BUILTIN[name] = _factory
            report.append(PluginResult(name=name, path=str(path), ok=True))
            logger.info("Registered %s plugin: %s ← %s", kind.upper(), name, path.name)
        except Exception as e:
            report.append(PluginResult(
                name=name, path=str(path),
                ok=False, error=f"register: {e}",
            ))
            logger.exception("Failed to register %s plugin %s", kind.upper(), path.name)

    _LAST_REPORT = report
    return report


def last_report() -> list[PluginResult]:
    return list(_LAST_REPORT)


def reload_plugins() -> list[PluginResult]:
    """Reload plugins — drops previously-registered FMU plugins first.

    Currently we don't track which entries in _BUILTIN came from plugins
    (Phase 2 limitation). For now reload simply re-runs discover_and_register
    which overwrites entries with the same name. Removing stale entries is a
    Phase 2.5 enhancement.
    """
    return discover_and_register()


def serialize_report() -> dict[str, Any]:
    rep = last_report()
    return {
        "total": len(rep),
        "loaded": sum(1 for r in rep if r.ok),
        "failed": sum(1 for r in rep if not r.ok),
        "items": [
            {"name": r.name, "path": r.path, "ok": r.ok, "error": r.error}
            for r in rep
        ],
    }
