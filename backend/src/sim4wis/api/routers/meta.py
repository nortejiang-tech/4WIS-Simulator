"""Version / strategy / model / status / reset / plugin endpoints."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from sim4wis.controller.registry import available_strategies
from sim4wis.core.simulator import get_simulator
from sim4wis.vehicle.model_registry import (
    model_ids,
    model_infos,
    model_layer,
    serialize_model_info,
)

router = APIRouter()


@router.get("/version")
async def version() -> dict[str, str]:
    from sim4wis import __version__
    return {"version": __version__}


@router.get("/strategies")
async def list_strategies() -> dict[str, list[str]]:
    return {"strategies": available_strategies()}


@router.get("/status")
async def status() -> dict[str, Any]:
    sim = get_simulator()
    s = sim.model.state
    return {
        "strategy": sim.strategy_name,
        "t": s.t,
        "pose": {"x": s.x, "y": s.y, "psi": s.psi},
        "running": sim._running,  # noqa: SLF001 — read-only debug peek
        "subscribers": len(sim.subscribers),
    }


class StrategySelect(BaseModel):
    name: str = Field(..., description="One of /api/strategies")


@router.post("/strategy")
async def set_strategy(body: StrategySelect) -> dict[str, str]:
    sim = get_simulator()
    try:
        sim.set_strategy(body.name)
    except KeyError as e:
        raise HTTPException(status_code=400, detail=f"unknown strategy: {e}") from e
    return {"strategy": sim.strategy_name}


@router.post("/reset")
async def reset() -> dict[str, str]:
    sim = get_simulator()
    sim.reset()
    return {"status": "reset"}


class ModelSelect(BaseModel):
    model_type: str = Field(..., description="One of /api/model available ids")


@router.post("/model")
async def set_model(body: ModelSelect) -> dict[str, str]:
    sim = get_simulator()
    try:
        sim.set_model_type(body.model_type)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    sim.reset()
    return {"model_type": sim.model_type, "layer": model_layer(sim.model_type)}


@router.get("/model")
async def get_model() -> dict[str, Any]:
    sim = get_simulator()
    return {
        "model_type": sim.model_type,
        "layer": model_layer(sim.model_type),
        "available": model_ids(),
        "models": [serialize_model_info(info) for info in model_infos()],
        "primary": model_ids(include_research=False),
    }


@router.get("/user_python/status")
async def user_python_status() -> dict[str, Any]:
    """Current status of the hot-reload Python strategy plugin."""
    sim = get_simulator()
    from sim4wis.controller.user_python import HotReloadStrategy
    if isinstance(sim.strategy, HotReloadStrategy):
        return sim.strategy.status_dict()
    # Not the active strategy — instantiate a probe to read from disk.
    try:
        probe = HotReloadStrategy(sim.params)
        return probe.status_dict()
    except Exception as exc:
        return {"status": "error", "error": str(exc), "last_reload": None, "file_path": ""}


@router.post("/user_python/reload")
async def user_python_reload() -> dict[str, Any]:
    """Force-reload the Python strategy plugin (e.g. after saving the file)."""
    sim = get_simulator()
    from sim4wis.controller.user_python import HotReloadStrategy
    if isinstance(sim.strategy, HotReloadStrategy):
        return sim.strategy.force_reload()
    raise HTTPException(status_code=400, detail="user_python is not the active strategy")


@router.get("/plugins")
async def list_plugins() -> dict[str, Any]:
    from sim4wis.controller.plugins.loader import serialize_report
    return serialize_report()


@router.post("/plugins/reload")
async def reload_plugins_endpoint() -> dict[str, Any]:
    from sim4wis.controller.plugins.loader import reload_plugins, serialize_report
    reload_plugins()
    return serialize_report()
