"""Scenario endpoints — static driving environments (city / track / plaza).

Fetched once on load and kept in sync via a version counter (mirrors /path);
loading a scenario also teleports the vehicle to the scenario's spawn pose.
"""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException

from sim4wis.core.simulator import get_simulator
from sim4wis.core.state import VehicleState
from sim4wis.environment import scenario as scn

router = APIRouter()


@router.get("/scenarios")
async def list_scenarios() -> dict[str, list[dict]]:
    return {"scenarios": scn.list_scenarios()}


@router.get("/scenario")
async def get_scenario() -> dict[str, Any]:
    active = scn.get_active()
    return {"version": scn.get_version(), "scenario": active.serialize() if active else None}


def _spawn(sim, pose: tuple[float, float, float]) -> None:
    init = VehicleState()
    init.x, init.y, init.psi = float(pose[0]), float(pose[1]), float(pose[2])
    sim.model.reset(init)


@router.post("/scenarios/{name}/load")
async def load_scenario(name: str) -> dict[str, Any]:
    try:
        version = scn.load_scenario(name)
    except KeyError as e:
        raise HTTPException(status_code=404, detail=f"unknown scenario: {name}") from e
    active = scn.get_active()
    sim = get_simulator()
    if active is not None:
        _spawn(sim, active.spawn)
    return {"version": version, "scenario": active.serialize() if active else None}


@router.post("/scenario/clear")
async def clear_scenario() -> dict[str, Any]:
    version = scn.clear_scenario()
    return {"version": version, "scenario": None}
