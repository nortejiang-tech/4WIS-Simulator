"""Fault injection REST endpoints.

GET  /api/faults         — list all configured faults
POST /api/faults         — add a new fault
PATCH /api/faults/{id}   — toggle active state
DELETE /api/faults/{id}  — remove a fault
DELETE /api/faults       — clear all faults
"""

from __future__ import annotations

from typing import Any, Literal

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from sim4wis.core.simulator import get_simulator
from sim4wis.fault import FaultConfig, FaultType

router = APIRouter()

WheelName = Literal["fl", "fr", "rl", "rr"]


class FaultRequest(BaseModel):
    fault_type: FaultType
    wheel: WheelName
    value: float = Field(0.0, description=(
        "stuck_angle [rad] | range_limit [rad] | bias [rad] | noise_std [rad]; "
        "ignored for motor_stuck_zero and sensor_dropout"
    ))
    active: bool = True


class ActivePatch(BaseModel):
    active: bool


@router.get("/faults")
async def list_faults() -> dict[str, Any]:
    sim = get_simulator()
    return {"faults": [f.to_dict() for f in sim.fault_injector.list_faults()]}


@router.post("/faults", status_code=201)
async def add_fault(body: FaultRequest) -> dict[str, Any]:
    sim = get_simulator()
    cfg = FaultConfig(
        fault_type=body.fault_type,
        wheel=body.wheel,
        value=body.value,
        active=body.active,
    )
    sim.fault_injector.add(cfg)
    return cfg.to_dict()


@router.patch("/faults/{fault_id}")
async def patch_fault(fault_id: str, body: ActivePatch) -> dict[str, Any]:
    sim = get_simulator()
    if not sim.fault_injector.set_active(fault_id, body.active):
        raise HTTPException(status_code=404, detail=f"fault {fault_id!r} not found")
    faults = sim.fault_injector.list_faults()
    f = next((x for x in faults if x.id == fault_id), None)
    if f is None:
        raise HTTPException(status_code=404, detail=f"fault {fault_id!r} not found")
    return f.to_dict()


@router.delete("/faults/{fault_id}", status_code=200)
async def delete_fault(fault_id: str) -> dict[str, Any]:
    sim = get_simulator()
    if not sim.fault_injector.remove(fault_id):
        raise HTTPException(status_code=404, detail=f"fault {fault_id!r} not found")
    return {"deleted": fault_id}


@router.delete("/faults", status_code=200)
async def clear_faults() -> dict[str, Any]:
    sim = get_simulator()
    sim.fault_injector.clear()
    return {"cleared": True}
