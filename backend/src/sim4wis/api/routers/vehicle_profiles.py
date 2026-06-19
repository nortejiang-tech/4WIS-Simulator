"""Vehicle profile endpoints."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from sim4wis.core.simulator import get_simulator
from sim4wis.project.params_codec import params_from_dict

router = APIRouter()


class ProfileSave(BaseModel):
    label: str | None = Field(None, max_length=80)
    description: str = ""
    vehicle: dict[str, Any] | None = None


@router.get("/vehicle-profiles")
async def list_vehicle_profiles() -> dict[str, Any]:
    from sim4wis.project.vehicle_profiles import list_profiles
    return {"profiles": list_profiles()}


@router.get("/vehicle-profiles/{name}")
async def get_vehicle_profile(name: str) -> dict[str, Any]:
    from sim4wis.project.vehicle_profiles import load_profile
    try:
        return load_profile(name)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=f"vehicle profile not found: {name}") from e


@router.post("/vehicle-profiles/{name}")
async def save_vehicle_profile(name: str, body: ProfileSave | None = None) -> dict[str, Any]:
    from sim4wis.project.vehicle_profiles import save_profile
    sim = get_simulator()
    if body and body.vehicle is not None:
        params = params_from_dict(body.vehicle, sim.params)
    else:
        params = sim.params
    path = save_profile(
        name,
        params,
        label=(body.label if body else None),
        description=(body.description if body else ""),
    )
    return {"status": "saved", "name": name, "path": str(path)}


@router.post("/vehicle-profiles/{name}/apply")
async def apply_vehicle_profile(name: str) -> dict[str, Any]:
    from sim4wis.project.vehicle_profiles import load_profile_params
    sim = get_simulator()
    try:
        params = load_profile_params(name)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=f"vehicle profile not found: {name}") from e
    sim.set_params(params)
    sim.reset()
    return {"status": "applied", "name": name}


@router.delete("/vehicle-profiles/{name}")
async def delete_vehicle_profile(name: str) -> dict[str, Any]:
    from sim4wis.project.vehicle_profiles import BUILTIN_LS9_ALIASES, BUILTIN_LS9_NAME, delete_profile
    if name == BUILTIN_LS9_NAME or name in BUILTIN_LS9_ALIASES:
        raise HTTPException(status_code=400, detail="built-in profile cannot be deleted")
    if not delete_profile(name):
        raise HTTPException(status_code=404, detail=f"vehicle profile not found: {name}")
    return {"status": "deleted", "name": name}
