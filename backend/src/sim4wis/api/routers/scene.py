"""Scene endpoints — base friction + disturbance CRUD (GUI editing)."""

from __future__ import annotations

from typing import Any, Literal

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from sim4wis.core.simulator import get_simulator
from sim4wis.environment.disturbance import Disturbance, disturbance_from_dict

router = APIRouter()


class SceneMu(BaseModel):
    mu: float = Field(..., ge=0.0, le=1.5, description="Base tyre-ground friction coefficient")


@router.post("/scene/mu")
async def set_scene_mu(body: SceneMu) -> dict[str, float]:
    """Set the base road friction μ live (surface-preset / manual tune)."""
    sim = get_simulator()
    sim.scene.base_mu = float(body.mu)
    sim.env.mu = float(body.mu)
    return {"base_mu": sim.scene.base_mu}


@router.get("/scene")
async def get_scene() -> dict[str, Any]:
    return get_simulator().scene.serialize()


class DisturbanceCreate(BaseModel):
    """Type-tagged disturbance creation. Bounds keep the sim numerically safe
    (e.g. stiffness cap prevents the Fz-pulse blowup fixed in Phase 3)."""

    type: Literal["ice_patch", "split_mu", "speed_bump", "slope"]
    x: float = Field(..., ge=-10_000.0, le=10_000.0)
    y: float = Field(..., ge=-10_000.0, le=10_000.0)
    width: float = Field(6.0, gt=0.0, le=100.0)
    length: float = Field(10.0, gt=0.0, le=200.0)
    heading: float = Field(0.0, ge=-3.1416, le=3.1416)
    # ice_patch
    mu: float | None = Field(None, ge=0.05, le=1.5)
    # split_mu
    mu_left: float | None = Field(None, ge=0.05, le=1.5)
    mu_right: float | None = Field(None, ge=0.05, le=1.5)
    # speed_bump
    height: float | None = Field(None, ge=0.0, le=0.3)
    stiffness: float | None = Field(None, gt=0.0, le=5e5)
    # slope
    angle: float | None = Field(None, ge=-0.35, le=0.35)

    def payload(self) -> dict[str, Any]:
        return self.model_dump(exclude_none=True)


class DisturbanceUpdate(BaseModel):
    """Partial update — same bounds as create, everything optional."""

    x: float | None = Field(None, ge=-10_000.0, le=10_000.0)
    y: float | None = Field(None, ge=-10_000.0, le=10_000.0)
    width: float | None = Field(None, gt=0.0, le=100.0)
    length: float | None = Field(None, gt=0.0, le=200.0)
    heading: float | None = Field(None, ge=-3.1416, le=3.1416)
    mu: float | None = Field(None, ge=0.05, le=1.5)
    mu_left: float | None = Field(None, ge=0.05, le=1.5)
    mu_right: float | None = Field(None, ge=0.05, le=1.5)
    height: float | None = Field(None, ge=0.0, le=0.3)
    stiffness: float | None = Field(None, gt=0.0, le=5e5)
    angle: float | None = Field(None, ge=-0.35, le=0.35)


def _gen_id(existing: set[str], type_id: str) -> str:
    n = 1
    while f"{type_id}_{n}" in existing:
        n += 1
    return f"{type_id}_{n}"


def _find(disturbances: list[Disturbance], dist_id: str) -> int:
    for i, d in enumerate(disturbances):
        if d.id == dist_id:
            return i
    raise HTTPException(status_code=404, detail=f"disturbance not found: {dist_id}")


@router.post("/scene/disturbances")
async def create_disturbance(body: DisturbanceCreate) -> dict[str, Any]:
    sim = get_simulator()
    payload = body.payload()
    payload["id"] = _gen_id({d.id for d in sim.scene.disturbances}, body.type)
    try:
        dist = disturbance_from_dict(payload)
    except (ValueError, TypeError) as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    sim.scene.disturbances.append(dist)
    return dist.serialize()


@router.put("/scene/disturbances/{dist_id}")
async def update_disturbance(dist_id: str, body: DisturbanceUpdate) -> dict[str, Any]:
    sim = get_simulator()
    i = _find(sim.scene.disturbances, dist_id)
    merged = sim.scene.disturbances[i].serialize()
    merged.update(body.model_dump(exclude_none=True))
    try:
        # Rebuild instead of mutating in place — same atomic-replace spirit as set_scene.
        dist = disturbance_from_dict(merged)
    except (ValueError, TypeError) as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    sim.scene.disturbances[i] = dist
    return dist.serialize()


@router.delete("/scene/disturbances/{dist_id}")
async def delete_disturbance(dist_id: str) -> dict[str, Any]:
    sim = get_simulator()
    i = _find(sim.scene.disturbances, dist_id)
    removed = sim.scene.disturbances.pop(i)
    return {"status": "deleted", "id": removed.id}


@router.post("/scene/clear")
async def clear_disturbances() -> dict[str, Any]:
    sim = get_simulator()
    n = len(sim.scene.disturbances)
    sim.scene.disturbances.clear()
    return {"status": "cleared", "removed": n, "base_mu": sim.scene.base_mu}
