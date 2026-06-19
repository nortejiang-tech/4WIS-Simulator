"""Trajectory / reference-path endpoints (step 17)."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

router = APIRouter()


@router.get("/path/templates")
async def path_templates() -> dict[str, list[str]]:
    from sim4wis.controller.path import list_templates
    return {"templates": list_templates()}


@router.get("/path")
async def get_path() -> dict[str, Any]:
    from sim4wis.controller.path import get_active_plan, get_version
    return {"version": get_version(), **get_active_plan().serialize()}


class WaypointsBody(BaseModel):
    points: list[list[float]] = Field(..., description="World-frame [[x,y],...]")
    closed: bool = False
    name: str = "custom"


@router.post("/path/waypoints")
async def set_path_waypoints(body: WaypointsBody) -> dict[str, Any]:
    from sim4wis.controller.path import plan_from_waypoints, set_active_plan
    plan = plan_from_waypoints(body.points, closed=body.closed, name=body.name)
    version = set_active_plan(plan)
    return {"version": version, **plan.serialize()}


class TemplateBody(BaseModel):
    name: str
    params: dict[str, float] = Field(default_factory=dict)


@router.post("/path/template")
async def set_path_template(body: TemplateBody) -> dict[str, Any]:
    from sim4wis.controller.path import plan_from_template, set_active_plan
    try:
        plan = plan_from_template(body.name, body.params)
    except KeyError as e:
        raise HTTPException(status_code=400, detail=f"unknown template: {e}") from e
    except TypeError as e:
        raise HTTPException(status_code=400, detail=f"bad template params: {e}") from e
    version = set_active_plan(plan)
    return {"version": version, **plan.serialize()}


@router.post("/path/clear")
async def clear_path() -> dict[str, Any]:
    from sim4wis.controller.path import clear_active_plan
    version = clear_active_plan()
    return {"version": version, "points": [], "cones": [], "closed": False, "name": ""}
