"""Trajectory / reference-path endpoints (step 17)."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

router = APIRouter()


@router.get("/path/templates")
async def path_templates() -> dict[str, Any]:
    """Template names (kept as a plain list for older clients) + the parameter
    specs the trajectory panel renders its inputs from."""
    from sim4wis.controller.path import list_templates, template_specs
    return {"templates": list_templates(), "specs": template_specs()}


def _vehicle_dimensions() -> dict[str, float]:
    """Drawn body size of the *active* vehicle, so ISO lane widths track it.

    Same factors as the frontend's `vehicleShape.bodyDimensions`, which is what
    the user actually sees between the cones.
    """
    from sim4wis.controller.path import BODY_WIDTH_FACTOR
    from sim4wis.core.simulator import get_simulator
    p = get_simulator().params
    return {
        "vehicle_width": max(p.track_front, p.track_rear) * BODY_WIDTH_FACTOR,
        "vehicle_length": p.wheelbase * 1.20,
    }


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
    anchor: str | None = Field(default=None,
                               description="Named scenario anchor to place the course on")


@router.post("/path/template")
async def set_path_template(body: TemplateBody) -> dict[str, Any]:
    from sim4wis.controller.path import plan_from_template, set_active_plan
    # Vehicle-derived geometry is injected unless the caller pinned it, so a
    # script/experiment with `params: {}` still gets a course sized for the car.
    params = {**_vehicle_dimensions(), **body.params}
    try:
        plan = plan_from_template(body.name, params, anchor=body.anchor)
    except KeyError as e:
        raise HTTPException(status_code=400, detail=f"unknown template or anchor: {e}") from e
    except TypeError as e:
        raise HTTPException(status_code=400, detail=f"bad template params: {e}") from e
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    version = set_active_plan(plan)
    return {"version": version, **plan.serialize()}


@router.post("/path/clear")
async def clear_path() -> dict[str, Any]:
    from sim4wis.controller.path import clear_active_plan
    version = clear_active_plan()
    return {"version": version, "points": [], "cones": [], "marks": [],
            "closed": False, "name": "", "label": "", "notes": ""}
