"""Project (YAML) endpoints — list / inspect / load / save."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from sim4wis.core.simulator import get_simulator

router = APIRouter()


class ProjectSave(BaseModel):
    name: str = Field(..., min_length=1, max_length=64)
    description: str = ""


@router.get("/projects")
async def list_projects_endpoint() -> dict[str, list[str]]:
    from sim4wis.project.io import list_projects
    return {"projects": list_projects()}


@router.get("/projects/{name}")
async def get_project(name: str) -> dict[str, Any]:
    from sim4wis.project.io import load_project
    try:
        proj = load_project(name)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=f"project not found: {name}") from e
    return proj.model_dump(mode="json")


@router.post("/projects/{name}/load")
async def load_project_into_sim(name: str) -> dict[str, Any]:
    """Read project YAML and apply it to the running simulator."""
    from sim4wis.project.io import load_project
    sim = get_simulator()
    try:
        proj = load_project(name)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=f"project not found: {name}") from e
    sim.set_params(proj.vehicle_params())
    try:
        sim.set_model_type(proj.vehicle.model)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    try:
        sim.set_strategy(proj.controller.type)
    except KeyError as e:
        raise HTTPException(status_code=400, detail=f"unknown strategy in project: {e}") from e
    sim.set_scene(proj.scene.to_scene())
    sim.reset()
    return {
        "status": "loaded", "name": name,
        "strategy": sim.strategy_name,
        "model_type": sim.model_type,
        "disturbances": len(sim.scene.disturbances),
    }


@router.post("/projects/{name}")
async def save_project_endpoint(name: str, body: ProjectSave | None = None) -> dict[str, Any]:
    """Snapshot the current sim state (params + strategy + scene) as `<name>.yaml`."""
    from sim4wis.project.io import save_project
    from sim4wis.project.schema import ProjectFile, SceneSection

    sim = get_simulator()
    desc = body.description if body else ""
    proj = ProjectFile.from_runtime(
        name=name,
        description=desc,
        params=sim.params,
        strategy_name=sim.strategy_name,
        model_type=sim.model_type,
        scene=SceneSection(**sim.scene.serialize()),
    )
    path = save_project(name, proj)
    return {"status": "saved", "name": name, "path": str(path)}
