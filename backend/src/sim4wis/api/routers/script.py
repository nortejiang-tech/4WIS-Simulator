"""Action-script endpoints — load / start / stop / status / library."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from sim4wis.core.simulator import get_simulator

router = APIRouter()


@router.get("/script/status")
async def script_status() -> dict[str, Any]:
    sim = get_simulator()
    st = sim.script_runner.status()
    return {
        "running": st.running,
        "current_action_idx": st.current_action_idx,
        "t_in_script": st.t_in_script,
        "script_name": st.script_name,
        "loop_count": st.loop_count,
    }


class ScriptLoad(BaseModel):
    yaml: str


@router.post("/script/load")
async def script_load(body: ScriptLoad) -> dict[str, Any]:
    import yaml
    from sim4wis.input.action_schema import Script

    try:
        data = yaml.safe_load(body.yaml) or {}
        script = Script.from_dict(data if isinstance(data, dict) else {"script": data})
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"script parse error: {e}") from e
    sim = get_simulator()
    try:
        sim.script_runner.load(script)
    except RuntimeError as e:
        raise HTTPException(status_code=409, detail=str(e)) from e

    # If the script declares a reference path/cone layout, lay it down so the
    # maneuver's ground markers are visible (solves "工况无地面标志").
    path_set = False
    if script.path_template and script.path_template.get("name"):
        from sim4wis.controller.path import plan_from_template, set_active_plan
        try:
            plan = plan_from_template(
                str(script.path_template["name"]),
                script.path_template.get("params") or {},
            )
            set_active_plan(plan)
            path_set = True
        except (KeyError, TypeError):
            pass  # bad template ref shouldn't block script loading

    return {
        "status": "loaded",
        "script_name": script.name,
        "actions": len(script.actions),
        "path_template": path_set,
    }


@router.post("/script/start")
async def script_start() -> dict[str, str]:
    sim = get_simulator()
    try:
        await sim.script_runner.start()
    except RuntimeError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    return {"status": "started"}


@router.post("/script/stop")
async def script_stop() -> dict[str, str]:
    sim = get_simulator()
    await sim.script_runner.stop()
    return {"status": "stopped"}


@router.get("/script/library")
async def script_library() -> dict[str, list[str]]:
    """List YAML scripts shipped under the data dir's scripts_lib/."""
    from sim4wis.paths import scripts_lib_dir
    root = scripts_lib_dir()
    if not root.exists():
        return {"scripts": []}
    return {"scripts": sorted(p.stem for p in root.glob("*.yaml"))}


@router.get("/script/library/{name}")
async def script_library_get(name: str) -> dict[str, str]:
    from sim4wis.paths import scripts_lib_dir
    path = scripts_lib_dir() / f"{name}.yaml"
    if not path.exists():
        raise HTTPException(status_code=404, detail=f"script not found: {name}")
    return {"name": name, "yaml": path.read_text(encoding="utf-8")}
