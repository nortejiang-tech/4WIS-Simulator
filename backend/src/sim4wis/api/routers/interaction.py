"""Visible lifecycle controls for the shared manual/script driving station."""
from typing import Literal

from fastapi import APIRouter
from pydantic import BaseModel, ConfigDict

from sim4wis.core.simulator import get_simulator

router = APIRouter(prefix="/interaction")


class Action(BaseModel):
    model_config = ConfigDict(extra="forbid")
    action: Literal["pause", "resume", "stop"]


@router.get("/status")
async def status():
    return get_simulator().interaction_status()


@router.post("/control")
async def control(body: Action):
    sim = get_simulator()
    if body.action == "stop":
        await sim.script_runner.stop()
        sim.release_input()
    else:
        sim.paused = body.action == "pause"
    return sim.interaction_status()
