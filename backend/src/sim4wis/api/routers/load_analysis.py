"""Steering load-analysis endpoints."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter
from pydantic import BaseModel, Field

from sim4wis.core.simulator import get_simulator
from sim4wis.project.params_codec import params_from_dict
from sim4wis.vehicle.geometry import wheel_angle_from_rack_travel
from sim4wis.vehicle.load_analysis import sweep_load_analysis, sweep_sensitivity

router = APIRouter()


class LoadSweepRequest(BaseModel):
    params: dict[str, Any] | None = None
    speeds: list[float] = Field(default_factory=lambda: [0.0, 2.0, 5.0, 10.0, 20.0])
    angles: list[float] = Field(default_factory=list)
    wheel_index: int = Field(0, ge=0, le=3)
    mode: str = "single_wheel"
    mu: float = Field(0.85, gt=0.05, le=2.0)


@router.post("/load-analysis/sweep")
async def load_analysis_sweep(body: LoadSweepRequest) -> dict[str, Any]:
    sim = get_simulator()
    params = params_from_dict(body.params, sim.params) if body.params else sim.params
    return sweep_load_analysis(
        params,
        speeds=body.speeds,
        angles=body.angles,
        wheel_index=body.wheel_index,
        mode=body.mode,
        mu=body.mu,
    )


class SensitivityRequest(BaseModel):
    params: dict[str, Any] | None = None
    vary_param: str
    values: list[float]
    speed: float = Field(8.0, ge=0.0, le=120.0)
    angles: list[float] = Field(default_factory=list)
    wheel_index: int = Field(0, ge=0, le=3)
    mu: float = Field(0.85, gt=0.05, le=2.0)


@router.post("/load-analysis/sensitivity")
async def load_analysis_sensitivity(body: SensitivityRequest) -> dict[str, Any]:
    sim = get_simulator()
    params = params_from_dict(body.params, sim.params) if body.params else sim.params
    return sweep_sensitivity(
        params,
        vary_param=body.vary_param,
        values=body.values,
        speed=body.speed,
        angles=body.angles,
        wheel_index=body.wheel_index,
        mu=body.mu,
    )


class RackSolveRequest(BaseModel):
    params: dict[str, Any] | None = None
    wheel_index: int = Field(0, ge=0, le=3)
    rack_travels: list[float] = Field(default_factory=list)
    hint_delta: float = 0.0


@router.post("/load-analysis/rack-solve")
async def load_analysis_rack_solve(body: RackSolveRequest) -> dict[str, Any]:
    """Solve δ (and linkage angles) for a list of rack travels — shared with the
    frontend mechanism diagram so the geometry can never disagree across the wall."""
    sim = get_simulator()
    params = params_from_dict(body.params, sim.params) if body.params else sim.params
    hint = float(body.hint_delta)
    solutions: list[dict[str, Any]] = []
    travels = body.rack_travels[:512]
    for t in travels:
        sol = wheel_angle_from_rack_travel(
            float(t),
            params.steering_geometry,
            int(body.wheel_index),
            hint_delta=hint,
        )
        if sol["valid"]:
            hint = float(sol["delta"])
        solutions.append(sol)
    return {"wheel_index": int(body.wheel_index), "solutions": solutions}
