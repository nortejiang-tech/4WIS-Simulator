"""REST API aggregator — one APIRouter per resource under `routers/`.

All URLs keep their historical `/api/...` shape; this module only mounts the
per-resource routers (split from the former 470-line single file).
"""

from __future__ import annotations

from fastapi import APIRouter

from sim4wis.api.routers import (
    agent,
    experiments,
    interaction,
    fault,
    load_analysis,
    meta,
    model_demo,
    params,
    path,
    projects,
    recording,
    scenario,
    scene,
    script,
    study,
    targets,
    vehicle_profiles,
)

router = APIRouter(prefix="/api", tags=["api"])
router.include_router(meta.router)
router.include_router(agent.router)
router.include_router(interaction.router)
router.include_router(fault.router)
router.include_router(load_analysis.router)
router.include_router(model_demo.router)
router.include_router(params.router)
router.include_router(scene.router)
router.include_router(scenario.router)
router.include_router(projects.router)
router.include_router(path.router)
router.include_router(script.router)
router.include_router(recording.router)
router.include_router(vehicle_profiles.router)
router.include_router(experiments.router)
router.include_router(study.router)
router.include_router(targets.router)
