"""Target-set and compliance endpoints.

    GET  /api/targets                   the library — every set, one line each
    GET  /api/targets/{ref}             one set in full, with sources
    POST /api/targets/{ref}/check       run an actuator sizing pass and judge it

`ref` is `name` (latest version) or `name@version`. Pinning the version is the
point of having one: a compliance record that quotes "eps_actuator" without a
version cannot be reproduced once the document moves.

The check endpoint is synchronous. A sizing pass is four short scenarios and
runs in a couple of seconds, unlike a study grid — there is nothing here worth
a job queue.
"""

from __future__ import annotations

import dataclasses

from fastapi import APIRouter, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel, ConfigDict

from sim4wis.core.state import VehicleParams
from sim4wis.steering.sizing import size_actuator
from sim4wis.targets import compliance, library, measure
from sim4wis.targets import report as target_report
from sim4wis.targets.spec import TargetError

router = APIRouter(tags=["targets"])


class CheckRequest(BaseModel):
    model_config = ConfigDict(protected_namespaces=())

    #: Vehicle profile to size against; the built-in default when absent.
    vehicle: str | None = None
    #: Return the rendered HTML table instead of JSON.
    html: bool = False


def _resolve(ref: str):
    try:
        return library.get(ref)
    except TargetError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e


@router.get("/targets")
async def list_targets() -> dict[str, object]:
    try:
        return {"sets": library.catalogue()}
    except TargetError as e:
        raise HTTPException(status_code=500, detail=str(e)) from e


@router.get("/targets/{ref}")
async def get_target_set(ref: str) -> dict[str, object]:
    ts = _resolve(ref)
    return {**ts.to_dict(), "ref": ts.ref, "digest": ts.digest()}


@router.post("/targets/{ref}/check")
async def check(ref: str, body: CheckRequest | None = None):
    ts = _resolve(ref)
    req = body or CheckRequest()

    if req.vehicle:
        from sim4wis.project.vehicle_profiles import load_profile_params

        try:
            params = load_profile_params(req.vehicle)
        except (FileNotFoundError, ValueError) as e:
            raise HTTPException(status_code=404, detail=str(e)) from e
    else:
        params = VehicleParams()

    # Sizing needs the plant; a request to check actuator targets is a request
    # to run it, not a reason to report every requirement as unevaluated.
    if not params.steering_system.enabled:
        params = dataclasses.replace(
            params,
            steering_system=dataclasses.replace(params.steering_system, enabled=True),
        )

    sizing = size_actuator(params)
    report = compliance.evaluate(ts, measure.from_sizing(sizing))
    if req.html:
        return HTMLResponse(target_report.render_html(
            report,
            notes=f"作动器选型走查 · 架构 {params.steering_system.architecture}",
        ))
    return {"compliance": report.to_dict(), "sizing": sizing.to_dict()}
