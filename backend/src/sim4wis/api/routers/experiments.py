"""Experiment / batch / run endpoints (platform refactor Phase A).

    GET    /api/experiments               list definitions
    GET    /api/experiments/{name}        full definition
    POST   /api/experiments/{name}        save definition (body = Experiment)
    DELETE /api/experiments/{name}
    GET    /api/maneuver-templates        steer-profile kinds + path templates

    POST   /api/batch                     start a batch → {job_id}
    GET    /api/batch                     list jobs (newest first)
    GET    /api/batch/{job_id}            progress + per-run KPIs
    POST   /api/batch/{job_id}/cancel

    GET    /api/runs                      run metadata, newest first
    GET    /api/runs/{run_id}             full meta.json
    GET    /api/runs/{run_id}/data        channels (?channels=a,b&decimate=N)
    DELETE /api/runs/{run_id}
"""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import FileResponse, Response, StreamingResponse
from pydantic import BaseModel, Field

from sim4wis.experiment import batch, store
from sim4wis.experiment.schema import Experiment

router = APIRouter()


# ---- experiment definitions -------------------------------------------------


@router.get("/experiments")
async def list_experiments_endpoint() -> dict[str, Any]:
    return {"experiments": store.list_experiments()}


@router.get("/experiments/{name}")
async def get_experiment(name: str) -> dict[str, Any]:
    try:
        return store.load_experiment(name).model_dump(mode="json")
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=f"experiment not found: {name}") from e
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e


@router.post("/experiments/{name}")
async def save_experiment_endpoint(name: str, body: Experiment) -> dict[str, Any]:
    body = body.model_copy(update={"name": name})
    try:
        path = store.save_experiment(body)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    return {"status": "saved", "name": name, "path": str(path)}


@router.delete("/experiments/{name}")
async def delete_experiment_endpoint(name: str) -> dict[str, Any]:
    try:
        ok = store.delete_experiment(name)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    if not ok:
        raise HTTPException(status_code=404, detail=f"experiment not found: {name}")
    return {"status": "deleted", "name": name}


@router.get("/maneuver-templates")
async def maneuver_templates() -> dict[str, Any]:
    from sim4wis.controller.path import list_templates
    return {
        "steer_kinds": ["constant", "step", "ramp", "sine", "sweep", "dlc"],
        "path_templates": list_templates(),
    }


# ---- batch -------------------------------------------------------------------


class BatchRequest(BaseModel):
    """Either a stored experiment name or an inline definition, plus variants."""

    name: str | None = None
    experiment: Experiment | None = None
    variants: list[dict[str, Any]] = Field(default_factory=list)


@router.post("/batch")
async def start_batch_endpoint(body: BatchRequest) -> dict[str, Any]:
    if body.experiment is not None:
        exp = body.experiment
    elif body.name:
        try:
            exp = store.load_experiment(body.name)
        except FileNotFoundError as e:
            raise HTTPException(status_code=404, detail=f"experiment not found: {body.name}") from e
    else:
        raise HTTPException(status_code=400, detail="provide `name` or inline `experiment`")
    if not exp.maneuver.steps:
        raise HTTPException(status_code=400, detail="experiment has no maneuver steps")
    try:
        job_id = await batch.start_batch(exp, body.variants)
    except (ValueError, KeyError, IndexError) as e:
        raise HTTPException(status_code=400, detail=f"variant expansion failed: {e}") from e
    return {"job_id": job_id}


@router.get("/batch")
async def list_batch_jobs() -> dict[str, Any]:
    return {"jobs": batch.list_jobs()}


@router.get("/batch/{job_id}")
async def batch_status(job_id: str) -> dict[str, Any]:
    job = batch.get_job(job_id)
    if job is None:
        raise HTTPException(status_code=404, detail=f"job not found: {job_id}")
    return job.to_dict()


@router.post("/batch/{job_id}/cancel")
async def batch_cancel(job_id: str) -> dict[str, Any]:
    if not batch.cancel_job(job_id):
        raise HTTPException(status_code=409, detail="job not running")
    return {"status": "cancelling", "job_id": job_id}


# ---- runs ----------------------------------------------------------------------


@router.get("/runs")
async def list_runs_endpoint() -> dict[str, Any]:
    return {"runs": store.list_runs()}


@router.get("/runs/{run_id}")
async def run_meta(run_id: str) -> dict[str, Any]:
    try:
        return store.load_run_meta(run_id)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=f"run not found: {run_id}") from e
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e


@router.get("/runs/{run_id}/data")
async def run_data(
    run_id: str,
    channels: str | None = Query(None, description="comma-separated channel names"),
    decimate: int = Query(1, ge=1, le=1000),
) -> dict[str, Any]:
    names = [c.strip() for c in channels.split(",") if c.strip()] if channels else None
    try:
        return store.load_run_channels(run_id, names, decimate)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=f"run not found: {run_id}") from e
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e


@router.delete("/runs/{run_id}")
async def delete_run_endpoint(run_id: str) -> dict[str, Any]:
    try:
        ok = store.delete_run(run_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    if not ok:
        raise HTTPException(status_code=404, detail=f"run not found: {run_id}")
    return {"status": "deleted", "run_id": run_id}


def _artifact(fn, *args):
    try: return fn(*args)
    except FileNotFoundError as exc:
        raise HTTPException(404, detail="run not found") from exc
    except ValueError as exc:
        raise HTTPException(422, detail=str(exc)) from exc


@router.get("/runs/{run_id}/artifacts")
def run_artifacts(run_id: str):
    from sim4wis.experiment.artifacts import manifest
    return _artifact(manifest, run_id)


@router.get("/runs/{run_id}/export.csv")
def run_csv(run_id: str):
    from sim4wis.experiment.artifacts import run_file
    path = _artifact(run_file, run_id, "data.csv")
    return FileResponse(path, media_type="text/csv", filename=f"{run_id}.csv")


@router.get("/runs/{run_id}/export.json")
def run_json(run_id: str):
    from sim4wis.experiment.artifacts import json_rows, run_file
    _artifact(run_file, run_id, "data.csv")
    return StreamingResponse(json_rows(run_id), media_type="application/json",
                             headers={"Content-Disposition": f'attachment; filename="{run_id}.json"'})


@router.get("/runs/{run_id}/plot.svg")
def run_plot(run_id: str, kind: str = Query("chart", pattern="^(chart|trajectory)$"), channel: str = "yaw_rate"):
    from sim4wis.experiment.artifacts import plot_svg
    return Response(_artifact(plot_svg, run_id, kind, channel), media_type="image/svg+xml")


@router.get("/runs/{run_id}/bundle.zip")
def run_bundle(run_id: str):
    import json
    import tempfile
    import zipfile
    from sim4wis.experiment.artifacts import manifest, plot_svg, run_file
    info = _artifact(manifest, run_id)
    output = tempfile.TemporaryFile()
    try:
        with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            archive.write(run_file(run_id, "data.csv"), "data.csv")
            archive.write(run_file(run_id, "meta.json"), "meta.json")
            archive.writestr("manifest.json", json.dumps(info, ensure_ascii=False, indent=2))
            for kind in ("trajectory", "chart"):
                try: archive.writestr(f"{kind}.svg", plot_svg(run_id, kind))
                except ValueError as exc:
                    archive.writestr(f"{kind}-unavailable.txt", str(exc))
        output.seek(0)
    except Exception:
        output.close()
        raise
    def chunks():
        try:
            while chunk := output.read(65536): yield chunk
        finally: output.close()
    return StreamingResponse(chunks(), media_type="application/zip",
                             headers={"Content-Disposition": f'attachment; filename="{run_id}.zip"'})
