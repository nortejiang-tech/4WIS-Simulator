"""Study endpoints — the contract the CLI and the MCP server both speak.

    GET  /api/study/capabilities        models, metrics, strategies — the first call
    POST /api/study/dry-run             validate a spec + cost estimate, runs nothing
    POST /api/study/run                 start a study → {job_id}
    GET  /api/study/jobs                job list, newest first
    GET  /api/study/jobs/{job_id}       progress / result
    GET  /api/study                     stored studies
    GET  /api/study/{study_id}          one study's summary
    GET  /api/study/{study_id}/report   the rendered report (HTML)
    GET  /api/study/trace/{run_id}      downsampled channels for one run

Two deliberate shapes here.

Nothing returns traces by default. `/trace/{run_id}` requires an explicit
channel list and caps the sample count, because a single run is ~670 samples
across ~35 channels and a grid of them is far past any useful context window.

The study job runs in a worker thread, exactly as `/api/batch` does, so the
realtime simulator loop stays responsive while a grid executes. A human can
keep driving while an agent runs a study.
"""

from __future__ import annotations

import asyncio
import logging
import time
import uuid
from dataclasses import dataclass, field
from typing import Any

from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import FileResponse

from sim4wis.controller.registry import available_strategies
from sim4wis.experiment import store as run_store
from sim4wis.study import runner, store
from sim4wis.study.metrics import describe_metrics
from sim4wis.study.spec import StudySpec
from sim4wis.vehicle.model_registry import model_infos

logger = logging.getLogger(__name__)
router = APIRouter()

#: Hard cap on samples returned by /trace — a guard, not a preference.
MAX_TRACE_POINTS = 2000


# ---------------------------------------------------------------------------
# Job registry
# ---------------------------------------------------------------------------


@dataclass
class StudyJob:
    id: str
    study: str
    total: int
    status: str = "running"              # running | done | error
    error: str = ""
    study_id: str | None = None
    result: dict[str, Any] | None = None
    started_at: float = field(default_factory=time.time)
    finished_at: float | None = None

    def to_dict(self, *, with_result: bool = True) -> dict[str, Any]:
        d: dict[str, Any] = {
            "job_id": self.id,
            "study": self.study,
            "status": self.status,
            "total": self.total,
            "error": self.error,
            "study_id": self.study_id,
            "elapsed_s": round((self.finished_at or time.time()) - self.started_at, 3),
        }
        if with_result:
            d["result"] = self.result
        return d


_JOBS: dict[str, StudyJob] = {}
_MAX_JOBS = 50


def _trim_jobs() -> None:
    if len(_JOBS) <= _MAX_JOBS:
        return
    for jid in sorted(_JOBS, key=lambda k: _JOBS[k].started_at)[: len(_JOBS) - _MAX_JOBS]:
        if _JOBS[jid].status != "running":
            _JOBS.pop(jid, None)


# ---------------------------------------------------------------------------
# Capabilities
# ---------------------------------------------------------------------------


@router.get("/study/capabilities")
async def capabilities() -> dict[str, Any]:
    """What can be asked, of which model, with which metrics.

    The intended first call for any agent: everything needed to write a valid
    spec, in one response, instead of discovering it endpoint by endpoint.
    """
    return {
        "models": [
            {"id": m.id, "label": m.label, "layer": m.layer, "description": m.description}
            for m in model_infos()
        ],
        "metrics": describe_metrics(),
        "strategies": available_strategies(),
        "steer_kinds": ["constant", "step", "ramp", "sine", "sweep", "dlc"],
        "steer_units": ["normalized", "front_deg"],
        "channels": runner.expected_channels(),
        "limits": {"max_trace_points": MAX_TRACE_POINTS},
        "notes": [
            "metrics[].requires names the model capability a metric needs; "
            "the envelope guard that enforces it is not in place yet, so a metric "
            "asked of a model that cannot produce it will come back missing rather "
            "than refused.",
            "solver axes (sweep[*].solve_for) are declared in the schema but not "
            "implemented yet.",
        ],
    }


# ---------------------------------------------------------------------------
# Dry run / run
# ---------------------------------------------------------------------------


@router.post("/study/dry-run")
async def dry_run_endpoint(body: StudySpec) -> dict[str, Any]:
    return runner.dry_run(body)


@router.post("/study/run")
async def run_endpoint(body: StudySpec) -> dict[str, Any]:
    check = runner.dry_run(body)
    if not check["ok"]:
        raise HTTPException(status_code=400, detail={"problems": check["problems"]})

    job = StudyJob(id=uuid.uuid4().hex[:12], study=body.study, total=check["runs"])
    _JOBS[job.id] = job
    _trim_jobs()

    async def _worker() -> None:
        try:
            study_id, result = await asyncio.to_thread(runner.run_sync, body)
            job.study_id = study_id
            job.result = result.to_summary()
            job.status = "done"
        except Exception as exc:                     # noqa: BLE001 - surfaced to the caller
            logger.exception("Study %s failed", body.study)
            job.status = "error"
            job.error = f"{type(exc).__name__}: {exc}"
        finally:
            job.finished_at = time.time()

    asyncio.create_task(_worker(), name=f"study:{job.id}")
    return {"job_id": job.id, "total": job.total, "estimate": check}


@router.get("/study/jobs")
async def list_jobs() -> dict[str, Any]:
    jobs = sorted(_JOBS.values(), key=lambda j: j.started_at, reverse=True)
    return {"jobs": [j.to_dict(with_result=False) for j in jobs]}


@router.get("/study/jobs/{job_id}")
async def get_job(job_id: str) -> dict[str, Any]:
    job = _JOBS.get(job_id)
    if job is None:
        raise HTTPException(status_code=404, detail=f"unknown study job: {job_id}")
    return job.to_dict()


# ---------------------------------------------------------------------------
# Stored studies
# ---------------------------------------------------------------------------


@router.get("/study")
async def list_studies() -> dict[str, Any]:
    return {"studies": store.list_studies()}


@router.get("/study/trace/{run_id}")
async def trace(
    run_id: str,
    channels: str = Query(..., description="comma-separated channel names — required"),
    max_points: int = Query(200, ge=2, le=MAX_TRACE_POINTS),
) -> dict[str, Any]:
    """Downsampled channels for one run.

    `channels` has no default on purpose. Returning everything is the failure
    mode this endpoint exists to prevent.
    """
    names = [c.strip() for c in channels.split(",") if c.strip()]
    if not names:
        raise HTTPException(status_code=400, detail="channels must name at least one channel")
    try:
        meta = run_store.load_run_meta(run_id)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=f"unknown run: {run_id}") from e

    n_samples = int(meta.get("n_samples") or 0)
    decimate = max(1, -(-n_samples // max_points)) if n_samples else 1
    data = run_store.load_run_channels(run_id, names, decimate=decimate)
    missing = [n for n in names if n not in data]
    return {
        "run_id": run_id,
        "decimate": decimate,
        "n_samples": len(data.get("t", [])),
        "of_total": n_samples,
        "missing_channels": missing,
        "channels": data,
    }


@router.get("/study/{study_id}")
async def get_study(study_id: str) -> dict[str, Any]:
    try:
        return store.load(study_id)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=f"unknown study: {study_id}") from e


@router.get("/study/{study_id}/report")
async def get_report(study_id: str) -> FileResponse:
    from sim4wis.paths import studies_dir

    path = studies_dir() / study_id / "report.html"
    if not path.is_file():
        summary = None
        try:
            summary = store.load(study_id)
        except FileNotFoundError:
            raise HTTPException(status_code=404, detail=f"unknown study: {study_id}") from None
        out = (summary or {}).get("report_path")
        if out:
            from pathlib import Path

            p = Path(out)
            if p.is_file():
                return FileResponse(p, media_type="text/html")
        raise HTTPException(status_code=404, detail=f"study {study_id} has no report")
    return FileResponse(path, media_type="text/html")
