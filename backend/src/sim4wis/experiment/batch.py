"""Batch execution — run an Experiment (plus variants) headless, store artifacts.

A *variant* is `{label, overrides}` where overrides is a flat dict of
dotted-path → value applied to the experiment's JSON form, e.g.:

    {"label": "80kmh",  "overrides": {"maneuver.steps.0.speed_kmh": 80}}
    {"label": "rws",    "overrides": {"strategy": "rear_wheel_steer"}}
    {"label": "light",  "overrides": {"vehicle.overrides.mass": 2400}}

No variants → the experiment runs once. Runs execute sequentially in a worker
thread (`asyncio.to_thread`) so the realtime simulator loop stays responsive;
each individual run is CPU-bound but short (a 10 s maneuver ≈ 0.2–0.5 s).
Progress is polled via the in-process job registry (REST `GET /api/batch/{id}`).
"""

from __future__ import annotations

import asyncio
import copy
import logging
import time
import uuid
from dataclasses import dataclass, field
from typing import Any

from sim4wis.experiment.kpi import compute_kpis
from sim4wis.experiment.schema import Experiment
from sim4wis.experiment.session import run_experiment
from sim4wis.experiment.store import save_run

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Variant expansion
# ---------------------------------------------------------------------------


def _set_dotted(d: dict[str, Any] | list, path: str, value: Any) -> None:
    """Set `value` at a dotted path inside nested dicts/lists (in place)."""
    parts = path.split(".")
    cur: Any = d
    for p in parts[:-1]:
        if isinstance(cur, list):
            cur = cur[int(p)]
        elif isinstance(cur, dict):
            if p not in cur or cur[p] is None:
                cur[p] = {}
            cur = cur[p]
        else:
            raise ValueError(f"cannot traverse {path!r} at {p!r}")
    last = parts[-1]
    if isinstance(cur, list):
        cur[int(last)] = value
    elif isinstance(cur, dict):
        cur[last] = value
    else:
        raise ValueError(f"cannot set {path!r}")


def expand_variants(
    exp: Experiment,
    variants: list[dict[str, Any]] | None,
) -> list[tuple[str, Experiment]]:
    """Return [(label, concrete Experiment)] — base experiment if no variants."""
    if not variants:
        return [(exp.name, exp)]
    out: list[tuple[str, Experiment]] = []
    base = exp.model_dump(mode="json")
    for i, v in enumerate(variants):
        label = str(v.get("label") or f"variant_{i}")
        payload = copy.deepcopy(base)
        for path, value in (v.get("overrides") or {}).items():
            _set_dotted(payload, str(path), value)
        out.append((label, Experiment.model_validate(payload)))
    return out


# ---------------------------------------------------------------------------
# Job registry + runner
# ---------------------------------------------------------------------------


@dataclass
class BatchJob:
    id: str
    total: int
    done: int = 0
    status: str = "running"          # running | done | error | cancelled
    error: str = ""
    current_label: str = ""
    run_ids: list[str] = field(default_factory=list)
    labels: list[str] = field(default_factory=list)
    kpis: list[dict[str, Any]] = field(default_factory=list)
    started_at: float = field(default_factory=time.time)
    finished_at: float | None = None
    _cancel: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "job_id": self.id,
            "status": self.status,
            "total": self.total,
            "done": self.done,
            "current_label": self.current_label,
            "error": self.error,
            "elapsed_s": round((self.finished_at or time.time()) - self.started_at, 3),
            "runs": [
                {"run_id": rid, "label": lbl, "kpis": k}
                for rid, lbl, k in zip(self.run_ids, self.labels, self.kpis)
            ],
        }


_JOBS: dict[str, BatchJob] = {}
_MAX_JOBS = 50


def get_job(job_id: str) -> BatchJob | None:
    return _JOBS.get(job_id)


def list_jobs() -> list[dict[str, Any]]:
    return [j.to_dict() for j in sorted(_JOBS.values(), key=lambda j: j.started_at, reverse=True)]


def cancel_job(job_id: str) -> bool:
    job = _JOBS.get(job_id)
    if job is None or job.status != "running":
        return False
    job._cancel = True
    return True


def _run_one(label: str, exp: Experiment, job_id: str) -> tuple[str, dict[str, Any]]:
    """Worker-thread body: execute + KPI + persist. Returns (run_id, kpis)."""
    result = run_experiment(exp)
    kpis = compute_kpis(result, exp)
    run_id = save_run(result, label=label, kpis=kpis, job_id=job_id)
    return run_id, kpis


async def start_batch(exp: Experiment, variants: list[dict[str, Any]] | None) -> str:
    """Expand variants, launch the background task, return the job id."""
    items = expand_variants(exp, variants)
    job = BatchJob(id=uuid.uuid4().hex[:12], total=len(items))
    _JOBS[job.id] = job
    # Trim the registry so long sessions don't grow unboundedly.
    if len(_JOBS) > _MAX_JOBS:
        for jid in sorted(_JOBS, key=lambda k: _JOBS[k].started_at)[: len(_JOBS) - _MAX_JOBS]:
            if _JOBS[jid].status != "running":
                _JOBS.pop(jid, None)

    async def _worker() -> None:
        try:
            for label, item in items:
                if job._cancel:
                    job.status = "cancelled"
                    break
                job.current_label = label
                run_id, kpis = await asyncio.to_thread(_run_one, label, item, job.id)
                job.run_ids.append(run_id)
                job.labels.append(label)
                job.kpis.append(kpis)
                job.done += 1
            if job.status == "running":
                job.status = "done"
        except Exception as exc:
            logger.exception("Batch job %s failed", job.id)
            job.status = "error"
            job.error = f"{type(exc).__name__}: {exc}"
        finally:
            job.current_label = ""
            job.finished_at = time.time()

    asyncio.create_task(_worker(), name=f"batch:{job.id}")
    return job.id


def run_batch_sync(exp: Experiment, variants: list[dict[str, Any]] | None) -> BatchJob:
    """Synchronous batch execution (tests / CLI — no event loop required)."""
    items = expand_variants(exp, variants)
    job = BatchJob(id=uuid.uuid4().hex[:12], total=len(items))
    _JOBS[job.id] = job
    try:
        for label, item in items:
            job.current_label = label
            run_id, kpis = _run_one(label, item, job.id)
            job.run_ids.append(run_id)
            job.labels.append(label)
            job.kpis.append(kpis)
            job.done += 1
        job.status = "done"
    except Exception as exc:
        logger.exception("Batch job %s failed", job.id)
        job.status = "error"
        job.error = f"{type(exc).__name__}: {exc}"
    finally:
        job.current_label = ""
        job.finished_at = time.time()
    return job
