"""Recording endpoints — start / stop / status / CSV export."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

from sim4wis.core.simulator import get_simulator

router = APIRouter()


class RecordingStart(BaseModel):
    """Optional channel selection. If omitted, current channels are kept."""
    channels: list[str] | None = None
    full_rate: bool = False


@router.post("/recording/start")
async def recording_start(body: RecordingStart | None = None) -> dict[str, Any]:
    sim = get_simulator()
    if body and body.channels is not None:
        from sim4wis.recorder.buffer import AVAILABLE_CHANNELS
        unknown = set(body.channels) - AVAILABLE_CHANNELS.keys()
        if unknown:
            raise HTTPException(422, detail=f"unknown channels: {sorted(unknown)}")
        sim.recorder.set_channels(body.channels)
    sim.record_full_rate = body.full_rate if body else False
    actual_hz = 1 / (sim.dt_sim if sim.record_full_rate else max(1, round(sim.dt_push / sim.dt_sim)) * sim.dt_sim)
    sim.recorder.cfg.rate_hz = actual_hz
    sim.recorder.max_samples = int(sim.recorder.cfg.buffer_seconds * actual_hz) + 60
    from sim4wis.project.params_codec import params_to_dict
    sim.recording_meta = {"experiment": {"name": "realtime_recording", "strategy": sim.strategy_name,
                                        "model_type": sim.model_type},
                          "vehicle_at_start": params_to_dict(sim.params),
                          "sampling": "every integration step" if sim.record_full_rate else "state-stream cadence"}
    sim._saved_recording = None
    sim.recorder.start()
    return sim.recorder.status()


@router.post("/recording/stop")
async def recording_stop() -> dict[str, Any]:
    sim = get_simulator()
    sim.recorder.stop()
    return sim.recorder.status()


@router.post("/recording/clear")
async def recording_clear() -> dict[str, Any]:
    """Discard a stopped recording without changing the current simulation."""
    sim = get_simulator()
    try:
        sim.recorder.clear()
    except RuntimeError as exc:
        raise HTTPException(409, detail=str(exc)) from exc
    sim._saved_recording = None
    sim.recording_meta = {}
    return sim.recorder.status()


@router.get("/recording/status")
async def recording_status() -> dict[str, Any]:
    return get_simulator().recorder.status()


@router.post("/recording/save")
async def recording_save() -> dict[str, Any]:
    from sim4wis.experiment import store
    from sim4wis.experiment.artifacts import manifest
    from sim4wis.experiment.kpi import compute_recorded_kpis
    from sim4wis.project.params_codec import params_to_dict
    sim = get_simulator()
    if sim.recorder.is_recording:
        raise HTTPException(409, detail="stop recording before saving")
    if getattr(sim, "_saved_recording", None):
        return manifest(sim._saved_recording)
    try:
        end = params_to_dict(sim.params)
        result = sim.recorder.result_snapshot({**sim.recording_meta, "vehicle_at_end": end,
            "configuration_changed": end != sim.recording_meta.get("vehicle_at_start")})
    except ValueError as exc:
        raise HTTPException(409, detail=str(exc)) from exc
    run_id = store.save_run(result, label="实时驾驶 / 脚本录制", kpis=compute_recorded_kpis(result))
    sim._saved_recording = run_id
    return manifest(run_id)


@router.get("/recording/export.csv")
async def recording_export_csv(
    channels: str | None = Query(
        default=None,
        description="Comma-separated channel names. If omitted, all recorded channels are exported.",
    ),
    from_t: float | None = Query(default=None, description="Lower bound on simulation time t [s]."),
    to_t: float | None = Query(default=None, description="Upper bound on simulation time t [s]."),
    include_strategy: bool = Query(default=True),
) -> StreamingResponse:
    sim = get_simulator()
    selected = [c.strip() for c in channels.split(",") if c.strip()] if channels else None

    def stream():
        yield from sim.recorder.to_csv(
            channels=selected, from_t=from_t, to_t=to_t,
            include_strategy=include_strategy,
        )

    import time
    filename = f"sim4wis_{int(time.time())}.csv"
    return StreamingResponse(
        stream(),
        media_type="text/csv",
        headers={"Content-Disposition": f'attachment; filename="{filename}"'},
    )
