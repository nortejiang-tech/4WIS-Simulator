"""Recording endpoints — start / stop / status / CSV export."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, Query
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

from sim4wis.core.simulator import get_simulator

router = APIRouter()


class RecordingStart(BaseModel):
    """Optional channel selection. If omitted, current channels are kept."""
    channels: list[str] | None = None


@router.post("/recording/start")
async def recording_start(body: RecordingStart | None = None) -> dict[str, Any]:
    sim = get_simulator()
    if body and body.channels is not None:
        sim.recorder.set_channels(body.channels)
    sim.recorder.start()
    return sim.recorder.status()


@router.post("/recording/stop")
async def recording_stop() -> dict[str, Any]:
    sim = get_simulator()
    sim.recorder.stop()
    return sim.recorder.status()


@router.get("/recording/status")
async def recording_status() -> dict[str, Any]:
    return get_simulator().recorder.status()


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
