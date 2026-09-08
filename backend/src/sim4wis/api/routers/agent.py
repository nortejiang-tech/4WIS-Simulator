"""OpenAPI-visible v1 step/observe API. Each session owns its simulator."""
from __future__ import annotations

from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import JSONResponse

from sim4wis.agent.session import (CONTRACT, MAX_SESSIONS, MAX_STEPS, STRATEGIES, TTL_S,
    REGISTRY_LOCK, SESSIONS, SessionConfig, SessionError, StepRequest, create_session, get_session)
from sim4wis.experiment.artifacts import manifest

router = APIRouter(prefix="/agent", tags=["agent-v1"])


def _call(fn, *args):
    try: return fn(*args)
    except SessionError as exc:
        raise HTTPException(exc.status, {"code": exc.code, "message": str(exc)}) from exc
    except (ValueError, TypeError, KeyError, FileNotFoundError) as exc:
        raise HTTPException(422, {"code": "invalid_config", "message": str(exc)}) from exc


@router.get("/capabilities")
def capabilities():
    from sim4wis.core.telemetry import AVAILABLE_CHANNELS
    from sim4wis.experiment.artifacts import channel_unit
    return {"contract": CONTRACT, "execution": "isolated fixed-step; GUI input never changes an Agent session",
            "models": ["kinematic", "simplified_dynamic", "multibody"], "strategies": sorted(STRATEGIES),
            "limits": {"max_steps_per_request": 1000, "max_steps_per_session": MAX_STEPS,
                       "max_sessions": MAX_SESSIONS, "idle_ttl_s": TTL_S, "max_preview_points": 1000},
            "units": {"time": "s", "length": "m", "angle": "rad", "speed": "m/s", "wheel_order": ["FL","FR","RL","RR"]},
            "schemas": {"create": SessionConfig.model_json_schema(), "step": StepRequest.model_json_schema()},
            "telemetry_channels": [{"name": name, "unit": channel_unit(name)} for name in AVAILABLE_CHANNELS],
            "outputs": ["state", "preview", "run_id", "full_csv", "full_json", "bundle_zip", "trajectory_svg", "chart_svg"],
            "workflow": ["create_session", "observe_session", "step_session", "export_session", "close_session"],
            "notes": ["Every step request supplies complete controls; omitted controls return to defaults.",
                      "Retry with the SAME request_id and identical body after an uncertain response.",
                      "expected_revision must match the last observed revision.",
                      "speed_target_ms and front_angle_rad bypass driver aids; pedal controls retain them.",
                      "Export records every accepted integration step; preview is explicitly sampled.",
                      "Kinematic force/acceleration outputs are unsupported placeholders; inspect state.accel.valid.",
                      "Sessions are in memory; export before closing or server restart. Long jobs use study/batch."]}


@router.post("/sessions", status_code=201)
def create(body: SessionConfig):
    return _call(create_session, body).observe()


@router.get("/sessions")
def list_sessions():
    with REGISTRY_LOCK: sessions = list(SESSIONS.values())
    items = []
    for session in sessions:
        with session.lock: items.append(session.observe())
    return {"contract": CONTRACT, "sessions": items}


@router.get("/sessions/{session_id}")
def observe(session_id: str):
    session = _call(get_session, session_id)
    with session.lock: return session.observe()


@router.post("/sessions/{session_id}/step")
def step(session_id: str, body: StepRequest):
    result = _call(_call(get_session, session_id).step, body)
    return JSONResponse(result, status_code=422 if result["status"] == "failed" else 200)


@router.get("/sessions/{session_id}/preview")
def preview(session_id: str, max_points: int = Query(300, ge=2, le=1000)):
    import math
    session = _call(get_session, session_id)
    with session.lock:
        n = len(session.t)
        indexes = sorted({round(i * (n-1) / (min(n,max_points)-1)) for i in range(min(n,max_points))}) if n > 1 else list(range(n))
        data = {"t": [session.t[i] for i in indexes]}
        for name in ("pose_x", "pose_y", "pose_psi", "vx", "vy", "yaw_rate", "delta_fl", "delta_fr", "delta_rl", "delta_rr"):
            data[name] = [v if math.isfinite(v) else None for i in indexes for v in [session.channels[name][i]]]
        return {"session_id": session_id, "revision": session.revision, "total_samples": n,
                "returned_samples": len(indexes), "decimated": len(indexes) < n, "data": data}


@router.post("/sessions/{session_id}/export")
def export(session_id: str):
    run_id = _call(_call(get_session, session_id).export)
    return manifest(run_id)


@router.delete("/sessions/{session_id}")
def close(session_id: str):
    session = _call(get_session, session_id)
    with session.lock:
        session.status = "closed"
        with REGISTRY_LOCK: SESSIONS.pop(session_id, None)
    return {"session_id": session_id, "status": "closed"}
