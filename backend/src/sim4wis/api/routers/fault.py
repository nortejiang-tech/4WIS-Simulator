"""Live fault API, with atomic schedule replacement and legacy single-fault CRUD."""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, ConfigDict
from sim4wis.core.simulator import get_simulator
from sim4wis.fault import FaultConfig
from sim4wis.fault.schema import FaultSchedule, FaultSpec

router = APIRouter()

class ActivePatch(BaseModel):
    model_config = ConfigDict(extra="forbid")
    active: bool

@router.get("/faults")
async def list_faults():
    return {"faults": get_simulator().fault_injector.status()["faults"]}

@router.get("/faults/schedule")
async def get_schedule():
    return get_simulator().fault_injector.status()

@router.post("/faults/schedule")
async def set_schedule(body: FaultSchedule):
    sim = get_simulator()
    try:
        sim.fault_injector.replace_schedule(body.faults, seed=body.seed, model=sim.model, now=sim.model.state.t)
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from exc
    return sim.fault_injector.status()

@router.post("/faults", status_code=201)
async def add_fault(body: FaultSpec):
    sim = get_simulator()
    cfg = FaultConfig.from_spec(body, len(sim.fault_injector.list_faults()))
    if body.id is None:
        import uuid
        cfg.id = uuid.uuid4().hex[:12]
    try:
        sim.fault_injector.validate_model([body], sim.model)
        sim.fault_injector.add(cfg)
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from exc
    return cfg.to_dict()

@router.patch("/faults/{fault_id}")
async def patch_fault(fault_id: str, body: ActivePatch):
    fi = get_simulator().fault_injector
    if not fi.set_active(fault_id, body.active):
        raise HTTPException(404, f"fault {fault_id!r} not found")
    return next(f.to_dict() for f in fi.list_faults() if f.id == fault_id)

@router.delete("/faults/{fault_id}")
async def delete_fault(fault_id: str):
    if not get_simulator().fault_injector.remove(fault_id):
        raise HTTPException(404, f"fault {fault_id!r} not found")
    return {"deleted": fault_id}

@router.delete("/faults")
async def clear_faults():
    get_simulator().fault_injector.clear()
    return {"cleared": True}
