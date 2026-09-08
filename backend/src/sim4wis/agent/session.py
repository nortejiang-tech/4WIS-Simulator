"""Bounded step/observe sessions; no wall-clock pacing or shared GUI state.

Each request supplies a complete control value held for an integer step count.
Revision checks prevent stale decisions; request ids make transport retries safe.
On numerical failure the partial trace remains inspectable and the session is
failed. No implicit reset can turn a failed experiment into a successful one.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import threading
import time
import uuid
from dataclasses import asdict
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from sim4wis.core.state import DriverInput
from sim4wis.core.step import advance_model
from sim4wis.core.telemetry import AVAILABLE_CHANNELS, sample_channels
from sim4wis.experiment import store
from sim4wis.experiment.kpi import compute_kpis
from sim4wis.experiment.schema import Experiment
from sim4wis.experiment.session import WHEELS, RunResult, SimSession

CONTRACT = "4wis.agent.v1"
MAX_STEPS = 20000
MAX_SESSIONS = 4
TTL_S = 3600
STRATEGIES = {"ackermann", "ideal_ackermann", "rear_wheel_steer", "crab", "zero_radius",
              "manual_wheel", "manual_body", "follow_trajectory", "fault_reconfig"}


class SessionError(Exception):
    def __init__(self, code: str, message: str, status: int = 409):
        super().__init__(message)
        self.code, self.status = code, status


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class SessionConfig(StrictModel):
    label: str = Field("Agent session", max_length=120)
    experiment: Experiment

    @model_validator(mode="after")
    def check(self):
        exp = self.experiment
        json.dumps(exp.model_dump(mode="json"), allow_nan=False)
        if exp.vehicle.profile and not re.fullmatch(r"[\w-]{1,64}", exp.vehicle.profile):
            raise ValueError("vehicle.profile must be a library name")
        if exp.strategy not in STRATEGIES:
            raise ValueError("isolated sessions support built-in structured strategies only")
        if exp.strategy == "follow_trajectory" and exp.path is None:
            raise ValueError("follow_trajectory requires an explicit isolated path")
        if exp.model_type not in {"kinematic", "simplified_dynamic", "multibody"}:
            raise ValueError("unknown model_type")
        if not math.isfinite(exp.dt) or not .0005 <= exp.dt <= .02:
            raise ValueError("session dt must be within 0.0005..0.02 s")
        if exp.maneuver.steps:
            raise ValueError("session control comes from step requests; use batch for maneuver steps")
        transient = {"speed_target_ms", "steer_raw_rad", "wheel_norm", "vx_frac", "vy_frac", "yaw_frac"}
        if transient.intersection(exp.mode_params):
            raise ValueError("dynamic control values belong in step.control, not experiment.mode_params")
        # Refuse typos rather than params_codec silently discarding them.
        from sim4wis.core.state import VehicleParams
        def validate_tree(value, reference, path="vehicle.overrides"):
            if not isinstance(value, dict):
                raise ValueError(f"{path} must be an object")
            for key, item in value.items():
                if key not in reference:
                    raise ValueError(f"unknown field: {path}.{key}")
                ref = reference[key]
                if isinstance(ref, dict):
                    validate_tree(item, ref, f"{path}.{key}")
                elif isinstance(item, (float, int)) and not math.isfinite(item):
                    raise ValueError(f"non-finite field: {path}.{key}")
        validate_tree(exp.vehicle.overrides, asdict(VehicleParams()))
        return self


class Control(StrictModel):
    throttle: float = Field(0, ge=0, le=1)
    brake: float = Field(0, ge=0, le=1)
    steering: float = Field(0, ge=-1, le=1)
    gear: Literal[-1, 0, 1] = 1
    handbrake: Literal[0, 1] = 0
    speed_target_ms: float | None = Field(None, ge=-60, le=60)
    front_angle_rad: float | None = Field(None, ge=-1.5, le=1.5)
    wheel_norm: list[float] | None = Field(None, min_length=4, max_length=4)
    body_fraction: list[float] | None = Field(None, min_length=3, max_length=3)

    @model_validator(mode="after")
    def ranges(self):
        for values in (self.wheel_norm, self.body_fraction):
            if values and any(not math.isfinite(x) or abs(x) > 1 for x in values):
                raise ValueError("wheel_norm/body_fraction values must be finite within [-1,1]")
        return self


class StepRequest(StrictModel):
    request_id: str = Field(min_length=1, max_length=80, pattern=r"^[A-Za-z0-9_.:-]+$")
    expected_revision: int = Field(ge=0, strict=True)
    steps: int = Field(ge=1, le=1000, strict=True)
    control: Control


class AgentSession:
    def __init__(self, config: SessionConfig):
        self.config = config
        self.id = uuid.uuid4().hex
        self.sim = SimSession(config.experiment)
        p = self.sim.params
        if not (p.mass > 0 and p.inertia_z > 0 and p.wheelbase > 0 and p.tire_radius > 0):
            raise ValueError("mass, inertia_z, wheelbase and tire_radius must be positive")
        self.driver = DriverInput(mode_params=dict(config.experiment.mode_params))
        self.revision = 0
        self.n_steps = 0
        self.status = "ready"
        self.error = None
        self.t: list[float] = []
        self.channels = {c: [] for c in AVAILABLE_CHANNELS}
        self.receipts: dict[str, tuple[str, dict]] = {}
        self.command_log: list[dict] = []
        self.exported: tuple[int, str] | None = None
        self.created = self.touched = time.time()
        self.lock = threading.RLock()

    def observe(self):
        def clean(value):
            if isinstance(value, dict): return {key: clean(item) for key, item in value.items()}
            if isinstance(value, list): return [clean(item) for item in value]
            if isinstance(value, float) and not math.isfinite(value): return None
            return value
        return clean(self._observe())

    def _observe(self):
        s = self.sim.model.state
        return {"contract": CONTRACT, "session_id": self.id, "label": self.config.label,
                "status": self.status, "revision": self.revision, "steps": self.n_steps,
                "samples": len(self.t), "dt": self.config.experiment.dt,
                "remaining_steps": MAX_STEPS - self.n_steps, "error": self.error,
                "model_type": self.config.experiment.model_type, "strategy": self.config.experiment.strategy,
                "state": {"t": float(s.t), "pose": {"x": float(s.x), "y": float(s.y), "psi": float(s.psi)},
                          "telemetry": {name: values[-1] for name, values in self.channels.items()} if self.t else {},
                          "velocity": {"vx": float(s.vx), "vy": float(s.vy), "yaw_rate": float(s.yaw_rate)},
                          "accel": {"ax": float(s.ax), "ay": float(s.ay), "valid": bool(s.grip_valid)},
                          "wheels": [{"name": w.upper(), "delta": float(s.delta[i]), "fz": float(s.fz[i])}
                                     for i, w in enumerate(WHEELS)]},
                "run_id": self.exported[1] if self.exported and self.exported[0] == self.revision else None,
                "validity": "internal verification only; no independent vehicle correlation"}

    def step(self, request: StepRequest):
        digest = hashlib.sha256(request.model_dump_json().encode()).hexdigest()
        with self.lock:
            self.touched = time.time()
            if request.request_id in self.receipts:
                old, response = self.receipts[request.request_id]
                if old != digest:
                    raise SessionError("request_conflict", "request_id already used with a different body")
                return {**response, "replayed": True}
            if self.status != "ready":
                raise SessionError("session_failed", "create a new session after a numerical failure")
            if request.expected_revision != self.revision:
                raise SessionError("revision_conflict", f"expected current revision {self.revision}")
            if self.n_steps + request.steps > MAX_STEPS:
                raise SessionError("step_limit", "session sample limit reached; export then create a new session")
            c = request.control
            strategy = self.config.experiment.strategy
            if c.speed_target_ms is not None and strategy in {"manual_body", "zero_radius"}:
                raise SessionError("unsupported_control", "this strategy has no longitudinal speed_target_ms input", 422)
            if c.front_angle_rad is not None and strategy not in {"ideal_ackermann", "rear_wheel_steer", "fault_reconfig"}:
                raise SessionError("unsupported_control", "front_angle_rad is unsupported by this strategy", 422)
            if c.wheel_norm is not None and strategy != "manual_wheel":
                raise SessionError("unsupported_control", "wheel_norm requires manual_wheel", 422)
            if c.body_fraction is not None and strategy != "manual_body":
                raise SessionError("unsupported_control", "body_fraction requires manual_body", 422)
            mode = dict(self.config.experiment.mode_params)
            if c.speed_target_ms is not None: mode["speed_target_ms"] = c.speed_target_ms
            if c.front_angle_rad is not None: mode["steer_raw_rad"] = c.front_angle_rad
            if c.wheel_norm is not None: mode["wheel_norm"] = c.wheel_norm
            if c.body_fraction is not None:
                mode.update(zip(("vx_frac", "vy_frac", "yaw_frac"), c.body_fraction))
            self.driver = DriverInput(throttle=c.throttle, brake=c.brake, steering=c.steering,
                                      gear=c.gear, handbrake=c.handbrake, mode_params=mode)
            before = self.n_steps
            try:
                for _ in range(request.steps):
                    cmd = self.sim.strategy.compute(self.driver, self.sim.model.state, self.config.experiment.dt)
                    self.sim._apply_faults(cmd)
                    s = advance_model(self.sim.model, self.sim.params, self.driver, cmd,
                                      self.config.experiment.dt, self.sim.env)
                    if not all(math.isfinite(float(x)) for x in (s.x, s.y, s.psi, s.vx, s.vy, s.yaw_rate)):
                        raise ArithmeticError("non-finite vehicle state")
                    sample = sample_channels(s, cmd, self.driver, self.sim.model)
                    self.t.append(float(s.t))
                    for name, value in sample.items():
                        self.channels[name].append(value)
                    self.n_steps += 1
            except Exception as exc:
                self.status, self.error = "failed", f"{type(exc).__name__}: {exc}"
            self.revision += 1
            self.command_log.append({**request.model_dump(mode="json"), "completed_steps": self.n_steps - before,
                                     "status": self.status})
            response = {**self.observe(), "completed_steps": self.n_steps - before, "replayed": False}
            self.receipts[request.request_id] = (digest, response)
            return response

    def result(self):
        return RunResult(self.t, self.channels, self.n_steps * self.config.experiment.dt, self.n_steps,
                         {"experiment": self.config.experiment.model_dump(mode="json"),
                          "source": "agent", "contract": CONTRACT, "session_id": self.id,
                          "resolved_vehicle": asdict(self.sim.params),
                          "config_sha256": hashlib.sha256(self.config.model_dump_json().encode()).hexdigest(),
                          "revision": self.revision, "execution_status": self.status,
                          "error": self.error, "record_hz": 1 / self.config.experiment.dt,
                          "sampling": "every accepted integration step", "command_log": self.command_log})

    def export(self):
        with self.lock:
            if not self.t:
                raise SessionError("no_samples", "advance the session before exporting")
            if self.exported and self.exported[0] == self.revision:
                return self.exported[1]
            result = self.result()
            kpis = compute_kpis(result, self.config.experiment)
            if self.config.experiment.model_type == "kinematic":
                for key in ("steer_energy_nms", "rack_force_peak_n", "slip_alpha_peak_deg"):
                    kpis.pop(key, None)
            run_id = store.save_run(result, label=self.config.label, kpis=kpis)
            self.exported = (self.revision, run_id)
            return run_id


SESSIONS: dict[str, AgentSession] = {}
REGISTRY_LOCK = threading.Lock()


def create_session(config):
    with REGISTRY_LOCK:
        for key, session in list(SESSIONS.items()):
            if time.time() - session.touched > TTL_S and session.lock.acquire(blocking=False):
                try: SESSIONS.pop(key, None)
                finally: session.lock.release()
        if len(SESSIONS) >= MAX_SESSIONS:
            raise SessionError("session_limit", "close an existing session before creating another", 429)
        session = AgentSession(config)
        SESSIONS[session.id] = session
        return session


def get_session(session_id):
    with REGISTRY_LOCK:
        session = SESSIONS.get(session_id)
        if session is None:
            raise SessionError("unknown_session", "session not found or expired", 404)
        session.touched = time.time()
        return session
