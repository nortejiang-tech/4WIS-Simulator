"""Simulator — owns the model, strategy, driver input, and the fixed-step loop.

This is the single application-wide instance accessed by the REST and
WebSocket endpoints. It runs an asyncio task that integrates the model at
`dt_sim` (default 5 ms / 200 Hz) and broadcasts a serialised state snapshot
to all subscribers at `dt_push` (default ~16.7 ms / 60 Hz).

Concurrency model:
    * One background task per Simulator (the loop).
    * One asyncio.Queue per subscribed WebSocket client. The loop performs
      a non-blocking put; if the queue is full (slow client), the oldest
      message is dropped — never block the sim loop.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from typing import Any

from sim4wis.controller.registry import available_strategies, make_strategy
from sim4wis.core.state import (
    ControlCommand,
    DriverInput,
    EnvironmentState,
    VehicleParams,
)
from sim4wis.environment.disturbance import Scene
from sim4wis.fault import FaultInjector
from sim4wis.recorder.buffer import Recorder, RecorderConfig
from sim4wis.core.derived import update_derived_outputs
from sim4wis.vehicle.base import VehicleModel
from sim4wis.vehicle.model_registry import make_vehicle_model
# ScriptRunner is imported lazily inside Simulator to avoid an import cycle
# (script.py imports Simulator).

logger = logging.getLogger(__name__)


class Simulator:
    """Application-wide simulator instance."""

    def __init__(
        self,
        params: VehicleParams | None = None,
        *,
        strategy_name: str = "ideal_ackermann",
        model_type: str = "kinematic",
        dt_sim: float = 0.005,    # 200 Hz internal
        dt_push: float = 1.0 / 60.0,  # ~60 Hz to clients
    ) -> None:
        self.params = params or VehicleParams()
        self.model_type = model_type
        self.model: VehicleModel = self._make_model(self.params, model_type)
        self.driver = DriverInput()
        self.scene = Scene()
        self.env = EnvironmentState(scene=self.scene, mu=self.scene.base_mu)
        self.dt_sim = dt_sim
        self.dt_push = dt_push
        self.strategy_name = strategy_name
        self.strategy = make_strategy(strategy_name, self.params)
        self.last_cmd: ControlCommand = ControlCommand.zero()
        self.subscribers: set[asyncio.Queue[dict]] = set()
        self.fault_injector = FaultInjector()
        self.recorder = Recorder(RecorderConfig(rate_hz=1.0 / dt_push))
        from sim4wis.input.script import ScriptRunner
        self.script_runner = ScriptRunner(self)
        self._task: asyncio.Task | None = None
        self._running = False

    # ---- lifecycle ----------------------------------------------------------

    async def start(self) -> None:
        if self._task is not None:
            return
        self._running = True
        self._task = asyncio.create_task(self._loop(), name="sim4wis-loop")
        logger.info("Simulator started (dt_sim=%g, dt_push=%g)", self.dt_sim, self.dt_push)

    async def stop(self) -> None:
        self._running = False
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
            self._task = None
        logger.info("Simulator stopped")

    # ---- subscriptions ------------------------------------------------------

    def subscribe(self, maxsize: int = 8) -> asyncio.Queue[dict]:
        q: asyncio.Queue[dict] = asyncio.Queue(maxsize=maxsize)
        self.subscribers.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue[dict]) -> None:
        self.subscribers.discard(q)

    # ---- mutation API used by HTTP / WS handlers ---------------------------

    def set_driver(self, *, throttle: float | None = None,
                   steering: float | None = None,
                   handbrake: int | None = None,
                   mode_params: dict[str, Any] | None = None) -> None:
        d = self.driver
        if throttle is not None:
            d.throttle = float(max(-1.0, min(1.0, throttle)))
        if steering is not None:
            d.steering = float(max(-1.0, min(1.0, steering)))
        if handbrake is not None:
            d.handbrake = int(handbrake)
        if mode_params is not None:
            d.mode_params = dict(mode_params)

    def set_strategy(self, name: str) -> None:
        if name not in available_strategies():
            raise KeyError(name)
        self.strategy = make_strategy(name, self.params)
        self.strategy_name = name
        # Clear the wheel-speed servo integrators so the new strategy doesn't
        # inherit accumulated integral torque (avoids a torque jump at switch).
        for servo in getattr(self.model, "servos", []):
            servo.reset()
        logger.info("Strategy switched to %s", name)

    def set_params(self, params: VehicleParams) -> None:
        """Replace vehicle parameters and reset the model in place."""
        self.params = params
        self.model = self._make_model(self.params, self.model_type)
        self.strategy = make_strategy(self.strategy_name, self.params)

    def set_model_type(self, model_type: str) -> None:
        """Switch the integration model."""
        self.model_type = model_type
        self.model = self._make_model(self.params, self.model_type)

    @staticmethod
    def _make_model(params: VehicleParams, model_type: str) -> VehicleModel:
        return make_vehicle_model(params, model_type)

    def set_scene(self, scene: Scene) -> None:
        """Replace the scene (disturbance list) atomically."""
        self.scene = scene
        self.env = EnvironmentState(scene=scene, mu=scene.base_mu)

    def reset(self) -> None:
        self.model.reset()

    # ---- main loop ----------------------------------------------------------

    async def _loop(self) -> None:
        loop = asyncio.get_event_loop()
        push_interval_steps = max(1, int(round(self.dt_push / self.dt_sim)))
        step_count = 0
        next_time = loop.time()

        while self._running:
            # 1) Control & integrate
            try:
                cmd = self.strategy.compute(self.driver, self.model.state)
                if self.fault_injector.has_active:
                    from sim4wis.core.state import ControlCommand
                    cmd = ControlCommand(
                        delta_cmd=self.fault_injector.apply_to_cmd(cmd.delta_cmd),
                        wheel_speed_cmd=cmd.wheel_speed_cmd,
                        icr_target_body=cmd.icr_target_body,
                    )
                self.last_cmd = cmd
                self.model.step(self.dt_sim, cmd, self.env)
                # Per-wheel steering centre + split-rack force chain (shared
                # with the headless batch SimSession).
                update_derived_outputs(self.model.state, self.params)
            except Exception:
                logger.exception("Simulator loop step failed")
                # Recover by resetting the model so we don't get NaN-locked.
                self.model.reset()

            step_count += 1

            # 2) Push downsampled state
            if step_count % push_interval_steps == 0:
                # Recorder writes happen at the push rate (60 Hz default).
                self.recorder.write(self.model.state, self.last_cmd, self.strategy_name)
                msg = self._serialize_state()
                stale_queues = []
                for q in self.subscribers:
                    if q.full():
                        # Drop the oldest to make room for the freshest sample.
                        try:
                            q.get_nowait()
                        except asyncio.QueueEmpty:
                            pass
                    try:
                        q.put_nowait(msg)
                    except asyncio.QueueFull:
                        stale_queues.append(q)
                for q in stale_queues:
                    self.subscribers.discard(q)

            # 3) Pace to wall clock so simulation runs in real time
            next_time += self.dt_sim
            now = loop.time()
            sleep = next_time - now
            if sleep > 0:
                try:
                    await asyncio.sleep(sleep)
                except asyncio.CancelledError:
                    return
            else:
                # We fell behind — drop accumulated lag, don't try to catch up
                # (running multiple steps without sleep would just compound).
                next_time = now

    def _serialize_state(self) -> dict:
        s = self.model.state
        cmd = self.last_cmd

        # Per-wheel effective μ at the current pose (works for both models —
        # it's a geometric query on the scene, independent of the integrator).
        cp, sp = math.cos(s.psi), math.sin(s.psi)
        wpb = s.wheel_pos_body

        def _wheel_mu(i: int) -> float:
            if self.scene is None:
                return float(self.env.mu)
            wx = s.x + cp * float(wpb[i, 0]) - sp * float(wpb[i, 1])
            wy = s.y + sp * float(wpb[i, 0]) + cp * float(wpb[i, 1])
            return float(self.scene.effective_mu([wx, wy]))

        def _nan2none(v) -> Any:
            try:
                x = float(v)
            except (TypeError, ValueError):
                return None
            if x != x or x in (float("inf"), float("-inf")):  # NaN/Inf
                return None
            return x

        reported_delta = self.fault_injector.apply_to_reported(s.delta)

        side_forces, force_source = _side_force_body_y(self.model, s.delta)
        side_left = float(side_forces[0] + side_forces[2])
        side_right = float(side_forces[1] + side_forces[3])

        return {
            "type": "state",
            "t": s.t,
            "wall": time.time(),
            "strategy": self.strategy_name,
            "fault_active": self.fault_injector.has_active,
            "driver": {
                "throttle": self.driver.throttle,
                "steering": self.driver.steering,
                "handbrake": self.driver.handbrake,
                "mode_params": dict(self.driver.mode_params),
            },
            "pose": {"x": s.x, "y": s.y, "psi": s.psi},
            "attitude": {"z": s.z, "roll": s.roll, "pitch": s.pitch},
            "velocity": {"vx": s.vx, "vy": s.vy, "yaw_rate": s.yaw_rate},
            "wheels": [
                {
                    "delta": float(reported_delta[i]),
                    "delta_true": float(s.delta[i]),
                    "delta_cmd": float(cmd.delta_cmd[i]),
                    "omega": float(s.wheel_omega[i]),
                    "fz": float(s.fz[i]),
                    "torque_steer": float(s.torque_steer[i]),
                    "susp_defl": float(s.susp_defl[i]),
                    "slip_alpha": _slip(self.model, "slip_alpha", i),
                    "slip_kappa": _slip(self.model, "slip_kappa", i),
                    "mu": _wheel_mu(i),
                    "pos_body": [
                        float(s.wheel_pos_body[i, 0]),
                        float(s.wheel_pos_body[i, 1]),
                    ],
                    "icr_body": [
                        _nan2none(s.wheel_icr_body[i, 0]),
                        _nan2none(s.wheel_icr_body[i, 1]),
                    ],
                    "icr_dev": _nan2none(s.wheel_icr_dev[i]),
                    "rack_force": float(s.rack_force[i]),
                    "motor_torque_demand": float(s.motor_torque_demand[i]),
                    "linkage_arm_tie_angle": _nan2none(s.linkage_arm_tie_angle[i]),
                    "linkage_tie_rack_angle": _nan2none(s.linkage_tie_rack_angle[i]),
                    "linkage_efficiency": float(s.linkage_efficiency[i]),
                    "tire_fx": _force(self.model, "tire_fx", i),
                    "tire_fy": _force(self.model, "tire_fy", i),
                    "side_force_body_y": float(side_forces[i]),
                    "force_source": force_source,
                }
                for i in range(4)
            ],
            "side_force_summary": {
                "left": side_left,
                "right": side_right,
                "total": side_left + side_right,
                "source": force_source,
            },
            "model_type": self.model_type,
            "icr_vehicle_body": [
                _nan2none(s.vehicle_icr_body[0]),
                _nan2none(s.vehicle_icr_body[1]),
            ],
            "icr_target_body": [
                _nan2none(cmd.icr_target_body[0]),
                _nan2none(cmd.icr_target_body[1]),
            ],
            "params": {
                "wheelbase": self.params.wheelbase,
                "track_front": self.params.track_front,
                "track_rear": self.params.track_rear,
                "tire_radius": self.params.tire_radius,
                "steer_limit": self.params.steer_limit,
                "v_max": self.params.v_max,
            },
            "scene": self.scene.serialize() if self.scene else None,
            "path_version": _path_version(),
            "scenario_version": _scenario_version(),
        }


# ----------------------------------------------------------------------------
# Application-wide singleton accessor
# ----------------------------------------------------------------------------

def _path_version() -> int:
    """Active reference-path version (frontend re-fetches the path on change)."""
    from sim4wis.controller.path import get_version
    return get_version()


def _scenario_version() -> int:
    """Active scenario version (frontend re-fetches the scenario geometry)."""
    from sim4wis.environment.scenario import get_version
    return get_version()


def _slip(model: VehicleModel, attr: str, i: int) -> float:
    """Read a slip diagnostic array from the dynamic model; 0.0 for kinematic."""
    arr = getattr(model, attr, None)
    if arr is None:
        return 0.0
    try:
        v = float(arr[i])
    except (TypeError, IndexError, ValueError):
        return 0.0
    if not (v == v):  # NaN
        return 0.0
    return v


def _force(model: VehicleModel, attr: str, i: int) -> float:
    """Read a tyre-force diagnostic array from dynamic models; 0 for kinematic."""
    arr = getattr(model, attr, None)
    if arr is None:
        return 0.0
    try:
        v = float(arr[i])
    except (TypeError, IndexError, ValueError):
        return 0.0
    if not (v == v):
        return 0.0
    return v


def _side_force_body_y(model: VehicleModel, delta) -> tuple[list[float], str]:
    """Per-wheel lateral force in body-Y, plus provenance label."""
    if not hasattr(model, "tire_fx") or not hasattr(model, "tire_fy"):
        return [0.0, 0.0, 0.0, 0.0], "kinematic_estimate"
    out: list[float] = []
    for i in range(4):
        fx = _force(model, "tire_fx", i)
        fy = _force(model, "tire_fy", i)
        di = float(delta[i])
        out.append(float(math.sin(di) * fx + math.cos(di) * fy))
    return out, "tire_model"


_sim: Simulator | None = None


def get_simulator() -> Simulator:
    """Return the application-wide Simulator (lazily constructed)."""
    global _sim
    if _sim is None:
        _sim = Simulator()
    return _sim


def reset_simulator() -> None:
    """Mainly for tests — drop the singleton so the next get_simulator() rebuilds."""
    global _sim
    _sim = None
