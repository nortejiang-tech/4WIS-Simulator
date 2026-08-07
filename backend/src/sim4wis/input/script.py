"""ScriptRunner — execute a Script against a Simulator over wall clock.

Design notes:
    * The runner is a single async task. It pulls actions in time order, sleeps
      to the next action's `t`, and dispatches to a per-action handler.
    * Ramps (throttle/steer/brake) span a duration; during a ramp the runner
      ticks at ~50 Hz to interpolate the driver values.
    * `wait_until` blocks until a predicate becomes true (polled at ~50 Hz):
        - `t`             until simulator state time exceeds value
        - `distance`      until distance travelled from script start exceeds value
        - `speed_below`   until |v| < value (used for "come to a stop")
    * The script overrides keyboard input *only while running*. On stop it
      leaves the driver state untouched (frontend's keyboard layer takes over
      again automatically on its next push).
    * If `loop` is true, the runner repeats the action list indefinitely
      (with t reset each loop) until stopped.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import dataclass

from sim4wis.core.simulator import Simulator
from sim4wis.input.action_schema import Action, Script

logger = logging.getLogger(__name__)

TICK_HZ = 50.0
TICK_DT = 1.0 / TICK_HZ


@dataclass
class ScriptStatus:
    running: bool
    current_action_idx: int
    t_in_script: float
    script_name: str
    loop_count: int


class ScriptRunner:
    def __init__(self, sim: Simulator) -> None:
        self.sim = sim
        self.script: Script | None = None
        self._task: asyncio.Task | None = None
        self._stop_event: asyncio.Event = asyncio.Event()
        self._current_idx: int = -1
        self._t_start_wall: float = 0.0
        self._t_start_sim: float = 0.0
        self._pose_at_start: tuple[float, float] = (0.0, 0.0)
        self._loop_count: int = 0

    # ---- public API --------------------------------------------------------

    def load(self, script: Script) -> None:
        if self.is_running:
            raise RuntimeError("script is running — stop it first")
        self.script = script
        self._current_idx = -1
        logger.info("Script loaded: %s (%d actions)", script.name, len(script.actions))

    async def start(self) -> None:
        if self.script is None:
            raise RuntimeError("no script loaded")
        if self.is_running:
            return
        self._stop_event.clear()
        self._task = asyncio.create_task(self._run(), name=f"script:{self.script.name}")

    async def stop(self) -> None:
        self._stop_event.set()
        if self._task is not None:
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
            self._task = None

    @property
    def is_running(self) -> bool:
        return self._task is not None and not self._task.done()

    def status(self) -> ScriptStatus:
        return ScriptStatus(
            running=self.is_running,
            current_action_idx=self._current_idx,
            t_in_script=(time.time() - self._t_start_wall) if self.is_running else 0.0,
            script_name=self.script.name if self.script else "",
            loop_count=self._loop_count,
        )

    # ---- runner loop -------------------------------------------------------

    async def _run(self) -> None:
        assert self.script is not None
        self._loop_count = 0
        try:
            while True:
                await self._run_once()
                self._loop_count += 1
                if not self.script.loop:
                    break
                if self._stop_event.is_set():
                    break
        finally:
            self._current_idx = -1

    async def _run_once(self) -> None:
        assert self.script is not None
        self._t_start_wall = time.time()
        self._t_start_sim = self.sim.model.state.t
        self._pose_at_start = (self.sim.model.state.x, self.sim.model.state.y)

        for i, action in enumerate(self.script.actions):
            self._current_idx = i
            await self._sleep_until_t(action.t)
            if self._stop_event.is_set():
                return
            try:
                await self._dispatch(action)
            except Exception:
                logger.exception("Script action %d (%s) failed", i, action.action)
            if self._stop_event.is_set():
                return

    async def _sleep_until_t(self, t: float) -> None:
        """Wait wall-clock until elapsed seconds since start == t."""
        target_wall = self._t_start_wall + t
        while True:
            remaining = target_wall - time.time()
            if remaining <= 0:
                return
            try:
                await asyncio.wait_for(self._stop_event.wait(), timeout=min(remaining, TICK_DT))
                return  # stopped
            except asyncio.TimeoutError:
                continue

    async def _dispatch(self, action: Action) -> None:
        name = action.action
        args = action.args
        if name == "drive":
            self.sim.set_driver(throttle=float(args["throttle"]), steering=float(args["steering"]))
        elif name == "set_strategy":
            self.sim.set_strategy(str(args["name"]))
        elif name == "set_mode_params":
            self.sim.set_driver(mode_params=dict(args["params"]))
        elif name == "throttle_ramp":
            await self._ramp("throttle", float(args["from_"]), float(args["to"]), float(args["duration"]))
        elif name == "steer_ramp":
            await self._ramp("steering", float(args["from_"]), float(args["to"]), float(args["duration"]))
        elif name == "brake":
            await self._brake(float(args["duration"]))
        elif name == "wait_until":
            await self._wait_until(args)
        elif name == "reset":
            self.sim.reset()
            self._pose_at_start = (self.sim.model.state.x, self.sim.model.state.y)
        elif name == "stop":
            self._stop_event.set()
        else:
            logger.warning("Unknown action %r — ignored", name)

    async def _ramp(self, channel: str, v0: float, v1: float, duration: float) -> None:
        if duration <= 0:
            self.sim.set_driver(**{channel: v1})
            return
        t_start = time.time()
        steps = max(1, int(duration / TICK_DT))
        for k in range(1, steps + 1):
            if self._stop_event.is_set():
                return
            alpha = k / steps
            value = v0 + (v1 - v0) * alpha
            self.sim.set_driver(**{channel: value})
            elapsed = time.time() - t_start
            wait = (k * duration / steps) - elapsed
            if wait > 0:
                try:
                    await asyncio.wait_for(self._stop_event.wait(), timeout=wait)
                    return
                except asyncio.TimeoutError:
                    pass

    async def _brake(self, duration: float) -> None:
        # Full friction-brake pedal (gear stays in D; reverse is a separate
        # concern). This is the new brake channel, not the legacy throttle=-1.
        # The throttle is lifted too: the legacy `throttle=-1` form implied it,
        # and scripts written against that expect `brake` to mean coast-and-stop
        # rather than "hold the previous throttle and fight it".
        self.sim.set_driver(throttle=0.0, brake=1.0)
        # Wait either duration or until vehicle stopped, whichever sooner.
        t_start = time.time()
        while time.time() - t_start < duration:
            if self._stop_event.is_set():
                return
            v = self.sim.model.state.vx
            if abs(v) < 0.05:
                break
            try:
                await asyncio.wait_for(self._stop_event.wait(), timeout=TICK_DT)
                return
            except asyncio.TimeoutError:
                pass
        self.sim.set_driver(brake=0.0)

    async def _wait_until(self, args: dict) -> None:
        t_target = args.get("t")
        dist_target = args.get("distance")
        speed_below = args.get("speed_below")
        while True:
            if self._stop_event.is_set():
                return
            s = self.sim.model.state
            if t_target is not None and s.t >= float(t_target):
                return
            if dist_target is not None:
                dx = s.x - self._pose_at_start[0]
                dy = s.y - self._pose_at_start[1]
                if math.hypot(dx, dy) >= float(dist_target):
                    return
            if speed_below is not None and abs(s.vx) < float(speed_below):
                return
            try:
                await asyncio.wait_for(self._stop_event.wait(), timeout=TICK_DT)
                return
            except asyncio.TimeoutError:
                pass
