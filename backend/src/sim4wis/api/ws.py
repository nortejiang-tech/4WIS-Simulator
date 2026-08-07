"""WebSocket endpoints for live state streaming and driver input.

Wire protocol — JSON messages over a single bidirectional connection:

    Server → Client
        { "type": "state", "t": …, "pose": {…}, "velocity": {…},
          "wheels": [...], "icr_vehicle_body": [x,y]|null,
          "icr_target_body": [x,y]|null, "strategy": str, … }

    Client → Server
        { "type": "driver",   "throttle": float, "steering": float,
                              "mode_params": {…}? }
        { "type": "strategy", "name": str }
        { "type": "reset" }

Both directions are JSON-encoded text frames. A simulator instance is
shared application-wide via `get_simulator()`.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from sim4wis.core.simulator import get_simulator

logger = logging.getLogger(__name__)
router = APIRouter()


@router.websocket("/ws/state")
async def stream_state(ws: WebSocket) -> None:
    await ws.accept()
    sim = get_simulator()
    queue = sim.subscribe(maxsize=8)
    logger.info("WS /ws/state connected — subscribers=%d", len(sim.subscribers))

    async def writer() -> None:
        """Drain the queue and write to the socket as fast as the client allows."""
        try:
            while True:
                msg = await queue.get()
                await ws.send_json(msg)
        except (WebSocketDisconnect, RuntimeError):
            pass

    async def reader() -> None:
        """Read client commands and apply them to the shared Simulator.

        A malformed frame is logged and skipped (one bad message must not kill
        the connection); only a disconnect ends the loop.
        """
        import json
        while True:
            try:
                raw = await ws.receive_text()
            except (WebSocketDisconnect, RuntimeError):
                return
            try:
                data = json.loads(raw)
            except (ValueError, TypeError):
                logger.warning("WS: skipping malformed frame (%d bytes)", len(raw))
                continue
            if isinstance(data, dict) and data.get("type") == "ping":
                # Heartbeat — echo so the client can detect half-open links.
                try:
                    await ws.send_json({"type": "pong", "t": data.get("t")})
                except (WebSocketDisconnect, RuntimeError):
                    return
                continue
            try:
                _apply_client_message(data, sim)
            except Exception:
                logger.exception("WS: failed applying client message")

    write_task = asyncio.create_task(writer(), name="ws-writer")
    read_task = asyncio.create_task(reader(), name="ws-reader")
    try:
        done, pending = await asyncio.wait(
            {write_task, read_task},
            return_when=asyncio.FIRST_COMPLETED,
        )
        for t in pending:
            t.cancel()
        for t in done:
            exc = t.exception()
            if exc and not isinstance(exc, (WebSocketDisconnect, asyncio.CancelledError)):
                logger.warning("WS task ended with %r", exc)
    finally:
        sim.unsubscribe(queue)
        logger.info("WS /ws/state disconnected — subscribers=%d", len(sim.subscribers))
        try:
            await ws.close()
        except Exception:
            pass


def _apply_client_message(data: Any, sim) -> None:
    """Apply a single decoded client message to the simulator."""
    if not isinstance(data, dict):
        return
    mtype = data.get("type")
    if mtype == "driver":
        sim.set_driver(
            throttle=data.get("throttle"),
            brake=data.get("brake"),
            gear=data.get("gear"),
            steering=data.get("steering"),
            handbrake=data.get("handbrake"),
            mode_params=data.get("mode_params"),
        )
    elif mtype == "strategy":
        name = data.get("name")
        if isinstance(name, str):
            try:
                sim.set_strategy(name)
            except KeyError:
                logger.warning("WS: unknown strategy %r", name)
    elif mtype == "reset":
        sim.reset()
    elif mtype == "steer_cmd":
        from sim4wis.controller.user_js import UserJsStrategy
        if isinstance(sim.strategy, UserJsStrategy):
            sim.strategy.set_steer_cmd(
                float(data.get("fl", 0.0)),
                float(data.get("fr", 0.0)),
                float(data.get("rl", 0.0)),
                float(data.get("rr", 0.0)),
            )
    else:
        logger.debug("WS: ignoring unknown message type %r", mtype)
