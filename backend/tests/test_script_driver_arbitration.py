"""Tests — who owns the driver channel while an action script runs.

The keyboard input loop pushes driver state at 50 Hz even when idle, which
used to clobber every script command within 20 ms of it being issued — a
started script could not actually drive the car while the GUI was connected
(found by the H1 render-smoothness e2e, which needs real motion). The
contract now: while the script runner is running, client driver messages are
ignored; stopping the script returns the channel to the client.
"""

from __future__ import annotations

import asyncio
import yaml

import pytest

from sim4wis.api.ws import _apply_client_message
from sim4wis.core.simulator import get_simulator, reset_simulator
from sim4wis.input.action_schema import Script
from sim4wis.paths import scripts_lib_dir


@pytest.fixture()
def sim():
    reset_simulator()
    yield get_simulator()
    reset_simulator()


async def test_a_running_script_owns_the_driver_channel(sim):
    raw = yaml.safe_load(
        (scripts_lib_dir() / "double_lane_change.yaml").read_text(encoding="utf-8")
    )
    sim.script_runner.load(
        Script.from_dict(raw if isinstance(raw, dict) else {"script": raw})
    )
    await sim.script_runner.start()
    await asyncio.sleep(0.05)  # let the t=0 actions (strategy + drive) land
    try:
        assert sim.script_runner.status().running
        assert sim.driver.throttle > 0.0, "the script's drive action must land"

        # The idle keyboard push must not take the channel back.
        _apply_client_message({"type": "driver", "throttle": 0.0}, sim)
        assert sim.driver.throttle > 0.0, (
            "client driver message clobbered a running script's command"
        )
    finally:
        await sim.script_runner.stop()


async def test_stopping_the_script_returns_the_channel_to_the_client(sim):
    raw = yaml.safe_load(
        (scripts_lib_dir() / "double_lane_change.yaml").read_text(encoding="utf-8")
    )
    sim.script_runner.load(
        Script.from_dict(raw if isinstance(raw, dict) else {"script": raw})
    )
    await sim.script_runner.start()
    await asyncio.sleep(0.05)
    await sim.script_runner.stop()

    _apply_client_message({"type": "driver", "throttle": 0.0}, sim)
    assert sim.driver.throttle == 0.0
