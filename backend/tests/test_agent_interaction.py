"""Cross-interface acceptance: isolation, retry semantics and exact artifacts."""
import asyncio
import csv
import hashlib
import io
import json
import zipfile

import numpy as np
import pytest
from fastapi.testclient import TestClient

from sim4wis.agent.session import SESSIONS, get_session
from sim4wis.api.ws import _apply_client_message
from sim4wis.core.simulator import Simulator, get_simulator, reset_simulator
from sim4wis.core.state import ControlCommand, DriverInput, VehicleState
from sim4wis.input.action_schema import Script
from sim4wis.main import create_app
from sim4wis.recorder.buffer import Recorder, RecorderConfig


@pytest.fixture
def client(tmp_path, monkeypatch):
    monkeypatch.setenv("SIM4WIS_RUNS_DIR", str(tmp_path / "runs"))
    SESSIONS.clear(); reset_simulator()
    # No lifespan: manual singleton stays stationary for the isolation oracle.
    with TestClient(create_app(), raise_server_exceptions=False) as api:
        sim = get_simulator()
        sim.paused = True
        yield api
    SESSIONS.clear(); reset_simulator()


def config(**experiment):
    return {"label": "acceptance", "experiment": {"name": "agent_test", "model_type": "simplified_dynamic",
            "strategy": "ideal_ackermann", **experiment}}


def create(client, **experiment):
    res = client.post("/api/agent/sessions", json=config(**experiment))
    assert res.status_code == 201, res.text
    return res.json()["session_id"]


def step(client, sid, request="a", revision=0, steps=100, **control):
    return client.post(f"/api/agent/sessions/{sid}/step", json={"request_id": request,
        "expected_revision": revision, "steps": steps, "control": {"throttle": .1, "steering": .05, **control}})


def test_discovery_and_openapi_describe_the_contract(client):
    caps = client.get("/api/agent/capabilities").json()
    assert caps["contract"] == "4wis.agent.v1"
    assert caps["units"]["wheel_order"] == ["FL", "FR", "RL", "RR"]
    assert caps["schemas"]["step"]["properties"]["steps"]["maximum"] == 1000
    assert "/api/agent/sessions/{session_id}/step" in client.get("/openapi.json").json()["paths"]


@pytest.mark.parametrize("model", ["kinematic", "simplified_dynamic", "multibody"])
def test_chunking_is_identical_and_human_instance_is_untouched(client, model):
    manual = get_simulator()
    manual.set_driver(throttle=.7, steering=-.2)
    before = (manual.model.state.t, manual.driver.throttle, manual.driver.steering)
    a, b = create(client, model_type=model), create(client, model_type=model)
    assert step(client, a).status_code == 200
    assert step(client, b, steps=40).status_code == 200
    assert step(client, b, request="b", revision=1, steps=60).status_code == 200
    for name, values in get_session(a).channels.items():
        np.testing.assert_array_equal(values, get_session(b).channels[name])
    assert get_session(a).t == get_session(b).t
    assert (manual.model.state.t, manual.driver.throttle, manual.driver.steering) == before


def test_retry_is_idempotent_and_stale_writers_are_refused(client):
    sid = create(client)
    original = step(client, sid).json()
    assert step(client, sid, "b", 1).status_code == 200
    replay = step(client, sid).json()
    assert replay["replayed"] and replay["state"] == original["state"]
    assert get_session(sid).n_steps == 200
    assert step(client, sid, steps=99).json()["detail"]["code"] == "request_conflict"
    assert step(client, sid, "c", 0).json()["detail"]["code"] == "revision_conflict"


@pytest.mark.parametrize("control", [{"throttle": 2}, {"steering": -2}, {"gear": 3},
                                      {"unknown_control": 1}, {"wheel_norm": [0,0,0,0]},
                                      {"body_fraction": [0,0,0]}])
def test_invalid_controls_never_advance(client, control):
    sid = create(client)
    res = step(client, sid, **control)
    assert res.status_code == 422, res.text
    assert get_session(sid).n_steps == 0


@pytest.mark.parametrize("overrides", [{"mass_typo": 5}, {"mass": 0}, {"inertia_z": 0}, {"tire_radius": 0}])
def test_bad_parameters_are_refused(client, overrides):
    res = client.post("/api/agent/sessions", json=config(vehicle={"overrides": overrides}))
    assert res.status_code == 422, res.text
    assert not SESSIONS


def test_dynamic_controls_cannot_latch_through_configuration(client):
    res = client.post("/api/agent/sessions", json=config(mode_params={"speed_target_ms": 10}))
    assert res.status_code == 422
    for strategy in ("manual_body", "zero_radius"):
        sid = create(client, strategy=strategy)
        assert step(client, sid, speed_target_ms=5).status_code == 422
        assert get_session(sid).n_steps == 0


def test_precise_artifact_roundtrip_and_immutable_run(client):
    sid = create(client)
    step(client, sid, steps=101)
    exported = client.post(f"/api/agent/sessions/{sid}/export").json()
    assert client.post(f"/api/agent/sessions/{sid}/export").json()["run_id"] == exported["run_id"]
    assert exported["samples"] == 101 and not exported["full_data_decimated"]
    raw = client.get(exported["outputs"]["csv"]).content
    assert hashlib.sha256(raw).hexdigest() == exported["csv_sha256"]
    rows = list(csv.DictReader(io.StringIO(raw.decode())))
    full = client.get(exported["outputs"]["json"]).json()
    assert len(rows) == len(full["rows"]) == 101
    assert float(rows[-1]["t"]) == get_session(sid).t[-1]
    for key, cell in rows[-1].items():
        assert full["rows"][-1][key] == (float(cell) if cell else None)
    archive = zipfile.ZipFile(io.BytesIO(client.get(exported["outputs"]["bundle"]).content))
    assert archive.read("data.csv") == raw
    assert {"trajectory.svg", "chart.svg", "meta.json", "manifest.json"} <= set(archive.namelist())
    meta = json.loads(archive.read("meta.json"))
    assert meta["source"] == "agent" and len(meta["command_log"]) == 1
    step(client, sid, "b", 1, steps=10)
    assert client.get(exported["outputs"]["csv"]).content == raw
    preview = client.get(f"/api/agent/sessions/{sid}/preview?max_points=9").json()
    assert preview["returned_samples"] == 9 and preview["decimated"]
    assert preview["data"]["t"][-1] == get_session(sid).t[-1]
    assert client.delete(f"/api/agent/sessions/{sid}").status_code == 200
    assert client.get(f"/api/agent/sessions/{sid}").status_code == 404
    assert client.get(exported["outputs"]["csv"]).status_code == 200


def test_failure_preserves_partial_trace_and_is_not_a_reset(client, monkeypatch):
    sid = create(client)
    step(client, sid, steps=4)
    session = get_session(sid)
    def broken(*args): raise ArithmeticError("injected failure")
    monkeypatch.setattr(session.sim.model, "step", broken)
    failed = step(client, sid, "b", 1)
    assert failed.status_code == 422
    assert failed.json()["status"] == "failed" and session.n_steps == 4
    assert "injected failure" in failed.json()["error"]
    assert step(client, sid, "b", 1).json()["replayed"]
    assert step(client, sid, "c", 2).status_code == 409
    saved = client.post(f"/api/agent/sessions/{sid}/export").json()
    assert saved["samples"] == 4


def test_session_limit_and_steps_limit_are_explicit(client):
    ids = [create(client) for _ in range(4)]
    assert client.post("/api/agent/sessions", json=config()).status_code == 429
    assert step(client, ids[0], steps=1001).status_code == 422
    get_session(ids[0]).n_steps = 20000
    assert step(client, ids[0], steps=1).json()["detail"]["code"] == "step_limit"


def test_lost_human_input_is_released_without_losing_controller_settings():
    sim = Simulator()
    _apply_client_message({"type": "driver", "throttle": .8, "steering": .3,
                           "mode_params": {"spin_rate_max_dps": 30, "vx_frac": .8}}, sim, "browser")
    sim.expire_input(sim.input_seen + 1)
    assert sim.driver.throttle == sim.driver.steering == 0
    assert sim.driver.mode_params == {"spin_rate_max_dps": 30}
    _apply_client_message({"type": "driver", "throttle": .5,
                           "mode_params": {"spin_rate_max_dps": 30, "wheel_norm": [.5, 0, 0, 0], "speed_target_ms": 5}}, sim, "browser")
    _apply_client_message({"type": "release_input"}, sim, "another_browser")
    assert sim.driver.throttle == .5
    _apply_client_message({"type": "release_input"}, sim, "browser")
    assert sim.driver.throttle == 0
    assert sim.driver.mode_params == {"spin_rate_max_dps": 30}
    assert sim.input_owner is None


async def test_script_waits_for_simulation_time_and_stops_with_neutral_input():
    sim = Simulator()
    sim.script_runner.load(Script.from_dict({"actions": [
        {"t": 0, "action": "drive", "throttle": .2, "steering": 0},
        {"t": .1, "action": "drive", "throttle": .6, "steering": 0},
        {"t": 1, "action": "stop"}]}))
    await sim.script_runner.start()
    try:
        await asyncio.sleep(.15)
        assert sim.driver.throttle == .2  # wall time cannot advance the script
        sim.model.state.t = .2
        await asyncio.sleep(.04)
        assert sim.driver.throttle == .6
    finally:
        await sim.script_runner.stop()
    assert sim.driver.throttle == 0


def test_recorder_reports_truncation_and_stops_on_time_reset():
    recorder = Recorder(RecorderConfig(channels=["vx", "driver_brake"]))
    recorder.max_samples = 2
    recorder.start()
    state = VehicleState()
    for t in (1, 2, 3):
        state.t = t
        recorder.write(state, ControlCommand.zero(), "crab", DriverInput(brake=.12345678901234567))
    assert recorder.status()["dropped_samples"] == 1
    state.t = 0
    recorder.write(state, ControlCommand.zero(), "crab")
    assert not recorder.is_recording and recorder.stop_reason == "simulation_time_reset"
    result = recorder.result_snapshot({})
    assert result.t == [2, 3] and result.meta["complete"] is False
    assert result.channels["driver_brake"][-1] == .12345678901234567


def test_recording_and_agent_publish_the_same_telemetry_and_units(client):
    from sim4wis.core.telemetry import AVAILABLE_CHANNELS, sample_channels
    from sim4wis.experiment.artifacts import channel_unit
    sid = create(client)
    step(client, sid, steps=20)
    session = get_session(sid)
    model = session.sim.model
    # Use the recorded actuator command from the same integration boundary.
    cmd = ControlCommand.zero()
    recorder = Recorder(RecorderConfig(channels=list(AVAILABLE_CHANNELS)))
    recorder.start()
    recorder.write(model.state, cmd, "ideal_ackermann", session.driver, model)
    recorder.stop()
    row = sample_channels(model.state, cmd, session.driver, model)
    for name in AVAILABLE_CHANNELS:
        np.testing.assert_equal(recorder._data[name][-1], row[name])
        assert channel_unit(name) != "unspecified", name
    assert row["tire_fx_fl"] == model.tire_fx[0]
    assert row["slip_alpha_fr"] == model.slip_alpha[1]
    assert channel_unit("grip_margin_lat_fl") == "N"
    assert channel_unit("grip_alpha_peak_rr") == "rad"
    exported = client.post(f"/api/agent/sessions/{sid}/export").json()
    assert len(exported["channels"]) == len(AVAILABLE_CHANNELS) + 1


def test_manual_recording_save_roundtrip_and_invalid_start_is_atomic(client):
    sim = get_simulator()
    assert client.post("/api/recording/start", json={"full_rate": True, "channels": ["vx", "driver_brake"]}).status_code == 200
    sim.model.state.t = 1.005
    sim.recorder.write(sim.model.state, ControlCommand.zero(), "crab", DriverInput(brake=.12345678901234567))
    before = (sim.recorder.status(), sim.recorder.cfg.rate_hz, sim.record_full_rate)
    assert client.post("/api/recording/start", json={"full_rate": False, "channels": ["invalid"]}).status_code == 422
    assert (sim.recorder.status(), sim.recorder.cfg.rate_hz, sim.record_full_rate) == before
    assert client.post("/api/recording/save").status_code == 409
    client.post("/api/recording/stop")
    saved = client.post("/api/recording/save").json()
    assert saved["complete"] and saved["samples"] == 1
    assert client.post("/api/recording/save").json()["run_id"] == saved["run_id"]
    full = client.get(saved["outputs"]["json"]).json()
    assert full["rows"][0]["driver_brake"] == .12345678901234567


def test_manual_recording_clear_requires_stop_and_discards_only_buffer(client):
    sim = get_simulator()
    assert client.post("/api/recording/start", json={"channels": ["vx"]}).status_code == 200
    sim.model.state.t = 1.0
    sim.recorder.write(sim.model.state, ControlCommand.zero(), "crab")
    before = sim.recorder.status()
    active = client.post("/api/recording/clear")
    assert active.status_code == 409
    assert sim.recorder.status() == before
    assert client.post("/api/recording/stop").status_code == 200
    cleared = client.post("/api/recording/clear")
    assert cleared.status_code == 200
    status = cleared.json()
    assert not status["recording"] and status["samples"] == 0
    assert status["from_t"] is None and status["to_t"] is None
    assert status["dropped_samples"] == 0 and status["stop_reason"] is None
    assert status["generation"] == before["generation"] + 1
    assert client.post("/api/recording/save").status_code == 409


async def test_zero_time_loop_remains_stoppable():
    sim = Simulator()
    sim.script_runner.load(Script.from_dict({"loop": True, "actions": []}))
    await sim.script_runner.start()
    await asyncio.sleep(.05)
    await asyncio.wait_for(sim.script_runner.stop(), timeout=.2)
    assert not sim.script_runner.is_running


@pytest.mark.parametrize("action", [
    {"t": float("nan"), "action": "stop"},
    {"t": -1, "action": "stop"},
    {"t": 0, "action": "brake", "duration": float("inf")},
    {"t": 0, "action": "drive", "throttle": float("nan"), "steering": 0},
])
def test_invalid_script_numbers_are_rejected_before_execution(action):
    with pytest.raises(ValueError): Script.from_dict({"actions": [action]})
