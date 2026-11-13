"""Independent fault/physics oracles: timing, mechanics, composition and host parity."""
import math
from dataclasses import replace

import numpy as np
import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from sim4wis.agent.session import AgentSession, FaultScheduleRequest, SessionConfig, SessionError, StepRequest
from sim4wis.core.simulator import Simulator, reset_simulator
from sim4wis.core.state import ControlCommand, DriverInput, EnvironmentState, VehicleParams
from sim4wis.core.step import advance_model
from sim4wis.experiment.schema import Experiment, Maneuver, ManeuverStep, SteerProfile
from sim4wis.experiment.session import SimSession
from sim4wis.fault import FaultInjector, passive_steering_step
from sim4wis.fault.schema import FaultSchedule, FaultSpec
from sim4wis.vehicle.model_registry import make_vehicle_model

MODELS = ("simplified_dynamic", "multibody")
KINDS = ("stuck_zero", "stuck_value", "limited", "stuck_hold", "free_caster",
         "sensor_bias", "sensor_noise", "sensor_dropout")


def step(model, faults, dt=.005, angle=.1):
    cmd = ControlCommand.zero()
    cmd.delta_cmd[:] = angle
    cmd.wheel_speed_cmd[:] = 5 / model.params.tire_radius
    return advance_model(model, model.params, DriverInput(), cmd, dt, EnvironmentState(), faults)


@pytest.mark.parametrize("body", [
    {"t_start": -1}, {"t_start": math.inf}, {"value": math.nan}, {"t_end": 0},
    {"j_steer": 0}, {"c_damp": -1}, {"tau_coulomb": -1}, {"eta_rev": 1.1},
    {"wheel": 4}, {"wheel": True}, {"unknown": 1},
    {"fault_type": "sensor_noise", "value": -.1}, {"fault_type": "limited", "value": -.1},
])
def test_invalid_faults_fail_before_execution(body):
    with pytest.raises(ValidationError):
        FaultSpec(**{"fault_type": "stuck_hold", "wheel": 0, **body})


def test_aliases_and_overlap_rules():
    assert FaultSpec(fault_type="motor_stuck_angle", wheel="FR").fault_type == "stuck_value"
    assert FaultSpec(fault_type="sensor_bias", wheel="rr").wheel == 3
    a = FaultSpec(fault_type="stuck_hold", wheel=0, t_start=1, t_end=2)
    for kind in KINDS[:5]:
        with pytest.raises(ValidationError, match="overlapping"):
            FaultSchedule(faults=[a, FaultSpec(fault_type=kind, wheel=0, t_start=1.5)])
    assert FaultSchedule(faults=[a, FaultSpec(fault_type="free_caster", wheel=0, t_start=2)])
    assert FaultSchedule(faults=[FaultSpec(fault_type=k, wheel=0) for k in KINDS[5:]])


@pytest.mark.parametrize("model_type", MODELS)
@pytest.mark.parametrize("wheel", range(4))
@pytest.mark.parametrize("kind", KINDS)
def test_every_fault_at_every_corner_in_both_force_models(model_type, wheel, kind):
    model = make_vehicle_model(VehicleParams(), model_type)
    fi = FaultInjector([FaultSpec(id="probe", fault_type=kind, wheel=wheel, value=.03, t_start=.021, t_end=.083)])
    for _ in range(24):
        s = step(model, fi)
        assert np.isfinite(np.r_[s.delta, s.fz, s.wheel_omega, s.vx, s.vy, s.yaw_rate]).all()
        assert np.min(s.fz) >= 0
        assert np.max(np.abs(s.delta)) <= model.params.steer_limit + 1e-12
        assert np.max(s.grip_util) <= 1 + 1e-8
        if .022 < s.t < .08 and kind == "sensor_bias":
            assert s.delta_reported[wheel] - s.delta[wheel] == pytest.approx(.03)
    assert [(e["event"], round(e["time"], 10)) for e in fi.events] == [("activated", .021), ("deactivated", .083)]
    assert fi.status()["faults"][0]["phase"] == "completed"


def test_non_grid_simultaneous_staggered_recovery_and_step_splitting():
    model = make_vehicle_model(VehicleParams(), "simplified_dynamic")
    fi = FaultInjector([FaultSpec(id=f"f{i}", fault_type="stuck_hold", wheel=i, t_start=t, t_end=.009)
                        for i, t in enumerate((.003, .003, .006, .008))])
    spans = []
    original = model.step
    def observed(dt, cmd, env):
        spans.append((model.state.t, dt, np.isfinite(cmd.steer_override).tolist()))
        return original(dt, cmd, env)
    model.step = observed
    step(model, fi, dt=.01)
    assert max(x[1] for x in spans) <= .00125 + 1e-12
    assert sum(x[1] for x in spans) == pytest.approx(.01)
    at = {round(start, 8): mode for start, _, mode in spans}
    assert at[.003] == [True, True, False, False]
    assert at[.008] == [True, True, True, True]
    assert at[.009] == [False] * 4
    assert model.state.t == pytest.approx(.01)
    assert [e["time"] for e in fi.events if e["event"] == "activated"] == [.003, .003, .006, .008]


@pytest.mark.parametrize("model_type", MODELS)
def test_jam_latches_mechanism_without_double_adding_toe(model_type):
    p = replace(VehicleParams(), static_toe_front=.02, static_toe_rear=.015)
    model = make_vehicle_model(p, model_type)
    for _ in range(20): step(model, FaultInjector(), angle=.12)
    before_act, before_delta = model._delta_act.copy(), model.state.delta.copy()
    fi = FaultInjector([FaultSpec(fault_type="stuck_hold", wheel=i) for i in range(4)])
    for _ in range(20): step(model, fi, angle=-.2)
    np.testing.assert_array_equal(model._delta_act, before_act)
    np.testing.assert_allclose(model.state.delta, before_delta, atol=1e-12)
    np.testing.assert_array_equal(model.state.steer_rate, 0)


@pytest.mark.parametrize("model_type", MODELS)
def test_passive_corner_bypasses_powered_lag_and_preserves_initial_rate(model_type):
    model = make_vehicle_model(VehicleParams(), model_type)
    model._delta_act[0] = .1
    model._steer_rate[0] = .4
    model.state.torque_steer[0] = 12
    fi = FaultInjector([FaultSpec(fault_type="free_caster", wheel=0, eta_rev=1,
                                 j_steer=3, c_damp=2, tau_coulomb=0)])
    step(model, fi, dt=.001, angle=-.3)
    expected_rate = (3*.4 - .001*12) / (3 + .001*2)
    assert model._delta_act[0] == pytest.approx(.1 + .001*expected_rate, abs=1e-12)
    assert model.state.steer_rate[0] == pytest.approx(expected_rate)


@pytest.mark.parametrize("dt", [.0005, .005, .05, .5])
def test_passive_energy_and_stiction(dt):
    angle, rate = 0., 2.
    energies = []
    for _ in range(50):
        angle, rate = passive_steering_step(angle, rate, 0, dt, inertia=3, damping=80, friction=5, limit=.6)
        energies.append(1.5 * rate**2)
    assert np.all(np.diff(energies) <= 1e-14)
    assert passive_steering_step(.1, 0, 4, dt, inertia=3, damping=80, friction=5, limit=.6) == (.1, 0)
    assert passive_steering_step(.59, 2, 100, .1, inertia=3, damping=0, friction=0, limit=.6) == (.6, 0)
    a, w = passive_steering_step(.1, .2, 20, dt, inertia=3, damping=80, friction=5, limit=.6)
    an, wn = passive_steering_step(-.1, -.2, -20, dt, inertia=3, damping=80, friction=5, limit=.6)
    assert (an, wn) == pytest.approx((-a, -w))


def test_sensor_composition_seed_order_dropout_and_pure_reads():
    specs = [FaultSpec(fault_type=k, wheel=0, value=v, t_start=t, t_end=end)
             for k,v,t,end in [("sensor_bias", .02, 0, None), ("sensor_noise", .01, 0, None),
                               ("sensor_dropout", 0, .02, .04)]]
    a, b = [make_vehicle_model(VehicleParams(), "simplified_dynamic") for _ in range(2)]
    fa, fb = FaultInjector(specs, seed=17), FaultInjector(list(reversed(specs)), seed=17)
    values = []
    internal = []
    original_sample = fa.sample
    def sample(model):
        original_sample(model)
        internal.append((model.state.t, float(model.state.delta_reported[0])))
    fa.sample = sample
    for _ in range(12):
        step(a, fa); step(b, fb)
        np.testing.assert_array_equal(a.state.delta_reported, b.state.delta_reported)
        previous = a.state.delta_reported.copy()
        for _ in range(7): fa.status()
        np.testing.assert_array_equal(previous, a.state.delta_reported)
        values.append(previous[0])
    last_valid = [value for t, value in internal if t < .02 - 1e-10][-1]
    assert values[3:7] == [last_valid] * 4  # hold last internal pre-activation sample
    assert values[7] != values[6]


def test_fault_configuration_failure_is_atomic_and_model_switch_is_guarded():
    sim = Simulator(model_type="simplified_dynamic")
    a = FaultSpec(fault_type="stuck_hold", wheel=0)
    sim.fault_injector.replace_schedule([a], model=sim.model)
    old = sim.fault_injector.status()
    with pytest.raises(ValueError):
        sim.fault_injector.replace_schedule([a, a], model=sim.model)
    assert old == sim.fault_injector.status()
    with pytest.raises(ValueError, match="dynamic"):
        sim.set_model_type("kinematic")
    assert sim.model_type == "simplified_dynamic"
    p = VehicleParams(); p.steering_system.enabled = True
    with pytest.raises(ValueError, match="independent"):
        sim.set_params(p)
    assert not sim.params.steering_system.enabled


def test_live_batch_and_agent_have_identical_trajectory_and_observations():
    faults = [FaultSpec(fault_type="stuck_hold", wheel=0, t_start=.203, t_end=.6),
              FaultSpec(fault_type="free_caster", wheel=3, t_start=.403, t_end=.8),
              FaultSpec(fault_type="sensor_noise", wheel=1, value=.01, t_start=.1)]
    exp = Experiment(name="parity", faults=faults, fault_seed=32, record_hz=200,
        maneuver=Maneuver(steps=[ManeuverStep(duration=1, speed_kmh=18,
            steer=SteerProfile(amplitude=2, unit="front_deg"))]))
    result = SimSession(exp).run()
    agent = AgentSession(SessionConfig(experiment=exp.model_copy(update={"maneuver": Maneuver()})))
    for i, n in enumerate((37, 63, 100)):
        response = agent.step(StepRequest(request_id=str(i), expected_revision=i, steps=n,
                control={"speed_target_ms": 5, "front_angle_rad": math.radians(2)}))
        assert response["status"] == "ready"
        agent.observe(); agent.observe()
    live = Simulator(model_type="simplified_dynamic")
    live.fault_injector.replace_schedule(faults, seed=32, model=live.model)
    live.driver = DriverInput(throttle=5/live.params.v_max,
        mode_params={"speed_target_ms": 5, "steer_raw_rad": math.radians(2)})
    for i in range(200):
        cmd = live.strategy.compute(live.driver, live.model.state, .005)
        advance_model(live.model, live.params, live.driver, cmd, .005, live.env, live.fault_injector)
        snapshot = live._serialize_state(); live._serialize_state()
        assert snapshot["t"] == pytest.approx(result.t[i])
        assert live.model.state.delta_reported[1] == agent.channels["delta_reported_fr"][i]
    for name in ("pose_x", "pose_y", "yaw_rate", "delta_fl", "delta_rr", "delta_reported_fr"):
        np.testing.assert_array_equal(result.channels[name], agent.channels[name])
    assert live.fault_injector.events == agent.sim.fault_injector.events


def test_agent_schedule_revision_retry_failure_and_export_metadata():
    agent = AgentSession(SessionConfig(experiment=Experiment(name="agent")))
    req = FaultScheduleRequest(request_id="schedule", expected_revision=0,
        faults=[FaultSpec(fault_type="stuck_hold", wheel=0, t_start=.023),
                FaultSpec(fault_type="stuck_hold", wheel=3, t_start=.023)])
    response = agent.set_fault_schedule(req)
    assert response["revision"] == 1
    assert agent.set_fault_schedule(req)["replayed"]
    with pytest.raises(SessionError, match="revision"):
        agent.set_fault_schedule(req.model_copy(update={"request_id": "stale"}))
    agent.step(StepRequest(request_id="run", expected_revision=1, steps=20, control={"front_angle_rad": .1}))
    assert agent.set_fault_schedule(req)["state"] == response["state"]
    with pytest.raises(SessionError, match="at or after"):
        agent.set_fault_schedule(FaultScheduleRequest(request_id="past", expected_revision=2, faults=req.faults))
    assert len(agent.result().meta["fault_timeline"]["events"]) == 2
    assert agent.revision == 2


def test_live_api_rejects_unknown_fields_and_conflicts_without_partial_write():
    from sim4wis.main import app
    reset_simulator(); api = TestClient(app)
    before = api.get("/api/faults/schedule").json()
    bad = {"faults": [{"fault_type": "sensor_bias", "wheel": 0, "typo": 1}]}
    assert api.post("/api/faults/schedule", json=bad).status_code == 422
    assert api.get("/api/faults/schedule").json() == before
    good = {"faults": [{"fault_type": "sensor_bias", "wheel": w, "value": .1, "t_start": 1} for w in (0, 3)]}
    assert api.post("/api/faults/schedule", json=good).status_code == 200
    assert len(api.get("/api/faults").json()["faults"]) == 2
    reset_simulator()


@pytest.mark.asyncio
async def test_live_numerical_failure_pauses_and_preserves_evidence(monkeypatch):
    import asyncio
    sim = Simulator(model_type="simplified_dynamic")
    sim.model.state.x = 12.

    def fail(*args):
        raise ArithmeticError("manufactured numerical failure")

    monkeypatch.setattr(sim.model, "step", fail)
    await sim.start()
    try:
        for _ in range(20):
            if sim.simulation_error:
                break
            await asyncio.sleep(.005)
        assert sim.paused
        assert sim.model.state.x == 12
        assert sim.model.state.t == 0
        assert "manufactured numerical failure" in sim._serialize_state()["simulation_error"]
    finally:
        await sim.stop()
    sim.reset()
    assert sim.simulation_error is None
