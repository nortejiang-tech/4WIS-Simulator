"""Tests — longitudinal architecture (drive-torque channel + pedal decoupling).

Phase 0 of the drive-form work. Two independent things are guarded here:

  * `longitudinal_mode` selects how the throttle reaches the wheels. The
    default stays the wheel-speed servo, so every existing baseline is
    untouched; "torque" hands `drive_torque_cmd` straight to the wheel-spin ODE
    and the servos sit out.
  * `mode_params["speed_target_ms"]` lets a validation run command a speed
    without going through the pedal at all — the longitudinal twin of
    `steer_raw_rad`. Without it the experiment layer round-trips its target
    through `throttle = v/v_max`, which only cancels while that mapping stays
    exactly linear over exactly [0, v_max]; any pedal curve, speed limiter or
    powertrain envelope would silently re-baseline every golden.
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from sim4wis.controller.longitudinal import (
    apply_brake_command,
    apply_drive_command,
    speed_command,
)
from sim4wis.controller.registry import make_strategy
from sim4wis.core.derived import update_derived_outputs
from sim4wis.core.simulator import Simulator
from sim4wis.core.state import (
    ControlCommand,
    DriverInput,
    EnvironmentState,
    VehicleParams,
)
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel
from sim4wis.vehicle.kinematic import KinematicModel

DT = 0.005


def _sim(mode: str = "speed_servo", model: str = "dynamic", **over) -> Simulator:
    p = replace(VehicleParams(), longitudinal_mode=mode, **over)
    sim = Simulator(dt_sim=DT, dt_push=0.05)
    sim.reset()
    sim.params = p
    sim.model = SimplifiedDynamicModel(p) if model == "dynamic" else KinematicModel(p)
    sim.model.reset()
    sim.strategy = make_strategy("ideal_ackermann", p)
    return sim


def _tick(sim: Simulator, env: EnvironmentState) -> ControlCommand:
    cmd = sim.strategy.compute(sim.driver, sim.model.state, DT)
    apply_brake_command(cmd, sim.driver, sim.params)
    apply_drive_command(cmd, sim.driver, sim.params)
    sim.model.step(DT, cmd, env)
    update_derived_outputs(sim.model.state, sim.params)
    return cmd


# ── 1. the default mode is untouched ─────────────────────────────────────────

def test_default_mode_is_speed_servo():
    assert VehicleParams().longitudinal_mode == "speed_servo"


def test_speed_servo_leaves_drive_torque_channel_empty():
    """In the default mode the wheel-speed servos own the drivetrain and the
    new channel must stay inert, or the two would double up."""
    sim = _sim("speed_servo")
    sim.set_driver(throttle=1.0, gear=1)
    cmd = _tick(sim, EnvironmentState(mu=0.85))
    assert np.allclose(cmd.drive_torque_cmd, 0.0)


def test_speed_servo_ignores_a_stray_drive_torque_command():
    """Belt and braces: even if something fills the channel, the default mode
    must not read it — that is what keeps the baselines safe."""
    p = VehicleParams()
    model = SimplifiedDynamicModel(p)
    model.reset()
    strat = make_strategy("ideal_ackermann", p)
    env = EnvironmentState(mu=0.85)
    drv = DriverInput(throttle=0.4, gear=1)

    def run(poison: float) -> float:
        model.reset()
        for servo in model.servos:
            servo.reset()
        for _ in range(400):
            cmd = strat.compute(drv, model.state, DT)
            apply_brake_command(cmd, drv, p)
            cmd.drive_torque_cmd = np.full(4, poison)
            model.step(DT, cmd, env)
        return float(model.state.vx)

    assert run(0.0) == run(9999.0)


# ── 2. torque mode ───────────────────────────────────────────────────────────

def test_torque_mode_fills_and_uses_the_channel():
    sim = _sim("torque")
    sim.set_driver(throttle=0.5, gear=1)
    cmd = _tick(sim, EnvironmentState(mu=0.85))
    expected = 0.5 * sim.params.motor_torque_max
    assert np.allclose(cmd.drive_torque_cmd, expected)


def test_torque_mode_disengages_the_speed_servos():
    """The servo integrators must stay clear, so switching modes mid-run cannot
    dump a stale integral into the drivetrain."""
    sim = _sim("torque")
    sim.set_driver(throttle=1.0, gear=1)
    for _ in range(200):
        _tick(sim, EnvironmentState(mu=0.85))
    assert all(servo.integral == 0.0 for servo in sim.model.servos)


def test_torque_mode_actually_accelerates():
    sim = _sim("torque")
    env = EnvironmentState(mu=0.85)
    sim.set_driver(throttle=1.0, gear=1)
    for _ in range(400):
        _tick(sim, env)
    assert sim.model.state.vx > 5.0


def test_reverse_gear_gives_negative_drive_torque():
    p = replace(VehicleParams(), longitudinal_mode="torque")
    cmd = ControlCommand.zero()
    apply_drive_command(cmd, DriverInput(throttle=0.5, gear=-1), p)
    assert np.all(cmd.drive_torque_cmd < 0.0)


def test_neutral_gives_no_drive_torque():
    p = replace(VehicleParams(), longitudinal_mode="torque")
    cmd = ControlCommand.zero()
    apply_drive_command(cmd, DriverInput(throttle=1.0, gear=0), p)
    assert np.allclose(cmd.drive_torque_cmd, 0.0)


@pytest.mark.parametrize("drv", [
    DriverInput(throttle=1.0, brake=1.0, gear=1),
    DriverInput(throttle=1.0, brake=0.0, handbrake=1, gear=1),
])
def test_brake_outranks_drive_torque(drv):
    """Same authority order the speed path uses — the pedal that removes energy
    wins, so left-foot braking cannot fight the drivetrain."""
    p = replace(VehicleParams(), longitudinal_mode="torque")
    cmd = ControlCommand.zero()
    apply_drive_command(cmd, drv, p)
    assert np.allclose(cmd.drive_torque_cmd, 0.0)


def test_kinematic_model_ignores_torque_mode():
    """Documented limitation: the kinematic model is geometric, has no wheel
    spin ODE and so no torque concept. It must keep following wheel_speed_cmd
    rather than silently standing still."""
    sim = _sim("torque", model="kinematic")
    env = EnvironmentState(mu=0.85)
    sim.set_driver(throttle=0.5, gear=1)
    for _ in range(200):
        _tick(sim, env)
    assert sim.model.state.vx > 1.0


# ── 3. the experiment speed channel is pedal-independent ─────────────────────

@pytest.mark.parametrize("throttle", [0.0, 0.1, 1.0])
@pytest.mark.parametrize("v_max", [10.0, 55.6, 200.0])
def test_speed_target_ignores_throttle_and_v_max(throttle, v_max):
    p = replace(VehicleParams(), v_max=v_max)
    drv = DriverInput(throttle=throttle, gear=1,
                      mode_params={"speed_target_ms": 12.5})
    assert speed_command(p, drv, v_actual=0.0, dt=DT) == pytest.approx(12.5)


def test_speed_target_survives_a_pedal_map_change():
    """The whole reason the channel exists: re-scaling the pedal must not move
    a validation run by a single float."""
    base = VehicleParams()
    tuned = replace(base, v_max=base.v_max * 0.5, v_max_reverse=2.0)
    drv = DriverInput(throttle=0.37, gear=1, mode_params={"speed_target_ms": -3.25})
    assert (speed_command(base, drv, 1.0, DT)
            == speed_command(tuned, drv, 1.0, DT)
            == pytest.approx(-3.25))


def test_speed_target_still_yields_to_the_brake():
    """The bypass replaces the pedal, not the safety ordering."""
    p = VehicleParams()
    drv = DriverInput(throttle=0.0, brake=1.0, gear=1,
                      mode_params={"speed_target_ms": 20.0})
    assert speed_command(p, drv, v_actual=10.0, dt=DT) < 10.0


def test_without_the_key_the_pedal_still_rules():
    p = VehicleParams()
    drv = DriverInput(throttle=0.5, gear=1)
    assert speed_command(p, drv, 0.0, DT) == pytest.approx(0.5 * p.v_max)


def test_session_drives_through_the_speed_channel():
    """The experiment layer must set the key every tick, on every strategy."""
    import yaml
    from sim4wis.experiment.schema import Experiment
    from sim4wis.experiment.session import SimSession

    root = Path(__file__).resolve().parents[2]
    exp = Experiment.model_validate(
        yaml.safe_load((root / "experiments" / "step_steer_60kmh.yaml").read_text())
    )
    session = SimSession(exp)
    original = session.strategy.compute
    seen: list[float] = []

    def spy(driver, state, dt=0.0):
        seen.append(driver.mode_params.get("speed_target_ms"))
        return original(driver, state, dt)

    session.strategy.compute = spy
    session.run()
    assert seen and all(v is not None for v in seen)
    assert max(seen) == pytest.approx(60.0 / 3.6, rel=1e-9)
