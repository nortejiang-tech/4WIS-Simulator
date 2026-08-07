"""Tests — friction-brake system (work-package A).

These exercise the end-to-end behaviour the plan §A5 wants to see, driving the
brake through the SAME wiring the live loop uses (`apply_brake_command`) rather
than hand-building `cmd.brake_cmd`. That distinction matters: the original
suite hard-coded the per-wheel array at full axle bias, so every "pedal" case
actually ran at full pedal and the tests could not see that pedal travel did
nothing.

Covered:
  * pedal position modulates stopping distance (monotonic, ~1/pedal),
  * full-pedal distance is close to the v²/(2μg) ideal,
  * the vehicle actually stops — no low-speed creep, no reversing,
  * stopping distance scales with surface μ, on the kinematic model too,
  * lockup happens at high pedal only, and a locked wheel loses lateral force,
  * the parking brake holds the car against full throttle,
  * the brake outranks the throttle and the gear selector,
  * legacy negative-throttle callers still map to the brake — and release it.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from sim4wis.controller.longitudinal import apply_brake_command, speed_command
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
G = 9.81


def _sim(model: str = "dynamic", params: VehicleParams | None = None) -> Simulator:
    p = params or VehicleParams()
    sim = Simulator(dt_sim=DT, dt_push=0.05)
    sim.reset()
    sim.params = p
    sim.model = (SimplifiedDynamicModel(p) if model == "dynamic" else KinematicModel(p))
    sim.model.reset()
    sim.strategy = make_strategy("ideal_ackermann", p)
    return sim


def _tick(sim: Simulator, env: EnvironmentState) -> None:
    """One control+integrate step, mirroring Simulator._loop."""
    cmd = sim.strategy.compute(sim.driver, sim.model.state, DT)
    apply_brake_command(cmd, sim.driver, sim.params)
    sim.model.step(DT, cmd, env)
    update_derived_outputs(sim.model.state, sim.params)


def _accelerate_to(sim: Simulator, env: EnvironmentState, kmh: float) -> float:
    sim.set_driver(throttle=min(1.0, (kmh / 3.6) / sim.params.v_max), brake=0.0, gear=1)
    for _ in range(20000):
        _tick(sim, env)
        if sim.model.state.vx >= kmh / 3.6 - 0.05:
            break
    return float(sim.model.state.vx)


def _stop(sim: Simulator, env: EnvironmentState, pedal: float,
          handbrake: int = 0) -> dict:
    """Brake to rest; report distance, time, lockup and whether it reversed."""
    sim.set_driver(throttle=0.0, brake=pedal, gear=1, handbrake=handbrake)
    x0, v0 = float(sim.model.state.x), float(sim.model.state.vx)
    locked_steps = reversed_seen = 0
    k = 0
    for k in range(120000):
        _tick(sim, env)
        if bool(np.any(sim.model.state.wheel_locked)):
            locked_steps += 1
        if sim.model.state.vx < -0.05:
            reversed_seen += 1
        if abs(sim.model.state.vx) < 0.05:
            break
    return {
        "v0": v0,
        "dist": abs(float(sim.model.state.x) - x0),
        "t_stop": k * DT,
        "locked_s": locked_steps * DT,
        "reversed": reversed_seen > 0,
        "vx_end": float(sim.model.state.vx),
    }


# ── 1. pedal travel actually modulates the stop ──────────────────────────────

def test_pedal_position_modulates_stopping_distance():
    """The whole point of a brake pedal. Regression guard: with the servo
    still holding full retarding authority during braking, 0.2 and 1.0 pedal
    produced the same 22 m / 11 s locked skid — 80% of travel was dead."""
    dists = []
    for pedal in (0.2, 0.4, 0.6, 1.0):
        sim = _sim()
        env = EnvironmentState(mu=0.85)
        _accelerate_to(sim, env, 60.0)
        dists.append(_stop(sim, env, pedal)["dist"])

    # strictly decreasing with pedal, and by a wide margin
    for a, b in zip(dists, dists[1:]):
        assert b < a * 0.95, f"pedal travel barely changed the stop: {dists}"
    # a light pedal must be dramatically longer than a full one
    assert dists[0] > 2.5 * dists[-1], f"pedal range too compressed: {dists}"


def test_full_pedal_distance_near_ideal():
    """Full pedal on dry asphalt should land near v²/(2·μ·g). A little longer
    is expected and correct: the actuator ramps in over brake_tau and the last
    phase is a locked skid, which gives slightly less than peak μ."""
    sim = _sim()
    env = EnvironmentState(mu=0.85)
    v0 = _accelerate_to(sim, env, 60.0)
    r = _stop(sim, env, 1.0)
    ideal = v0 * v0 / (2 * 0.85 * G)
    assert ideal <= r["dist"] <= ideal * 1.25, (
        f"stop {r['dist']:.2f} m vs ideal {ideal:.2f} m"
    )


def test_stop_is_prompt_and_does_not_creep_or_reverse():
    """A full stop from 60 km/h takes ~2 s. Regression guard: the brake torque
    used to vanish at ω = 0 (sign(0) = 0), so the rear wheels spun faster than
    the body and pushed the car — it crawled below 1 m/s for ~9 s."""
    sim = _sim()
    env = EnvironmentState(mu=0.85)
    v0 = _accelerate_to(sim, env, 60.0)
    r = _stop(sim, env, 1.0)
    ideal_t = v0 / (0.85 * G)
    assert r["t_stop"] < ideal_t * 1.6, f"took {r['t_stop']:.2f} s (ideal {ideal_t:.2f})"
    assert not r["reversed"], "vehicle reversed while braking"
    assert r["vx_end"] >= -0.02


# ── 2. stopping distance scales with surface μ ───────────────────────────────

@pytest.mark.parametrize("model", ["dynamic", "kinematic"])
def test_brake_distance_scales_with_mu(model):
    """Lower μ → longer stop, on BOTH models.

    The kinematic model has no tyre, so its rate limit must be fed the real
    surface μ via `state.mu_avg`. Regression guard: `mu_eff` defaulted to 0.85
    and no caller ever passed it, so a kinematic stop on ice was exactly as
    short as on dry asphalt — the very defect work-package A set out to fix.
    """
    dists = {}
    for mu in (0.85, 0.40):
        sim = _sim(model)
        env = EnvironmentState(mu=mu)
        _accelerate_to(sim, env, 50.0)
        dists[mu] = _stop(sim, env, 1.0)["dist"]
    ratio = dists[0.40] / dists[0.85]
    expected = 0.85 / 0.40
    assert ratio == pytest.approx(expected, rel=0.25), (
        f"{model}: distance ratio {ratio:.2f}, expected ≈{expected:.2f} ({dists})"
    )


def test_kinematic_brake_rate_follows_state_mu():
    """The rate limit is brake·μ·g — read straight off speed_command."""
    p = VehicleParams()
    drv = DriverInput(throttle=0.0, brake=1.0, gear=1)
    for mu in (0.85, 0.30):
        v_cmd = speed_command(p, drv, v_actual=10.0, dt=DT, mu_eff=mu)
        assert 10.0 - v_cmd == pytest.approx(mu * G * DT, rel=0.01)


# ── 3. lockup is a high-pedal event, and costs lateral grip ──────────────────

def test_lockup_only_at_high_pedal():
    """Lockup must be reachable (no ABS) but must not consume the usable range.
    Regression guard: the per-wheel torque was the whole-vehicle budget without
    the /2, so every wheel locked from ~0.3 pedal and every stop was a skid."""
    sim = _sim()
    env = EnvironmentState(mu=0.85)
    _accelerate_to(sim, env, 60.0)
    mid = _stop(sim, env, 0.5)
    assert mid["locked_s"] == 0.0, "a half pedal should not lock the wheels"

    sim = _sim()
    env = EnvironmentState(mu=0.85)
    _accelerate_to(sim, env, 60.0)
    full = _stop(sim, env, 1.0)
    assert full["locked_s"] > 0.5, "a full pedal on dry asphalt should lock up"


def test_locked_wheel_loses_lateral_force():
    """Friction-ellipse consequence of lockup: Fx saturates, Fy collapses."""
    from sim4wis.vehicle.tire import make_tire
    p = VehicleParams()
    tire = make_tire(p)
    fz = p.mass * G / 4.0
    _fx, fy_grip, _ = tire.forces(0.087, 0.0, fz, 0.85)      # α=5°, rolling
    _fx, fy_lock, _ = tire.forces(0.087, -0.99, fz, 0.85)    # α=5°, locked
    assert abs(fy_lock) < abs(fy_grip) * 0.4


def test_wheel_locked_flag_is_true_while_sliding():
    """The flag must be set when the wheel is held against a moving car — the
    original test (|ω| > 1e-3) went False exactly when lockup became total."""
    sim = _sim()
    env = EnvironmentState(mu=0.85)
    _accelerate_to(sim, env, 60.0)
    sim.set_driver(throttle=0.0, brake=1.0, gear=1)
    saw_locked_with_stopped_wheel = False
    for _ in range(2000):
        _tick(sim, env)
        st = sim.model.state
        if abs(st.vx) < 0.05:
            break
        if np.any(st.wheel_locked & (np.abs(st.wheel_omega) < 1e-3)):
            saw_locked_with_stopped_wheel = True
    assert saw_locked_with_stopped_wheel


# ── 4. parking brake ─────────────────────────────────────────────────────────

def test_handbrake_holds_against_full_throttle():
    """Regression guard: the handbrake only gated the rear motors, so the front
    speed servo drove the car to v_max (200 km/h) with the parking brake on."""
    sim = _sim()
    env = EnvironmentState(mu=0.85)
    sim.set_driver(throttle=1.0, brake=0.0, gear=1, handbrake=1)
    for _ in range(2000):    # 10 s
        _tick(sim, env)
    assert abs(sim.model.state.vx) < 0.5, (
        f"parking brake did not hold: reached {sim.model.state.vx * 3.6:.1f} km/h"
    )


def test_handbrake_alone_commands_a_stop():
    p = VehicleParams()
    drv = DriverInput(throttle=0.0, brake=0.0, gear=1, handbrake=1)
    assert speed_command(p, drv, v_actual=10.0, dt=DT) < 10.0


# ── 5. brake authority ordering ──────────────────────────────────────────────

def test_brake_outranks_throttle():
    """Left-foot braking, or a wheel whose pedals overlap, must not launch."""
    p = VehicleParams()
    drv = DriverInput(throttle=1.0, brake=1.0, gear=1)
    assert speed_command(p, drv, v_actual=0.0, dt=DT) == 0.0
    assert speed_command(p, drv, v_actual=10.0, dt=DT) < 10.0


def test_brake_works_in_neutral():
    """A service brake does not care which gear is selected."""
    p = VehicleParams()
    drv = DriverInput(throttle=0.0, brake=1.0, gear=0)
    assert speed_command(p, drv, v_actual=15.0, dt=DT) < 15.0


def test_neutral_without_brake_coasts():
    p = VehicleParams()
    drv = DriverInput(throttle=1.0, brake=0.0, gear=0)
    assert speed_command(p, drv, v_actual=15.0, dt=DT) == pytest.approx(15.0)


def test_reverse_is_speed_capped():
    p = VehicleParams()
    drv = DriverInput(throttle=1.0, brake=0.0, gear=-1)
    assert speed_command(p, drv, v_actual=0.0, dt=DT) == pytest.approx(-p.v_max_reverse)


# ── 6. actuator command wiring ───────────────────────────────────────────────

def test_apply_brake_command_splits_by_axle_bias():
    p = VehicleParams()
    cmd = ControlCommand.zero()
    apply_brake_command(cmd, DriverInput(brake=1.0, gear=1), p)
    assert cmd.brake_cmd[0] == pytest.approx(p.brake_bias_front)
    assert cmd.brake_cmd[1] == pytest.approx(p.brake_bias_front)
    assert cmd.brake_cmd[2] == pytest.approx(1.0 - p.brake_bias_front)
    assert cmd.brake_cmd[3] == pytest.approx(1.0 - p.brake_bias_front)


def test_handbrake_does_not_out_brake_a_full_pedal():
    """The parking brake applies the rear axle's own share, not a full clamp —
    otherwise pulling it mid-stop spikes rear torque above the pedal's."""
    p = VehicleParams()
    cmd = ControlCommand.zero()
    apply_brake_command(cmd, DriverInput(brake=1.0, handbrake=1, gear=1), p)
    assert cmd.brake_cmd[2] == pytest.approx(1.0 - p.brake_bias_front)
    assert cmd.handbrake == 1


# ── 7. legacy protocol ───────────────────────────────────────────────────────

def test_legacy_negative_throttle_maps_to_brake():
    sim = Simulator(dt_sim=DT, dt_push=0.05)
    sim.reset()
    sim.set_driver(throttle=-1.0)   # legacy signed form, no brake arg
    assert sim.driver.throttle == 0.0
    assert sim.driver.brake == 1.0


def test_legacy_positive_throttle_releases_the_brake():
    """Regression guard: a legacy caller owns both channels through one axis and
    has no way to release the brake. Latching it meant any old client that
    braked once drove with the brake on for the rest of the session."""
    sim = Simulator(dt_sim=DT, dt_push=0.05)
    sim.reset()
    sim.set_driver(throttle=-0.8)
    assert sim.driver.brake == pytest.approx(0.8)
    sim.set_driver(throttle=0.8)
    assert sim.driver.brake == 0.0
    assert sim.driver.throttle == pytest.approx(0.8)


def test_explicit_brake_channel_is_not_treated_as_legacy():
    sim = Simulator(dt_sim=DT, dt_push=0.05)
    sim.reset()
    sim.set_driver(throttle=0.5, brake=0.3)
    assert sim.driver.throttle == pytest.approx(0.5)
    assert sim.driver.brake == pytest.approx(0.3)


def test_gear_validation_rejects_invalid():
    sim = Simulator(dt_sim=DT, dt_push=0.05)
    sim.reset()
    with pytest.raises(ValueError):
        sim.set_driver(gear=2)


# ── 8. servo must not fight the brake ────────────────────────────────────────

def test_servo_disengages_while_braking():
    """Clamping the servo output to ≤0 is not enough: as a *speed* servo it
    then pours its full retarding torque in on top of the friction brake, with
    no pedal dependence at all. It must disengage."""
    from sim4wis.vehicle.wheel_servo import WheelSpeedServo
    servo = WheelSpeedServo(kp=200.0, ki=50.0, torque_limit=2000.0)
    for _ in range(5):                       # prime the integrator
        servo.update(omega_actual=10.0, omega_cmd=20.0, dt=DT)
    for _ in range(10):
        assert servo.update(10.0, 20.0, DT, brake_active=True) == 0.0
    assert servo.integral == 0.0, "integrator must not stay wound up"
    # large negative error must not produce retarding torque either
    assert servo.update(20.0, 0.0, DT, brake_active=True) == 0.0
