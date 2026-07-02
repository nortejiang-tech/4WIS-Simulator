"""Integration tests — time-domain models (simplified dynamic + multibody).

These cover the v0.9.0 physics-completion pass that aligned the driving
workbench with the quasi-static load page:

    * aero drag + rolling resistance in body X (both models)
    * per-axle aero lift unloading the tyres at speed (both models)
    * static toe / camber thrust reaching the kingpin chain (both models)
    * longitudinal slope gravity in the multibody model (previously only the
      simplified model had it — the Slope disturbance never slowed the
      multibody car)

Test style: drive the model open-loop with a constant wheel-speed command and
δ_cmd = 0, long enough to settle, then assert on steady-state quantities.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from sim4wis.core.state import ControlCommand, EnvironmentState, VehicleParams
from sim4wis.environment.disturbance import Scene, Slope
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel
from sim4wis.vehicle.model_core import body_resistance_force
from sim4wis.vehicle.multibody import MultiBodyModel

DT = 0.005


def _cruise_cmd(params: VehicleParams, speed: float) -> ControlCommand:
    omega = speed / params.tire_radius
    return ControlCommand(
        delta_cmd=np.zeros(4),
        wheel_speed_cmd=np.full(4, omega),
        icr_target_body=np.array([np.nan, np.nan]),
    )


def _settle(model, cmd, env, seconds: float, speed0: float | None = None):
    if speed0 is not None:
        model.state.vx = speed0
        model.state.wheel_omega[:] = speed0 / model.params.tire_radius
    for _ in range(int(seconds / DT)):
        model.step(DT, cmd, env)
    return model.state


@pytest.fixture
def params() -> VehicleParams:
    return VehicleParams()


@pytest.mark.parametrize("model_cls", [SimplifiedDynamicModel, MultiBodyModel])
def test_cruise_requires_drive_torque_against_resistance(model_cls, params) -> None:
    """At steady cruise the wheel servos must deliver the drag+Crr torque."""
    speed = 25.0  # 90 km/h — aero term dominates, unambiguous signal
    model = model_cls(params)
    env = EnvironmentState(mu=0.9)
    cmd = _cruise_cmd(params, speed)
    _settle(model, cmd, env, seconds=4.0, speed0=speed)

    # Speed is held by the servos despite the resistance...
    assert model.state.vx == pytest.approx(speed, abs=0.5)

    # ...and the total tyre drive force ≈ the resistance force.
    f_needed = -body_resistance_force(params, model.state.vx)
    assert f_needed > 500.0  # sanity: resistance is a real force at 90 km/h
    f_drive = float(np.sum(model.tire_fx))
    assert f_drive == pytest.approx(f_needed, rel=0.25)


@pytest.mark.parametrize("model_cls", [SimplifiedDynamicModel, MultiBodyModel])
def test_aero_lift_unloads_tyres_at_speed(model_cls, params) -> None:
    """Total Fz at high speed < static weight (per-axle lift, ∝ v²)."""
    env = EnvironmentState(mu=0.9)

    slow = model_cls(params)
    _settle(slow, _cruise_cmd(params, 5.0), env, seconds=3.0, speed0=5.0)
    fz_slow = float(np.sum(slow.state.fz))

    fast = model_cls(params)
    _settle(fast, _cruise_cmd(params, 40.0), env, seconds=3.0, speed0=40.0)
    fz_fast = float(np.sum(fast.state.fz))

    q = 0.5 * params.air_density * params.frontal_area * 40.0**2
    lift_expected = (params.aero_lift_coeff_front + params.aero_lift_coeff_rear) * q
    assert fz_fast < fz_slow - 0.5 * lift_expected
    assert fz_slow == pytest.approx(params.mass * 9.81, rel=0.03)


@pytest.mark.parametrize("model_cls", [SimplifiedDynamicModel, MultiBodyModel])
def test_toe_camber_load_the_kingpin_but_keep_straight_line(model_cls, params) -> None:
    """δ_cmd = 0 straight-ahead: alignment loads each kingpin (τ ≠ 0) while the
    left/right forces cancel so the vehicle still tracks straight."""
    speed = 20.0
    model = model_cls(params)
    env = EnvironmentState(mu=0.9)
    _settle(model, _cruise_cmd(params, speed), env, seconds=4.0, speed0=speed)

    s = model.state
    # Straight-line: no meaningful yaw or lateral drift.
    assert abs(s.yaw_rate) < 1e-3
    assert abs(s.vy) < 0.05
    # Reported wheel angle includes static toe (FL = −toe_front, FR = +toe_front).
    assert s.delta[0] == pytest.approx(-params.static_toe_front, abs=2e-4)
    assert s.delta[1] == pytest.approx(+params.static_toe_front, abs=2e-4)
    # Alignment side forces load the steering chain even at δ_cmd = 0.
    assert np.all(np.abs(model.tire_fy[:2]) > 20.0)
    # Mirror symmetry: left/right lateral forces cancel at the body level.
    assert abs(float(np.sum(model.tire_fy))) < 0.2 * float(np.max(np.abs(model.tire_fy)))


def test_toe_camber_zeroed_recovers_zero_kingpin_torque(params) -> None:
    """With toe/camber/lift/drive all zero, straight-ahead τ_steer ≈ 0."""
    clean = dataclasses.replace(
        params,
        static_toe_front=0.0,
        static_toe_rear=0.0,
        camber_thrust_coeff=0.0,
        rolling_resistance_coeff=0.0,   # otherwise Fx_drive×scrub loads the
        drag_coeff_cd=0.0,              # kingpin — the load page's A1 term
        suspension=dataclasses.replace(params.suspension, camber=0.0),
    )
    model = SimplifiedDynamicModel(clean)
    env = EnvironmentState(mu=0.9)
    _settle(model, _cruise_cmd(clean, 20.0), env, seconds=3.0, speed0=20.0)
    assert np.all(np.abs(model.state.torque_steer) < 1.0)


@pytest.mark.parametrize("model_cls", [SimplifiedDynamicModel, MultiBodyModel])
def test_uphill_slope_needs_more_drive_force(model_cls, params) -> None:
    """Inside a Slope region the drive force must additionally carry
    m·g·sin(grade). This is the regression test for the multibody model,
    which previously had no longitudinal grade gravity at all."""
    speed = 10.0
    angle = 0.06  # ≈ 3.4° grade
    # Region spans x ∈ [0, 400] so the vehicle starts at the bottom edge
    # (ground_z = 0) and climbs — NOT in the middle of a 12 m-high ramp.
    scene = Scene(
        base_mu=0.9,
        disturbances=[
            Slope(id="s1", x=200.0, y=0.0, width=50.0, length=400.0, angle=angle),
        ],
    )
    env_slope = EnvironmentState(scene=scene, mu=scene.base_mu)
    env_flat = EnvironmentState(mu=0.9)

    on_slope = model_cls(params)
    _settle(on_slope, _cruise_cmd(params, speed), env_slope, seconds=4.0, speed0=speed)
    f_slope = float(np.sum(on_slope.tire_fx))

    flat = model_cls(params)
    _settle(flat, _cruise_cmd(params, speed), env_flat, seconds=4.0, speed0=speed)
    f_flat = float(np.sum(flat.tire_fx))

    extra = f_slope - f_flat
    expected = params.mass * 9.81 * np.sin(angle)
    assert extra == pytest.approx(expected, rel=0.30)


def test_tire_load_sensitive_cornering_stiffness_unit() -> None:
    """c_α(Fz) = c_α0·(Fz/Fz_nom)^p at the tyre-model level (C3-1)."""
    from sim4wis.vehicle.tire import LinearTireModel, PacejkaTireModel

    nom = 7000.0
    for cls in (LinearTireModel, PacejkaTireModel):
        tire = cls(fz_nom=nom, load_exp=0.8)
        alpha = 0.005  # deep in the linear region
        _, fy_nom, _ = tire.forces(alpha, 0.0, nom, 1.0)
        _, fy_15, _ = tire.forces(alpha, 0.0, 1.5 * nom, 1.0)
        assert fy_15 / fy_nom == pytest.approx(1.5**0.8, rel=0.02), cls.__name__
        # Off (defaults): small-slip stiffness independent of load. Pacejka's
        # peak still scales with D = μ·Fz, so allow its slight curvature
        # difference at finite α (1 %); the linear model is exact.
        tire_off = cls()
        _, a, _ = tire_off.forces(alpha, 0.0, nom, 1.0)
        _, b, _ = tire_off.forces(alpha, 0.0, 1.5 * nom, 1.0)
        assert a == pytest.approx(b, rel=1e-2), cls.__name__


def test_load_sensitivity_changes_cornering_but_not_straight_line(params) -> None:
    """Time-domain c_α(Fz): measurable in a steady corner (lateral transfer
    reduces axle capacity), invisible in straight-line driving."""
    on = params  # default: tire_load_sensitivity_time_domain = True
    off = dataclasses.replace(params, tire_load_sensitivity_time_domain=False)
    env = EnvironmentState(mu=0.9)
    speed = 15.0

    def steady_yaw(p: VehicleParams, steering: float) -> float:
        from sim4wis.controller.registry import make_strategy
        from sim4wis.core.state import DriverInput
        model = SimplifiedDynamicModel(p)
        strat = make_strategy("ideal_ackermann", p)
        model.state.vx = speed
        model.state.wheel_omega[:] = speed / p.tire_radius
        driver = DriverInput(throttle=speed / p.v_max, steering=steering)
        for _ in range(int(5.0 / DT)):
            cmd = strat.compute(driver, model.state)
            model.step(DT, cmd, env)
        return model.state.yaw_rate

    yaw_on = steady_yaw(on, 0.10)
    yaw_off = steady_yaw(off, 0.10)
    assert yaw_on != pytest.approx(yaw_off, rel=1e-4)   # physics actually differs
    assert abs(yaw_on - yaw_off) / abs(yaw_off) < 0.15  # ...but is a correction, not a regime change

    # Straight line: symmetric loads → no pull either way.
    assert abs(steady_yaw(on, 0.0)) < 1e-3
    assert abs(steady_yaw(off, 0.0)) < 1e-3


@pytest.mark.parametrize("model_cls", [SimplifiedDynamicModel, MultiBodyModel])
def test_no_finite_escape_under_full_lock(model_cls, params) -> None:
    """Numerical robustness: full-lock steering at speed stays finite."""
    model = model_cls(params)
    env = EnvironmentState(mu=0.9)
    cmd = ControlCommand(
        delta_cmd=np.full(4, params.steer_limit),
        wheel_speed_cmd=np.full(4, 15.0 / params.tire_radius),
        icr_target_body=np.array([np.nan, np.nan]),
    )
    _settle(model, cmd, env, seconds=3.0, speed0=15.0)
    s = model.state
    for v in (s.vx, s.vy, s.yaw_rate, s.x, s.y, s.psi):
        assert np.isfinite(v)
    assert np.all(np.isfinite(s.fz))
    assert np.all(np.isfinite(s.torque_steer))
