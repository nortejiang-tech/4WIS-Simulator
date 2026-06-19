"""KinematicModel integration tests."""

from __future__ import annotations

import numpy as np
import pytest

from sim4wis.controller.registry import make_strategy
from sim4wis.core.state import DriverInput, EnvironmentState, VehicleParams
from sim4wis.vehicle.kinematic import KinematicModel


@pytest.fixture
def model() -> KinematicModel:
    return KinematicModel(VehicleParams())


def test_straight_line(model: KinematicModel) -> None:
    """With zero steering and full throttle, vehicle should move straight at v_max."""
    strat = make_strategy("ideal_ackermann", model.params)
    env = EnvironmentState()
    dt = 0.005
    driver = DriverInput(throttle=1.0, steering=0.0)
    for _ in range(int(1.0 / dt)):  # 1 second
        cmd = strat.compute(driver, model.state)
        model.step(dt, cmd, env)
    # Should be ~v_max metres ahead of origin in world X.
    assert abs(model.state.x - model.params.v_max) < 0.05
    assert abs(model.state.y) < 0.01
    assert abs(model.state.psi) < 0.001


def test_ideal_ackermann_circle(model: KinematicModel) -> None:
    """Steady-state circular driving with ideal Ackermann should produce a closed
    circle of the expected radius."""
    strat = make_strategy("ideal_ackermann", model.params)
    env = EnvironmentState()
    dt = 0.005
    driver = DriverInput(throttle=0.3, steering=0.3)
    # The ICR target tells us the expected radius (= |y_R|).
    cmd0 = strat.compute(driver, model.state)
    expected_radius = abs(cmd0.icr_target_body[1])

    # Simulate ~one revolution
    omega = model.params.v_max * driver.throttle / expected_radius
    T = 2 * np.pi / omega
    steps = int(T / dt)
    for _ in range(steps):
        cmd = strat.compute(driver, model.state)
        model.step(dt, cmd, env)

    # Should be back to the starting pose, approximately.
    assert abs(model.state.x) < expected_radius * 0.03
    assert abs(model.state.y) < expected_radius * 0.03


def test_crab_pure_lateral(model: KinematicModel) -> None:
    """Crab at ±90° throttle 1 should translate the vehicle without yaw."""
    strat = make_strategy("crab", model.params)
    env = EnvironmentState()
    dt = 0.005
    driver = DriverInput(throttle=1.0, steering=1.0)  # δ = +steer_limit
    initial_psi = model.state.psi
    for _ in range(int(0.5 / dt)):
        cmd = strat.compute(driver, model.state)
        model.step(dt, cmd, env)
    # ψ should be unchanged (no yaw in crab)
    assert abs(model.state.psi - initial_psi) < 1e-6
    # Lateral motion present (y or x changed in body-aligned direction)
    assert (model.state.x**2 + model.state.y**2) > 0.1


def test_zero_radius_no_translation(model: KinematicModel) -> None:
    """Zero-radius spin should leave (x, y) ~ unchanged but ψ rotates."""
    strat = make_strategy("zero_radius", model.params)
    env = EnvironmentState()
    dt = 0.005
    driver = DriverInput(throttle=0.5, steering=+1.0)
    for _ in range(int(0.5 / dt)):
        cmd = strat.compute(driver, model.state)
        model.step(dt, cmd, env)
    # Body origin should barely move (numerical drift only)
    assert abs(model.state.x) < 0.01
    assert abs(model.state.y) < 0.01
    # But ψ should have changed appreciably
    assert abs(model.state.psi) > 0.1
