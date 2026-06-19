"""Unit tests — pure-pursuit curvature speed planning + parameter overrides."""

from __future__ import annotations

import numpy as np
import pytest

from sim4wis.controller.follow_trajectory import FollowTrajectoryStrategy
from sim4wis.controller.path import plan_from_template, set_active_plan, clear_active_plan
from sim4wis.core.state import DriverInput, VehicleParams, VehicleState


@pytest.fixture(autouse=True)
def _clean_path():
    yield
    clear_active_plan()


def _wheel_speed_to_v(cmd, params: VehicleParams) -> float:
    return float(np.mean(cmd.wheel_speed_cmd)) * params.tire_radius


def test_tight_arc_caps_speed() -> None:
    """On a small-radius arc, v_cmd must respect v ≤ sqrt(ay_max/κ)."""
    params = VehicleParams()
    set_active_plan(plan_from_template("arc", {"radius": 8.0}))
    strat = FollowTrajectoryStrategy(params)
    state = VehicleState()   # at origin; template starts near origin
    driver = DriverInput(throttle=1.0)   # ask for v_max
    cmd = strat.compute(driver, state)
    v = _wheel_speed_to_v(cmd, params)
    ay_max = FollowTrajectoryStrategy.AY_MAX
    kappa = 1.0 / 8.0
    v_cap = (ay_max / kappa) ** 0.5
    assert v <= v_cap * 1.15   # small tolerance: wheel speeds ≠ body speed exactly


def test_straight_path_not_throttled() -> None:
    params = VehicleParams()
    set_active_plan(plan_from_template("straight", {}))
    strat = FollowTrajectoryStrategy(params)
    cmd = strat.compute(DriverInput(throttle=0.0, mode_params={"cruise_speed": 6.0}),
                        VehicleState())
    v = _wheel_speed_to_v(cmd, params)
    assert v == pytest.approx(6.0, rel=0.1)


def test_mode_params_override_ay_max() -> None:
    """A tighter ay_max must produce a lower speed cap on the same arc."""
    params = VehicleParams()
    set_active_plan(plan_from_template("arc", {"radius": 8.0}))
    strat = FollowTrajectoryStrategy(params)
    v_hi = _wheel_speed_to_v(
        strat.compute(DriverInput(throttle=1.0, mode_params={"ay_max": 6.0}), VehicleState()),
        params)
    v_lo = _wheel_speed_to_v(
        strat.compute(DriverInput(throttle=1.0, mode_params={"ay_max": 1.5}), VehicleState()),
        params)
    assert v_lo < v_hi


def test_no_path_degenerates_to_straight() -> None:
    params = VehicleParams()
    clear_active_plan()
    strat = FollowTrajectoryStrategy(params)
    cmd = strat.compute(DriverInput(throttle=0.3), VehicleState())
    assert np.allclose(cmd.delta_cmd, 0.0)
