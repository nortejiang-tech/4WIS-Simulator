"""Manual gamepad strategies — direct per-wheel + holonomic body motion."""

from __future__ import annotations

import numpy as np
import pytest

from sim4wis.controller.registry import make_strategy
from sim4wis.core.state import DriverInput, VehicleParams, VehicleState


@pytest.fixture
def params() -> VehicleParams:
    return VehicleParams()


def test_manual_wheel_passes_normalised_angles(params) -> None:
    strat = make_strategy("manual_wheel", params)
    # Front/rear-independent grouping: front +0.5, rear −0.3.
    driver = DriverInput(throttle=0.4, mode_params={"wheel_norm": [0.5, 0.5, -0.3, -0.3]})
    cmd = strat.compute(driver, VehicleState())
    lim = params.steer_limit
    assert cmd.delta_cmd[0] == pytest.approx(0.5 * lim)
    assert cmd.delta_cmd[1] == pytest.approx(0.5 * lim)
    assert cmd.delta_cmd[2] == pytest.approx(-0.3 * lim)
    assert cmd.delta_cmd[3] == pytest.approx(-0.3 * lim)
    # Throttle → equal wheel spins.
    v = 0.4 * params.v_max
    assert np.allclose(cmd.wheel_speed_cmd, v / params.tire_radius)


def test_manual_wheel_clips_and_defaults(params) -> None:
    strat = make_strategy("manual_wheel", params)
    over = strat.compute(DriverInput(mode_params={"wheel_norm": [9, -9, 0, 0]}), VehicleState())
    assert over.delta_cmd[0] == pytest.approx(params.steer_limit)
    assert over.delta_cmd[1] == pytest.approx(-params.steer_limit)
    # Missing mode_params → straight, no crash.
    zero = strat.compute(DriverInput(), VehicleState())
    assert np.allclose(zero.delta_cmd, 0.0)


def test_manual_wheel_per_wheel_grouping_is_independent(params) -> None:
    """Per-wheel layout: each wheel a distinct angle (kinematically free)."""
    strat = make_strategy("manual_wheel", params)
    cmd = strat.compute(
        DriverInput(mode_params={"wheel_norm": [0.2, -0.4, 0.6, -0.1]}), VehicleState()
    )
    lim = params.steer_limit
    assert list(np.round(cmd.delta_cmd / lim, 3)) == [0.2, -0.4, 0.6, -0.1]


def test_manual_body_pure_crab(params) -> None:
    """vy only → all wheels align to the crab direction, zero yaw."""
    strat = make_strategy("manual_body", params)
    cmd = strat.compute(
        DriverInput(mode_params={"vx_frac": 0.0, "vy_frac": 0.5, "yaw_frac": 0.0}),
        VehicleState(),
    )
    # Pure lateral motion → every wheel steered near ±90° (arctan2(vy,0)).
    assert np.all(np.abs(np.abs(cmd.delta_cmd) - np.pi / 2) < 0.2) or \
        np.allclose(np.abs(cmd.delta_cmd), params.steer_limit, atol=0.2)


def test_manual_body_forward_plus_yaw_curves(params) -> None:
    """vx + yaw → a finite ICR (curved path), not straight."""
    strat = make_strategy("manual_body", params)
    cmd = strat.compute(
        DriverInput(mode_params={"vx_frac": 0.4, "vy_frac": 0.0, "yaw_frac": 0.3}),
        VehicleState(),
    )
    assert np.all(np.isfinite(cmd.icr_target_body))
    # Front and rear wheels take different angles under combined vx+yaw.
    assert abs(cmd.delta_cmd[0] - cmd.delta_cmd[2]) > 1e-3


def test_manual_body_idle_is_straight(params) -> None:
    strat = make_strategy("manual_body", params)
    cmd = strat.compute(DriverInput(mode_params={}), VehicleState())
    assert np.allclose(cmd.delta_cmd, 0.0)
    assert not np.all(np.isfinite(cmd.icr_target_body))  # ICR at infinity
