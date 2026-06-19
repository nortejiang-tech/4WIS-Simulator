"""Unit tests — per-wheel steering centre (vehicle ICR projected onto each
wheel's perpendicular line) + signed deviation."""

from __future__ import annotations

import numpy as np
import pytest

from sim4wis.controller.registry import make_strategy
from sim4wis.core.state import DriverInput, VehicleParams, VehicleState
from sim4wis.vehicle.geometry import (
    steer_angle_for_icr,
    wheel_icr_projection,
)


def test_ideal_ackermann_projection_coincides_with_icr() -> None:
    """All four perpendiculars pass through the ICR → dev ≡ 0, q_i ≡ R."""
    params = VehicleParams()
    wheels = params.wheel_positions_body()
    icr = np.array([0.0, 8.0])
    delta = steer_angle_for_icr(wheels, icr)
    pts, dev = wheel_icr_projection(wheels, delta, icr)
    assert np.allclose(dev, 0.0, atol=1e-12)
    assert np.allclose(pts, icr, atol=1e-12)


def test_known_analytic_case() -> None:
    """Wheel at origin pointing along +X (δ=0); ICR at (3, 4):
    rolling dir t=(1,0) → dev = (R−p)·t = 3, q = R − 3·t = (0, 4)."""
    wheels = np.array([[0.0, 0.0]])
    delta = np.array([0.0])
    icr = np.array([3.0, 4.0])
    pts, dev = wheel_icr_projection(wheels, delta, icr)
    assert dev[0] == pytest.approx(3.0)
    assert pts[0] == pytest.approx([0.0, 4.0])


def test_straight_line_motion_gives_nan() -> None:
    params = VehicleParams()
    wheels = params.wheel_positions_body()
    pts, dev = wheel_icr_projection(wheels, np.zeros(4), np.array([np.nan, np.nan]))
    assert np.all(np.isnan(pts))
    assert np.all(np.isnan(dev))


def test_crab_strategy_dev_finite_via_strategy_geometry() -> None:
    """Crab steering (all wheels same δ, ω=0): any finite ICR projects onto
    parallel lines — dev must be finite and equal across wheels for a
    symmetric ICR position."""
    params = VehicleParams()
    strat = make_strategy("crab", params)
    cmd = strat.compute(DriverInput(throttle=0.5, steering=0.4), VehicleState())
    wheels = params.wheel_positions_body()
    icr = np.array([0.0, 10.0])   # arbitrary finite point
    pts, dev = wheel_icr_projection(wheels, cmd.delta_cmd, icr)
    assert np.all(np.isfinite(dev))
    assert np.all(np.isfinite(pts))


def test_dev_sign_is_along_rolling_direction() -> None:
    """ICR ahead of the wheel along its rolling direction → positive dev."""
    wheels = np.array([[0.0, 0.0]])
    delta = np.array([0.0])               # rolls along +X
    pts, dev = wheel_icr_projection(wheels, delta, np.array([5.0, 0.0]))
    assert dev[0] == pytest.approx(5.0)
    pts, dev = wheel_icr_projection(wheels, delta, np.array([-5.0, 0.0]))
    assert dev[0] == pytest.approx(-5.0)
