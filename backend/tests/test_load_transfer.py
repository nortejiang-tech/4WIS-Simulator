"""Unit tests — quasi-static vertical load distribution."""

from __future__ import annotations

import numpy as np
import pytest

from sim4wis.core.state import VehicleParams
from sim4wis.vehicle.load_transfer import vertical_loads

G = 9.81


@pytest.fixture
def params() -> VehicleParams:
    return VehicleParams()


def test_static_total_equals_weight(params: VehicleParams) -> None:
    fz = vertical_loads(params, ax=0.0, ay=0.0)
    assert float(np.sum(fz)) == pytest.approx(params.mass * G, rel=1e-9)


def test_static_front_rear_split(params: VehicleParams) -> None:
    fz = vertical_loads(params, 0.0, 0.0)
    b = params.wheelbase - params.cg_to_front
    front_share = (fz[0] + fz[1]) / (params.mass * G)
    assert front_share == pytest.approx(b / params.wheelbase, rel=1e-9)
    # Left/right symmetric
    assert fz[0] == pytest.approx(fz[1])
    assert fz[2] == pytest.approx(fz[3])


def test_forward_accel_shifts_to_rear(params: VehicleParams) -> None:
    fz = vertical_loads(params, ax=3.0, ay=0.0)
    fz0 = vertical_loads(params, 0.0, 0.0)
    assert fz[2] > fz0[2] and fz[3] > fz0[3]   # rear gains
    assert fz[0] < fz0[0] and fz[1] < fz0[1]   # front loses
    assert float(np.sum(fz)) == pytest.approx(params.mass * G, rel=1e-9)


def test_left_accel_shifts_to_right(params: VehicleParams) -> None:
    fz = vertical_loads(params, ax=0.0, ay=4.0)   # ay > 0 = leftward accel
    fz0 = vertical_loads(params, 0.0, 0.0)
    assert fz[1] > fz0[1] and fz[3] > fz0[3]   # right side gains
    assert fz[0] < fz0[0] and fz[2] < fz0[2]


def test_loads_never_negative(params: VehicleParams) -> None:
    fz = vertical_loads(params, ax=-50.0, ay=50.0)   # absurd accel
    assert np.all(fz >= 0.0)
