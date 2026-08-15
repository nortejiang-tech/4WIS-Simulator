"""Tests — by-wire corner actuator sizing (direction 6).

The over-envelope study (scripts/devtools/over_envelope_study.py) produced
three numbers: the dynamic requirement (~81 N·m), the survival floor
(~60 N·m), and — as the sizing cross-check — the finding that the earlier
"104 N·m" basis mislabelled the tyre lateral force as the rack force. The
rack chain gives ~2× that. These tests pin what the sizing module now
reports, including the honest failing verdict for the current default.
"""

from __future__ import annotations

import dataclasses

from sim4wis.core.state import VehicleParams
from sim4wis.steering.sizing import (
    CORNER_OVER_ENVELOPE_FLOOR_NM,
    CORNER_OVER_ENVELOPE_PEAK_NM,
    size_corner_actuator,
)


def _params(peak_nm: float) -> VehicleParams:
    p = VehicleParams()
    st = dataclasses.replace(p.steering_system, enabled=True)
    ac = dataclasses.replace(st.angle_control, plant_peak_torque_nm=peak_nm)
    return dataclasses.replace(
        p, steering_system=dataclasses.replace(st, angle_control=ac))


def test_the_static_basis_is_the_rack_chain_not_the_tyre_force():
    out = size_corner_actuator(_params(120.0))
    # The mislabelled basis: 5.2 kN tire_fy × pinion ≈ 104 N·m.
    assert 95.0 < out["static_tire_fy_basis_nm"] < 115.0
    # The honest rack chain is ~2× that.
    assert out["static_parking_nm"] > 1.8 * out["static_tire_fy_basis_nm"]
    assert 190.0 < out["static_parking_nm"] < 230.0


def test_dynamic_over_envelope_numbers_are_pinned():
    out = size_corner_actuator(_params(120.0))
    assert out["dynamic_over_envelope_nm"] == CORNER_OVER_ENVELOPE_PEAK_NM
    assert out["over_envelope_floor_nm"] == CORNER_OVER_ENVELOPE_FLOOR_NM


def test_the_default_peak_is_undersized_against_the_static_basis():
    # The 120 N·m default covers the dynamic case but not the static rack
    # chain — the verdict says so instead of laundering it.
    out = size_corner_actuator(_params(120.0))
    assert out["driven_by"] == "parking_full_lock"
    assert not out["pass"]
    assert out["required_peak_nm"] > 120.0


def test_a_peak_covering_the_static_basis_passes():
    out = size_corner_actuator(_params(250.0))
    assert out["pass"]
    assert out["margin_pct"] > 0.0


def test_the_dynamic_requirement_is_below_the_default():
    out = size_corner_actuator(_params(120.0))
    assert out["dynamic_over_envelope_nm"] < 120.0
