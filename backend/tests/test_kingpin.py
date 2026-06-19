"""Unit tests — kingpin steering-resistance torque."""

from __future__ import annotations

import numpy as np

from sim4wis.core.state import SuspensionParams
from sim4wis.vehicle.kingpin import kingpin_torque

FZ = np.full(4, 7100.0)
ZEROS = np.zeros(4)


def test_zero_forces_straight_ahead_zero_torque() -> None:
    tau = kingpin_torque(fx=ZEROS, fy=ZEROS, mz=ZEROS, fz=FZ,
                         suspension=SuspensionParams(), delta=ZEROS)
    assert np.allclose(tau, 0.0)


def test_kpi_term_monotonic_in_delta() -> None:
    """With pure vertical load, τ grows monotonically with δ (KPI centring)."""
    susp = SuspensionParams()
    prev = 0.0
    for d in (0.05, 0.15, 0.3, 0.5):
        tau = kingpin_torque(fx=ZEROS, fy=ZEROS, mz=ZEROS, fz=FZ,
                             suspension=susp, delta=np.full(4, d))
        assert tau[0] > prev
        prev = float(tau[0])


def test_lateral_force_lever() -> None:
    susp = SuspensionParams()
    fy = np.full(4, 1000.0)
    tau = kingpin_torque(fx=ZEROS, fy=fy, mz=ZEROS, fz=ZEROS,
                         suspension=susp, delta=ZEROS, tire_radius=0.395)
    lever = susp.scrub_radius + 0.395 * np.tan(susp.caster_angle)
    assert np.allclose(tau, 1000.0 * lever, rtol=1e-9)


def test_fx_scrub_term() -> None:
    susp = SuspensionParams()
    fx = np.full(4, 2000.0)
    tau = kingpin_torque(fx=fx, fy=ZEROS, mz=ZEROS, fz=ZEROS,
                         suspension=susp, delta=ZEROS)
    assert np.allclose(tau, 2000.0 * susp.scrub_radius, rtol=1e-9)


def test_mz_passthrough() -> None:
    mz = np.full(4, -30.0)
    tau = kingpin_torque(fx=ZEROS, fy=ZEROS, mz=mz, fz=ZEROS,
                         suspension=SuspensionParams(), delta=ZEROS)
    assert np.allclose(tau, -30.0)
