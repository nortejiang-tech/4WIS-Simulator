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


def test_mz_enters_as_negative() -> None:
    """Pneumatic SAT enters τ as −Mz (report §5.5).

    The tyre's Mz = −Fy·t_p is the raw aligning moment; its contribution to
    steering *effort* is −Mz, so it reinforces (not cancels) the Fy·mechanical-
    trail self-centring.
    """
    mz = np.full(4, -30.0)
    tau = kingpin_torque(fx=ZEROS, fy=ZEROS, mz=mz, fz=ZEROS,
                         suspension=SuspensionParams(), delta=ZEROS)
    assert np.allclose(tau, 30.0)


def test_pneumatic_and_mechanical_trail_reinforce() -> None:
    """The pneumatic SAT and mechanical (caster) trail must self-centre in the
    SAME direction — including Mz must INCREASE |τ|, never shrink it.

    Guards against the prior cancellation bug where t_p was added to the Fy
    lever AND Mz was summed with the wrong sign, nulling the pneumatic effect.
    """
    susp = SuspensionParams()
    fy = np.full(4, -6000.0)          # cornering lateral force
    t_p = 0.03
    mz = -fy * t_p                    # tyre raw SAT = −Fy·t_p
    tau_mech_only = kingpin_torque(fx=ZEROS, fy=fy, mz=ZEROS, fz=ZEROS,
                                   suspension=susp, delta=ZEROS, tire_radius=0.395)
    tau_with_pneu = kingpin_torque(fx=ZEROS, fy=fy, mz=mz, fz=ZEROS,
                                   suspension=susp, delta=ZEROS, tire_radius=0.395)
    # same sign, and the pneumatic channel strictly increases the magnitude
    assert np.all(np.sign(tau_with_pneu) == np.sign(tau_mech_only))
    assert np.all(np.abs(tau_with_pneu) > np.abs(tau_mech_only))
