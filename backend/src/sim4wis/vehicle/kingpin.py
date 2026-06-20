"""Kingpin steering-resistance torque from contact-patch forces + suspension geometry.

Industry-standard estimation following Reimpell §3.10 / Pacejka §9, with the
mechanical and pneumatic trail kept as *separate* channels (Steering Handbook
Ch.6 / change-report §5.5):

    τ_steer = Fy · (scrub + mechanical_trail)      ← lateral force, MECHANICAL trail
            + Fx · scrub_radius                     ← driving/braking torque transfer
            − Mz_self_aligning                      ← tyre pneumatic SAT (own channel)
            + Fz · sin(KPI) · scrub · sin(δ)        ← kingpin-inclination jacking

where mechanical_trail = tire_radius · tan(caster) is the caster trail.

Pneumatic vs mechanical trail — DO NOT double-count
---------------------------------------------------
The tyre's self-aligning torque ``Mz`` (from ``tire.py``) is ``Mz = −Fy·t_p``,
i.e. it *already* carries the pneumatic-trail contribution, and in the Pacejka
model ``t_p`` decays with slip (``t_p(α)``). Therefore the pneumatic trail must
enter the kingpin moment ONLY through ``Mz`` and must NOT also be folded into
the Fy lever arm — doing both double-counts it. An earlier version added a
constant ``t_pneumatic_extra`` into the Fy lever *and* summed ``Mz``; since the
lever used a constant ``t_p`` while ``Mz`` used the slip-decayed ``t_p(α)``,
the two nearly cancelled and the pneumatic self-centring all but vanished at
small angles (exactly where δ_eq lives). Removed.

Sign of the Mz channel
----------------------
The Fy/Fx/KPI terms above are written as *actuator effort* (τ_steer > 0 means
the actuator must work CCW, i.e. it opposes the tyre's self-centring). The
tyre's ``Mz = −Fy·t_p`` is the raw aligning moment, which has the OPPOSITE sign
to "actuator effort"; the pneumatic trail is self-centring just like the
mechanical (caster) trail, so its contribution to τ_steer is ``−Mz`` (which
reinforces the ``Fy·mechanical_trail`` term rather than cancelling it).

Sign convention: τ_steer > 0 means the actuator must work CCW (positive
δ direction). The driver "feels" this as steering effort.

The KPI jacking term is `∝ sin(δ)` so it is 0 straight-ahead and grows with
steer — the wheel only "lifts the car" once turned. The earlier model used a
constant `Fz·sin(KPI)·scrub` offset, which swamped the lateral-force term at
small angles and made τ non-monotonic.

What is deliberately NOT modelled here:
    * Fx coupling through a tilted kingpin axis (caster/KPI rotated frame).
      In a strict cross-product derivation this contributes additional
      `−sin(caster)·scrub·Fz` and `±sin(KPI)·trail·Fz` style terms; for our
      sizing-grade use these are absorbed into the KPI jacking term above and
      the experimentally calibrated Reimpell coefficients.
    * Self-steer from drive force at non-zero δ via mechanical_trail. An
      earlier version had `m_from_fx_caster = −Fx·trail·sin(δ)` but that
      cross-product is geometrically null (Fx ∥ trail for a longitudinal
      trail offset) — it was removed.
"""

from __future__ import annotations

import math

import numpy as np

from sim4wis.core.state import SuspensionParams


def kingpin_torque_terms(
    fx: np.ndarray,         # (4,) tyre Fx in wheel-aligned frame [N]
    fy: np.ndarray,         # (4,) tyre Fy in wheel-aligned frame [N]
    mz: np.ndarray,         # (4,) tyre self-aligning torque [N·m]
    fz: np.ndarray,         # (4,) tyre vertical load [N]
    suspension: SuspensionParams,
    *,
    delta: np.ndarray | None = None,  # (4,) actual steer angles [rad]
    tire_radius: float = 0.33,
) -> dict[str, np.ndarray]:
    """Per-wheel kingpin moment broken into its four physical contributions.

    Single source of truth for both ``kingpin_torque`` (which sums these) and
    the model-doc page's term-decomposition demo.

    Returns dict of length-4 arrays:
        m_fy  — lateral force × (scrub + caster MECHANICAL trail)
        m_fx  — longitudinal force × scrub radius
        m_mz  — pneumatic self-aligning torque as steering resistance (−Mz);
                reinforces m_fy (mechanical + pneumatic trail both self-centre)
        m_kpi — kingpin-inclination jacking, ∝ sin(δ)
    """
    scrub = suspension.scrub_radius
    caster = suspension.caster_angle
    kpi = suspension.kingpin_inclination
    if delta is None:
        delta = np.zeros_like(fy)

    # 1) Lateral force × effective lever (scrub + caster MECHANICAL trail only).
    #    Pneumatic trail is NOT added here — it lives entirely in the Mz channel
    #    below (see module docstring: avoids double-counting t_p).
    mechanical_trail = tire_radius * math.tan(caster)
    lever_y = scrub + mechanical_trail
    m_from_fy = fy * lever_y

    # 2) Driving/braking torque around kingpin via scrub radius.
    m_from_fx = fx * scrub

    # 3) Tyre self-aligning torque (pneumatic-trail channel). Mz = −Fy·t_p(α) is
    #    the raw aligning moment; its contribution to steering *effort* is −Mz,
    #    so it adds to the mechanical-trail self-centring rather than opposing it.
    m_from_mz = -np.asarray(mz, dtype=np.float64)

    # 4) Kingpin-inclination jacking — the wheel "lifts the car" as it turns,
    #    so the restoring moment grows with sin(δ) (≈0 straight-ahead).
    m_from_kpi = fz * math.sin(kpi) * scrub * np.sin(delta)

    return {
        "m_fy": m_from_fy,
        "m_fx": m_from_fx,
        "m_mz": m_from_mz,
        "m_kpi": m_from_kpi,
    }


def kingpin_torque(
    fx: np.ndarray,         # (4,) tyre Fx in wheel-aligned frame [N]
    fy: np.ndarray,         # (4,) tyre Fy in wheel-aligned frame [N]
    mz: np.ndarray,         # (4,) tyre self-aligning torque [N·m]
    fz: np.ndarray,         # (4,) tyre vertical load [N]
    suspension: SuspensionParams,
    *,
    delta: np.ndarray | None = None,  # (4,) actual steer angles [rad]
    tire_radius: float = 0.33,
) -> np.ndarray:
    """Compute per-wheel kingpin moment (4,) — sum of the four physical terms."""
    terms = kingpin_torque_terms(
        fx, fy, mz, fz, suspension,
        delta=delta, tire_radius=tire_radius,
    )
    return terms["m_fy"] + terms["m_fx"] + terms["m_mz"] + terms["m_kpi"]
