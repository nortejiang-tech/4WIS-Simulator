"""Kingpin steering-resistance torque from contact-patch forces + suspension geometry.

Industry-standard estimation following Reimpell §3.10 / Pacejka §9:

    τ_steer = Fy · (scrub + mechanical_trail)      ← lateral force self-centering
            + Fx · scrub_radius                     ← driving/braking torque transfer
            + Mz_self_aligning                      ← tyre pneumatic SAT
            + Fz · sin(KPI) · scrub · sin(δ)        ← kingpin-inclination jacking

where mechanical_trail = tire_radius · tan(caster) is the caster trail.

Note on the Fy lever arm: a strict 3D moment-about-kingpin-axis derivation
gives `Fy · trail − Fx · scrub` for a (caster, KPI) tilted axis, with second-
order Fz cross-terms. Reimpell's "Fy · (scrub + trail)" lumps the first-order
3D coupling into a single effective lever; it overestimates the Fy moment by
~30% at LS9-class scrub/trail but matches the industry convention used in
EPS sizing tables. We keep it for that reason; if you need a strict derivation
write a multibody force-element block instead.

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
    t_pneumatic_extra: float = 0.0,   # extra mechanical trail (cumulative)
) -> dict[str, np.ndarray]:
    """Per-wheel kingpin moment broken into its four physical contributions.

    Single source of truth for both ``kingpin_torque`` (which sums these) and
    the model-doc page's term-decomposition demo.

    Returns dict of length-4 arrays:
        m_fy  — lateral force × (scrub + mechanical + pneumatic trail)
        m_fx  — longitudinal force × scrub radius
        m_mz  — tyre self-aligning torque (pass-through)
        m_kpi — kingpin-inclination jacking, ∝ sin(δ)
    """
    scrub = suspension.scrub_radius
    caster = suspension.caster_angle
    kpi = suspension.kingpin_inclination
    if delta is None:
        delta = np.zeros_like(fy)

    # 1) Lateral force × effective lever (Reimpell scrub + caster mechanical trail).
    mechanical_trail = tire_radius * math.tan(caster) + t_pneumatic_extra
    lever_y = scrub + mechanical_trail
    m_from_fy = fy * lever_y

    # 2) Driving/braking torque around kingpin via scrub radius.
    m_from_fx = fx * scrub

    # 3) Tyre self-aligning torque (pneumatic trail effect; pass through).
    m_from_mz = np.asarray(mz, dtype=np.float64)

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
    t_pneumatic_extra: float = 0.0,   # extra mechanical trail (cumulative)
) -> np.ndarray:
    """Compute per-wheel kingpin moment (4,) — sum of the four physical terms."""
    terms = kingpin_torque_terms(
        fx, fy, mz, fz, suspension,
        delta=delta, tire_radius=tire_radius, t_pneumatic_extra=t_pneumatic_extra,
    )
    return terms["m_fy"] + terms["m_fx"] + terms["m_mz"] + terms["m_kpi"]
