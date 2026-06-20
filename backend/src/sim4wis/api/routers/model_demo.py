"""Teaching-only, read-only endpoints for the model-doc page.

Every curve here is produced by the *same* foundation the rest of the app uses
(``vehicle.model_core`` / ``vehicle.tire`` / ``vehicle.kingpin`` /
``vehicle.load_analysis``), so the explanation page demonstrates the real model
rather than reimplementing formulas on the frontend. The page's own stated
boundary — "frontend charts consume backend results, they don't reimplement
physics" — is honoured here.

All three endpoints accept an optional ``params`` override (same shape as
``/api/params``); when omitted they use the running simulator's params.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from fastapi import APIRouter
from pydantic import BaseModel, Field

from sim4wis.core.simulator import get_simulator
from sim4wis.core.state import VehicleParams
from sim4wis.project.params_codec import params_from_dict
from sim4wis.vehicle.kingpin import kingpin_torque_terms
from sim4wis.vehicle.load_analysis import sweep_load_analysis
from sim4wis.vehicle.load_transfer import vertical_loads
from sim4wis.vehicle.model_core import (
    G_ACCEL,
    load_sensitive_cornering_stiffness,
    quasi_static_wheel_loads,
    solve_steady_state_body,
)
from sim4wis.vehicle.tire import pacejka_combined_forces

router = APIRouter()


def _resolve_params(raw: dict[str, Any] | None) -> VehicleParams:
    sim = get_simulator()
    return params_from_dict(raw, sim.params) if raw else sim.params


# ---------------------------------------------------------------------------
# Demo 1: bicycle slip gain & linear-region width vs speed
# ---------------------------------------------------------------------------


class BicycleGainRequest(BaseModel):
    params: dict[str, Any] | None = None
    speeds_kmh: list[float] = Field(default_factory=lambda: list(range(0, 205, 10)))
    ref_delta_deg: float = Field(1.0, gt=0.0, le=10.0)
    wheel_index: int = Field(0, ge=0, le=3)
    alpha_peak_deg: float = Field(7.0, gt=1.0, le=20.0)  # reference slip at tyre peak


@router.post("/model/demo/bicycle-gain")
async def bicycle_gain(body: BicycleGainRequest) -> dict[str, Any]:
    """Slip gain |α_wheel/δ| and implied saturation steer δ_sat vs speed.

    For a single steered wheel the steady-state bicycle solve gives the body
    (β, r), hence α_wheel = β + r·x/V − δ. The gain grows ~(1 + mV²/(8CαL)),
    so the linear region (δ_sat = α_peak/gain) shrinks with speed — exactly the
    high-speed behaviour the load page exposes.
    """
    p = _resolve_params(body.params)
    wp = p.wheel_positions_body()
    wheel_index = int(np.clip(body.wheel_index, 0, 3))
    fz_static = vertical_loads(p, ax=0.0, ay=0.0)
    ref_delta = math.radians(body.ref_delta_deg)
    alpha_peak = math.radians(body.alpha_peak_deg)

    points: list[dict[str, Any]] = []
    for v_kmh in body.speeds_kmh[:80]:
        v = max(float(v_kmh) / 3.6, 1e-3)
        loads = quasi_static_wheel_loads(p, fz_static, v)
        deltas = np.zeros(4)
        deltas[wheel_index] = ref_delta
        beta, yaw_rate = solve_steady_state_body(
            speed=v, delta=deltas, c_alpha=loads.c_alpha,
            wheel_positions_body=wp, mass=float(p.mass),
        )
        alpha_w = beta + yaw_rate * float(wp[wheel_index, 0]) / v - ref_delta
        gain = abs(alpha_w / ref_delta) if ref_delta > 1e-9 else 0.0
        delta_sat_deg = math.degrees(alpha_peak / gain) if gain > 1e-9 else float("nan")
        points.append({
            "speed_kmh": float(v_kmh),
            "slip_gain": float(gain),
            "delta_sat_deg": float(delta_sat_deg),
        })
    return {"wheel_index": wheel_index, "ref_delta_deg": body.ref_delta_deg, "points": points}


# ---------------------------------------------------------------------------
# Demo 2: tyre Fy vs slip angle, parameterised by Fz / mu
# ---------------------------------------------------------------------------


class TireCurveRequest(BaseModel):
    params: dict[str, Any] | None = None
    fz: float = Field(7000.0, gt=100.0, le=40000.0)
    mu: float = Field(0.85, gt=0.05, le=2.0)
    alpha_max_deg: float = Field(15.0, gt=1.0, le=40.0)
    points: int = Field(81, ge=11, le=241)


@router.post("/model/demo/tire-curve")
async def tire_curve(body: TireCurveRequest) -> dict[str, Any]:
    """Pacejka Fy(α) at a chosen vertical load — the real ``pacejka_combined_forces``.

    Cornering stiffness is the load-sensitive value at this Fz, so the small-slip
    slope and the μ·Fz peak both move with load just like the foundation model.
    """
    p = _resolve_params(body.params)
    c_alpha = float(load_sensitive_cornering_stiffness(p, np.full(4, body.fz))[0])
    c_kappa = float(p.tire_c_kappa)
    cx = float(getattr(p, "tire_cx", 1.65))
    cy = float(getattr(p, "tire_cy", 1.30))
    ex = float(getattr(p, "tire_ex", -0.5))
    ey = float(getattr(p, "tire_ey", -1.0))

    n = int(np.clip(body.points, 11, 241))
    alphas = np.linspace(-math.radians(body.alpha_max_deg), math.radians(body.alpha_max_deg), n)
    cap = body.mu * body.fz
    curve = []
    for a in alphas:
        _fx, fy = pacejka_combined_forces(
            alpha=float(a), kappa=0.0, fz=body.fz, mu=body.mu,
            c_alpha=c_alpha, c_kappa=c_kappa, cx=cx, cy=cy, ex=ex, ey=ey,
        )
        curve.append({"alpha_deg": math.degrees(float(a)), "fy": float(fy)})
    return {
        "fz": body.fz, "mu": body.mu, "c_alpha": c_alpha,
        "mu_fz": float(cap), "curve": curve,
    }


# ---------------------------------------------------------------------------
# Demo 3: kingpin torque four-term decomposition vs steer angle
# ---------------------------------------------------------------------------


class KingpinBreakdownRequest(BaseModel):
    params: dict[str, Any] | None = None
    speed_kmh: float = Field(30.0, ge=0.0, le=250.0)
    angle_max_deg: float = Field(20.0, gt=1.0, le=45.0)
    points: int = Field(61, ge=11, le=161)
    wheel_index: int = Field(0, ge=0, le=3)
    mu: float = Field(0.85, gt=0.05, le=2.0)
    body_coupling: str = Field("vehicle", pattern="^(vehicle|isolated)$")


@router.post("/model/demo/kingpin-breakdown")
async def kingpin_breakdown(body: KingpinBreakdownRequest) -> dict[str, Any]:
    """τ_kingpin split into Fy / Fx / Mz / KPI terms vs δ for one wheel.

    Reuses ``sweep_load_analysis`` to produce the *real* per-δ tyre forces
    (Fx, Fy, Mz, Fz) — exactly what the load page shows — then decomposes the
    kingpin moment with ``kingpin_torque_terms`` (the same helper ``kingpin_torque``
    sums). No physics is duplicated.
    """
    p = _resolve_params(body.params)
    wheel_index = int(np.clip(body.wheel_index, 0, 3))
    speed = float(body.speed_kmh) / 3.6
    n = int(np.clip(body.points, 11, 161))
    amax = math.radians(body.angle_max_deg)
    angles = list(np.linspace(-amax, amax, n))

    sweep = sweep_load_analysis(
        p, speeds=[speed], angles=angles, wheel_index=wheel_index,
        mode="single_wheel", mu=body.mu,
        body_coupling=body.body_coupling,
    )
    rows = [r for r in sweep["rows"] if int(r["wheel_index"]) == wheel_index]
    rows.sort(key=lambda r: float(r["delta_cmd"]))

    out = []
    for r in rows:
        terms = kingpin_torque_terms(
            fx=np.array([float(r["tire_fx"])]),
            fy=np.array([float(r["tire_fy"])]),
            mz=np.array([float(r["tire_mz"])]),
            fz=np.array([float(r["fz"])]),
            suspension=p.suspension,
            delta=np.array([float(r["delta"])]),
            tire_radius=p.tire_radius,
            t_pneumatic_extra=p.tire_t_pneumatic,
        )
        out.append({
            "delta_cmd_deg": math.degrees(float(r["delta_cmd"])),
            "m_fy": float(terms["m_fy"][0]),
            "m_fx": float(terms["m_fx"][0]),
            "m_mz": float(terms["m_mz"][0]),
            "m_kpi": float(terms["m_kpi"][0]),
            "torque_steer": float(r["torque_steer"]),
        })
    return {"wheel_index": wheel_index, "speed_kmh": body.speed_kmh, "curve": out}
