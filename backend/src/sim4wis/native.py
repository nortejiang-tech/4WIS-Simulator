"""Checked adapter to the optional C++17 numerical core.

The Python implementation remains the oracle and extension/plugin fallback.
Selection does not change control cadence, fault scheduling or solver settings.
"""

from __future__ import annotations

import os
from contextlib import contextmanager
from contextvars import ContextVar

import numpy as np

from sim4wis.vehicle.tire import (
    _BUILTIN_FORCE_METHODS,
    _BUILTIN_STIFFNESS_METHODS,
    PacejkaTireModel,
)

try:
    from sim4wis import _native
except ImportError:
    _native = None

if _native is not None and _native.ABI_VERSION != 1:
    raise RuntimeError("incompatible sim4wis native core ABI; rebuild the extension")

_override: ContextVar[str | None] = ContextVar("sim4wis_engine", default=None)
_tire_methods = _BUILTIN_FORCE_METHODS
_stiffness_methods = _BUILTIN_STIFFNESS_METHODS


def requested_engine() -> str:
    mode = _override.get() or os.environ.get("SIM4WIS_ENGINE", "auto")
    if mode not in {"auto", "python", "native"}:
        raise ValueError("SIM4WIS_ENGINE must be auto, python or native")
    return mode


def core():
    mode = requested_engine()
    if mode == "python":
        return None
    if _native is None and mode == "native":
        raise RuntimeError(
            "native core requested but unavailable; install a native wheel or build backend"
        )
    return _native


@contextmanager
def use_engine(mode: str):
    """Context-local selection for differential tests/benchmarks, without global mutation."""
    if mode not in {"auto", "python", "native"}:
        raise ValueError("unknown simulation engine")
    token = _override.set(mode)
    try:
        core()  # fail immediately for a requested but missing native build
        yield
    finally:
        _override.reset(token)


def engine_info(model=None) -> dict:
    lib = core()
    info = {
        "requested": requested_engine(),
        "native_available": _native is not None,
        "numerical_core": "cpp17" if lib is not None else "python",
        "abi": 1 if lib is not None else None,
        "native_source_sha256": getattr(lib, "SOURCE_SHA256", None),
        "integration": "RK4 body + nonlinear backward-Euler wheel spin; partitioned",
    }
    if model is not None:
        info["body_core"] = getattr(model, "_body_core", "python")
        info["body_fallback_reason"] = getattr(model, "_body_fallback_reason", None)
    return info


def tire_parameters(tire):
    # Exact types AND original methods: injected/custom force laws remain Python.
    cls = type(tire)
    if (
        cls not in _tire_methods
        or getattr(tire.forces, "__func__", None) is not _tire_methods[cls]
        or getattr(tire.c_alpha_eff, "__func__", None) is not _stiffness_methods[cls]
    ):
        return None
    return np.array(
        [
            float(cls is PacejkaTireModel),
            tire.c_alpha,
            tire.c_kappa,
            tire.t_pneumatic,
            tire.fz_nom,
            tire.load_exp,
            getattr(tire, "cx", 1.65),
            getattr(tire, "cy", 1.30),
            getattr(tire, "ex", -0.5),
            getattr(tire, "ey", -1.0),
            getattr(tire, "alpha_sl", 0.2),
        ],
        dtype=np.float64,
    )


def _body_parameters(p):
    # Layout is the P enum in native/core.hpp; ABI tests cover both sides.
    return np.array(
        [
            p.mass,
            p.inertia_z,
            p.wheelbase,
            p.cg_to_front,
            p.tire_radius,
            p.rolling_resistance_coeff,
            p.air_density,
            p.drag_coeff_cd,
            p.frontal_area,
            p.camber_thrust_coeff,
            p.suspension.camber,
            p.tire_c_alpha,
            p.tire_load_sensitivity_time_domain,
            p.tire_load_sensitivity_exp,
            p.tire_mu_load_sensitivity,
            p.tire_relax_length,
            p.steer_limit,
            p.track_front,
            p.track_rear,
        ],
        dtype=np.float64,
    )


def _kc_parameters(model):
    from sim4wis.vehicle.kc import AxleKC, ComplianceCoeffs, KinematicCurve, VehicleKC

    kc = model._kc
    if kc is None:
        return np.zeros(14)
    if type(kc) is not VehicleKC or any(
        type(a) is not AxleKC
        or type(a.compliance) is not ComplianceCoeffs
        or any(type(c) is not KinematicCurve for c in (a.toe, a.camber))
        for a in (kc.front, kc.rear)
    ):
        return None
    # Repack, including programmatically supplied mutable curve sequences.
    # A frozen dataclass alone does not make its referenced objects immutable.
    values = []
    for axle in (kc.front, kc.rear):
        c = axle.compliance
        values.extend([c.toe_per_fy, c.toe_per_fx, c.toe_per_mz, c.camber_per_fy, c.camber_per_fz])
    for axle in (kc.front, kc.rear):
        for curve in (axle.toe, axle.camber):
            values.append(len(curve.jounce_m))
            for x, y in zip(curve.jounce_m, curve.value, strict=True):
                values.extend((x, y))
    result = np.asarray(values, dtype=np.float64)
    return result


def integrate(model, kind, y, dt, delta, mus, road, *, fz_offset=None, hooks_original=True):
    lib = core()
    reason = None
    tp = tire_parameters(model.tire) if lib is not None else None
    kc = _kc_parameters(model) if lib is not None and kind == 1 else np.zeros(14)
    if lib is None:
        reason = "Python selected or native extension unavailable"
    elif not hooks_original:
        reason = "custom model or overridden model hook"
    elif tp is None:
        reason = "custom Python tyre force law"
    elif kc is None:
        reason = "custom Python K&C law"
    if reason is not None:
        model._body_core, model._body_fallback_reason = "python", reason
        return None
    p = model.params
    frozen = np.empty((4, 15))
    frozen[:, :2] = p.wheel_positions_body() if kind == 0 else model._wheels
    frozen[:, 2] = delta
    frozen[:, 3] = mus
    frozen[:, 4] = np.clip(model._fz_base + fz_offset, 0.0, p.mass * 9.81) if kind == 0 else 0.0
    frozen[:, 5] = model.state.wheel_omega
    frozen[:, 6] = model._alpha_scale
    frozen[:, 7] = model._alpha_lag
    frozen[:, 8] = model._kc_camber if kind == 0 else 0.0
    frozen[:, 9] = model._toe
    frozen[:, 10] = road
    frozen[:, 11] = model._kc_fx
    frozen[:, 12] = model._kc_fy
    frozen[:, 13] = model._kc_fz
    frozen[:, 14] = model._kc_mz
    suspension = (
        np.array(
            [
                model.m,
                model.m_u,
                model.m_s,
                model.h_cg,
                model.k_s,
                model.c_s,
                model.k_t,
                model._arb_f,
                model._arb_r,
                model._arb_body,
                model.I_roll,
                model.I_pitch,
                model._sprung_cg_x,
                model._q_aero,
                model._cl_f,
                model._cl_r,
                *model._W,
            ],
            dtype=np.float64,
        )
        if kind
        else np.empty(0)
    )
    out = np.empty((2, len(y)))
    lib.integrate(
        kind,
        dt,
        _body_parameters(p),
        tp,
        frozen,
        suspension,
        kc,
        np.ascontiguousarray(y, dtype=np.float64),
        out,
    )
    model._body_core, model._body_fallback_reason = "cpp17", None
    return out[0], out[1]


def wheel_spin(tire, dt, radius, inertia, min_speed, omega, vx, alpha, fz, mu, torque, brake):
    lib = core()
    tp = tire_parameters(tire) if lib is not None else None
    if tp is None:
        return None
    data = np.column_stack((omega, vx, alpha, fz, mu, torque, brake))
    out = np.empty((4, 5))
    lib.spin(tp, np.array([dt, radius, inertia, min_speed]), data, out)
    return out


def linkage(delta, geometry, eta, fallback):
    lib = core()
    if lib is None:
        return None
    values = []
    for prefix in ("front", "rear"):
        values.extend(
            [
                getattr(geometry, prefix + "_" + name)
                for name in ("outer_x", "outer_y", "inner_x", "inner_y")
            ]
        )
        values.extend(
            (
                np.deg2rad(getattr(geometry, prefix + "_rack_axis_deg")),
                getattr(geometry, prefix + "_rack_travel_limit"),
            )
        )
    out = np.empty((7, 4))
    lib.linkage(
        np.ascontiguousarray(delta, dtype=np.float64).reshape(4),
        np.asarray(values, dtype=np.float64),
        eta,
        fallback,
        out,
    )
    keys = (
        "arm_tie_angle",
        "tie_rack_angle",
        "efficiency",
        "arm_length",
        "rack_travel",
        "rack_travel_derivative",
        "valid",
    )
    return {key: out[i].astype(bool) if key == "valid" else out[i] for i, key in enumerate(keys)}
