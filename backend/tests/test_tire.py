"""Unit tests — tire models (linear + Pacejka) and the make_tire factory."""

from __future__ import annotations

import math

import pytest

from sim4wis.core.state import VehicleParams
from sim4wis.vehicle.tire import (
    LinearTireModel,
    PacejkaTireModel,
    make_tire,
    pacejka_combined_forces,
    pacejka_pure_slip_forces,
)

FZ = 7100.0   # ~ LS9 static per-wheel load [N]
MU = 0.85


def test_linear_small_slip_slopes() -> None:
    t = LinearTireModel()
    fx, fy, _ = t.forces(alpha=0.0, kappa=0.01, fz=FZ, mu=MU)
    assert fx == pytest.approx(0.01 * t.c_kappa)
    fx, fy, _ = t.forces(alpha=0.01, kappa=0.0, fz=FZ, mu=MU)
    assert fy == pytest.approx(-0.01 * t.c_alpha)


def test_linear_friction_circle_cap() -> None:
    t = LinearTireModel()
    fx, fy, _ = t.forces(alpha=0.3, kappa=0.5, fz=FZ, mu=MU)
    assert math.hypot(fx, fy) <= MU * FZ * (1 + 1e-9)


def test_pacejka_small_slip_matches_linear() -> None:
    """Origin slope must equal the shared stiffnesses (servo tuning relies on it)."""
    p = PacejkaTireModel()
    lin = LinearTireModel()
    for kappa in (1e-4, 5e-4):
        fx_p, _, _ = p.forces(0.0, kappa, FZ, MU)
        fx_l, _, _ = lin.forces(0.0, kappa, FZ, MU)
        assert fx_p == pytest.approx(fx_l, rel=0.02)
    for alpha in (1e-4, 5e-4):
        _, fy_p, _ = p.forces(alpha, 0.0, FZ, MU)
        _, fy_l, _ = lin.forces(alpha, 0.0, FZ, MU)
        assert fy_p == pytest.approx(fy_l, rel=0.02)


def test_pacejka_peak_location_and_value() -> None:
    p = PacejkaTireModel()
    d = MU * FZ
    alphas = [i * 0.002 for i in range(1, 400)]   # 0.002 … 0.8 rad
    fys = [abs(p.forces(a, 0.0, FZ, MU)[1]) for a in alphas]
    peak = max(fys)
    a_peak = alphas[fys.index(peak)]
    # MF peak |Fy| ≈ D (sin passes through 1 since Cy > 1), at a few degrees.
    assert peak == pytest.approx(d, rel=0.03)
    assert 0.02 < a_peak < 0.4
    # Past the peak the curve relaxes toward the asymptote D·sin(Cy·π/2) < D.
    assert fys[-1] < peak
    assert fys[-1] == pytest.approx(d * math.sin(p.cy * math.pi / 2), rel=0.1)
    # Never exceeds D.
    assert all(f <= d * (1 + 1e-9) for f in fys)


def test_pacejka_combined_slip_ellipse() -> None:
    p = PacejkaTireModel()
    d = MU * FZ
    fx, fy, _ = p.forces(alpha=0.15, kappa=0.15, fz=FZ, mu=MU)
    assert (fx / d) ** 2 + (fy / d) ** 2 <= 1.0 + 1e-9


def test_pacejka_shared_function_matches_model_class() -> None:
    p = PacejkaTireModel()
    alpha = 0.07
    kappa = 0.05

    fx_model, fy_model, _ = p.forces(alpha=alpha, kappa=kappa, fz=FZ, mu=MU)
    fx_fn, fy_fn = pacejka_combined_forces(
        alpha=alpha,
        kappa=kappa,
        fz=FZ,
        mu=MU,
        c_alpha=p.c_alpha,
        c_kappa=p.c_kappa,
        cx=p.cx,
        cy=p.cy,
        ex=p.ex,
        ey=p.ey,
    )

    assert fx_fn == pytest.approx(fx_model)
    assert fy_fn == pytest.approx(fy_model)


def test_pacejka_pure_slip_can_exceed_ellipse_before_clip() -> None:
    p = PacejkaTireModel()
    d = MU * FZ

    fx0, fy0 = pacejka_pure_slip_forces(
        alpha=0.12,
        kappa=0.12,
        fz=FZ,
        mu=MU,
        c_alpha=p.c_alpha,
        c_kappa=p.c_kappa,
        cx=p.cx,
        cy=p.cy,
        ex=p.ex,
        ey=p.ey,
    )
    fx, fy = pacejka_combined_forces(
        alpha=0.12,
        kappa=0.12,
        fz=FZ,
        mu=MU,
        c_alpha=p.c_alpha,
        c_kappa=p.c_kappa,
        cx=p.cx,
        cy=p.cy,
        ex=p.ex,
        ey=p.ey,
    )

    assert (fx0 / d) ** 2 + (fy0 / d) ** 2 > 1.0
    assert (fx / d) ** 2 + (fy / d) ** 2 <= 1.0 + 1e-9


def test_pacejka_mu_scaling() -> None:
    """Halving μ halves the saturated force (disturbance compatibility)."""
    p = PacejkaTireModel()
    _, fy_hi, _ = p.forces(0.5, 0.0, FZ, 0.8)   # deep saturation
    _, fy_lo, _ = p.forces(0.5, 0.0, FZ, 0.4)
    assert fy_lo == pytest.approx(fy_hi / 2.0, rel=0.02)


def test_pacejka_zero_load_is_safe() -> None:
    p = PacejkaTireModel()
    fx, fy, mz = p.forces(0.2, 0.2, fz=0.0, mu=MU)
    assert abs(fx) < 1e-2 and abs(fy) < 1e-2 and abs(mz) < 1e-2


def test_make_tire_factory() -> None:
    lin = make_tire(VehicleParams())
    assert isinstance(lin, LinearTireModel)
    pac = make_tire(VehicleParams(tire_model="pacejka", tire_c_alpha=90_000.0))
    assert isinstance(pac, PacejkaTireModel)
    assert pac.c_alpha == 90_000.0
