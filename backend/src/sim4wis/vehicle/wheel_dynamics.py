"""Implicit wheel-spin integration using the selected tyre's actual force.

The residual is I*(w1-w0) = dt*(T - R*Fx(w1)). Replacing Fx with Ck*kappa
changes the equilibrium of a nonlinear/combined-slip tyre; this solver keeps
the real force law and a friction-capacity bracket. No external solver needed.
"""
from __future__ import annotations

import math


def implicit_spin(*, omega: float, vx: float, alpha: float, fz: float, mu: float,
                  torque: float, dt: float, radius: float, inertia: float,
                  denom: float, tire) -> float:
    cap = max(mu * fz, 0.0)
    if cap == 0.0 or dt == 0.0:
        return omega + dt * torque / inertia

    def residual(w: float) -> float:
        fx = tire.forces(alpha, (radius * w - vx) / denom, fz, mu)[0]
        return inertia * (w - omega) - dt * (torque - radius * fx)

    scale = max(inertia * abs(omega), dt * abs(torque), 1.0)
    tol = 1e-11 * scale
    f0 = residual(omega)
    if abs(f0) <= tol:
        return omega
    # Every physical tyre force lies inside +/- mu*Fz. Hence the residual
    # has opposite signs at these bounds even in saturation and in reverse.
    lo = omega + dt * (torque - radius * cap) / inertia
    hi = omega + dt * (torque + radius * cap) / inertia
    w = min(max(omega, lo), hi)
    for _ in range(48):
        f = residual(w)
        if abs(f) <= tol:
            return w
        if f > 0.0:
            hi = w
        else:
            lo = w
        if hi - lo <= 1e-12 * max(abs(w), 1.0):
            return 0.5 * (hi + lo)
        eps = 1e-5 * max(abs(w), 1.0)
        slope = (residual(w + eps) - residual(w - eps)) / (2.0 * eps)
        candidate = w - f / slope if slope > 1e-12 else math.nan
        # Safeguarded Newton; the bracket is retained on the falling shoulder.
        w = candidate if lo < candidate < hi else 0.5 * (lo + hi)
    raise ArithmeticError("wheel-spin implicit solve did not converge")
