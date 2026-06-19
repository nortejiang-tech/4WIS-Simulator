"""Quasi-static vertical-load distribution (4-wheel).

Static split + longitudinal transfer (during accel/brake) + lateral
transfer (during cornering). This is the Phase 2 "rigid body, no
suspension travel" approximation; Phase 3 multi-body will add suspension
compliance.

Sign / index convention (matches WheelIndex):
    [0]=FL  [1]=FR  [2]=RL  [3]=RR
    ax > 0   → forward accel  → load shifts to rear
    ay > 0   → leftward accel → load shifts to right (centrifugal goes right)
"""

from __future__ import annotations

import numpy as np

from sim4wis.core.state import VehicleParams

G = 9.81


def vertical_loads(params: VehicleParams, ax: float, ay: float) -> np.ndarray:
    """Return per-wheel vertical loads Fz [N] under given body accelerations."""
    m = params.mass
    L = params.wheelbase
    a = params.cg_to_front
    b = L - a
    h = getattr(params, "cg_height", 0.55)
    tF = params.track_front
    tR = params.track_rear

    # Static per-wheel (each axle has 2 wheels — divide by 2)
    fz_static_f = m * G * b / (2.0 * L)
    fz_static_r = m * G * a / (2.0 * L)

    # Longitudinal transfer (per wheel: divide by 2 for the 2 wheels on each axle)
    dfz_long = m * ax * h / (2.0 * L)

    # Lateral transfer (proportional to axle weight share)
    dfz_lat_f = (m * ay * h * b / L) / tF
    dfz_lat_r = (m * ay * h * a / L) / tR

    fz = np.array([
        fz_static_f - dfz_long - dfz_lat_f,   # FL
        fz_static_f - dfz_long + dfz_lat_f,   # FR
        fz_static_r + dfz_long - dfz_lat_r,   # RL
        fz_static_r + dfz_long + dfz_lat_r,   # RR
    ])
    # Tyres can lift off — clamp to ≥ 0.
    return np.maximum(fz, 0.0)
