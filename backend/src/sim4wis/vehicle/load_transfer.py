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

    # Longitudinal transfer (per wheel: divide by 2 for the 2 wheels on each axle)
    dfz_long = m * ax * h / (2.0 * L)

    # Lateral transfer — how the roll couple m·ay·h is shared between the axles.
    #
    # Real cars distribute it by ROLL STIFFNESS, and that distribution is the
    # main handling-balance lever: the axle taking more transfer loses more
    # grip (cornering stiffness rises slower than load, c_α ∝ Fz^p with p<1),
    # so a front-biased split makes the car understeer.
    #
    # `roll_stiffness_front_frac = 0` keeps the original behaviour, which split
    # by static axle *weight* — a car that is 51/49 by weight then gets a 51/49
    # roll-couple split and comes out essentially neutral (measured understeer
    # gradient 0.07 deg/g, where a production car sits at 1–4).
    #
    # Either way the total roll couple is conserved:
    #     ΔFz_f·tF + ΔFz_r·tR = m·ay·h
    eps_f = float(getattr(params.suspension, "roll_stiffness_front_frac", 0.0))
    if eps_f <= 0.0:
        eps_f = b / L                      # legacy: static weight share
    eps_f = min(max(eps_f, 0.0), 1.0)
    roll_couple = m * ay * h
    dfz_lat_f = eps_f * roll_couple / tF
    dfz_lat_r = (1.0 - eps_f) * roll_couple / tR

    # Limit each transfer by the available support. Clamping four negative
    # loads independently invents vertical force (and therefore tyre grip).
    # Beyond lift-off this reduced model saturates support; it cannot predict
    # rollover/heave. The total supported weight must still remain m*g.
    front = float(np.clip(2.0 * (fz_static_f - dfz_long), 0.0, m * G))
    rear = m * G - front
    dfz_lat_f = float(np.clip(dfz_lat_f, -front / 2, front / 2))
    dfz_lat_r = float(np.clip(dfz_lat_r, -rear / 2, rear / 2))
    return np.array([front / 2 - dfz_lat_f, front / 2 + dfz_lat_f,
                     rear / 2 - dfz_lat_r, rear / 2 + dfz_lat_r])
