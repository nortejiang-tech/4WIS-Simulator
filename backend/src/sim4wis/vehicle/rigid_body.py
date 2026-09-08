"""Planar rigid-body mechanics with an explicit reference point.

The public pose/velocity is at the axle midpoint O, whereas inertia_z and
Newton's force balance refer to the centre of mass G. X_G/O = L/2 - a.
Keeping that offset here avoids changing existing project/telemetry frames.
"""
from __future__ import annotations

import math

import numpy as np

from sim4wis.core.state import VehicleParams


def planar_derivatives(params: VehicleParams, vx: float, vy: float, yaw_rate: float,
                       fx: float, fy: float, moment_origin: float) -> np.ndarray:
    """Return (u_dot, v_dot, r_dot) at O; forces expressed in body axes.

    v_G = v_O + r*e; M_G = M_O - e*Fy. The e*r² and e*r_dot terms
    are required for an accelerating reference point away from the CG.
    """
    e = params.wheelbase / 2.0 - params.cg_to_front
    r_dot = (moment_origin - e * fy) / params.inertia_z
    return np.array([
        fx / params.mass + yaw_rate * vy + e * yaw_rate**2,
        fy / params.mass - yaw_rate * vx - e * r_dot,
        r_dot,
    ])


def cg_acceleration(params: VehicleParams, vx: float, vy: float, yaw_rate: float,
                    derivatives: np.ndarray) -> tuple[float, float]:
    """Inertial CG acceleration in body axes, from derivatives at O."""
    e = params.wheelbase / 2.0 - params.cg_to_front
    return (float(derivatives[0] - yaw_rate * vy - e * yaw_rate**2),
            float(derivatives[1] + yaw_rate * vx + e * derivatives[2]))


def integrate_pose(x: float, y: float, psi: float, vx: float, vy: float,
                   yaw_rate: float, dt: float) -> tuple[float, float, float]:
    """Exact SE(2) increment for a constant body twist over dt.

    For varying velocities callers supply their time-centred twist. sinc
    handles straight driving continuously, without a small-radius singularity.
    """
    half = 0.5 * yaw_rate * dt
    sinc = math.sin(half) / half if abs(half) > 1e-8 else 1.0 - half * half / 6.0
    c, s = math.cos(psi + half), math.sin(psi + half)
    return (x + dt * sinc * (vx * c - vy * s),
            y + dt * sinc * (vx * s + vy * c), psi + 2.0 * half)


def road_warp(wheel_positions: np.ndarray, road_z: np.ndarray) -> np.ndarray:
    """Wheel-height residual after removing a rigid support plane.

    Absolute altitude and a constant grade/bank are body pose, not suspension
    compression. The planar model can only approximate the remaining warp.
    """
    if not np.any(road_z):
        return np.zeros(4)
    plane = np.column_stack((np.ones(4), wheel_positions))
    coeff, *_ = np.linalg.lstsq(plane, road_z, rcond=None)
    return road_z - plane @ coeff
