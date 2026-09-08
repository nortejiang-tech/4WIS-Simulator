"""Kinematic 4WIS vehicle model — Phase 1.

Assumes:
    * Each wheel rolls without slip in its own rolling direction.
    * Vehicle is a rigid body on a flat ground (z, roll, pitch ignored).
    * Tyre side-slip = 0, weight transfer = 0 (these are Phase 2 territory).

Update rule per step (algebraic velocity + exact constant-twist pose):

    1. Accept commanded (δ_i, ω_i) from the controller as ground truth — i.e.
       the kinematic model does no actuator dynamics. (Phase 2 will add a
       first-order lag and torque-bandwidth limit.)

    2. The four (wheel position, δ_i, |v_i| = ω_i · r_tire) measurements
       overdetermine the rigid-body planar motion (3 DoF). We solve the
       least-squares problem:

           For each wheel i with rolling direction t_i = (cos δ_i, sin δ_i):
              v_origin + ω × r_i  =  s_i · t_i      (s_i = signed wheel speed)
           ⇒ vx + (-ω · y_i) = s_i · cos δ_i
              vy + (+ω · x_i) = s_i · sin δ_i

       i.e. 8 equations in 3 unknowns (vx, vy, ω). Solve by lstsq.

    3. Integrate pose in world frame:
           dx/dt = vx·cos ψ - vy·sin ψ
           dy/dt = vx·sin ψ + vy·cos ψ
           dψ/dt = ω

    4. Bookkeeping:
           - vehicle ICR in body frame from (vx, vy, ω)
           - simple steering-resistance torque (static friction model placeholder)
           - copy commanded δ/ω into state.delta / state.wheel_omega
"""

from __future__ import annotations

import numpy as np

from sim4wis.core.state import (
    N_WHEELS,
    ControlCommand,
    EnvironmentState,
    VehicleParams,
    VehicleState,
)
from sim4wis.vehicle.base import VehicleModel
from sim4wis.vehicle.geometry import vehicle_icr_from_velocity
from sim4wis.vehicle.load_transfer import vertical_loads
from sim4wis.vehicle.rigid_body import integrate_pose


class KinematicModel(VehicleModel):
    """Phase 1 — pure-rolling kinematic model.

    Wheel commands are accepted as ground truth; body velocities are solved
    from the wheel kinematic constraints in a least-squares sense.
    """

    def __init__(self, params: VehicleParams) -> None:
        super().__init__(params)
        # static vertical load (used as a Phase-1 placeholder for steering
        # resistance torque)
        self.state.fz = vertical_loads(params, 0.0, 0.0)

    def reset(self, init: VehicleState | None = None) -> None:
        super().reset(init)
        self.state.fz = vertical_loads(self.params, 0.0, 0.0)

    def step(
        self,
        dt: float,
        cmd: ControlCommand,
        env: EnvironmentState,
    ) -> VehicleState:
        s = self.state
        p = self.params

        # 0. Surface friction under the vehicle. This model has no tyre and so
        #    no per-wheel μ, but the driver-facing layers (kinematic brake rate,
        #    steering soft limit) still need to know what the road offers —
        #    without it they silently assume dry asphalt everywhere.
        scene = getattr(env, "scene", None)
        base_mu = env.mu if scene is None else getattr(scene, "base_mu", env.mu)
        if scene is not None:
            local = scene.wheel_env(np.array([s.x, s.y]))
            if local.mu_override is not None:
                base_mu = local.mu_override
        s.mu_avg = float(base_mu)

        # 1. Adopt commanded steer angles and wheel speeds (Phase-1 assumption).
        #    This model is geometric — it has no wheel-spin ODE and therefore no
        #    torque concept, so `longitudinal_mode == "torque"` does not apply
        #    here and `drive_torque_cmd` is ignored. Drive-form behaviour needs
        #    simplified_dynamic or multibody; the state frame reports the
        #    effective mode so the UI can say so.
        s.delta[:] = cmd.delta_cmd
        s.wheel_omega[:] = cmd.wheel_speed_cmd

        # 2. Least-squares solve for body velocities (vx, vy, ω)
        wheels = p.wheel_positions_body()  # (4, 2)
        s.wheel_pos_body = wheels
        delta = s.delta  # (4,)
        s_vel = s.wheel_omega * p.tire_radius  # signed linear wheel speed (4,)
        ct = np.cos(delta)
        st = np.sin(delta)

        # Constraint matrix A @ [vx, vy, ω]^T = b
        # 8 rows: for each wheel, x-component and y-component of velocity.
        #   row 2i  :  vx + (-y_i) ω = s_i · cos δ_i
        #   row 2i+1:  vy + (+x_i) ω = s_i · sin δ_i
        A = np.zeros((2 * N_WHEELS, 3))
        b = np.zeros(2 * N_WHEELS)
        for i in range(N_WHEELS):
            xi, yi = wheels[i]
            A[2 * i, 0] = 1.0
            A[2 * i, 2] = -yi
            b[2 * i] = s_vel[i] * ct[i]
            A[2 * i + 1, 1] = 1.0
            A[2 * i + 1, 2] = +xi
            b[2 * i + 1] = s_vel[i] * st[i]

        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        s.vx, s.vy, s.yaw_rate = float(sol[0]), float(sol[1]), float(sol[2])
        # Specific force is deliberately NOT reported by this model.
        #
        # It solves body velocity algebraically from the commanded wheel
        # speeds, so it has no acceleration of its own: any step in the command
        # teleports the velocity and a finite difference of that goes with it —
        # a standing start reads 5560 m/s² (≈570 g). Low-passing only scales
        # the artefact (still 505 m/s² at τ = 50 ms) because the input is
        # genuinely unbounded, and a plausible-looking number is worse than
        # none: it would be recorded, plotted on the g-g chart and read as
        # measurement.
        #
        # ax/ay therefore stay at zero here, consistent with `grip_valid`
        # remaining False — this model reports what it can support and nothing
        # more. Use simplified_dynamic or multibody for anything force-based.
        s.ax = 0.0
        s.ay = 0.0

        # 3. Exact pose increment for the commanded constant body twist.
        s.x, s.y, s.psi = integrate_pose(s.x, s.y, s.psi, s.vx, s.vy, s.yaw_rate, dt)

        # 4. Derived geometry — vehicle ICR (body frame)
        s.vehicle_icr_body = vehicle_icr_from_velocity(s.vx, s.vy, s.yaw_rate)

        # 5. Phase-1 steering-resistance torque (placeholder).
        #    τ_i  ≈  µ · F_z · scrub_radius · sign(δ̇_i_proxy)
        #    Without an actuator lag we don't have δ̇, so use a coarse static
        #    moment scaled by |δ| (so torque grows as wheel turns away from 0).
        mu_kingpin = getattr(p.suspension, "kingpin_mu", 0.6)
        scrub = max(p.suspension.scrub_radius, 0.015)  # safety floor 15 mm
        s.torque_steer = mu_kingpin * s.fz * scrub * np.sign(delta) * np.minimum(
            np.abs(delta) / p.steer_limit, 1.0
        )

        # 6. Time
        s.t += dt
        return s
