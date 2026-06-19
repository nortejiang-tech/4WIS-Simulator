"""Multibody 4WIS vehicle model (step 19) — 14 DOF, fixed-step RK4.

Degrees of freedom (14):
    * Sprung body 6:  x, y (planar) · ψ yaw · z heave · φ roll · θ pitch
    * Suspension  4:  per-corner unsprung vertical position z_u[i]
    * Wheel spin  4:  ω_w[i]

This is the high-fidelity model. Over `SimplifiedDynamicModel` it adds:
    * Real vertical dynamics — heave/bounce over bumps (quarter-car per corner
      with tyre vertical stiffness + suspension spring/damper).
    * Roll & pitch as actual DOF → load transfer is *dynamic* (lags lateral /
      longitudinal acceleration through the roll/pitch modes) instead of the
      quasi-static `load_transfer.vertical_loads()` used by the simplified model.
    * Bump-steer driven by **actual suspension deflection** (not the road-height
      hack the simplified model uses).

Design choice: implemented directly in NumPy (no Pinocchio / PyDy dependency)
to stay consistent with the rest of the codebase and the real-time budget.
At dt_sim = 5 ms the stiffest mode (wheel hop ≈ 11 Hz) is well within RK4
stability.

Frames & signs (see docs/design.md §2):
    z up, roll φ + = right side down, pitch θ + = nose up. Body origin = axle
    midpoint; CG is at body-x = L/2 − cg_to_front.
"""

from __future__ import annotations

import math

import numpy as np

from sim4wis.core.state import (
    ControlCommand,
    EnvironmentState,
    N_WHEELS,
    VehicleParams,
    VehicleState,
)
from sim4wis.vehicle.base import VehicleModel
from sim4wis.vehicle.geometry import steer_actuator, vehicle_icr_from_velocity
from sim4wis.vehicle.kingpin import kingpin_torque
from sim4wis.vehicle.model_core import rotate_wheel_forces_to_body, wheel_slip_kinematics
from sim4wis.vehicle.tire import TireModel, make_tire
from sim4wis.vehicle.wheel_servo import WheelSpeedServo

G = 9.81

# RK4 state vector layout (length 24)
#   0:x 1:y 2:ψ 3:z 4:roll 5:pitch 6..9:z_u  10:vx 11:vy 12:ω 13:vz
#   14:roll_rate 15:pitch_rate 16..19:zu_dot 20..23:ω_w
IX, IY, IPSI, IZ, IROLL, IPITCH = 0, 1, 2, 3, 4, 5
IZU = slice(6, 10)
IVX, IVY, IOMEGA, IVZ, IROLLR, IPITCHR = 10, 11, 12, 13, 14, 15
IZUD = slice(16, 20)
IWW = slice(20, 24)


class MultiBodyModel(VehicleModel):
    """High-fidelity 3D body + 4 quarter-car suspensions + 4 wheel spins."""

    def __init__(self, params: VehicleParams, tire: TireModel | None = None) -> None:
        super().__init__(params)
        self.tire = tire or make_tire(params)
        t_max = getattr(params, "motor_torque_max", 2000.0)
        self.servos = [
            WheelSpeedServo(
                kp=getattr(params, "servo_kp", 200.0),
                ki=getattr(params, "servo_ki", 50.0),
                torque_limit=t_max,
            )
            for _ in range(N_WHEELS)
        ]
        self.iw = getattr(params, "wheel_inertia", 1.2)

        self._precompute()

        # Internal velocity / unsprung states not held in VehicleState.
        self._vz = 0.0
        self._roll_rate = 0.0
        self._pitch_rate = 0.0
        self._zu = np.zeros(N_WHEELS)
        self._zu_dot = np.zeros(N_WHEELS)
        self._delta_act = np.zeros(N_WHEELS)  # steering actuator state

        # Diagnostics
        self.slip_alpha = np.zeros(N_WHEELS)
        self.slip_kappa = np.zeros(N_WHEELS)
        self.tire_fx = np.zeros(N_WHEELS)
        self.tire_fy = np.zeros(N_WHEELS)
        self.tire_mz = np.zeros(N_WHEELS)

        self.state.fz = self._corner_static_full.copy()

    def _precompute(self) -> None:
        p = self.params
        self.m = p.mass
        self.m_u = getattr(p, "unsprung_mass", 55.0)
        self.m_s = self.m - N_WHEELS * self.m_u
        L = p.wheelbase
        a = p.cg_to_front
        b = L - a
        # Sprung static corner loads (front pair / rear pair)
        wf = self.m_s * G * b / (2.0 * L)
        wr = self.m_s * G * a / (2.0 * L)
        self._W = np.array([wf, wf, wr, wr])              # FL,FR,RL,RR
        self._corner_static_full = self._W + self.m_u * G  # tyre vertical static
        self.x_cg = L / 2.0 - a                            # CG x in body frame
        self.h_cg = getattr(p, "cg_height", 0.55)

        self.k_s = p.suspension.spring_rate
        self.c_s = p.suspension.damper_rate
        self.arb = p.suspension.anti_roll_rate
        self.k_t = getattr(p, "tire_vertical_stiffness", 280_000.0)
        self.bump_steer = getattr(p, "bump_steer_coeff", 0.0)

        # Roll / pitch inertia — box approximation from sprung mass + dimensions.
        L_body = 1.5 * L
        W_body = max(p.track_front, p.track_rear)
        H_body = 2.0 * self.h_cg
        self.I_roll = self.m_s * (W_body**2 + H_body**2) / 12.0
        self.I_pitch = self.m_s * (L_body**2 + H_body**2) / 12.0

        self._wheels = p.wheel_positions_body()           # (4,2)
        self._toe_sign = np.sign(self._wheels[:, 1])      # +1 left, −1 right

    # ---- API ---------------------------------------------------------------

    def reset(self, init: VehicleState | None = None) -> None:
        super().reset(init)
        for s in self.servos:
            s.reset()
        self._vz = 0.0
        self._roll_rate = 0.0
        self._pitch_rate = 0.0
        self._zu = np.zeros(N_WHEELS)
        self._zu_dot = np.zeros(N_WHEELS)
        self._delta_act = np.zeros(N_WHEELS)
        self.slip_alpha[:] = 0.0
        self.slip_kappa[:] = 0.0
        self.state.fz = self._corner_static_full.copy()
        self.state.susp_defl = np.zeros(N_WHEELS)

    def step(self, dt: float, cmd: ControlCommand, env: EnvironmentState) -> VehicleState:
        s = self.state
        p = self.params

        # Per-wheel road height + μ at the current planar pose (held over the step).
        road_z, wheel_mus = self._road_and_mu(env)
        t_motor = self._motor_torques(cmd, s.wheel_omega, dt)
        # Steering actuator: actual δ tracks δ_cmd with lag + rate limit.
        self._delta_act = steer_actuator(
            self._delta_act, cmd.delta_cmd, dt,
            getattr(p, "steer_tau", 0.06), getattr(p, "steer_rate_max", 8.0),
        )
        delta_cmd = self._delta_act.astype(float)

        y0 = np.empty(24)
        y0[IX], y0[IY], y0[IPSI] = s.x, s.y, s.psi
        y0[IZ], y0[IROLL], y0[IPITCH] = s.z, s.roll, s.pitch
        y0[IZU] = self._zu
        y0[IVX], y0[IVY], y0[IOMEGA] = s.vx, s.vy, s.yaw_rate
        y0[IVZ], y0[IROLLR], y0[IPITCHR] = self._vz, self._roll_rate, self._pitch_rate
        y0[IZUD] = self._zu_dot
        y0[IWW] = s.wheel_omega

        def f(y):
            return self._derivatives(y, delta_cmd, t_motor, wheel_mus, road_z)

        k1 = f(y0)
        k2 = f(y0 + 0.5 * dt * k1)
        k3 = f(y0 + 0.5 * dt * k2)
        k4 = f(y0 + dt * k3)
        y1 = y0 + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        s.x, s.y, s.psi = float(y1[IX]), float(y1[IY]), float(y1[IPSI])
        s.z, s.roll, s.pitch = float(y1[IZ]), float(y1[IROLL]), float(y1[IPITCH])
        self._zu = y1[IZU]
        s.vx, s.vy, s.yaw_rate = float(y1[IVX]), float(y1[IVY]), float(y1[IOMEGA])
        self._vz = float(y1[IVZ])
        self._roll_rate = float(y1[IROLLR])
        self._pitch_rate = float(y1[IPITCHR])
        self._zu_dot = y1[IZUD]
        s.wheel_omega[:] = y1[IWW]

        # Derived / reported quantities (recompute forces at the final state).
        self._finalize(s, delta_cmd, road_z, wheel_mus)
        s.t += dt
        return s

    # ---- internals ---------------------------------------------------------

    def _motor_torques(self, cmd: ControlCommand, omega_actual: np.ndarray, dt: float) -> np.ndarray:
        out = np.zeros(N_WHEELS)
        for i in range(N_WHEELS):
            out[i] = self.servos[i].update(
                omega_actual=float(omega_actual[i]),
                omega_cmd=float(cmd.wheel_speed_cmd[i]),
                dt=dt,
            )
        return out

    def _road_and_mu(self, env: EnvironmentState) -> tuple[np.ndarray, np.ndarray]:
        s = self.state
        cp, sp = math.cos(s.psi), math.sin(s.psi)
        scene = getattr(env, "scene", None)
        base = env.mu if scene is None else getattr(scene, "base_mu", env.mu)
        road_z = np.zeros(N_WHEELS)
        mus = np.full(N_WHEELS, base)
        if scene is not None:
            for i in range(N_WHEELS):
                xw = s.x + cp * self._wheels[i, 0] - sp * self._wheels[i, 1]
                yw = s.y + sp * self._wheels[i, 0] + cp * self._wheels[i, 1]
                local = scene.wheel_env(np.array([xw, yw]))
                road_z[i] = local.ground_z
                if local.mu_override is not None:
                    mus[i] = local.mu_override
        return road_z, mus

    def _suspension_forces(self, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return (F_spring up on body, F_tire_z up on unsprung, comp) per corner."""
        xw = self._wheels[:, 0]
        yw = self._wheels[:, 1]
        z, roll, pitch = y[IZ], y[IROLL], y[IPITCH]
        vz, rr, pr = y[IVZ], y[IROLLR], y[IPITCHR]
        z_u = y[IZU]
        zu_dot = y[IZUD]

        z_corner = z + xw * pitch + yw * roll
        z_corner_rate = vz + xw * pr + yw * rr
        comp = z_u - z_corner                      # suspension compression vs static
        comp_rate = zu_dot - z_corner_rate
        f_spring = np.maximum(self._W + self.k_s * comp + self.c_s * comp_rate, 0.0)

        f_tire_z = np.maximum(self._corner_static_full + self.k_t * (self._road_z - z_u), 0.0)
        return f_spring, f_tire_z, comp

    def _derivatives(self, y, delta_cmd, t_motor, wheel_mus, road_z) -> np.ndarray:
        self._road_z = road_z  # used inside _suspension_forces
        p = self.params
        xw = self._wheels[:, 0]
        yw = self._wheels[:, 1]

        f_spring, f_tire_z, comp = self._suspension_forces(y)

        # Bump-steer from actual suspension compression.
        delta = delta_cmd + self.bump_steer * comp * self._toe_sign
        delta = np.clip(delta, -p.steer_limit, p.steer_limit)

        vx, vy, omega = y[IVX], y[IVY], y[IOMEGA]
        wheel_omega = y[IWW]

        fx_body = np.zeros(N_WHEELS)
        fy_body = np.zeros(N_WHEELS)
        fx_wheel = np.zeros(N_WHEELS)
        kin = wheel_slip_kinematics(
            vx=float(vx),
            vy=float(vy),
            yaw_rate=float(omega),
            wheel_omega=wheel_omega,
            delta=delta,
            wheel_positions_body=self._wheels,
            tire_radius=p.tire_radius,
        )

        fy_wheel = np.zeros(N_WHEELS)
        for i in range(N_WHEELS):
            alpha = float(kin.alpha[i])
            kappa = float(kin.kappa[i])
            fx, fy, mz = self.tire.forces(alpha, kappa, float(f_tire_z[i]), float(wheel_mus[i]))
            fx_wheel[i] = fx
            fy_wheel[i] = fy
            self.slip_alpha[i] = alpha
            self.slip_kappa[i] = kappa
            self.tire_fx[i] = fx
            self.tire_fy[i] = fy
            self.tire_mz[i] = mz

        fx_body, fy_body = rotate_wheel_forces_to_body(fx_wheel, fy_wheel, delta)

        fx_total = float(np.sum(fx_body))
        fy_total = float(np.sum(fy_body))
        mz_total = float(np.sum(xw * fy_body - yw * fx_body + self.tire_mz))

        # Planar (whole-vehicle mass) with Coriolis terms.
        m = self.m
        vx_dot = fx_total / m + omega * vy
        vy_dot = fy_total / m - omega * vx
        omega_dot = mz_total / p.inertia_z
        ax_body = vx_dot - omega * vy
        ay_body = vy_dot + omega * vx

        # Vertical body (sprung mass) — gravity balanced by spring statics.
        vz_dot = (float(np.sum(f_spring)) - self.m_s * G) / self.m_s

        # Roll: spring moment + lateral-accel roll moment − ARB.
        q_roll = float(np.sum(yw * f_spring))
        m_roll_ext = self.m_s * ay_body * self.h_cg
        roll_dd = (q_roll + m_roll_ext - self.arb * y[IROLL]) / self.I_roll

        # Pitch: spring moment − gravity pitch moment + long-accel pitch moment.
        q_pitch = float(np.sum(xw * f_spring)) - self.m_s * G * self.x_cg
        m_pitch_ext = self.m_s * ax_body * self.h_cg
        pitch_dd = (q_pitch + m_pitch_ext) / self.I_pitch

        # Unsprung vertical (quarter-car).
        zu_dd = (f_tire_z - f_spring - self.m_u * G) / self.m_u

        # Wheel spin.
        omega_w_dot = (t_motor - p.tire_radius * fx_wheel) / self.iw

        # Pose kinematics.
        cp, sp = math.cos(y[IPSI]), math.sin(y[IPSI])
        dy = np.zeros(24)
        dy[IX] = vx * cp - vy * sp
        dy[IY] = vx * sp + vy * cp
        dy[IPSI] = omega
        dy[IZ] = y[IVZ]
        dy[IROLL] = y[IROLLR]
        dy[IPITCH] = y[IPITCHR]
        dy[IZU] = y[IZUD]
        dy[IVX] = vx_dot
        dy[IVY] = vy_dot
        dy[IOMEGA] = omega_dot
        dy[IVZ] = vz_dot
        dy[IROLLR] = roll_dd
        dy[IPITCHR] = pitch_dd
        dy[IZUD] = zu_dd
        dy[IWW] = omega_w_dot
        return dy

    def _finalize(self, s: VehicleState, delta_cmd, road_z, wheel_mus) -> None:
        # Recompute suspension state + reported Fz / steering torque at final y.
        y = np.empty(24)
        y[IZ], y[IROLL], y[IPITCH] = s.z, s.roll, s.pitch
        y[IZU] = self._zu
        y[IVZ], y[IROLLR], y[IPITCHR] = self._vz, self._roll_rate, self._pitch_rate
        y[IZUD] = self._zu_dot
        self._road_z = road_z
        f_spring, f_tire_z, comp = self._suspension_forces(y)
        s.fz = f_tire_z
        s.susp_defl = comp
        # Apply bump-steer to the reported actual steer angle too.
        delta = np.clip(delta_cmd + self.bump_steer * comp * self._toe_sign,
                        -self.params.steer_limit, self.params.steer_limit)
        s.delta[:] = delta
        s.vehicle_icr_body = vehicle_icr_from_velocity(s.vx, s.vy, s.yaw_rate)
        s.torque_steer = kingpin_torque(
            fx=self.tire_fx, fy=self.tire_fy, mz=self.tire_mz, fz=s.fz,
            suspension=self.params.suspension, delta=s.delta,
            tire_radius=self.params.tire_radius,
            t_pneumatic_extra=getattr(self.tire, "t_pneumatic", 0.03),
        )
        s.wheel_pos_body = self._wheels
