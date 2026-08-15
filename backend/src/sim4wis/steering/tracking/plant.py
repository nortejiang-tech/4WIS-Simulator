"""The torque-mode corner actuator — wheel domain, rigid or compliant.

The feedback controllers of the tracking layer command a torque; this is the
object they command. In its default (rigid) form it is deliberately simple:

    J·θ̈ + b·θ̇ + F_c·sign(θ̇) + τ_load = τ   with |τ| ≤ τ_peak

in the road-wheel domain (rad / rad·s⁻¹ / N·m at the wheel), semi-implicit
on the rate. Coulomb friction carries a stiction rule: while the rate is
negligible and the net torque is inside the friction band, the actuator
holds — that is what makes the centre deadband a *physical* result of the
actuator rather than a control artefact.

**Transmission refinement (iteration direction 3).** A real corner module
is not rigid: the motor drives through a gear/belt whose finite stiffness
puts a two-mass resonance in the loop, and whose backlash is a dead band
the loop only discovers when it crosses it. With `transmission_stiffness`
set, the plant becomes two masses coupled through a spring with an
optional backlash dead zone:

    J_m·θ̈_m = τ − τ_k − b_m·θ̇_m
    J_w·θ̈_w = τ_k − b_w·θ̇_w − F_c·sign(θ̇_w) − τ_load
    τ_k = k·(x − backlash·sign(x))·[|x| > backlash],  x = θ_m − θ_w

The rigid default (stiffness None) keeps the original single-mass code
path untouched — bit-exactness for every existing run — and the compliant
path is an evaluation dimension: controllers tuned on the rigid model meet
the resonance and the dead band for the first time, and how each structure
copes (the observer absorbs it, the integrator winds against it) is
exactly the comparison the layer exists to make.

Deliberately NOT here: the motor's back-EMF envelope and thermal derating
(torque-source model — `steering/motor.py`), and the rate limit stays an
optional constraint.
"""

from __future__ import annotations

import math

#: Below this rate [rad/s] the actuator is considered at rest for stiction.
_RATE_EPS = 1e-9


class CornerActuatorPlant:
    def __init__(
        self,
        *,
        inertia_kgm2: float = 0.6,
        damping_nms_per_rad: float = 4.0,
        coulomb_friction_nm: float = 0.5,
        #: The sized default (see AngleControlParams.plant_peak_torque_nm):
        #: the rack-chain parking basis / 0.8 usage = 259 → 260 N·m.
        peak_torque_nm: float = 260.0,
        rate_limit_rad_s: float | None = None,
        transmission_stiffness_nms_per_rad: float | None = None,
        backlash_rad: float = 0.0,
        motor_inertia_fraction: float = 0.2,
    ) -> None:
        self.j = float(inertia_kgm2)
        self.b = float(damping_nms_per_rad)
        self.fc = float(coulomb_friction_nm)
        self.peak = float(peak_torque_nm)
        self.rate_limit = rate_limit_rad_s
        #: None → rigid single mass (the original, bit-exact path).
        self.k_trans = (None if transmission_stiffness_nms_per_rad is None
                        else float(transmission_stiffness_nms_per_rad))
        self.backlash = max(float(backlash_rad), 0.0)
        frac = min(max(float(motor_inertia_fraction), 0.05), 0.95)
        #: Two-mass split, wheel-side carries the friction and the load.
        self.j_m = self.j * frac
        self.j_w = self.j * (1.0 - frac)
        #: Wheel share of the damping; the motor side keeps a proportional
        #: share so the rigid limit's damping splits consistently.
        self.b_m = self.b * frac
        self.b_w = self.b * (1.0 - frac)
        self.angle = 0.0          # wheel-side angle — what the tyre sees
        self.rate = 0.0
        self._theta_m = 0.0       # motor-side angle
        self._omega_m = 0.0
        self._tau_k = 0.0         # coupling torque, for diagnostics

    def reset(self, angle: float = 0.0) -> None:
        self.angle = float(angle)
        self.rate = 0.0
        self._theta_m = float(angle)
        self._omega_m = 0.0
        self._tau_k = 0.0

    @property
    def motor_angle(self) -> float:
        return self._theta_m

    @property
    def coupling_torque(self) -> float:
        return self._tau_k

    def _coupling(self) -> float:
        """Spring torque across the transmission, with backlash dead zone."""
        x = self._theta_m - self.angle
        if abs(x) <= self.backlash:
            return 0.0
        return self.k_trans * (x - math.copysign(self.backlash, x))

    def step(self, dt: float, torque_cmd: float, load_torque: float) -> None:
        """Advance one step. Returns nothing; state is on the instance."""
        dt = max(float(dt), 1e-9)
        tau = max(-self.peak, min(self.peak, float(torque_cmd)))
        if self.k_trans is None:
            # Rigid single mass — the original path, byte for byte.
            net = tau - float(load_torque) - self.b * self.rate
            if abs(self.rate) <= _RATE_EPS and abs(net) <= self.fc:
                net = 0.0  # stiction holds
            elif abs(self.rate) <= _RATE_EPS:
                net -= math.copysign(self.fc, net)
            else:
                net -= math.copysign(self.fc, self.rate)
            self.rate += net / max(self.j, 1e-9) * dt
            if self.rate_limit is not None:
                self.rate = max(-self.rate_limit, min(self.rate_limit, self.rate))
            self.angle += self.rate * dt
            return

        # Compliant two-mass path.
        self._tau_k = self._coupling()
        # Motor side: no stiction model — the gearbox preloads it.
        net_m = tau - self._tau_k - self.b_m * self._omega_m
        self._omega_m += net_m / max(self.j_m, 1e-9) * dt
        if self.rate_limit is not None:
            self._omega_m = max(-self.rate_limit,
                                min(self.rate_limit, self._omega_m))
        self._theta_m += self._omega_m * dt
        # Wheel side: friction + stiction + load, as in the rigid path.
        net_w = self._tau_k - float(load_torque) - self.b_w * self.rate
        if abs(self.rate) <= _RATE_EPS and abs(net_w) <= self.fc:
            net_w = 0.0
        elif abs(self.rate) <= _RATE_EPS:
            net_w -= math.copysign(self.fc, net_w)
        else:
            net_w -= math.copysign(self.fc, self.rate)
        self.rate += net_w / max(self.j_w, 1e-9) * dt
        self.angle += self.rate * dt
