"""The torque-mode corner actuator — a 2nd-order plant, wheel domain.

The feedback controllers of the tracking layer command a torque; this is the
object they command. It is deliberately simple:

    J·θ̈ + b·θ̇ + F_c·sign(θ̇) + τ_load = τ   with |τ| ≤ τ_peak

in the road-wheel domain (rad / rad·s⁻¹ / N·m at the wheel), semi-implicit
on the rate. Coulomb friction carries a stiction rule: while the rate is
negligible and the net torque is inside the friction band, the actuator
holds — that is what makes the centre deadband a *physical* result of the
actuator rather than a control artefact.

Deliberately NOT here: the motor's back-EMF envelope and thermal derating.
Those live in ``steering/motor.py`` and belong to a torque-source model;
the control layer's first job is the position loop. A rate limit at the
wheel is available as an optional constraint for studies that want the
legacy behaviour's clipping.

The load torque is what the caller (the vehicle coupling) computes from the
tyre/rack force chain; the plant treats it as an exogenous disturbance.
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
        peak_torque_nm: float = 40.0,
        rate_limit_rad_s: float | None = None,
    ) -> None:
        self.j = float(inertia_kgm2)
        self.b = float(damping_nms_per_rad)
        self.fc = float(coulomb_friction_nm)
        self.peak = float(peak_torque_nm)
        self.rate_limit = rate_limit_rad_s
        self.angle = 0.0
        self.rate = 0.0

    def reset(self, angle: float = 0.0) -> None:
        self.angle = float(angle)
        self.rate = 0.0

    def step(self, dt: float, torque_cmd: float, load_torque: float) -> None:
        """Advance one step. Returns nothing; state is on the instance."""
        dt = max(float(dt), 1e-9)
        tau = max(-self.peak, min(self.peak, float(torque_cmd)))
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
