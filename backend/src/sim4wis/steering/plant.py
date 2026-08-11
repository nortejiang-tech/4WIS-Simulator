"""The mechanical chain — hand wheel, torsion bar, pinion.

This is the piece that makes hand-wheel torque exist as a signal.

Convention: **the hand-wheel angle is imposed.** That is the steering-robot
convention, and it is the right default for three reasons — every objective
test procedure drives angle and measures torque, every existing input path in
this simulator (keyboard, gamepad, script, experiment manoeuvre) produces a
position, and it makes torque an *output*, which is precisely the signal that
was missing. Driving by applied torque, which a force-feedback wheel needs,
is the same equations with the column DOF freed and comes later.

    θ_sw (imposed)
      │  τ_tb = K(θ_sw − θ_p) + c(ω_sw − ω_p)          ← torsion bar
      │  τ_sensor = clip(K(θ_sw − θ_p))                ← what the ECU reads
      │      └ assist map → motor → τ_assist
      ▼
    J_eq ω̇_p = τ_tb + τ_assist − τ_load − τ_friction   ← pinion
      │
      └ τ_hand = J_c α_sw + c_c ω_sw + friction + τ_tb ← what the driver feels

`τ_load` is supplied by the caller from the existing tyre → kingpin → linkage
chain. This module does not re-derive it.

Two details that decide whether the result is worth anything:

**Coulomb friction is solved for stick, not just sign().** `μ·sign(ω)` chatters
at zero crossing and, worse, cannot hold anything still — so the model would
have no torque deadband and no hysteresis loop, which is most of what on-centre
feel *is*. Here friction that cannot be overcome exactly cancels the applied
torque and the pinion stays put.

**The torque sensor reads the spring term only.** A real torsion-bar sensor
measures twist. Feeding the damping term into the assist map as well would be
free phase lead the hardware does not have, and would flatter the control law.

STATUS — NOT YET VALIDATED
--------------------------
Nothing integrates this module yet, and it must not be wired into the vehicle
until the defect below is resolved.

Bench probe, full-lock parking against a 10 kN rack load:

    assist off   peak hand torque  96.5 N.m      (plausible for an unassisted
                                                  2.9 t SUV at full lock)
    assist on    peak hand torque 174.3 N.m      (WRONG — assist must reduce
                                                  effort, never raise it)

The on-centre behaviour is right: at 100 km/h with +/-10 deg the peak hand
torque is 1.3 N.m, the stick logic engages, and the hysteresis loop has area,
which is the mechanism ISO 13674 measures.

Sub-stepping was needed and is not the whole story. The assist loop multiplies
the effective torsion stiffness by the boost ratio (~59x at parking), moving
the column mode from 2.5 Hz to roughly 19 Hz, which a 5 ms step cannot carry;
sub-stepping at 0.2 ms fixed the 100 km/h case (2.42 -> 1.30 N.m) but left
parking unchanged, so a second, non-numerical defect remains. Suspects, in
order: the motor speed envelope saturating assist right where the load peaks
(available torque falls to 3.14 N.m against a 3.97 N.m demand), the sensor
saturation interacting with the map's edge-hold, and the sign or referral of
the load term at large pinion angle.

Resolve by instrumenting one parking sweep step by step rather than by
adjusting parameters — the numbers above are self-consistent under several
wrong hypotheses, which is why guessing has not worked.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from sim4wis.steering.assist import AssistMap
from sim4wis.steering.motor import Motor
from sim4wis.steering.params import SteeringSystemParams


@dataclass
class SteeringPlantState:
    """One step of the steering system. All torques N·m, angles rad."""

    pinion_angle: float = 0.0
    pinion_rate: float = 0.0
    hand_angle: float = 0.0
    hand_rate: float = 0.0
    torsion_torque: float = 0.0        # 扭杆实际传递力矩（含阻尼项）
    torque_sensor: float = 0.0         # 传感器读数（仅弹性项，带饱和）
    assist_torque: float = 0.0         # 折算到小齿轮的助力 [N·m]
    hand_torque: float = 0.0           # 方向盘处手力矩 —— 本层的核心新增信号
    load_torque: float = 0.0           # 来自轮胎的负载力矩（折算到小齿轮）
    friction_torque: float = 0.0
    motor_torque: float = 0.0          # 电机轴转矩
    motor_speed: float = 0.0
    motor_heat: float = 0.0
    motor_saturated: bool = False
    stuck: bool = False                # 静摩擦锁死（中心区死区的来源）

    def to_channels(self) -> dict[str, float]:
        return {
            "steer_hand_torque": self.hand_torque,
            "steer_torque_sensor": self.torque_sensor,
            "steer_assist_torque": self.assist_torque,
            "steer_motor_torque": self.motor_torque,
            "steer_motor_speed": self.motor_speed,
            "steer_motor_heat": self.motor_heat,
            "steer_pinion_angle": self.pinion_angle,
            "steer_friction_torque": self.friction_torque,
        }


@dataclass
class SteeringPlant:
    """Column + torsion bar + pinion, driven by an imposed hand-wheel angle."""

    params: SteeringSystemParams
    assist_map: AssistMap
    pinion_radius: float = 0.020
    #: Defaults to the plant's own reduction (see SteeringSystemParams).
    motor_gear_ratio: float = 0.0
    #: Pinion-shaft inertia that is not the rack or the motor (shaft, gear).
    pinion_inertia: float = 2.0e-3
    #: Largest internal step [s]. The assist loop raises the effective torsion
    #: stiffness by the boost ratio — around 59x at parking on the default map
    #: — which moves the column mode from 2.5 Hz to roughly 19 Hz. A 5 ms
    #: vehicle step cannot integrate that: the first version of this model rang
    #: hard enough that assist *raised* parking effort from 97 to 167 N.m.
    #: Sub-stepping is the cheap fix (this is scalar arithmetic, and real EPS
    #: plants run at kHz inside a slower vehicle loop).
    max_substep_s: float = 2.0e-4

    state: SteeringPlantState = field(default_factory=SteeringPlantState)
    _motor: Motor = field(init=False, repr=False)
    _prev_hand_rate: float = field(default=0.0, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.motor_gear_ratio <= 0.0:
            self.motor_gear_ratio = self.params.motor_gear_ratio
        self._motor = Motor(self.params.motor)

    # ---- helpers -----------------------------------------------------------

    def reset(self, pinion_angle: float = 0.0) -> None:
        self.state = SteeringPlantState(pinion_angle=pinion_angle)
        self._motor.reset()
        self._prev_hand_rate = 0.0

    @property
    def equivalent_inertia(self) -> float:
        """J_eq at the pinion: shaft + rack referred through r_p + rotor through N."""
        r = max(self.pinion_radius, 1e-6)
        n = max(self.motor_gear_ratio, 1e-6)
        return (
            self.pinion_inertia
            + self.params.rack.mass * r * r
            + self.params.motor.inertia * n * n
        )

    def natural_frequency_hz(self) -> float:
        """Column mode frequency — the thing that decides the usable step size.

        Reported rather than assumed: a torsion bar stiff enough or an inertia
        small enough puts this above what a 5 ms step can carry, and the caller
        should be told rather than silently integrating nonsense.
        """
        k = self.params.column.torsion_stiffness
        return math.sqrt(k / max(self.equivalent_inertia, 1e-12)) / (2.0 * math.pi)

    # ---- step --------------------------------------------------------------

    def _substep(
        self,
        dt: float,
        *,
        hand_angle: float,
        hand_rate: float,
        rack_force: float,
        speed_ms: float,
        assist_enabled: bool = True,
    ) -> SteeringPlantState:
        """Advance one step.

        `rack_force` [N] is the tyre-side load from the existing kingpin →
        linkage chain, positive when it opposes a positive pinion rotation.
        """
        p = self.params
        s = self.state
        dt = max(float(dt), 1e-9)
        r = max(self.pinion_radius, 1e-6)
        n = max(self.motor_gear_ratio, 1e-6)

        k_tb = p.column.torsion_stiffness
        twist = float(hand_angle) - s.pinion_angle
        rate_diff = float(hand_rate) - s.pinion_rate
        torsion = k_tb * twist + p.column.torsion_damping * rate_diff

        # The sensor measures twist only — see the module docstring.
        sensor = max(-p.column.sensor_range_nm,
                     min(p.column.sensor_range_nm, k_tb * twist))

        # Assist: map → motor (with its envelope) → back to the pinion.
        assist_pinion = 0.0
        if assist_enabled:
            want_pinion = self.assist_map.assist_torque(sensor, speed_ms)
            motor_cmd = want_pinion / n
            m = self._motor.step(dt, motor_cmd, s.pinion_rate * n)
            assist_pinion = m.torque * n
        else:
            m = self._motor.step(dt, 0.0, s.pinion_rate * n)

        load = float(rack_force) * r

        # Viscous first; Coulomb is resolved against the net torque below.
        viscous = p.rack.viscous_n_per_mps * r * r * s.pinion_rate
        applied = torsion + assist_pinion - load - viscous
        coulomb = p.rack.coulomb_friction_n * r

        j = self.equivalent_inertia
        # Torque needed to bring the pinion to rest exactly at the end of this
        # step. If Coulomb friction can supply it, the pinion sticks — this is
        # what creates the on-centre deadband and the hysteresis loop.
        hold = applied + j * s.pinion_rate / dt
        if abs(hold) <= coulomb:
            friction = -applied - j * s.pinion_rate / dt
            rate = 0.0
            stuck = True
        else:
            friction = -math.copysign(coulomb, s.pinion_rate if abs(s.pinion_rate) > 1e-9
                                      else applied)
            rate = s.pinion_rate + (applied + friction) / j * dt
            stuck = False

        angle = s.pinion_angle + rate * dt

        # Hand-wheel torque: what a steering robot's load cell would read.
        hand_accel = (float(hand_rate) - self._prev_hand_rate) / dt
        self._prev_hand_rate = float(hand_rate)
        col_friction = (math.copysign(p.column.coulomb_friction, hand_rate)
                        if abs(hand_rate) > 1e-6 else 0.0)
        hand_torque = (
            p.column.inertia * hand_accel
            + p.column.damping * float(hand_rate)
            + col_friction
            + torsion
        )

        self.state = SteeringPlantState(
            pinion_angle=angle,
            pinion_rate=rate,
            hand_angle=float(hand_angle),
            hand_rate=float(hand_rate),
            torsion_torque=torsion,
            torque_sensor=sensor,
            assist_torque=assist_pinion,
            hand_torque=hand_torque,
            load_torque=load,
            friction_torque=friction + (-viscous),
            motor_torque=m.torque,
            motor_speed=m.speed,
            motor_heat=m.heat,
            motor_saturated=m.saturated,
            stuck=stuck,
        )
        return self.state

    def step(
        self,
        dt: float,
        *,
        hand_angle: float,
        hand_rate: float,
        rack_force: float,
        speed_ms: float,
        assist_enabled: bool = True,
    ) -> SteeringPlantState:
        """Advance one vehicle step, sub-stepping internally for stability.

        The inputs are held across the sub-steps: they come from the outer loop
        and have no finer information to offer.
        """
        dt = max(float(dt), 1e-9)
        n = max(1, min(256, math.ceil(dt / max(self.max_substep_s, 1e-9))))
        h = dt / n
        state = self.state
        for _ in range(n):
            state = self._substep(
                h,
                hand_angle=hand_angle,
                hand_rate=hand_rate,
                rack_force=rack_force,
                speed_ms=speed_ms,
                assist_enabled=assist_enabled,
            )
        return state

    def assisted_mode_hz(self, boost_ratio: float) -> float:
        """Column mode once assist closes the loop — the binding constraint.

        `natural_frequency_hz` reports the open-loop mode, which is the
        reassuring number and the wrong one to size a step against.
        """
        k = self.params.column.torsion_stiffness * (1.0 + max(boost_ratio, 0.0))
        return math.sqrt(k / max(self.equivalent_inertia, 1e-12)) / (2.0 * math.pi)
