"""The mechanical chain — hand wheel, torsion bar, pinion.

This is the piece that makes hand-wheel torque exist as a signal.

Convention: **angle is commanded, and the driver is a spring-damper onto it.**
Every objective test procedure drives angle and measures torque, and every
input path in this simulator produces a position, so angle is the input. But
the thing holding the wheel is not a rigid constraint — it has finite
stiffness, damping and strength, and leaving that out is what made the model
ring in the one place hardware does not (see below).

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

Damping is not optional
-----------------------
Assist is proportional feedback on twist, so it multiplies the effective
stiffness the pinion inertia works against. On the default map that is 59x at
parking, which leaves the loop at zeta = 0.0036 on mechanical damping alone.
Built without a damping term, this model did exactly what the hardware would:
the pinion overshot the hand wheel, twist went negative, assist reversed, and
it diverged until the torque sensor sat pinned at saturation with 87 degrees of
twist. Assist *raised* parking effort from 96.5 to 174.3 N.m.

That is why every production EPS carries a damping function. The gain here is
scheduled on the local boost so the loop damping ratio holds; a real
calibration uses a table, and this is the shape that table approximates.

The driver has to be compliant
------------------------------
With assist saturated the motor clips everything it was asked for, damping
included, and the pinion is left as a mass on the torsion bar at zeta ~ 0.03.
An imposed angle is infinitely stiff and absorbs none of that, so the model
rang — and worst in the *marginal* band, a motor that almost holds the load,
which is exactly the band an actuator-sizing pass operates in. A real pair of
arms is compliant, damped and finite in strength, and absorbs it.

Freeing the column DOF removed the band entirely. Sweeping motor size at
full-lock parking, 20.7 kN:

    2.0 N.m   17.98 N.m at the hand wheel, 1404 rpm
    4.0       10.32                        1672
    5.5        4.63                        1539
    8.0        4.66                        1539
    12.0       4.66                        1539

Monotone, no oscillation anywhere, and an undersized motor now reads as heavy
steering rather than as a numerical excursion.

Step-size note: the column mode is ~10 Hz (and 5x+ once assist closes the
loop), so the plant cannot be driven by a command held constant across the
whole 5 ms vehicle step — the peak hand torque used to come out ~1/3 low
because the peak fell between outer samples (D5). The coupling in
`vehicle/steering_link.py` therefore advances the plant at a 0.5 ms inner
rate with the commanded angle interpolated across the span; callers that
drive `step()` directly still get a zero-order hold and should read RMS
unless they use a comparably fine step.

Plausible, not validated: no bench or vehicle data backs these numbers, and
the parameters are engineering estimates. See docs/v2_steering_platform_plan.md
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from sim4wis.steering.assist import AssistMap
from sim4wis.steering.motor import Motor
from sim4wis.steering.params import SteeringSystemParams

#: Inner rate of the steering layer [s]. The vehicle outer loop runs at its own
#: step (5 ms default); the column mode (~10 Hz, and 5x+ once assist closes the
#: loop) cannot be resolved by a command held across that whole span, so every
#: path that drives the plant for a *peak* quantity advances it 10x finer with
#: the commanded angle interpolated across the span — a ramp, not a staircase.
#: 0.5 ms vs a 0.25 ms reference agree to <2% on the peak hand torque.
STEERING_INNER_DT = 0.5e-3


@dataclass
class SteeringPlantState:
    """One step of the steering system. All torques N·m, angles rad."""

    pinion_angle: float = 0.0
    pinion_rate: float = 0.0
    hand_angle: float = 0.0            # 方向盘实际角（自由度，非强加）
    hand_rate: float = 0.0
    commanded_hand_angle: float = 0.0  # 驾驶员/机器人想要的角度
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
    hand_limited: bool = False         # 保持该角度所需手力矩超出人/机器人能力

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
        rack_resistance: float = 0.0,
    ) -> SteeringPlantState:
        """Advance one step.

        `rack_force` [N] is the tyre-side load from the existing kingpin →
        linkage chain, positive when it opposes a positive pinion rotation. It
        is **angle-directed**: it restores toward centre, like the aligning
        torque of a rolling tyre.

        `rack_resistance` [N] is the part of the tyre load that is *dissipative*
        rather than restoring — the scrub of a tyre pivoting in place. It
        opposes motion and holds a stationary rack, so it is resolved with the
        rack's own Coulomb friction rather than added to `rack_force`. The
        distinction is not pedantic: a standing tyre at full lock resists being
        turned with several hundred N.m and does **not** spring back when
        released, and a model that signs that load by angle instead has built a
        400 N.m centring spring that exists nowhere.
        """
        p = self.params
        s = self.state
        dt = max(float(dt), 1e-9)
        r = max(self.pinion_radius, 1e-6)
        n = max(self.motor_gear_ratio, 1e-6)

        k_tb = p.column.torsion_stiffness

        # The driver is a spring-damper onto the commanded angle, not a rigid
        # constraint. `hand_angle` is what they are reaching for; `sw` is where
        # the wheel actually is.
        sw, sw_rate = s.hand_angle, s.hand_rate
        driver = (p.column.grip_stiffness * (float(hand_angle) - sw)
                  + p.column.grip_damping * (float(hand_rate) - sw_rate))
        limit = p.column.hand_torque_limit_nm
        hand_limited = abs(driver) > limit
        if hand_limited:
            driver = math.copysign(limit, driver)

        twist = sw - s.pinion_angle
        rate_diff = sw_rate - s.pinion_rate
        torsion = k_tb * twist + p.column.torsion_damping * rate_diff

        # The sensor measures twist only — see the module docstring.
        sensor = max(-p.column.sensor_range_nm,
                     min(p.column.sensor_range_nm, k_tb * twist))

        # Assist: map → motor (with its envelope) → back to the pinion.
        assist_pinion = 0.0
        if assist_enabled:
            # Boost curve, plus the damping that makes the loop stable at all.
            # Scheduled on the local boost so the loop damping ratio holds:
            # c = 2*zeta*sqrt(K_eff*J) with K_eff = K_tb*(1 + boost).
            # Schedule on the boost the motor can actually *deliver*, not the
            # one the map asks for. Past saturation the map keeps promising
            # more gain while the loop gain has stopped rising, so scheduling
            # on the map over-damps in one direction and mis-tunes the loop in
            # the other — which is how a request the hardware cannot meet turns
            # into an oscillation instead of into heavy steering. A sizing pass
            # drives past capability on purpose, so this path is normal, not
            # exceptional.
            boost = self.assist_map.boost_ratio(sensor, speed_ms)
            # Cap by the motor's *static* capability, not by what is available
            # at this instant. Using the instantaneous ceiling made the damping
            # gain depend on motor speed, which depends on the damping — a
            # feedback loop of my own making, and it chattered exactly at the
            # motor size that almost holds the load (5.5 N.m rang at 85 000 rpm
            # while both 3.0 and 8.0 were well behaved). A damping schedule
            # exists to keep zeta sane across the operating range; it has no
            # business reacting within a step.
            ceiling = self.params.motor.peak_torque * n
            if abs(sensor) > 1e-9:
                boost = min(boost, ceiling / abs(sensor))
            k_eff = k_tb * (1.0 + boost)
            c_damp = (2.0 * p.assist_damping_ratio
                      * math.sqrt(max(k_eff * self.equivalent_inertia, 0.0)))
            # Damp the **twist rate**, not the pinion rate.
            #
            # The unstable mode is the torsion bar's: its state is the twist
            # and its velocity is (pinion_rate - hand_rate), so that is the
            # signal to damp. Damping the pinion's absolute rate also stabilises
            # the mode, and charges the whole manoeuvre for it — the term then
            # opposes steering itself, so the faster you turn the less help you
            # get. Measured on the default map: at a parking rate of 193 deg/s
            # at the pinion it removed 195 N.m of the 413 N.m the map had asked
            # for, which put the driver on their 25 N.m limit, lost them the
            # wheel, and let a fully loaded rack drive the pinion backwards
            # until the run left physics entirely (345 000 rpm).
            #
            # In steady turning the pinion tracks the hand wheel and this term
            # vanishes, which is the point. In the oscillation the relative
            # rate *is* the oscillation, so it bites in full. Production EPS
            # damping functions are scheduled to behave this way for the same
            # reason; here the structure gets it rather than a schedule.
            want_pinion = (self.assist_map.assist_torque(sensor, speed_ms)
                           - c_damp * (s.pinion_rate - sw_rate))
            motor_cmd = want_pinion / n
            m = self._motor.step(dt, motor_cmd, s.pinion_rate * n)
            assist_pinion = m.torque * n
        else:
            m = self._motor.step(dt, 0.0, s.pinion_rate * n)

        load = float(rack_force) * r

        # Viscous first; Coulomb is resolved against the net torque below.
        viscous = p.rack.viscous_n_per_mps * r * r * s.pinion_rate
        applied = torsion + assist_pinion - load - viscous
        coulomb = (p.rack.coulomb_friction_n + abs(float(rack_resistance))) * r

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

        # Column DOF: the wheel accelerates under what the driver applies minus
        # what the torsion bar takes back. Hand torque is the *applied* torque,
        # which is what a robot's load cell reads and what the driver feels.
        col_friction = (math.copysign(p.column.coulomb_friction, sw_rate)
                        if abs(sw_rate) > 1e-6 else 0.0)
        sw_accel = ((driver - torsion - p.column.damping * sw_rate - col_friction)
                    / max(p.column.inertia, 1e-9))
        sw_rate_new = sw_rate + sw_accel * dt
        sw_new = sw + sw_rate_new * dt
        hand_torque = driver

        self.state = SteeringPlantState(
            pinion_angle=angle,
            pinion_rate=rate,
            hand_angle=sw_new,
            hand_rate=sw_rate_new,
            commanded_hand_angle=float(hand_angle),
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
            hand_limited=hand_limited,
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
        rack_resistance: float = 0.0,
    ) -> SteeringPlantState:
        """Advance one vehicle step, sub-stepping internally for stability.

        The inputs are held across the sub-steps. The vehicle coupling
        (`steering_link.plant_front_angle`) calls this at a 0.5 ms inner rate
        with the command already interpolated, so a direct caller using the
        full outer step gets a coarser zero-order hold of the command than the
        vehicle models do.
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
                rack_resistance=rack_resistance,
            )
        return state

    def assisted_mode_hz(self, boost_ratio: float) -> float:
        """Column mode once assist closes the loop — the binding constraint.

        `natural_frequency_hz` reports the open-loop mode, which is the
        reassuring number and the wrong one to size a step against.
        """
        k = self.params.column.torsion_stiffness * (1.0 + max(boost_ratio, 0.0))
        return math.sqrt(k / max(self.equivalent_inertia, 1e-12)) / (2.0 * math.pi)
