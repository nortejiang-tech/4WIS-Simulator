"""By-wire front axle — angle tracking and road-feel synthesis.

A mechanically-steered car has one problem: transmit what the tyres are doing
to the driver's hands, with as little corruption as possible. A by-wire car has
two, and they are independent:

    road wheel actuator   put the wheels where they were asked to be
    feedback actuator     make the driver feel something

Nothing connects them but software. That is the cost — road feel has to be
*designed*, and a poor design feels like a video game — and it is also the
entire point, because the path that used to carry road information also carried
kickback, torque steer and every impact the tyre ever took. Cutting it lets you
keep the information and drop the assault.

So the feedback torque here is a composition rather than a measurement:

    tau_hand = road-force term      scaled rack force, low-passed
             + centring term        spring toward centre, speed-scheduled
             + damping              proportional to hand rate
             + friction             a little, or the wheel feels like a toy
             clipped to the feedback motor's capability

The low-pass on the rack-force term is the one that matters most. Passing rack
force straight through reproduces exactly the kickback a mechanical column
would have delivered, which throws away the reason for going by-wire in the
first place. `test_kickback_is_filtered` pins that.

Angle deviation — commanded minus actual road wheel angle — is monitored rather
than assumed small. On a by-wire car it is a safety-relevant signal: it is how
you find out the actuator has saturated, stalled, or lost a phase, and it is
the trigger for the `angle_deviation` failure mode the architecture registry
lists.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from sim4wis.steering.motor import Motor
from sim4wis.steering.params import MotorParams


@dataclass
class ByWireParams:
    """Two actuators and a feel calibration."""

    # ---- road wheel actuator ----------------------------------------------
    #: Closed-loop position bandwidth [Hz]. Production SBW front actuators sit
    #: around 8-15 Hz; above that the mechanical resonances of the rack start
    #: to matter more than the controller.
    tracking_bandwidth_hz: float = 10.0
    #: Rate limit at the road wheel [rad/s]. This is what an evasive input runs
    #: into, and it is the reason a by-wire car can feel fine everywhere except
    #: at the limit.
    rate_limit: float = 6.0
    #: Deviation beyond which the monitor flags [rad] (~2.9 deg).
    angle_deviation_limit: float = 0.05

    # ---- feedback actuator -------------------------------------------------
    feedback_peak_nm: float = 8.0
    #: Hand torque per kN of rack force, at low and high speed [N·m/kN].
    #: Less at parking, or the driver fights the same force the assist exists
    #: to remove; more at speed, where the information is worth having.
    road_feel_low: float = 0.18
    road_feel_high: float = 0.55
    road_feel_v_ref: float = 22.0        # [m/s] where the blend is halfway
    #: Cut-off of the low-pass on the road-force term [Hz]. The kickback filter.
    road_feel_lowpass_hz: float = 6.0
    #: Centring spring [N·m/rad] at the hand wheel, and how much it grows with
    #: speed. A by-wire car has no caster torque reaching the wheel, so without
    #: this it simply does not return.
    centring_low: float = 0.8
    centring_high: float = 3.2
    damping: float = 0.55                # [N·m·s/rad]
    friction: float = 0.25               # [N·m] — "solid" rather than toy-like

    motor: MotorParams = field(default_factory=lambda: MotorParams(
        peak_torque=8.0, continuous_torque=4.0, bandwidth_hz=60.0,
        no_load_speed_rpm=1200.0, inertia=6.0e-5,
    ))


@dataclass
class ByWireState:
    road_wheel_angle: float = 0.0
    road_wheel_rate: float = 0.0
    commanded_angle: float = 0.0
    angle_deviation: float = 0.0
    deviation_flag: bool = False
    hand_torque: float = 0.0
    road_feel_torque: float = 0.0
    centring_torque: float = 0.0
    feedback_saturated: bool = False
    rate_limited: bool = False

    def to_channels(self) -> dict[str, float]:
        return {
            "steer_hand_torque": self.hand_torque,
            "steer_road_feel_torque": self.road_feel_torque,
            "steer_centring_torque": self.centring_torque,
            "steer_angle_deviation": self.angle_deviation,
            "steer_road_wheel_angle": self.road_wheel_angle,
        }


class ByWirePlant:
    """Front axle with no mechanical connection to the hand wheel."""

    def __init__(self, params: ByWireParams | None = None) -> None:
        self.p = params or ByWireParams()
        self.state = ByWireState()
        self._feedback = Motor(self.p.motor)
        self._road_force_filtered = 0.0

    def reset(self, angle: float = 0.0) -> None:
        self.state = ByWireState(road_wheel_angle=angle)
        self._feedback.reset()
        self._road_force_filtered = 0.0

    # ---- feel calibration ---------------------------------------------------

    def _blend(self, low: float, high: float, speed_ms: float) -> float:
        """Speed schedule shared by the road-feel and centring gains."""
        v = abs(float(speed_ms))
        w = v / (v + max(self.p.road_feel_v_ref, 1e-6))
        return low + (high - low) * w

    def road_feel_gain(self, speed_ms: float) -> float:
        return self._blend(self.p.road_feel_low, self.p.road_feel_high, speed_ms)

    def centring_gain(self, speed_ms: float) -> float:
        return self._blend(self.p.centring_low, self.p.centring_high, speed_ms)

    # ---- step ---------------------------------------------------------------

    def step(
        self,
        dt: float,
        *,
        hand_angle: float,
        hand_rate: float,
        delta_cmd: float,
        rack_force: float,
        speed_ms: float,
    ) -> ByWireState:
        """Advance both actuators one step.

        `delta_cmd` is the road wheel angle the control law asked for; the hand
        wheel is an independent input that only drives the feel.
        """
        p = self.p
        s = self.state
        dt = max(float(dt), 1e-9)

        # --- road wheel actuator: bandwidth-limited tracking, rate limited ---
        tau = 1.0 / max(2.0 * math.pi * p.tracking_bandwidth_hz, 1e-6)
        alpha = 1.0 - math.exp(-dt / tau)
        target = s.road_wheel_angle + (float(delta_cmd) - s.road_wheel_angle) * alpha
        max_step = p.rate_limit * dt
        step_taken = target - s.road_wheel_angle
        rate_limited = abs(step_taken) > max_step
        if rate_limited:
            step_taken = math.copysign(max_step, step_taken)
        angle = s.road_wheel_angle + step_taken
        rate = step_taken / dt

        deviation = float(delta_cmd) - angle

        # --- feedback actuator: synthesise a torque --------------------------
        # Low-pass the rack force before it reaches the hand. This is the whole
        # kickback argument for by-wire: keep the information, drop the impact.
        f_tau = 1.0 / max(2.0 * math.pi * p.road_feel_lowpass_hz, 1e-6)
        f_alpha = 1.0 - math.exp(-dt / f_tau)
        self._road_force_filtered += (float(rack_force) - self._road_force_filtered) * f_alpha

        road = self.road_feel_gain(speed_ms) * (self._road_force_filtered / 1000.0)
        centring = self.centring_gain(speed_ms) * float(hand_angle)
        damping = p.damping * float(hand_rate)
        friction = (math.copysign(p.friction, hand_rate)
                    if abs(hand_rate) > 1e-6 else 0.0)

        want = road + centring + damping + friction
        m = self._feedback.step(dt, want, float(hand_rate))

        self.state = ByWireState(
            road_wheel_angle=angle,
            road_wheel_rate=rate,
            commanded_angle=float(delta_cmd),
            angle_deviation=deviation,
            deviation_flag=abs(deviation) > p.angle_deviation_limit,
            hand_torque=m.torque,
            road_feel_torque=road,
            centring_torque=centring,
            feedback_saturated=m.saturated,
            rate_limited=rate_limited,
        )
        return self.state
