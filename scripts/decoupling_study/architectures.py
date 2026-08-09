"""The steering-decoupling ladder: four architectures, defined by hardware.

The question the study asks is what each additional *decoupled degree of
freedom* in the steering system is worth. To answer it honestly the four rungs
have to differ by things a supplier would quote — angle authority and actuator
bandwidth — not by which control law we happen to run on them. Control law is
the independent variable of a separate sweep (`ALGORITHMS`), run on fixed
hardware, so that "what the architecture buys" and "what the algorithm buys"
never get mixed up.

All four are held to ONE angle envelope — front +/-40 deg, rear +/-7 deg — so
that the comparison is between steering SYSTEMS and not between steering
ANGLES. What an extra degree of rear authority is worth is a different
question with its own study (scripts/rear_angle_study).

    L0  EPS            front axle only, mechanical column, fixed ratio
    L1  EPS + RWS      + rear axle within the shared envelope, slow actuator
    L2  SBW + RWS      front decoupled from the column: variable ratio and
                       higher bandwidth, same rear angle as L1
    L3  4WIS           four independently steered wheels, same envelope again:
                       only per-wheel ICR placement and actuator speed remain

Modelling limitations, stated up front because they bound what the numbers
mean:

  * `steer_tau` / `steer_rate_max` are vehicle-level parameters, so a rung gets
    ONE actuator bandwidth standing for the steering system as a whole. Where a
    real L1 car has a fast front and a slow rear, this study gives it a single
    intermediate value. That understates L1's front-axle response and
    overstates its rear-axle response; the net effect on the metrics reported
    here is small because the rear angle is small, but it is not zero.
  * Rear angle authority is imposed by clipping the rear axle command inside
    the harness rather than by a per-axle `steer_limit` in the model. The clip
    happens before the actuator, so the limit is on the command, not on a
    mechanical endstop that the actuator would wind up against.
  * The shared envelope deliberately takes away L3's headline manoeuvres. Crab
    and zero-radius need rear angles several times 7 deg, so under this
    constraint 4WIS simply cannot do them. That is a real consequence of the
    constraint, not an oversight, and the report says so rather than quietly
    reporting 4WIS as "barely better than L2".
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

from sim4wis.core.state import VehicleParams

#: Angle envelope every architecture is held to, so the ladder compares
#: steering SYSTEMS rather than steering ANGLES. Without this, most of what
#: looked like an architecture effect at low speed was really just L3 being
#: allowed five times the rear angle of L1. What each extra degree of rear
#: authority is worth is a separate question, answered by its own study
#: (scripts/rear_angle_study).
#:
#: L0 keeps 0 deg of rear angle. That is not a limit choice — a front-steer-only
#: car has no rear actuator to limit.
FRONT_LIMIT_DEG = 40.0
REAR_LIMIT_DEG = 7.0


@dataclass(frozen=True)
class Architecture:
    key: str
    label: str
    label_cn: str
    dof: int                      # independently commandable steering DOF
    rear_limit_deg: float         # rear axle angle authority (0 = no rear steer)
    steer_tau: float              # actuator first-order lag [s]
    steer_rate_max: float         # actuator rate limit [rad/s]
    variable_ratio: bool          # is the front angle decoupled from the column
    per_wheel: bool               # can each wheel be commanded independently
    blurb: str

    def params(self, base: VehicleParams | None = None) -> VehicleParams:
        """Vehicle parameters carrying this rung's steering hardware."""
        return replace(base or VehicleParams(),
                       steer_limit=math.radians(FRONT_LIMIT_DEG),
                       steer_tau=self.steer_tau,
                       steer_rate_max=self.steer_rate_max)

    @property
    def rear_limit_rad(self) -> float:
        return math.radians(self.rear_limit_deg)


LADDER: tuple[Architecture, ...] = (
    Architecture(
        key="L0", label="EPS", label_cn="前轮 EPS", dof=1,
        rear_limit_deg=0.0, steer_tau=0.090, steer_rate_max=6.0,
        variable_ratio=False, per_wheel=False,
        blurb="Mechanical column with electric assist. One steering DOF: the "
              "rear axle tracks the body, so yaw and sideslip are rigidly "
              "coupled by the chassis.",
    ),
    Architecture(
        key="L1", label="EPS + RWS", label_cn="EPS + 后轮转向", dof=2,
        rear_limit_deg=REAR_LIMIT_DEG, steer_tau=0.075, steer_rate_max=7.0,
        variable_ratio=False, per_wheel=False,
        blurb="A rear-steer actuator adds a second DOF, but the front angle is "
              "still whatever the driver's hands did. Rear authority is the "
              "shared +/-7 deg envelope.",
    ),
    Architecture(
        key="L2", label="SBW + RWS", label_cn="线控前轮 + 后轮转向", dof=2,
        rear_limit_deg=REAR_LIMIT_DEG, steer_tau=0.045, steer_rate_max=12.0,
        variable_ratio=True, per_wheel=False,
        blurb="Steer-by-wire breaks the mechanical link, so the front angle "
              "becomes a controlled variable too: variable ratio, higher "
              "bandwidth, and front-axle authority available to the stability "
              "controller rather than only to the driver. Same rear angle as "
              "L1 — what improves is how fast and how precisely it is placed.",
    ),
    Architecture(
        key="L3", label="4WIS", label_cn="四轮独立转向", dof=4,
        rear_limit_deg=REAR_LIMIT_DEG, steer_tau=0.030, steer_rate_max=15.0,
        variable_ratio=True, per_wheel=True,
        blurb="Four independently steered wheels. Held to the same angle "
              "envelope as L1/L2, so what is left is per-wheel ICR placement "
              "and the fastest actuator. Note what the envelope costs it: crab "
              "and zero-radius need rear angles far beyond 7 deg, so under this "
              "constraint 4WIS cannot perform its signature manoeuvres at all.",
    ),
)

BY_KEY = {a.key: a for a in LADDER}


@dataclass(frozen=True)
class Algorithm:
    key: str
    label: str
    label_cn: str
    mode_params: dict = field(default_factory=dict)
    needs_rear: bool = True
    blurb: str = ""


#: Control laws, swept on FIXED hardware so the algorithm contribution is
#: separable from the architecture contribution.
ALGORITHMS: tuple[Algorithm, ...] = (
    Algorithm("none", "front only", "仅前轮",
              {"rws_mode": "fixed_ratio", "rear_ratio": 0.0}, needs_rear=False,
              blurb="Rear axle held straight — the L0 control law, run on "
                    "whatever hardware, as the reference point."),
    Algorithm("fixed", "fixed ratio", "定比例",
              {"rws_mode": "fixed_ratio", "rear_ratio": -0.4},
              blurb="delta_r = k*delta_f with k constant. The cheapest law: "
                    "one number, no sensing beyond steering angle."),
    Algorithm("schedule", "speed schedule", "速度调度",
              {"rws_mode": "speed_schedule"},
              blurb="k = k(v), sampled from the analytic zero-sideslip ratio. "
                    "Counter-phase below the crossover, in-phase above it. "
                    "Open loop, so it is only as good as the model it was "
                    "sampled from."),
    Algorithm("yaw_fb", "yaw feedback", "横摆反馈",
              {"rws_mode": "yaw_feedback"},
              blurb="delta_r = g1*delta_f + g2*yaw. Closed loop on the measured "
                    "yaw rate, so it does not depend on knowing C_alpha."),
    Algorithm("transient", "transient", "瞬态补偿",
              {"rws_mode": "transient"},
              blurb="Steady-state schedule plus a counter-steer term on "
                    "d(delta_f)/dt — targets the phase lag specifically, at the "
                    "cost of amplifying steering-sensor noise."),
    Algorithm("model_follow", "model following", "模型跟随",
              {"rws_mode": "model_following"},
              blurb="Zero-sideslip feed-forward plus feedback on the error "
                    "against a reference yaw rate. The most capable of the "
                    "five, and the one that needs the most vehicle knowledge."),
)

ALGO_BY_KEY = {a.key: a for a in ALGORITHMS}

#: The control law each rung is given when the sweep is over ARCHITECTURES.
#: Each rung gets the most capable law its hardware can actually execute, so
#: the ladder comparison is "best available at this hardware level".
DEFAULT_LAW = {
    "L0": "none",
    "L1": "schedule",
    "L2": "model_follow",
    "L3": "model_follow",
}
