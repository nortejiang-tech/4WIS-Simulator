"""The steering-decoupling ladder: four architectures, defined by hardware.

The question the study asks is what each additional *decoupled degree of
freedom* in the steering system is worth. To answer it honestly the four rungs
have to differ by things a supplier would quote — angle authority and actuator
bandwidth — not by which control law we happen to run on them. Control law is
the independent variable of a separate sweep (`ALGORITHMS`), run on fixed
hardware, so that "what the architecture buys" and "what the algorithm buys"
never get mixed up.

    L0  EPS            front axle only, mechanical column, fixed ratio
    L1  EPS + RWS      + rear axle, +/-3 deg, slow rear actuator
    L2  SBW + RWS      front decoupled from the column: variable ratio,
                       higher bandwidth; rear authority up to +/-5 deg
    L3  4WIS           four independently steered wheels, full +/-35 deg

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
  * L3 is given the platform's full +/-35 deg on all four wheels. That is this
    project's own vehicle, not a production 4WIS car; read L3 as "what full
    decoupling makes available", not as a shipping specification.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

from sim4wis.core.state import VehicleParams


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
        rear_limit_deg=3.0, steer_tau=0.075, steer_rate_max=7.0,
        variable_ratio=False, per_wheel=False,
        blurb="A rear-steer actuator adds a second DOF, but the front angle is "
              "still whatever the driver's hands did. Rear authority is the "
              "+/-3 deg typical of a production rear-axle module.",
    ),
    Architecture(
        key="L2", label="SBW + RWS", label_cn="线控前轮 + 后轮转向", dof=2,
        rear_limit_deg=5.0, steer_tau=0.045, steer_rate_max=12.0,
        variable_ratio=True, per_wheel=False,
        blurb="Steer-by-wire breaks the mechanical link, so the front angle "
              "becomes a controlled variable too: variable ratio, higher "
              "bandwidth, and front-axle authority available to the stability "
              "controller rather than only to the driver.",
    ),
    Architecture(
        key="L3", label="4WIS", label_cn="四轮独立转向", dof=4,
        rear_limit_deg=35.0, steer_tau=0.030, steer_rate_max=15.0,
        variable_ratio=True, per_wheel=True,
        blurb="Four independently steered wheels. The ICR is no longer "
              "constrained to a line through the rear axle, so yaw rate and "
              "sideslip become independently commandable — and manoeuvres with "
              "no Ackermann solution at all (crab, zero-radius) open up.",
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
