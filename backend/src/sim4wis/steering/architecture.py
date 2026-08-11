"""Steering architectures — the configuration everything else hangs off.

Today `strategy` conflates two independent things: *what the control law does*
(Ackermann, zero sideslip, crab) and *what hardware it runs on* (column assist,
by-wire, rear axle present or not). They are orthogonal. Zero-sideslip rear
steering is the same control law whether the front axle is a mechanical EPS or
by-wire; loss of assist is a failure mode of an EPS and meaningless on a
by-wire front axle. Keeping them fused is why EPS, SBW, RWS and 4WIS can only
be four parallel names rather than four configurations of one system.

An architecture answers four questions, and each one is load-bearing somewhere:

    what exists      which sensors and actuators are physically present
    what it can do   which quantities can be measured at all
    how it can fail  which failure modes are even applicable
    how it is geared the reduction from motor to pinion, which differs by an
                     order of magnitude between column and rack assist

That last one settles something left open in V1. The legacy
`SteeringGeometryParams.motor_gear_ratio` of 10 implies a 20.8 N.m motor for a
10.4 kN parking force, where production hardware is 3-6 N.m. It was never wrong
in general — it is roughly right for *column* assist through a worm gear, and
badly wrong for *rack* assist through a ball screw. The number belongs to the
architecture, not to the vehicle.

Capabilities use the same vocabulary as the study layer's metric registry, so
the envelope guard extends to architecture without a second mechanism: asking
a C-EPS configuration for rear-axle phase is refused the same way asking the
kinematic model for lateral acceleration is.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

# ---------------------------------------------------------------------------
# Capabilities — what a configuration can be asked about
# ---------------------------------------------------------------------------

CAP_HAND_TORQUE = "hand_torque"           # 手力矩（中心区手感、回正性的前提）
CAP_TORQUE_SENSOR = "torque_sensor"       # 扭杆扭矩传感器信号
CAP_ASSIST_CAL = "assist_calibration"     # 有助力曲线可标定
CAP_ANGLE_TRACKING = "angle_tracking"     # 线控前轴的角度跟踪
CAP_ROAD_FEEL = "road_feel_synthesis"     # 路感合成（线控才需要）
CAP_REAR_STEER = "rear_steer"             # 后轮转向
CAP_PER_WHEEL = "per_wheel_steer"         # 四轮独立
CAP_REDUNDANCY = "redundancy"             # 冗余通道/降级切换

FrontPath = Literal["mechanical", "by_wire"]
AssistAt = Literal["column", "pinion", "dual_pinion", "rack", "none"]
RearAxle = Literal["none", "coupled", "independent"]


@dataclass(frozen=True)
class Architecture:
    """One steering system configuration."""

    id: str
    label: str
    label_en: str
    description: str

    front_path: FrontPath
    assist_at: AssistAt
    rear_axle: RearAxle
    front_independent: bool

    sensors: frozenset[str]
    actuators: frozenset[str]
    provides: frozenset[str]
    failure_modes: frozenset[str]

    #: Motor-to-pinion reduction for this architecture's drive. See the module
    #: docstring — this is why one global number could not be right.
    motor_gear_ratio: float

    def can(self, capability: str) -> bool:
        return capability in self.provides

    def to_dict(self) -> dict[str, object]:
        return {
            "id": self.id,
            "label": self.label,
            "label_en": self.label_en,
            "description": self.description,
            "front_path": self.front_path,
            "assist_at": self.assist_at,
            "rear_axle": self.rear_axle,
            "front_independent": self.front_independent,
            "sensors": sorted(self.sensors),
            "actuators": sorted(self.actuators),
            "provides": sorted(self.provides),
            "failure_modes": sorted(self.failure_modes),
            "motor_gear_ratio": self.motor_gear_ratio,
        }


# Failure-mode vocabulary. EPS and by-wire fail in genuinely different ways,
# which is most of why the distinction has to be explicit.
EPS_FAULTS = frozenset({
    "assist_loss",        # 助力丢失 —— 手力矩瞬间跳到无助力水平
    "over_assist",        # 过助力 —— 转向过轻/自激
    "self_steer",         # 自转向 —— 无输入的助力输出
    "assist_oscillation", # 助力振荡 —— 阻尼失效（V1 里真实复现过）
    "torque_sensor_fault",
})
BY_WIRE_FAULTS = frozenset({
    "comms_loss",
    "angle_deviation",    # 指令角与实际角偏差超限
    "feedback_actuator_loss",  # 路感作动器失效 —— 转向仍可用但无手感
    "redundancy_switch",
})
REAR_FAULTS = frozenset({
    "rear_stuck",         # 后轮卡死（已有研究）
    "rear_authority_loss",
    "rear_centring_failure",
})
CORNER_FAULTS = frozenset({
    "single_wheel_failure",   # 单轮失效（已有研究）
})

_MECH_SENSORS = frozenset({"hand_angle", "torque_sensor", "motor_angle", "motor_current"})
_BY_WIRE_SENSORS = frozenset({
    "hand_angle", "hand_torque_sensor", "road_wheel_angle", "motor_current",
})


def _eps(
    id_: str, label: str, label_en: str, description: str,
    assist_at: AssistAt, ratio: float,
) -> Architecture:
    """A mechanically-connected, electrically-assisted front axle."""
    return Architecture(
        id=id_, label=label, label_en=label_en, description=description,
        front_path="mechanical", assist_at=assist_at, rear_axle="none",
        front_independent=False,
        sensors=_MECH_SENSORS,
        actuators=frozenset({"assist_motor"}),
        provides=frozenset({CAP_HAND_TORQUE, CAP_TORQUE_SENSOR, CAP_ASSIST_CAL}),
        failure_modes=EPS_FAULTS,
        motor_gear_ratio=ratio,
    )


#: Reduction ratios are derived, not chosen:
#:   column assist  worm gear, ~20:1 at the column, which is the pinion here
#:   rack assist    ball screw, 2*pi*i_belt/lead * r_pinion
#:                  = 2*pi*2.5/0.005 * 0.020 = 63
#: A dual-pinion drive sits between the two.
REGISTRY: tuple[Architecture, ...] = (
    _eps("c_eps", "管柱助力 C-EPS", "column-assist EPS",
         "助力电机作用在转向管柱上。成本低、布置易，助力点在扭杆之后、齿条之前，"
         "所以柱与万向节的摩擦与柔度都在路感通路上。", "column", 20.0),
    _eps("p_eps", "小齿轮助力 P-EPS", "pinion-assist EPS",
         "助力作用在小齿轮上，绕过管柱摩擦，路感优于 C-EPS。", "pinion", 30.0),
    _eps("dp_eps", "双小齿轮 DP-EPS", "dual-pinion EPS",
         "驾驶员与助力各用一个小齿轮，助力不经过驾驶员通路，"
         "可用更大助力而不牺牲手感。", "dual_pinion", 45.0),
    _eps("r_eps", "齿条助力 R-EPS", "rack-assist EPS",
         "助力经滚珠丝杠直接作用在齿条上，力容量最大，重车与高负载首选。"
         "这是本工具的默认架构。", "rack", 63.0),
    Architecture(
        id="sbw", label="线控转向 SBW", label_en="steer-by-wire",
        description="前轴无机械连接。路感由反馈作动器合成，转向由前轮作动器执行；"
                    "传动比可任意调度，但手感必须被设计出来而不是被传递过来。",
        front_path="by_wire", assist_at="none", rear_axle="none",
        front_independent=False,
        sensors=_BY_WIRE_SENSORS,
        actuators=frozenset({"road_wheel_actuator", "feedback_actuator"}),
        provides=frozenset({
            CAP_HAND_TORQUE, CAP_ANGLE_TRACKING, CAP_ROAD_FEEL, CAP_REDUNDANCY,
        }),
        failure_modes=BY_WIRE_FAULTS,
        motor_gear_ratio=63.0,
    ),
    Architecture(
        id="eps_rws", label="EPS + 后轮转向", label_en="EPS front + RWS rear",
        description="前轴机械 EPS，后轴独立作动器。前后协同的主流量产形态。",
        front_path="mechanical", assist_at="rack", rear_axle="coupled",
        front_independent=False,
        sensors=_MECH_SENSORS | frozenset({"rear_actuator_angle"}),
        actuators=frozenset({"assist_motor", "rear_actuator"}),
        provides=frozenset({
            CAP_HAND_TORQUE, CAP_TORQUE_SENSOR, CAP_ASSIST_CAL, CAP_REAR_STEER,
        }),
        failure_modes=EPS_FAULTS | REAR_FAULTS,
        motor_gear_ratio=63.0,
    ),
    Architecture(
        id="sbw_rws", label="SBW + 后轮转向", label_en="by-wire front + RWS rear",
        description="前后全线控。传动比与前后相位都可自由调度，"
                    "代价是手感完全靠合成、且冗余要求最高。",
        front_path="by_wire", assist_at="none", rear_axle="coupled",
        front_independent=False,
        sensors=_BY_WIRE_SENSORS | frozenset({"rear_actuator_angle"}),
        actuators=frozenset({
            "road_wheel_actuator", "feedback_actuator", "rear_actuator",
        }),
        provides=frozenset({
            CAP_HAND_TORQUE, CAP_ANGLE_TRACKING, CAP_ROAD_FEEL,
            CAP_REDUNDANCY, CAP_REAR_STEER,
        }),
        failure_modes=BY_WIRE_FAULTS | REAR_FAULTS,
        motor_gear_ratio=63.0,
    ),
    Architecture(
        id="4wis", label="四轮独立转向 4WIS", label_en="four-wheel independent steering",
        description="四角各自独立作动。本工具原有能力最强的形态——蟹行、零半径、"
                    "单轮失效重构都在这里；手感同样必须合成。",
        front_path="by_wire", assist_at="none", rear_axle="independent",
        front_independent=True,
        sensors=_BY_WIRE_SENSORS | frozenset({"corner_actuator_angle"}),
        actuators=frozenset({"corner_actuator", "feedback_actuator"}),
        provides=frozenset({
            CAP_HAND_TORQUE, CAP_ANGLE_TRACKING, CAP_ROAD_FEEL,
            CAP_REDUNDANCY, CAP_REAR_STEER, CAP_PER_WHEEL,
        }),
        failure_modes=BY_WIRE_FAULTS | REAR_FAULTS | CORNER_FAULTS,
        motor_gear_ratio=63.0,
    ),
)

BY_ID: dict[str, Architecture] = {a.id: a for a in REGISTRY}

#: What the tool behaves as when nothing is chosen — the mechanical/EPS front
#: axle the existing steering-feel layer effectively assumes.
DEFAULT_ID = "r_eps"


class ArchitectureError(ValueError):
    """An unknown architecture, or one asked for something it does not have."""


def get(architecture_id: str) -> Architecture:
    if architecture_id not in BY_ID:
        raise ArchitectureError(
            f"unknown architecture {architecture_id!r}; "
            f"available: {', '.join(sorted(BY_ID))}"
        )
    return BY_ID[architecture_id]


def describe_all() -> list[dict[str, object]]:
    return [a.to_dict() for a in REGISTRY]


def check_capabilities(architecture_id: str, wanted: list[str]) -> list[dict[str, str]]:
    """Report which requested capabilities this architecture cannot supply.

    Same shape as the study layer's capability refusal, including the
    `use_instead` that lets a caller repair its own request rather than guess.
    """
    arch = get(architecture_id)
    problems: list[dict[str, str]] = []
    for cap in wanted:
        if arch.can(cap):
            continue
        alternatives = sorted(a.id for a in REGISTRY if a.can(cap))
        problems.append({
            "capability": cap,
            "architecture": architecture_id,
            "why": f"{arch.label} 没有提供 {cap} 所需的传感器/作动器",
            "use_instead": ", ".join(alternatives) or "（无架构提供该能力）",
        })
    return problems


def applicable_faults(architecture_id: str, wanted: list[str]) -> list[str]:
    """Requested failure modes that make no sense on this architecture.

    Injecting loss-of-assist into a by-wire front axle is not a conservative
    test; it is a meaningless one, and quietly accepting it would put a result
    in a safety report that describes nothing.
    """
    arch = get(architecture_id)
    return [f for f in wanted if f not in arch.failure_modes]
