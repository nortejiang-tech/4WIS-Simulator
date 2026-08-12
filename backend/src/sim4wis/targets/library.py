"""The target library — shipped requirement sets, plus whatever the user adds.

Two shipped sets, and the second one is shipped *because* nothing can evaluate
it yet.

`eps_actuator` is fully checkable today from one sizing pass: it is the
actuator specification written as requirements, which is what goes to a
supplier. `steering_feel` is the ISO 13674-flavoured on-centre side, and until
the objective-test library lands (v2 V3) a compliance run against it reports
覆盖不全 and names the measurement it wanted. That is the correct state of
affairs during development and it is worth showing rather than hiding: a
requirements document exists before the test capability does, and the gap
between them is the work list.

**Provenance, honestly.** None of these numbers is measured on this vehicle.
The actuator limits are the fitted hardware's own ratings and a stated design
margin, which are exact. The effort and feel bands are industry practice for
this class of vehicle, which is a defensible starting point and not a
measurement — every entry says which it is in its `source`, and the product's
validity boundary applies here as everywhere else.

Loading: shipped sets are in code so the packaged build always has them; files
under `targets_dir()` extend the library. A file that claims a `name@version`
already shipped is refused rather than shadowing it, because a requirement set
that silently changed under a version everyone quotes is worse than an error.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from pydantic import ValidationError

from sim4wis.paths import targets_dir
from sim4wis.targets.spec import TargetError, TargetSet

_MOTOR_PEAK_NM = 8.0            # MotorParams.peak_torque
_MOTOR_CONT_NM = 4.5            # MotorParams.continuous_torque
_MOTOR_SPEED_RPM = 8000.0       # MotorParams.no_load_speed_rpm
_DESIGN_MARGIN = 0.80           # aim to use 80% of the rating

_RATING = ("所选电机铭牌值（sim4wis.steering.params.MotorParams）")
_MARGIN_WHY = ("目标值取铭牌 80%：留出公差、老化、低温摩擦增大与最恶劣工况的余量；"
               "限值即铭牌，越过就是选型不成立")


def _eps_actuator() -> TargetSet:
    return TargetSet(
        name="eps_actuator",
        version=1,
        title="EPS 作动器选型要求",
        applies_to="LS9 级 2.9 t SUV，R-EPS，默认助力标定",
        owner="系统工程",
        notes=("对着一次 sizing 走查判定。限值来自所选电机铭牌与整车电气预算，"
               "属于硬约束；手力矩目标来自同级别产品的普遍预期，属于工程判断。"),
        entries=[
            {
                "id": "motor_peak_torque",
                "metric": "required_peak_torque_nm",
                "limit": f"<= {_MOTOR_PEAK_NM}",
                "target": f"<= {_MOTOR_PEAK_NM * _DESIGN_MARGIN}",
                "unit": "N·m",
                "source": _RATING,
                "rationale": _MARGIN_WHY,
            },
            {
                "id": "motor_continuous_torque",
                "metric": "required_rms_torque_nm",
                "limit": f"<= {_MOTOR_CONT_NM}",
                "target": f"<= {_MOTOR_CONT_NM * _DESIGN_MARGIN}",
                "unit": "N·m",
                "source": _RATING,
                "rationale": ("按占空比加权的 RMS，不是各工况的平均：把泊车和"
                              "高速巡航等权平均出来的连续转矩没有意义"),
            },
            {
                "id": "motor_peak_speed",
                "metric": "required_peak_speed_rpm",
                "limit": f"<= {_MOTOR_SPEED_RPM}",
                "target": f"<= {_MOTOR_SPEED_RPM * _DESIGN_MARGIN}",
                "unit": "rpm",
                "source": _RATING,
                "rationale": ("转矩在空载转速处线性归零，所以工作转速逼近铭牌"
                              "意味着可用转矩塌掉 —— 峰值转矩合格也救不了"),
            },
            {
                "id": "electrical_power",
                "metric": "required_peak_power_w",
                "limit": "<= 600",
                "target": "<= 480",
                "unit": "W",
                "source": ("12 V 电气预算：EPS 分配 50 A 峰值（工程约束，非实测）。"
                           "指标是电机**机械轴功率**，实际电流还要除以驱动效率，"
                           "所以这条限值是电气需求的下界，不是等价换算"),
                "rationale": ("峰值功率超预算表现为电压跌落与助力抖动，不是电机烧毁；"
                              "重型车 EPS 转 48 V 通常就是被这一条推过去的"),
            },
            {
                "id": "thermal_repeat_parking",
                "metric": "peak_thermal_state",
                "at": "scenario == parking_repeat",
                "limit": "<= 0.8",
                "target": "<= 0.6",
                "unit": "—",
                "source": "热降额阈值留 20% 余量（工程判断）",
                "rationale": "连续挪车第三把才是最热的一把；单次泊车永远测不到它",
            },
            {
                "id": "parking_hand_torque",
                "metric": "peak_hand_torque_nm",
                "at": "scenario == parking_full_lock",
                "limit": "<= 5.0",
                "target": "<= 4.0",
                "unit": "N·m",
                "source": "同级别乘用车原地泊车手力矩的普遍预期区间（工程判断，非实测）",
                "rationale": "原地全锁是手力矩的最恶劣点，也是用户最常抱怨的一点",
            },
        ],
    )


def _steering_feel() -> TargetSet:
    return TargetSet(
        name="steering_feel",
        version=1,
        title="转向手感要求（中心区 / 回正）",
        applies_to="LS9 级 2.9 t SUV，任意前轴架构",
        owner="系统工程",
        notes=("**当前工具尚无法测量这些指标** —— 客观试验与指标库（v2 V3）落地后才有"
               "测量值。在此之前对它做符合性检查会报「覆盖不全」并点名缺哪个测量，"
               "这正是它现在的用处：需求先立，试验后补。"),
        entries=[
            {
                "id": "onc_torque_gradient",
                "metric": "onc_torque_gradient_nm_per_deg",
                "at": "speed_kmh == 100",
                "limit": [0.15, 0.60],
                "target": [0.25, 0.45],
                "unit": "N·m/deg",
                "source": "ISO 13674-1 weave 试验口径；带宽取同级别车的常见范围（工程判断）",
                "rationale": ("两侧都是要求：太小方向发飘、驾驶员不敢松手，"
                              "太大高速转向沉重且指向迟钝。写成单边就丢掉一半需求"),
            },
            {
                "id": "onc_torque_deadband",
                "metric": "onc_torque_deadband_deg",
                "at": "speed_kmh == 100",
                "limit": "<= 1.2",
                "target": "<= 0.8",
                "unit": "deg",
                "source": "ISO 13674-1；限值按同级别车常见上限（工程判断）",
                "rationale": "死区主要由齿条库仑摩擦决定，是中心区手感的第一成因",
            },
            {
                "id": "return_residual_angle",
                "metric": "return_residual_angle_deg",
                "at": "all",
                "limit": "abs <= 8.0",
                "target": "abs <= 4.0",
                "unit": "deg",
                "source": "回正性试验（无统一国际标准，按内部惯例）",
                "rationale": "残余角由逆效率与摩擦决定；蜗轮蜗杆架构最容易在这里失分",
            },
        ],
    )


#: Shipped sets, keyed by `name@version`.
BUILTIN: dict[str, TargetSet] = {
    s.ref: s for s in (_eps_actuator(), _steering_feel())
}


def _load_file(path: Path) -> TargetSet:
    raw: Any = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise TargetError(f"{path.name}: a target set must be a mapping")
    try:
        return TargetSet.model_validate(raw)
    except ValidationError as e:
        # Pydantic wraps the schema's own TargetError, and a stack of internal
        # validation frames is not what somebody who mistyped a YAML key needs
        # to read. Re-raise with the file named.
        reasons = "; ".join(
            str(err.get("msg", "")).removeprefix("Value error, ") for err in e.errors()
        )
        raise TargetError(f"{path.name}: {reasons}") from e


def load_all() -> dict[str, TargetSet]:
    """Shipped sets plus every readable file under `targets_dir()`."""
    out = dict(BUILTIN)
    root = targets_dir()
    if not root.is_dir():
        return out
    for path in sorted(root.glob("*.y*ml")):
        ts = _load_file(path)
        if ts.ref in BUILTIN:
            raise TargetError(
                f"{path.name} declares {ts.ref}, which is a shipped set. "
                "Bump `version` instead — a quoted requirement version must not "
                "mean two different documents."
            )
        out[ts.ref] = ts
    return out


def get(ref: str) -> TargetSet:
    """Resolve `name` (latest version) or `name@version`."""
    sets = load_all()
    if "@" in ref:
        if ref not in sets:
            raise TargetError(
                f"unknown target set {ref!r}; available: {', '.join(sorted(sets))}"
            )
        return sets[ref]
    matching = [ts for ts in sets.values() if ts.name == ref]
    if not matching:
        raise TargetError(
            f"unknown target set {ref!r}; available: "
            f"{', '.join(sorted({ts.name for ts in sets.values()})) or 'none'}"
        )
    return max(matching, key=lambda ts: ts.version)


def catalogue() -> list[dict[str, Any]]:
    """One line per set, for the CLI listing and the capability endpoint."""
    return [
        {
            "name": ts.name, "version": ts.version, "ref": ts.ref,
            "title": ts.title, "applies_to": ts.applies_to, "owner": ts.owner,
            "entries": len(ts.entries),
            "must": sum(1 for e in ts.entries if e.severity == "must"),
            "metrics": ts.metrics(),
            "builtin": ts.ref in BUILTIN,
            "digest": ts.digest(),
            "notes": ts.notes,
        }
        for ts in sorted(load_all().values(), key=lambda t: (t.name, t.version))
    ]
