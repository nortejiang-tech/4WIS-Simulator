"""Metric registry — what a study is allowed to measure, and how.

Two tiers, deliberately.

**Builtin** metrics are the server-side KPIs `experiment/kpi.py` already
computes for every run. A study naming one of these costs nothing extra: the
value is read straight out of the run's stored KPIs, so a study and the GUI's
analysis page can never disagree about what `yaw_gain_dps` means.

**Procedure** metrics are the objective-test library: quantities that are only
defined for a particular manoeuvre and need the whole trace to extract, such as
the ISO 13674 on-centre numbers read off a weave's hysteresis loops. They are
neither a stored KPI (they would be computed for every run that cannot support
them) nor a one-line formula (the extraction is an analysis, not an
expression). Each one **refuses** rather than returning a plausible number when
the run cannot support it — a weave metric computed from a straight-line run
would otherwise come back as a very good on-centre result.

**Expression** metrics are written in the spec as a formula over the run's
channels. This tier is intentionally weak — no statements, no attribute
access, no imports, no lambdas — because it is the tier an agent may write
without review. It covers the large majority of derived quantities
(steady-state values, peaks, ratios, unit conversions). What it cannot express
— correlation-based phase estimation, injected-noise sensitivity, anything
needing its own simulation — belongs to the plugin tier and is out of scope
here.

Each metric declares `requires`: the model capability it needs. Nothing
enforces it yet; the envelope guard that consumes it is the next phase. The
field is declared now so the registry does not have to be revisited then.
"""

from __future__ import annotations

import ast
import math
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from sim4wis.study.spec import ExprMetric

# ---------------------------------------------------------------------------
# Capabilities (consumed by the envelope guard in a later phase)
# ---------------------------------------------------------------------------

CAP_POSE = "pose_kinematics"
CAP_ACCEL = "body_accel"
CAP_SLIP = "tyre_slip"
CAP_EFFORT = "steering_effort"
CAP_ATTITUDE = "attitude"
CAP_DRIVETRAIN = "drivetrain"
#: Needs the steering system modelled as a plant — hand torque exists only when
#: `vehicle.steering_system.enabled` is set.
CAP_PLANT = "steering_plant"


@dataclass(frozen=True)
class MetricInfo:
    name: str
    unit: str
    description: str
    requires: str = CAP_POSE
    #: Whether a solver axis may search on this metric (needs one scalar per
    #: run and monotonicity within the bracket).
    solvable: bool = False
    source: str = "builtin"


BUILTIN: dict[str, MetricInfo] = {
    m.name: m
    for m in (
        MetricInfo("icr_dev_peak_m", "m", "峰值瞬心偏差（各轮取最大）"),
        MetricInfo("icr_dev_rms_m", "m", "瞬心偏差 RMS"),
        MetricInfo("yaw_rate_peak_dps", "°/s", "峰值横摆角速度", solvable=True),
        MetricInfo("vy_peak_kmh", "km/h", "峰值侧向速度"),
        MetricInfo("speed_error_rms_kmh", "km/h", "车速跟踪 RMS 误差"),
        MetricInfo("yaw_gain_dps", "°/s", "阶跃稳态横摆增益", solvable=True),
        MetricInfo("yaw_rise_time_s", "s", "横摆 10→90% 上升时间"),
        MetricInfo("yaw_overshoot_pct", "%", "横摆超调"),
        MetricInfo("yaw_settling_time_s", "s", "横摆 5% 稳定时间"),
        MetricInfo("slip_alpha_peak_deg", "°", "峰值轮胎侧偏角", requires=CAP_SLIP),
        MetricInfo("rack_force_peak_n", "N", "峰值齿条力", requires=CAP_EFFORT),
        MetricInfo("steer_energy_nms", "N·m·s", "转向作动能量", requires=CAP_EFFORT),
    )
}


# ---------------------------------------------------------------------------
# Procedure tier — the objective-test library
# ---------------------------------------------------------------------------

#: Procedure name → the analysis it runs over a whole record.
#:
#: One entry per test, not per metric: a weave yields ten numbers from one pass
#: over the data, and running the analysis ten times would be both slow and a
#: way for two metrics of the same run to disagree.
ANALYSES: dict[str, Callable[[np.ndarray, dict[str, np.ndarray]], Any]] = {}

#: Metric name → (procedure, accessor, info).
PROCEDURE: dict[str, tuple[str, Callable[[Any], float], MetricInfo]] = {}


def _register_weave() -> None:
    from sim4wis.study import oncentre

    def analysis(t: np.ndarray, ch: dict[str, np.ndarray]) -> Any:
        def get(name: str) -> np.ndarray | None:
            v = ch.get(name)
            return None if v is None else np.asarray(v, dtype=float)

        angle = get("steer_hand_angle")
        torque = get("steer_hand_torque")
        if angle is None or torque is None:
            raise oncentre.ProcedureError(
                "本次运行没有 steer_hand_angle / steer_hand_torque 通道"
            )
        return oncentre.analyse_weave(
            t, hand_angle=angle, hand_torque=torque,
            plant_active=get("steer_plant_active"),
            ay=get("ay"), yaw_rate=get("yaw_rate"),
        )

    ANALYSES["weave"] = analysis

    deg = 180.0 / math.pi
    entries: tuple[tuple[str, str, str, Callable[[Any], float]], ...] = (
        ("onc_torque_gradient_nm_per_deg", "N·m/°",
         "中心区力矩梯度（零转角处，随试验幅值变化 —— 必须连同工况一起引用）",
         lambda w: w.torque_gradient_nm_per_rad / deg),
        ("onc_torque_gradient_nm_per_g", "N·m/g",
         "中心区力矩梯度（对侧向加速度）—— 与传动比无关，且对幅值稳健",
         lambda w: _need(w.torque_gradient_nm_per_g, "需要 ay 通道与足够的侧向加速度")),
        ("onc_torque_at_0_1g_nm", "N·m", "0.1 g 处的手力矩",
         lambda w: _need(w.torque_at_0_1g_nm, "需要 ay 通道")),
        ("onc_torque_hysteresis_nm", "N·m",
         "力矩迟滞：零转角处上下行两支的间距（摩擦感）",
         lambda w: w.torque_hysteresis_nm),
        ("onc_torque_deadband_deg", "°",
         "角度死区：零力矩处上下行两支的间距 —— 车开始回应之前能走过的角度",
         lambda w: w.angle_deadband_rad * deg),
        ("onc_angle_gradient_deg_per_g", "°/g", "转向灵敏度（方向盘转角/侧向加速度）",
         lambda w: _need(w.angle_gradient_rad_per_g, "需要 ay 通道") * deg),
        ("onc_yaw_phase_lag_deg", "°", "横摆角速度相对方向盘转角的相位滞后",
         lambda w: _need(w.yaw_phase_lag_deg, "需要 yaw_rate 通道")),
        # The achieved test condition. Reported as metrics rather than left
        # implicit because the angle-domain quantities above genuinely move
        # with amplitude — a gradient quoted without its condition is not a
        # measurement, and this is how a report is forced to carry both.
        ("onc_ay_amplitude_g", "g", "实际达到的侧向加速度幅值（试验工况）",
         lambda w: w.ay_amplitude / 9.81),
        ("onc_sw_amplitude_deg", "°", "实际达到的方向盘转角幅值（试验工况）",
         lambda w: w.angle_amplitude_rad * deg),
        ("onc_frequency_hz", "Hz", "实测扫掠频率（由过零点估计，不取自 spec）",
         lambda w: w.frequency_hz),
    )
    for name, unit, desc, getter in entries:
        PROCEDURE[name] = (
            "weave", getter,
            MetricInfo(name, unit, desc, requires=CAP_PLANT, source="procedure"),
        )


def _need(value: float | None, why: str) -> float:
    from sim4wis.study.oncentre import ProcedureError

    if value is None:
        raise ProcedureError(why)
    return float(value)


def _register_tracking_step() -> None:
    from sim4wis.study import tracking_step

    def analysis(t: np.ndarray, ch: dict[str, np.ndarray]) -> Any:
        return tracking_step.analyse_tracking_step(t, ch)

    ANALYSES["tracking_step"] = analysis
    entries: tuple[tuple[str, str, str, Callable[[Any], float]], ...] = ()
    for w in ("fl", "rl"):
        entries += (
            (f"trk_rise_s_{w}", "s", f"{w.upper()} 轮阶跃上升时间（10→90%）",
             lambda wm, _w=w: wm.get(_w).rise_s),
            (f"trk_overshoot_pct_{w}", "%", f"{w.upper()} 轮阶跃超调",
             lambda wm, _w=w: wm.get(_w).overshoot_pct),
            (f"trk_settle_s_{w}", "s", f"{w.upper()} 轮阶跃调节时间（±2% 带）",
             lambda wm, _w=w: wm.get(_w).settle_s),
            (f"trk_ss_err_rad_{w}", "rad", f"{w.upper()} 轮稳态误差",
             lambda wm, _w=w: wm.get(_w).ss_err_rad),
            (f"trk_peak_dev_rad_{w}", "rad", f"{w.upper()} 轮峰值跟踪偏差",
             lambda wm, _w=w: wm.get(_w).peak_dev_rad),
        )
    for name, unit, desc, getter in entries:
        PROCEDURE[name] = (
            "tracking_step", getter,
            MetricInfo(name, unit, desc, requires=CAP_PLANT, source="procedure"),
        )


def _register_tracking_sweep() -> None:
    from sim4wis.study import tracking_sweep

    def analysis(t: np.ndarray, ch: dict[str, np.ndarray]) -> Any:
        return tracking_sweep.analyse_tracking_sweep(t, ch)

    ANALYSES["tracking_sweep"] = analysis
    entries: tuple[tuple[str, str, str, Callable[[Any], float]], ...] = (
        ("trk_amp_ratio_0_5hz", "—", "0.5 Hz 幅值比（实际转角/指令）",
         lambda m: m.get(0.5)[0]),
        ("trk_amp_ratio_1hz", "—", "1 Hz 幅值比",
         lambda m: m.get(1.0)[0]),
        ("trk_amp_ratio_2hz", "—", "2 Hz 幅值比",
         lambda m: m.get(2.0)[0]),
        ("trk_amp_ratio_5hz", "—", "5 Hz 幅值比",
         lambda m: m.get(5.0)[0]),
        ("trk_amp_ratio_10hz", "—", "10 Hz 幅值比",
         lambda m: m.get(10.0)[0]),
        ("trk_phase_lag_2hz", "°", "2 Hz 相位滞后",
         lambda m: m.get(2.0)[1]),
        ("trk_bw_hz", "Hz", "−3 dB 跟踪带宽（对数插值；None=超出扫掠上界）",
         lambda m: _need_bw(m)),
    )
    for name, unit, desc, getter in entries:
        PROCEDURE[name] = (
            "tracking_sweep", getter,
            MetricInfo(name, unit, desc, requires=CAP_PLANT, source="procedure"),
        )


def _need_bw(m: Any) -> float:
    from sim4wis.study.tracking_sweep import ProcedureError

    if m.bandwidth_hz is None:
        raise ProcedureError(
            f"全扫掠段幅值比仍 ≥0.707 —— 带宽超出上界 {m.max_frequency_hz} Hz，"
            "只知道下界；扩频段后重测")
    return float(m.bandwidth_hz)


def _register_tracking_disturbance() -> None:
    from sim4wis.study import tracking_disturbance

    def analysis(t: np.ndarray, ch: dict[str, np.ndarray]) -> Any:
        return tracking_disturbance.analyse_tracking_disturbance(t, ch)

    ANALYSES["tracking_disturbance"] = analysis
    entries: tuple[tuple[str, str, str, Callable[[Any], float]], ...] = (
        ("trk_dist_onset_s", "s", "负载扰动检测到的时刻（齿条力最大阶跃）",
         lambda m: m.onset_s),
        ("trk_dist_peak_dev_rad", "rad", "扰动后峰值跟踪偏差（fl）",
         lambda m: m.peak_dev_rad),
        ("trk_dist_recover_s", "s", "扰动后回到 ±0.01 rad 带内的时间；"
         "未恢复为拒绝给出", lambda m: _need_recover(m)),
        ("trk_dist_rack_step_n", "N", "引发本次恢复过程的齿条力阶跃幅值",
         lambda m: m.rack_step_n),
    )
    for name, unit, desc, getter in entries:
        PROCEDURE[name] = (
            "tracking_disturbance", getter,
            MetricInfo(name, unit, desc, requires=CAP_PLANT, source="procedure"),
        )


def _need_recover(m: Any) -> float:
    from sim4wis.study.tracking_disturbance import ProcedureError

    if m.recover_s is None:
        raise ProcedureError("扰动后偏差未回到带内 —— 如实报告，不给恢复时间")
    return float(m.recover_s)


_register_weave()
_register_tracking_step()
_register_tracking_sweep()
_register_tracking_disturbance()


def describe_metrics() -> list[dict[str, Any]]:
    """Registry as data — what `describe_capabilities` will serve."""
    infos = [*BUILTIN.values(), *(info for _, _, info in PROCEDURE.values())]
    return [
        {
            "name": m.name,
            "unit": m.unit,
            "description": m.description,
            "requires": m.requires,
            "solvable": m.solvable,
            "source": m.source,
        }
        for m in infos
    ]


# ---------------------------------------------------------------------------
# Expression tier
# ---------------------------------------------------------------------------

_ALLOWED_NODES: tuple[type[ast.AST], ...] = (
    ast.Expression, ast.BinOp, ast.UnaryOp, ast.Call, ast.Name, ast.Load,
    ast.Constant, ast.Compare, ast.BoolOp, ast.IfExp, ast.Tuple, ast.List,
    ast.Add, ast.Sub, ast.Mult, ast.Div, ast.FloorDiv, ast.Mod, ast.Pow,
    ast.USub, ast.UAdd, ast.Not,
    ast.Eq, ast.NotEq, ast.Lt, ast.LtE, ast.Gt, ast.GtE,
    ast.And, ast.Or,
)


def _steady(x: np.ndarray, frac: float = 0.2) -> float:
    """Mean over the last `frac` of the record — the settled value.

    Studies ask for "the steady-state value" constantly and everyone writes a
    slightly different tail window; having one here means two metrics in the
    same table are at least measured the same way.
    """
    x = np.asarray(x, dtype=np.float64)
    if x.size == 0:
        return float("nan")
    n = max(1, int(round(x.size * min(max(frac, 1e-3), 1.0))))
    return float(np.nanmean(x[-n:]))


def _peak(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=np.float64)
    return float(np.nanmax(np.abs(x))) if x.size else float("nan")


def _rms(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=np.float64)
    return float(np.sqrt(np.nanmean(np.square(x)))) if x.size else float("nan")


_FUNCS: dict[str, Callable[..., Any]] = {
    "abs": np.abs,
    "min": np.nanmin,
    "max": np.nanmax,
    "mean": np.nanmean,
    "median": np.nanmedian,
    "sum": np.nansum,
    "sqrt": np.sqrt,
    "atan2": np.arctan2,
    "atan": np.arctan,
    "sin": np.sin,
    "cos": np.cos,
    "tan": np.tan,
    "degrees": np.degrees,
    "radians": np.radians,
    "clip": np.clip,
    "sign": np.sign,
    "first": lambda x: float(np.asarray(x, dtype=np.float64)[0]),
    "last": lambda x: float(np.asarray(x, dtype=np.float64)[-1]),
    "peak": _peak,
    "rms": _rms,
    "steady": _steady,
}

_CONSTS: dict[str, float] = {"pi": math.pi, "g": 9.80665}


class ExpressionError(ValueError):
    """A metric expression that is malformed, unsafe, or references nothing."""


def validate_expression(expr: str, channels: list[str]) -> None:
    """Raise ExpressionError unless `expr` is safe and resolvable.

    Checked without running anything, so `--dry-run` catches a bad metric
    before the grid does.
    """
    try:
        tree = ast.parse(expr, mode="eval")
    except SyntaxError as e:
        raise ExpressionError(f"{expr!r}: syntax error: {e.msg}") from e

    known = set(channels) | set(_FUNCS) | set(_CONSTS) | {"t"}
    for node in ast.walk(tree):
        if not isinstance(node, _ALLOWED_NODES):
            raise ExpressionError(
                f"{expr!r}: {type(node).__name__} is not allowed in a metric expression"
            )
        if isinstance(node, ast.Call):
            if not isinstance(node.func, ast.Name) or node.func.id not in _FUNCS:
                raise ExpressionError(
                    f"{expr!r}: only these functions are available: "
                    f"{', '.join(sorted(_FUNCS))}"
                )
            if node.keywords:
                raise ExpressionError(f"{expr!r}: keyword arguments are not supported")
        if isinstance(node, ast.Name) and node.id not in known:
            raise ExpressionError(
                f"{expr!r}: unknown name {node.id!r}. "
                "It must be a channel of the run, a helper function, or pi/g."
            )


def evaluate_expression(expr: str, t: list[float], channels: dict[str, list[float]]) -> float:
    """Evaluate a validated expression over one run's channels."""
    ns: dict[str, Any] = {}
    ns.update(_CONSTS)
    ns.update(_FUNCS)
    ns["t"] = np.asarray(t, dtype=np.float64)
    for k, v in channels.items():
        ns[k] = np.asarray(v, dtype=np.float64)

    validate_expression(expr, list(channels))
    try:
        value = eval(compile(ast.parse(expr, mode="eval"), "<metric>", "eval"),  # noqa: S307
                     {"__builtins__": {}}, ns)
    except Exception as e:                          # noqa: BLE001 - reported, not raised
        raise ExpressionError(f"{expr!r}: {type(e).__name__}: {e}") from e

    arr = np.asarray(value, dtype=np.float64)
    if arr.ndim != 0:
        raise ExpressionError(
            f"{expr!r}: produced an array of shape {arr.shape}, not a single number. "
            "Wrap it in steady(), peak(), rms(), mean() or last()."
        )
    return float(arr)


# ---------------------------------------------------------------------------
# Collection
# ---------------------------------------------------------------------------


@dataclass
class MetricValues:
    values: dict[str, float] = field(default_factory=dict)
    errors: dict[str, str] = field(default_factory=dict)


def collect(
    names: list[str],
    exprs: list[ExprMetric],
    kpis: dict[str, Any],
    t: list[float] | None = None,
    channels: dict[str, list[float]] | None = None,
) -> MetricValues:
    """Resolve one run's metrics: builtins from stored KPIs, the rest evaluated.

    A metric that cannot be produced records an error rather than a NaN — a
    NaN in a comparison table is indistinguishable from a measurement that
    genuinely came out undefined.
    """
    out = MetricValues()
    wanted_procedures = {PROCEDURE[n][0] for n in names if n in PROCEDURE}
    analyses: dict[str, Any] = {}
    analysis_errors: dict[str, str] = {}
    if wanted_procedures:
        arrays = ({k: np.asarray(v, dtype=float) for k, v in channels.items()}
                  if channels is not None else None)
        for proc in sorted(wanted_procedures):
            if t is None or arrays is None:
                analysis_errors[proc] = f"{proc} 指标需要该 run 的通道数据"
                continue
            try:
                analyses[proc] = ANALYSES[proc](np.asarray(t, dtype=float), arrays)
            except Exception as e:                   # noqa: BLE001 - recorded per metric
                analysis_errors[proc] = f"{type(e).__name__}: {e}"

    for n in names:
        if n in PROCEDURE:
            proc, getter, _ = PROCEDURE[n]
            if proc in analysis_errors:
                out.errors[n] = analysis_errors[proc]
                continue
            try:
                out.values[n] = float(getter(analyses[proc]))
            except Exception as e:                   # noqa: BLE001 - recorded per metric
                out.errors[n] = f"{type(e).__name__}: {e}"
        elif n in kpis and kpis[n] is not None:
            try:
                out.values[n] = float(kpis[n])
            except (TypeError, ValueError):
                out.errors[n] = f"KPI {n!r} is not a number: {kpis[n]!r}"
        elif n in BUILTIN:
            out.errors[n] = (
                f"builtin metric {n!r} was not produced by this run — "
                "it may need a maneuver kind this study does not use "
                "(step-response metrics need a step-kind steer segment)"
            )
        else:
            out.errors[n] = f"unknown metric {n!r}"

    if exprs:
        if t is None or channels is None:
            for m in exprs:
                out.errors[m.name] = "expression metric needs the run's channels"
        else:
            for m in exprs:
                out.values.pop(m.name, None)
                out.errors.pop(m.name, None)
                try:
                    out.values[m.name] = evaluate_expression(m.expr, t, channels)
                except ExpressionError as e:
                    out.errors[m.name] = str(e)
    return out
