"""Metric registry — what a study is allowed to measure, and how.

Two tiers, deliberately.

**Builtin** metrics are the server-side KPIs `experiment/kpi.py` already
computes for every run. A study naming one of these costs nothing extra: the
value is read straight out of the run's stored KPIs, so a study and the GUI's
analysis page can never disagree about what `yaw_gain_dps` means.

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


def describe_metrics() -> list[dict[str, Any]]:
    """Registry as data — what `describe_capabilities` will serve."""
    return [
        {
            "name": m.name,
            "unit": m.unit,
            "description": m.description,
            "requires": m.requires,
            "solvable": m.solvable,
            "source": m.source,
        }
        for m in BUILTIN.values()
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
    for n in names:
        if n in kpis and kpis[n] is not None:
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
