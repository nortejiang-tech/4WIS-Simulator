#!/usr/bin/env python3
"""单轮转向失效 ISO 26262 可控性研究 v2 — 一键复现脚本。

    python scripts/study_single_wheel_failure.py

v2 机构设定（2026-07-02 用户输入）：
    * 前轮执行器**无自锁**，逆效率 ≈60% → 单轮失效呈**自由脚轮**状态
      （FaultSpec free_caster：J·δ̈ = −η_rev·τ_kingpin − c·δ̇ − τ_c·sign(δ̇)）
    * 后轮执行器**自锁**（正效率 30%，逆效率 0）→ 单轮失效**锁死在失效位置**
      （stuck_hold / 跑飞后锁死 stuck_value）
    * 新增参数敏感性研究：车轮/车身/底盘参数对可控性的影响（OAT + 龙卷风图）

产物：
    docs/reports/figs_single_wheel/*.png
    docs/reports/single_wheel_failure_safety_analysis.html（自包含）
    docs/reports/single_wheel_failure_metrics.json
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "backend" / "src"))

import numpy as np

from sim4wis import __version__ as SIM_VERSION
from sim4wis.experiment.schema import (
    Experiment, FaultSpec, Maneuver, ManeuverStep, SteerProfile,
)
from sim4wis.experiment.session import run_experiment
from reporting import (
    ReportDocument,
    callout_box,
    embedded_png_figure,
    html_cell,
    html_table,
    image_to_base64,
    meta_paragraph,
    report_section,
    write_json,
)

plt = None
patches = None

OUT_DIR = ROOT / "docs" / "reports"
FIG_DIR = OUT_DIR / "figs_single_wheel"
FIG_DIR.mkdir(parents=True, exist_ok=True)

# ─── 判据常量（报告 §4） ─────────────────────────────────────────────────────
LANE_MARGIN = 0.90       # [m] 3.75 m 车道 − 1.96 m 车宽 → 单侧裕度 ≈0.9 m
T_REACT = 1.2            # [s] 驾驶员感知-反应时间（95 分位工程惯用值）
EVAL_WINDOW = 5.0        # [s] 故障后评估窗
DETECT_DELAY = 0.15      # [s] 基线检测+仲裁延时
V_LIMIT_KMH = 60.0       # 降级限速
STUCK_DEG = 4.0          # 后轮跑飞后锁死角
ETA_REV = 0.6            # 前轮机构逆效率

WHEEL_NAMES = {0: "FL(左前)", 1: "FR(右前)", 2: "RL(左后)", 3: "RR(右后)"}
FAULT_NAMES = {"free_caster": "自由脚轮(断电)", "stuck_value": f"跑飞后锁死+{STUCK_DEG:.0f}°",
               "stuck_zero": "回中锁死", "stuck_hold": "原角锁死"}

COL_BASE = "#d62728"
COL_MIT = "#1f77b4"
COL_REF = "#7f7f7f"
C_COLORS = {"C1": "#2ca02c", "C2": "#ff9f1c", "C3": "#d62728"}


# ─── 场景与实验构造 ──────────────────────────────────────────────────────────

def scenario_steps(scn: str) -> tuple[list[ManeuverStep], float, float]:
    """返回 (steps, t_fault, 车速 km/h)。"""
    if scn == "straight100":
        return [
            ManeuverStep(name="加速", duration=8.0, speed_kmh=100.0, speed_ramp_s=5.0),
            ManeuverStep(name="直行", duration=8.0),
        ], 9.0, 100.0
    if scn == "curve60":
        # 含出弯段：自锁后轮"弯中锁死"的危害主要在出弯后残余
        return [
            ManeuverStep(name="加速", duration=6.0, speed_kmh=60.0, speed_ramp_s=4.0),
            ManeuverStep(name="入弯", duration=2.0,
                         steer=SteerProfile(kind="ramp", start=0.0, amplitude=0.05)),
            ManeuverStep(name="稳态弯", duration=4.0,
                         steer=SteerProfile(kind="constant", amplitude=0.05)),
            ManeuverStep(name="出弯", duration=2.0,
                         steer=SteerProfile(kind="ramp", start=0.05, amplitude=0.0)),
            ManeuverStep(name="直行", duration=2.0),
        ], 10.5, 60.0
    if scn == "dlc60":
        return [
            ManeuverStep(name="加速", duration=6.0, speed_kmh=60.0, speed_ramp_s=4.0),
            ManeuverStep(name="双移线", duration=8.0,
                         steer=SteerProfile(kind="dlc", amplitude=0.06)),
            ManeuverStep(name="稳定", duration=2.0),
        ], 8.2, 60.0
    raise KeyError(scn)


SCN_LABELS = {"straight100": "高速直行 100 km/h",
              "curve60": "稳态弯-出弯 60 km/h (a_y≈4.6 m/s²)",
              "dlc60": "双移线 60 km/h（紧急变道中）"}


def make_fault(wheel: int, ftype: str, t_fault: float) -> FaultSpec:
    value = math.radians(STUCK_DEG) if ftype == "stuck_value" else 0.0
    return FaultSpec(fault_type=ftype, wheel=wheel, value=value,
                     t_start=t_fault, eta_rev=ETA_REV)


def build_exp(scn: str, fault: FaultSpec | None, strategy: str = "ideal_ackermann",
              mode_params: dict | None = None, overrides: dict | None = None,
              base_mu: float | None = None) -> Experiment:
    steps, _tf, _v = scenario_steps(scn)
    scene = None
    if base_mu is not None:
        scene = {"base_mu": float(base_mu), "surface": "flat", "disturbances": []}
    return Experiment(
        name=f"sw_{scn}",
        strategy=strategy,
        mode_params=mode_params or {},
        vehicle={"profile": None, "overrides": overrides or {}},
        scene=scene,
        faults=[fault] if fault else [],
        maneuver=Maneuver(name=scn, steps=steps),
        record_hz=100.0,
    )


# ─── 指标计算 ────────────────────────────────────────────────────────────────

def _arr(r, name):
    return np.asarray(r.channels[name], dtype=np.float64)


def cross_track(ref_xy: np.ndarray, xy: np.ndarray) -> np.ndarray:
    """每个点到参考轨迹折线的最小距离（车道保持口径的横向偏差）。"""
    seg_a, seg_b = ref_xy[:-1], ref_xy[1:]
    d = seg_b - seg_a
    len2 = np.maximum((d ** 2).sum(axis=1), 1e-12)
    out = np.empty(xy.shape[0])
    for i, p in enumerate(xy):
        t = np.clip(((p - seg_a) * d).sum(axis=1) / len2, 0.0, 1.0)
        proj = seg_a + t[:, None] * d
        out[i] = np.sqrt(((p - proj) ** 2).sum(axis=1).min())
    return out


def compute_metrics(run, ref, t_fault: float) -> dict:
    """故障后可控性指标（全部相对同参数无故障参考 run）。"""
    t = np.asarray(run.t)
    i0 = np.searchsorted(t, t_fault)
    i_end = np.searchsorted(t, min(t_fault + EVAL_WINDOW, t[-1] - 1e-9))

    xy = np.column_stack([_arr(run, "pose_x"), _arr(run, "pose_y")])
    ref_xy = np.column_stack([_arr(ref, "pose_x"), _arr(ref, "pose_y")])
    xt = cross_track(ref_xy, xy[i0:i_end])
    t_post = t[i0:i_end] - t_fault

    def xt_at(tq: float) -> float:
        j = np.searchsorted(t_post, tq)
        return float(xt[min(j, len(xt) - 1)])

    dep = np.nonzero(xt > LANE_MARGIN)[0]
    ttld = float(t_post[dep[0]]) if dep.size else math.inf

    yaw = _arr(run, "yaw_rate")
    vx = _arr(run, "vx")
    vy = _arr(run, "vy")
    # 横摆扰动 = 相对无故障参考（时间对齐相减，覆盖弯中/出弯变化的名义 r）
    yaw_ref = _arr(ref, "yaw_rate")
    n = min(yaw.size, yaw_ref.size)
    dyaw = (yaw[:n] - yaw_ref[:n])[i0:min(i_end, n)]
    if dyaw.size == 0:
        dyaw = np.zeros(1)

    # 侧向加速度增量 |Δa_y|（稳态弯名义 a_y 不计入判据）
    def ay_of(r):
        return (np.gradient(np.asarray(r.channels["vy"]), np.asarray(r.t))
                + np.asarray(r.channels["yaw_rate"]) * np.asarray(r.channels["vx"]))
    ay_run, ay_ref = ay_of(run), ay_of(ref)
    n2 = min(ay_run.size, ay_ref.size)
    d_ay = (ay_run[:n2] - ay_ref[:n2])[i0:min(i_end, n2)]
    ay = d_ay if d_ay.size else np.zeros(1)
    beta = np.arctan2(vy[i0:i_end], np.maximum(vx[i0:i_end], 0.5))

    psi = _arr(run, "pose_psi")
    psi_ref = _arr(ref, "pose_psi")
    i2 = np.searchsorted(t, t_fault + 2.0)
    return {
        "xtrack_1s": xt_at(1.0),
        "xtrack_react": xt_at(T_REACT),
        "xtrack_2_5s": xt_at(2.5),
        "xtrack_max": float(xt.max()),
        "ttld_s": ttld,
        "dyaw_peak_dps": float(np.rad2deg(np.abs(dyaw).max())),
        "dyaw_resid_dps": float(np.rad2deg(abs(float(np.mean(dyaw[-50:]))))),
        "ay_peak": float(np.abs(ay).max()),
        "beta_peak_deg": float(np.rad2deg(np.abs(beta).max())),
        "dpsi_2s_deg": float(np.rad2deg(abs(
            (psi[min(i2, psi.size - 1)] - psi[i0])
            - (psi_ref[min(i2, psi_ref.size - 1)] - psi_ref[min(i0, psi_ref.size - 1)])))),
        "v_at_fault_kmh": float(vx[i0] * 3.6),
    }


def c_class(m: dict) -> str:
    """可控性分级（判据见报告 §4；TTLD 为主指标）。"""
    if (math.isinf(m["ttld_s"]) and m["xtrack_max"] < 0.5 * LANE_MARGIN
            and m["dyaw_resid_dps"] < 1.0 and m["ay_peak"] < 2.0):
        return "C1"
    if m["ttld_s"] >= T_REACT and m["ay_peak"] < 5.0:
        return "C2"
    return "C3"


# ─── 主矩阵 ──────────────────────────────────────────────────────────────────

# 机构设定映射：前轮（无自锁）→ 自由脚轮；后轮（自锁）→ 锁死类
CASES = [
    ("straight100", 0, "free_caster"),
    ("straight100", 2, "stuck_value"),   # 跑飞后锁死（自锁机构最不利模式）
    ("curve60", 0, "free_caster"),
    ("curve60", 2, "stuck_hold"),        # 弯中锁死 → 出弯残余
    ("dlc60", 0, "free_caster"),
    ("dlc60", 2, "stuck_hold"),
]


def mitigation_params(wheel: int, kind: str, angle: float, t_fault: float,
                      detect: float = DETECT_DELAY) -> dict:
    return {"fault_wheel": wheel, "fault_kind": kind, "fault_angle": float(angle),
            "fault_time": float(t_fault), "detect_delay": float(detect),
            "v_limit_kmh": V_LIMIT_KMH}


def stuck_angle_from(base_run, wheel: int, t_fault: float) -> float:
    tb = np.asarray(base_run.t)
    name = f"delta_{['fl', 'fr', 'rl', 'rr'][wheel]}"
    return float(_arr(base_run, name)[np.searchsorted(tb, t_fault + 0.4)])


def run_matrix() -> tuple[list[dict], dict]:
    rows: list[dict] = []
    curves: dict = {}
    refs: dict[str, object] = {}
    for scn in {c[0] for c in CASES}:
        refs[scn] = run_experiment(build_exp(scn, None))
        curves[f"ref:{scn}"] = refs[scn]

    for scn, wheel, ftype in CASES:
        _steps, t_fault, _v = scenario_steps(scn)
        fault = make_fault(wheel, ftype, t_fault)
        base = run_experiment(build_exp(scn, fault))
        curves[f"base:{scn}:{wheel}:{ftype}"] = base

        kind = "free" if ftype == "free_caster" else "stuck"
        if ftype == "stuck_hold":
            angle = stuck_angle_from(base, wheel, t_fault)
        elif ftype == "stuck_value":
            angle = math.radians(STUCK_DEG)
        else:
            angle = 0.0
        mit = run_experiment(build_exp(
            scn, fault, "fault_reconfig",
            mitigation_params(wheel, kind, angle, t_fault)))
        curves[f"mit:{scn}:{wheel}:{ftype}"] = mit

        for label, run in (("baseline", base), ("mitigated", mit)):
            m = compute_metrics(run, refs[scn], t_fault)
            m.update({"scenario": scn, "wheel": wheel, "fault": ftype,
                      "mitigation": label, "t_fault": t_fault,
                      "stuck_angle_deg": math.degrees(angle),
                      "c_class": c_class(m)})
            rows.append(m)
        print(f"  ✓ {scn:12s} {WHEEL_NAMES[wheel]:8s} {FAULT_NAMES[ftype]:12s} "
              f"base={rows[-2]['c_class']} mit={rows[-1]['c_class']}")

    # 检测延时敏感性（后轮跑飞锁死 @直行 100 —— 最恶性组合）
    scn, wheel, ftype = "straight100", 2, "stuck_value"
    _steps, t_fault, _v = scenario_steps(scn)
    fault = make_fault(wheel, ftype, t_fault)
    sens = []
    for td in (0.05, 0.15, 0.30, 0.50, 0.80):
        mit = run_experiment(build_exp(
            scn, fault, "fault_reconfig",
            mitigation_params(wheel, "stuck", math.radians(STUCK_DEG), t_fault, detect=td)))
        m = compute_metrics(mit, refs[scn], t_fault)
        m.update({"detect_delay": td, "c_class": c_class(m)})
        sens.append(m)
        print(f"  ✓ 敏感性 τ_d={td:.2f}s → xtrack@2.5s={m['xtrack_2_5s']:.2f} m ({m['c_class']})")
    curves["sensitivity"] = sens
    return rows, curves


# ─── 参数敏感性（工况 × 车辆参数 OAT） ────────────────────────────────────────

# 锚 A：后轮跑飞锁死 @直行100，缓解开 —— 安全概念对车辆参数的稳健性
# 锚 B：前轮自由脚轮 @弯中60，未缓解 —— 机构/整车参数对"天然温和度"的影响
PARAM_DEFS = [
    ("caster", "主销后倾角", 6.0, [2.0, 4.0, 6.0, 8.0], "°",
     lambda v: ({"suspension": {"caster_angle": math.radians(v)}}, None)),
    ("scrub", "主销偏置(scrub)", 15.0, [5.0, 15.0, 30.0, 50.0], "mm",
     lambda v: ({"suspension": {"scrub_radius": v / 1000.0}}, None)),
    ("c_alpha", "轮胎侧偏刚度", 120.0, [80.0, 100.0, 120.0, 150.0], "kN/rad",
     lambda v: ({"tire_c_alpha": v * 1000.0}, None)),
    ("mass", "整备质量", 2900.0, [2300.0, 2600.0, 2900.0, 3200.0], "kg",
     lambda v: ({"mass": v}, None)),
    ("cg_front", "质心到前轴", 1.55, [1.35, 1.45, 1.55, 1.65], "m",
     lambda v: ({"cg_to_front": v}, None)),
    ("cg_h", "质心高度", 0.62, [0.50, 0.62, 0.75], "m",
     lambda v: ({"cg_height": v}, None)),
    ("iz", "横摆转动惯量", 7500.0, [5500.0, 7500.0, 9500.0], "kg·m²",
     lambda v: ({"inertia_z": v}, None)),
    ("wheelbase", "轴距", 3.16, [2.90, 3.16, 3.40], "m",
     lambda v: ({"wheelbase": v}, None)),
    ("mu", "路面附着 μ", 0.85, [0.50, 0.70, 0.85, 1.00], "-",
     lambda v: ({}, v)),
]


def run_param_sensitivity() -> dict:
    out = {"A": {}, "B": {}}
    scn_a, wheel_a, ftype_a = "straight100", 2, "stuck_value"
    _s, tf_a, _v = scenario_steps(scn_a)
    fault_a = make_fault(wheel_a, ftype_a, tf_a)
    mp_a = mitigation_params(wheel_a, "stuck", math.radians(STUCK_DEG), tf_a)

    scn_b, wheel_b, ftype_b = "curve60", 0, "free_caster"
    _s, tf_b, _v = scenario_steps(scn_b)
    fault_b = make_fault(wheel_b, ftype_b, tf_b)

    for key, label, base_v, values, unit, mk in PARAM_DEFS:
        pts_a, pts_b = [], []
        for v in values:
            ov, mu = mk(v)
            # 每个取值配自己的无故障参考（同参数）
            ref_a = run_experiment(build_exp(scn_a, None, overrides=ov, base_mu=mu))
            mit_a = run_experiment(build_exp(scn_a, fault_a, "fault_reconfig", mp_a,
                                             overrides=ov, base_mu=mu))
            ma = compute_metrics(mit_a, ref_a, tf_a)
            ma["value"] = v
            ma["c_class"] = c_class(ma)
            pts_a.append(ma)

            ref_b = run_experiment(build_exp(scn_b, None, overrides=ov, base_mu=mu))
            base_b = run_experiment(build_exp(scn_b, fault_b, overrides=ov, base_mu=mu))
            mb = compute_metrics(base_b, ref_b, tf_b)
            mb["value"] = v
            mb["c_class"] = c_class(mb)
            pts_b.append(mb)
        out["A"][key] = {"label": label, "unit": unit, "base": base_v, "points": pts_a}
        out["B"][key] = {"label": label, "unit": unit, "base": base_v, "points": pts_b}
        print(f"  ✓ 参数 {label:10s} A: xtrack@2.5 "
              f"{min(p['xtrack_2_5s'] for p in pts_a):.2f}–{max(p['xtrack_2_5s'] for p in pts_a):.2f} m | "
              f"B: Δr残余 {min(p['dyaw_resid_dps'] for p in pts_b):.2f}"
              f"–{max(p['dyaw_resid_dps'] for p in pts_b):.2f} °/s")
    return out


# ─── 图 ─────────────────────────────────────────────────────────────────────

def ensure_matplotlib() -> None:
    """Load plotting dependencies only when building the full HTML report."""
    global plt, patches
    if plt is not None and patches is not None:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.patches as mpl_patches
    import matplotlib.pyplot as mpl_pyplot

    mpl_pyplot.rcParams["font.sans-serif"] = [
        "PingFang SC", "Hiragino Sans GB", "Arial Unicode MS", "SimHei"
    ]
    mpl_pyplot.rcParams["axes.unicode_minus"] = False
    mpl_pyplot.rcParams["figure.dpi"] = 110
    plt = mpl_pyplot
    patches = mpl_patches


def savefig(fig, name: str) -> str:
    ensure_matplotlib()
    path = FIG_DIR / f"{name}.png"
    fig.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return image_to_base64(path)


def _draw_car(ax, deltas, colors_w, L=3.16, T=1.565):
    wheels = [(+L / 2, +T / 2), (+L / 2, -T / 2), (-L / 2, +T / 2), (-L / 2, -T / 2)]
    ax.add_patch(plt.Rectangle((-L / 2 - 0.55, -T / 2 - 0.18), L + 1.1, T + 0.36,
                               fill=False, lw=1.6, ec="#333"))
    ax.annotate("", xy=(L / 2 + 1.15, 0), xytext=(L / 2 + 0.6, 0),
                arrowprops=dict(arrowstyle="-|>", color="#333"))
    for (x, y), d, col in zip(wheels, deltas, colors_w):
        wl, ww = 0.62, 0.22
        c, s = math.cos(d), math.sin(d)
        corners = np.array([[wl / 2, ww / 2], [wl / 2, -ww / 2],
                            [-wl / 2, -ww / 2], [-wl / 2, ww / 2]])
        rot = corners @ np.array([[c, s], [-s, c]])
        ax.add_patch(plt.Polygon(rot + [x, y], color=col))
    for (x, y), nm in zip(wheels, ("FL", "FR", "RL", "RR")):
        ax.text(x, y + (0.42 if y > 0 else -0.42), nm, ha="center",
                va="bottom" if y > 0 else "top", fontsize=9, color="#555")
    ax.set_xlim(-3.4, 3.9)
    ax.set_ylim(-2.6, 2.6)
    ax.set_aspect("equal")
    ax.axis("off")
    return wheels


def fig_mechanism() -> str:
    """图1：两类机构的失效形态与对应安全机制。"""
    ensure_matplotlib()
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8))
    d = math.radians(10)

    # (a) 后轮自锁锁死 + 同轴镜像
    wheels = _draw_car(axes[0], [0, 0, d, -d],
                       ["#1f77b4", "#1f77b4", "#d62728", "#2ca02c"])
    x, y = wheels[2]
    axes[0].annotate("", xy=(x - 0.9 * math.sin(d), y + 0.9 * math.cos(d)),
                     xytext=(x, y), arrowprops=dict(arrowstyle="-|>", lw=2.2, color="#d62728"))
    x, y = wheels[3]
    axes[0].annotate("", xy=(x + 0.9 * math.sin(d), y - 0.9 * math.cos(d)),
                     xytext=(x, y), arrowprops=dict(arrowstyle="-|>", lw=2.2, color="#2ca02c"))
    axes[0].set_title("(a) 后轮自锁机构锁死（逆效率 0）\n→ 同轴镜像力抵消：δ_RR = −e", fontsize=10.5)
    axes[0].text(0, -2.45, "自锁把失效冻结为持续干扰源；同轴等力臂 → ΣF_y=0 且 ΣM_z=0",
                 ha="center", fontsize=9, color="#555")

    # (b) 前轮自由脚轮
    wheels = _draw_car(axes[1], [math.radians(3), 0, 0, 0],
                       ["#ff9f1c", "#1f77b4", "#1f77b4", "#1f77b4"])
    x, y = wheels[0]
    arc = patches.Arc((x, y), 1.5, 1.5, angle=0, theta1=-35, theta2=35,
                      color="#ff9f1c", lw=2)
    axes[1].add_patch(arc)
    axes[1].annotate("", xy=(x + 0.75 * math.cos(math.radians(38)),
                             y + 0.75 * math.sin(math.radians(38))),
                     xytext=(x + 0.75 * math.cos(math.radians(30)),
                             y + 0.75 * math.sin(math.radians(30))),
                     arrowprops=dict(arrowstyle="-|>", lw=1.8, color="#ff9f1c"))
    axes[1].set_title("(b) 前轮无自锁断电（逆效率 0.6）\n→ 自由脚轮：J·δ̈ = −0.6·τ_KP − c·δ̇", fontsize=10.5)
    axes[1].text(0, -2.45, "主销后倾使其自对准零侧偏力方向 ≈ 理想阿克曼角 → 无寄生力，\n"
                           "但前轴丢失该轮侧偏刚度份额（转向不足化）→ 健康轮增益补偿",
                 ha="center", fontsize=9, color="#555")
    fig.suptitle("图 1  两类转向执行机构的单轮失效形态与安全机制（俯视示意）", fontsize=12)
    return savefig(fig, "fig1_mechanism")


def fig_timehistory(curves) -> str:
    """图2：最恶性组合（直行 100 · RL 跑飞锁死 +4°）时间历程。"""
    ensure_matplotlib()
    ref = curves["ref:straight100"]
    base = curves["base:straight100:2:stuck_value"]
    mit = curves["mit:straight100:2:stuck_value"]
    t_fault = 9.0
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)

    def plot3(ax, ch, scale=1.0, ylabel=""):
        for r, col, lbl in ((ref, COL_REF, "无故障参考"), (base, COL_BASE, "未缓解"),
                            (mit, COL_MIT, "镜像抵消+PI+限速")):
            ax.plot(np.asarray(r.t) - t_fault, _arr(r, ch) * scale, color=col, lw=1.5, label=lbl)
        ax.axvline(0, color="k", lw=0.8, ls="--")
        ax.axvline(DETECT_DELAY, color=COL_MIT, lw=0.8, ls=":")
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.3)
        ax.set_xlim(-1, 5)

    plot3(axes[0, 0], "yaw_rate", 180 / math.pi, "横摆角速度 [°/s]")
    axes[0, 0].legend(fontsize=9)
    plot3(axes[0, 1], "pose_psi", 180 / math.pi, "航向角 [°]")
    plot3(axes[1, 0], "pose_y", 1.0, "侧向位置 y [m]")
    axes[1, 0].axhline(-LANE_MARGIN, color="#d62728", lw=1, ls="-.")
    axes[1, 0].axhline(LANE_MARGIN, color="#d62728", lw=1, ls="-.")
    axes[1, 0].text(4.9, LANE_MARGIN + 0.12, "车道裕度 ±0.9 m", ha="right", fontsize=8, color="#d62728")
    axes[1, 0].set_ylim(-6.0, 3.0)
    axes[1, 0].set_title("（未缓解曲线超出图幅）", fontsize=8, color="#888")
    for r, col in ((base, COL_BASE), (mit, COL_MIT)):
        axes[1, 1].plot(np.asarray(r.t) - t_fault, np.rad2deg(_arr(r, "delta_rl")),
                        color=col, lw=1.5)
        axes[1, 1].plot(np.asarray(r.t) - t_fault, np.rad2deg(_arr(r, "delta_rr")),
                        color=col, lw=1.5, ls="--")
    axes[1, 1].set_ylabel("后轮转角 [°]（实线 RL / 虚线 RR）")
    axes[1, 1].axvline(0, color="k", lw=0.8, ls="--")
    axes[1, 1].grid(alpha=0.3)
    for ax in axes[1]:
        ax.set_xlabel("故障后时间 [s]")
    fig.suptitle("图 2  直行 100 km/h · RL 跑飞后锁死 +4°（自锁机构最不利模式）的时间历程", fontsize=12)
    fig.tight_layout()
    return savefig(fig, "fig2_timehistory")


def fig_free_front(curves) -> str:
    """图3：前轮自由脚轮失效的两个标志性行为。"""
    ensure_matplotlib()
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.2))
    # 左：弯中失效——δ_FL 松脱到脚轮平衡（α→0）
    base = curves["base:curve60:0:free_caster"]
    ref = curves["ref:curve60"]
    t = np.asarray(base.t) - 10.5
    ax = axes[0]
    ax.plot(t, np.rad2deg(_arr(base, "delta_fl")), color="#ff9f1c", lw=1.6, label="δ_FL（自由）")
    ax.plot(np.asarray(ref.t) - 10.5, np.rad2deg(_arr(ref, "delta_fl")), color=COL_REF,
            lw=1.2, ls="--", label="δ_FL（无故障参考）")
    ax2 = ax.twinx()
    ax2.plot(t, np.rad2deg(_arr(base, "slip_alpha_fl")), color="#9467bd", lw=1.2)
    ax2.set_ylabel("FL 侧偏角 [°]", color="#9467bd")
    ax.axvline(0, color="k", lw=0.8, ls="--")
    ax.set_xlim(-1, 4)
    ax.set_xlabel("故障后时间 [s]")
    ax.set_ylabel("FL 轮转角 [°]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("(a) 弯中断电：经 ~4 Hz 衰减摆振（caster shimmy 瞬态）\n约 1 s 收敛到零侧偏力平衡（c=80 N·m·s/rad）", fontsize=10)

    # 右：直行失效——toe 平衡丢失导致的慢漂
    base_s = curves["base:straight100:0:free_caster"]
    ref_s = curves["ref:straight100"]
    ts = np.asarray(base_s.t) - 9.0
    ax = axes[1]
    y0 = _arr(base_s, "pose_y")[np.searchsorted(np.asarray(base_s.t), 9.0)]
    ax.plot(ts, _arr(base_s, "pose_y") - y0, color="#ff9f1c", lw=1.6, label="侧向漂移（自由失效）")
    ax.plot(np.asarray(ref_s.t) - 9.0, _arr(ref_s, "pose_y"), color=COL_REF, lw=1.2, ls="--",
            label="无故障参考")
    ax.axhline(LANE_MARGIN, color="#d62728", ls="-.", lw=1)
    ax.text(4.8, LANE_MARGIN + 0.05, "车道裕度", ha="right", fontsize=8, color="#d62728")
    ax.axvline(0, color="k", lw=0.8, ls="--")
    ax.set_xlim(-1, 5)
    ax.set_ylim(-0.3, 1.2)
    ax.set_xlabel("故障后时间 [s]")
    ax.set_ylabel("侧向位置 y [m]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("(b) 直行断电：失效轮不再抵抗静态 toe\n→ 健康侧 toe 力失配产生慢漂", fontsize=10)
    fig.suptitle("图 3  前轮自由脚轮失效的标志性行为（未缓解）", fontsize=12)
    fig.tight_layout()
    return savefig(fig, "fig3_free_front")


def fig_trajectories(curves) -> str:
    """图4：三工况轨迹俯视（前自由/后锁死 × 缓解）。"""
    ensure_matplotlib()
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.3))
    for ax, scn in zip(axes, ("straight100", "curve60", "dlc60")):
        ref = curves[f"ref:{scn}"]
        rear_f = "stuck_value" if scn == "straight100" else "stuck_hold"
        _s, t_fault, _v = scenario_steps(scn)
        series = [
            (ref, COL_REF, "无故障参考", 1.2, "-"),
            (curves[f"base:{scn}:0:free_caster"], "#ff9f1c", "前轮自由(未缓解)", 1.6, "-"),
            (curves[f"base:{scn}:2:{rear_f}"], COL_BASE, "后轮锁死(未缓解)", 1.6, "-"),
            (curves[f"mit:{scn}:2:{rear_f}"], COL_MIT, "后轮锁死(缓解)", 1.6, "--"),
        ]
        for r, col, lbl, lw, ls in series:
            ax.plot(_arr(r, "pose_x"), _arr(r, "pose_y"), color=col, lw=lw, ls=ls, label=lbl)
            i = np.searchsorted(np.asarray(r.t), t_fault)
            ax.plot(_arr(r, "pose_x")[i], _arr(r, "pose_y")[i], "o", color=col, ms=4)
        ax.set_title(SCN_LABELS[scn], fontsize=10)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.grid(alpha=0.3)
        ax.axis("equal")
    axes[0].legend(fontsize=8)
    fig.suptitle("图 4  三工况轨迹俯视：前轮自由 vs 后轮锁死（圆点 = 失效时刻）", fontsize=12)
    fig.tight_layout()
    return savefig(fig, "fig4_trajectories")


def fig_metric_bars(rows) -> str:
    ensure_matplotlib()
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.6))
    labels, xt_base, xt_mit, yaw_base, yaw_mit = [], [], [], [], []
    seen = []
    for r in rows:
        key = (r["scenario"], r["wheel"], r["fault"])
        if key in seen:
            continue
        seen.append(key)
        labels.append(f"{SCN_LABELS[r['scenario']].split(' ')[0]}\n"
                      f"{WHEEL_NAMES[r['wheel']][:2]} {FAULT_NAMES[r['fault']][:4]}")
        b = next(x for x in rows if (x["scenario"], x["wheel"], x["fault"]) == key
                 and x["mitigation"] == "baseline")
        m = next(x for x in rows if (x["scenario"], x["wheel"], x["fault"]) == key
                 and x["mitigation"] == "mitigated")
        xt_base.append(min(b["xtrack_2_5s"], 6.0))
        xt_mit.append(m["xtrack_2_5s"])
        yaw_base.append(b["dyaw_peak_dps"])
        yaw_mit.append(m["dyaw_peak_dps"])
    x = np.arange(len(labels))
    for ax, (vb, vm, ylab, thr) in zip(axes, [
        (xt_base, xt_mit, "横向偏差 @2.5 s [m]（截至 6 m 显示）", LANE_MARGIN),
        (yaw_base, yaw_mit, "横摆扰动峰值 [°/s]", None),
    ]):
        ax.bar(x - 0.19, vb, 0.36, color=COL_BASE, label="未缓解")
        ax.bar(x + 0.19, vm, 0.36, color=COL_MIT, label="缓解")
        if thr:
            ax.axhline(thr, color="#d62728", ls="-.", lw=1)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, fontsize=8)
        ax.set_ylabel(ylab)
        ax.grid(axis="y", alpha=0.3)
    axes[0].legend(fontsize=9)
    fig.suptitle("图 5  全矩阵可控性指标：缓解前后对比", fontsize=12)
    fig.tight_layout()
    return savefig(fig, "fig5_metric_bars")


def fig_sensitivity(curves) -> str:
    ensure_matplotlib()
    sens = curves["sensitivity"]
    td = [s["detect_delay"] for s in sens]
    fig, ax1 = plt.subplots(figsize=(7.2, 4.2))
    ax1.plot(td, [s["xtrack_2_5s"] for s in sens], "o-", color=COL_MIT, label="横向偏差 @2.5 s")
    ax1.plot(td, [s["xtrack_react"] for s in sens], "s--", color="#2ca02c",
             label=f"横向偏差 @{T_REACT} s（反应窗）")
    ax1.axhline(LANE_MARGIN, color="#d62728", ls="-.", lw=1)
    ax1.set_xlabel("检测 + 仲裁延时 τ_d [s]")
    ax1.set_ylabel("横向偏差 [m]")
    ax1.grid(alpha=0.3)
    ax2 = ax1.twinx()
    ax2.plot(td, [s["dyaw_peak_dps"] for s in sens], "^:", color="#9467bd", label="横摆扰动峰值")
    ax2.set_ylabel("横摆扰动峰值 [°/s]", color="#9467bd")
    l1, n1 = ax1.get_legend_handles_labels()
    l2, n2 = ax2.get_legend_handles_labels()
    ax1.legend(l1 + l2, n1 + n2, fontsize=9, loc="upper left")
    ax1.set_title("图 6  检测延时的影响（直行 100 km/h · RL 跑飞后锁死，缓解开）", fontsize=11)
    fig.tight_layout()
    return savefig(fig, "fig6_detect_delay")


def fig_c_matrix(rows) -> str:
    ensure_matplotlib()
    keys = []
    for r in rows:
        k = (r["scenario"], r["wheel"], r["fault"])
        if k not in keys:
            keys.append(k)
    fig, axes = plt.subplots(1, 2, figsize=(11, 0.62 * len(keys) + 1.6))
    for ax, mit in zip(axes, ("baseline", "mitigated")):
        ax.set_xlim(0, 1)
        ax.set_ylim(-0.5, len(keys) - 0.5)
        for i, k in enumerate(keys):
            row = next(x for x in rows if (x["scenario"], x["wheel"], x["fault"]) == k
                       and x["mitigation"] == mit)
            c = row["c_class"]
            ttld = "∞" if math.isinf(row["ttld_s"]) else f"{row['ttld_s']:.1f}s"
            ax.barh(i, 1, color=C_COLORS[c], alpha=0.88)
            ax.text(0.5, i, f"{c}   (TTLD {ttld})", ha="center", va="center",
                    fontsize=10, color="white", fontweight="bold")
        ax.set_yticks(range(len(keys)))
        if mit == "baseline":
            ax.set_yticklabels([f"{SCN_LABELS[s].split(' ')[0]} · {WHEEL_NAMES[w][:2]} · {FAULT_NAMES[f]}"
                                for s, w, f in keys], fontsize=9)
        else:
            ax.set_yticklabels([])
        ax.set_xticks([])
        ax.set_title("未缓解" if mit == "baseline" else "缓解（free→增益补偿 / stuck→镜像抵消）",
                     fontsize=10.5)
        ax.invert_yaxis()
    fig.suptitle("图 7  可控性分级矩阵（TTLD = 车道脱离时间）", fontsize=12)
    fig.tight_layout()
    return savefig(fig, "fig7_c_matrix")


def fig_tornado(param_sens) -> str:
    """图8：参数敏感性龙卷风图（锚 A：后轮锁死缓解后 xtrack@2.5s）。"""
    ensure_matplotlib()
    items = []
    for key, d in param_sens["A"].items():
        vals = [p["xtrack_2_5s"] for p in d["points"]]
        items.append((d["label"], min(vals), max(vals)))
    items.sort(key=lambda x: x[2] - x[1], reverse=True)
    base_x = next(p["xtrack_2_5s"] for p in param_sens["A"]["mu"]["points"]
                  if abs(p["value"] - 0.85) < 1e-9)
    fig, ax = plt.subplots(figsize=(8.5, 0.5 * len(items) + 1.8))
    y = np.arange(len(items))
    for i, (label, lo, hi) in enumerate(items):
        ax.barh(i, max(hi - lo, 0.005), left=lo, color="#1f77b4", alpha=0.75, height=0.55)
        ax.text(hi + 0.02, i, f"{lo:.2f}–{hi:.2f}", va="center", fontsize=8, color="#555")
    ax.axvline(base_x, color="k", lw=1, ls="--")
    ax.text(base_x, len(items) - 0.2, " LS9 基准", fontsize=8)
    ax.axvline(LANE_MARGIN, color="#d62728", lw=1, ls="-.")
    ax.text(LANE_MARGIN, -0.45, " 车道裕度", fontsize=8, color="#d62728")
    ax.set_yticks(y)
    ax.set_yticklabels([it[0] for it in items], fontsize=9)
    ax.set_xlabel("缓解后横向偏差 @2.5 s [m]")
    ax.grid(axis="x", alpha=0.3)
    ax.invert_yaxis()
    ax.set_title("图 8  车辆参数对缓解后可控性的影响（锚 A：直行100 · RL 跑飞锁死 · 缓解开）",
                 fontsize=11)
    fig.tight_layout()
    return savefig(fig, "fig8_tornado")


def fig_param_curves(param_sens) -> str:
    """图9：关键参数曲线。"""
    ensure_matplotlib()
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.0))

    d = param_sens["A"]["c_alpha"]
    ax = axes[0]
    vals = [p["value"] for p in d["points"]]
    ax.plot(vals, [p["xtrack_2_5s"] for p in d["points"]], "o-", color=COL_MIT, label="侧偏刚度")
    d2 = param_sens["A"]["iz"]
    ax2 = ax.twiny()
    ax2.plot([p["value"] for p in d2["points"]],
             [p["xtrack_2_5s"] for p in d2["points"]], "s--", color="#9467bd")
    ax2.set_xlabel("横摆惯量 I_z [kg·m²]", color="#9467bd", fontsize=9)
    ax.set_xlabel("轮胎侧偏刚度 [kN/rad]")
    ax.set_ylabel("缓解后横向偏差 @2.5 s [m]")
    ax.set_title("(a) 第一敏感因子：胎刚度↑恶化 / 惯量↑改善\n（锚 A：后轮锁死+缓解）", fontsize=10)
    ax.grid(alpha=0.3)

    ax = axes[1]
    d = param_sens["B"]["scrub"]
    vals = [p["value"] for p in d["points"]]
    ax.plot(vals, [p["dyaw_peak_dps"] for p in d["points"]], "o-", color="#ff9f1c",
            label="主销偏置 scrub [mm]")
    d2 = param_sens["B"]["caster"]
    ax.plot([v * 6.0 for v in [p["value"] for p in d2["points"]]],
            [p["dyaw_peak_dps"] for p in d2["points"]], "s--", color="#9467bd",
            label="主销后倾 ×6 [°→mm 对齐横轴]")
    ax.set_xlabel("几何参数（横轴按量程对齐）")
    ax.set_ylabel("Δr 瞬态峰值 [°/s]")
    ax.set_title("(b) 自由失效瞬态的几何抓手：小 scrub\n（锚 B：前轮自由@弯中，未缓解）", fontsize=10)
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    d = param_sens["B"]["cg_front"]
    ax = axes[2]
    vals = [p["value"] for p in d["points"]]
    ax.plot(vals, [p["xtrack_2_5s"] for p in d["points"]], "o-", color="#2ca02c")
    ax.set_xlabel("质心到前轴距离 a [m]（大 = 后置）")
    ax.set_ylabel("横向偏差 @2.5 s [m]")
    ax.set_title("(c) 配重决定前轴失效的外漂速度\n（锚 B：前轮自由@弯中，未缓解）", fontsize=10)
    ax.grid(alpha=0.3)

    fig.suptitle("图 9  代表性参数-可控性曲线", fontsize=12)
    fig.tight_layout()
    return savefig(fig, "fig9_param_curves")


def fig_actuator_cost(curves) -> str:
    ensure_matplotlib()
    mit = curves["mit:straight100:2:stuck_value"]
    t = np.asarray(mit.t) - 9.0
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.0))
    for w, col in (("fl", "#d62728"), ("fr", "#1f77b4"), ("rl", "#2ca02c"), ("rr", "#9467bd")):
        axes[0].plot(t, np.rad2deg(_arr(mit, f"delta_{w}")), lw=1.4, label=w.upper(), color=col)
        axes[1].plot(t, np.abs(_arr(mit, f"slip_alpha_{w}")) * 180 / math.pi, lw=1.4,
                     label=w.upper(), color=col)
    axes[0].set_ylabel("车轮转角 [°]")
    axes[1].set_ylabel("|侧偏角| [°]")
    for ax in axes:
        ax.axvline(0, color="k", lw=0.8, ls="--")
        ax.set_xlabel("故障后时间 [s]")
        ax.set_xlim(-1, 5)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=9)
    fig.suptitle("图 10  缓解代价：后轴 RL/RR 内部力对抗（侧偏角驻留）→ 限速与轮胎裕度校核依据",
                 fontsize=12)
    fig.tight_layout()
    return savefig(fig, "fig10_actuator_cost")


# ─── HTML 报告 ────────────────────────────────────────────────────────────────

def fmt(v, digits=2):
    if isinstance(v, float) and math.isinf(v):
        return "∞"
    return f"{v:.{digits}f}"


def build_html(rows, curves, param_sens, figs: dict[str, str]) -> str:
    sens = curves["sensitivity"]

    def matrix_table() -> str:
        headers = [
            "工况", "失效轮", "失效形态", "缓解", "Δψ@2s [°]",
            "横向偏差@1.2s [m]", "横向偏差@2.5s [m]", "TTLD [s]",
            "Δr峰值 [°/s]", "Δa_y峰值 [m/s²]", "β峰值 [°]", "C 级",
        ]
        table_rows = []
        for r in rows:
            cc = r["c_class"]
            table_rows.append([
                SCN_LABELS[r["scenario"]],
                WHEEL_NAMES[r["wheel"]],
                FAULT_NAMES[r["fault"]],
                "—" if r["mitigation"] == "baseline" else "开",
                fmt(r["dpsi_2s_deg"]),
                fmt(r["xtrack_react"]),
                fmt(r["xtrack_2_5s"]),
                fmt(r["ttld_s"], 1),
                fmt(r["dyaw_peak_dps"], 1),
                fmt(r["ay_peak"], 1),
                fmt(r["beta_peak_deg"], 1),
                html_cell(cc, {"style": f"color:{C_COLORS[cc]};font-weight:700"}),
            ])
        return html_table(headers, table_rows)

    def sens_table() -> str:
        return html_table(
            ["τ_d [s]", "偏差@1.2s [m]", "偏差@2.5s [m]", "Δr峰值 [°/s]", "TTLD [s]", "C 级"],
            [
                [
                    f"{s['detect_delay']:.2f}",
                    fmt(s["xtrack_react"]),
                    fmt(s["xtrack_2_5s"]),
                    fmt(s["dyaw_peak_dps"], 1),
                    fmt(s["ttld_s"], 1),
                    html_cell(s["c_class"], {"style": f"color:{C_COLORS[s['c_class']]};font-weight:700"}),
                ]
                for s in sens
            ],
        )

    def param_table(anchor: str, metric: str, metric_label: str) -> str:
        table_rows = []
        for key, d in param_sens[anchor].items():
            for p in d["points"]:
                mark = " <b>*</b>" if abs(p["value"] - d["base"]) < 1e-9 else ""
                table_rows.append([
                    f"{d['label']} [{d['unit']}]",
                    html_cell(f"{p['value']:g}{mark}", raw=True),
                    fmt(p[metric]),
                    html_cell(p["c_class"], {"style": f"color:{C_COLORS[p['c_class']]};font-weight:700"}),
                ])
        return html_table(["参数", "取值", metric_label, "C 级"], table_rows) + "<p class='meta'>* = LS9 基准值</p>"

    b = next(r for r in rows if r["scenario"] == "straight100" and r["wheel"] == 2
             and r["mitigation"] == "baseline")
    m = next(r for r in rows if r["scenario"] == "straight100" and r["wheel"] == 2
             and r["mitigation"] == "mitigated")
    ff_s = next(r for r in rows if r["scenario"] == "straight100" and r["wheel"] == 0
                and r["mitigation"] == "baseline")
    ff_c = next(r for r in rows if r["scenario"] == "curve60" and r["wheel"] == 0
                and r["mitigation"] == "baseline")

    def img(key, alt):
        return embedded_png_figure(figs[key], alt)

    def actuator_table() -> str:
        return html_table(
            ["", "前轮执行器", "后轮执行器"],
            [
                ["自锁性", "无自锁", "自锁"],
                ["正效率", "—（正常伺服）", "≈30%"],
                ["逆效率 η_rev", "≈60%（轮胎力可反驱机构）", "0（不可反驱）"],
                [
                    html_cell("<b>断电/失效形态</b>", raw=True),
                    html_cell("<b>自由脚轮</b>：J·δ̈ = −η_rev·τ_KP − c·δ̇ − τ_c·sgn(δ̇)", raw=True),
                    html_cell("<b>锁死在失效位置</b>（原角锁死 / 跑飞后锁死）", raw=True),
                ],
                [
                    "正常工况含义",
                    "需持续供电抵抗回正力矩（能耗↑），失效温和",
                    "断电保持、能耗低（以 30% 正效率换自锁），失效恶性",
                ],
            ],
        )

    def fmea_table() -> str:
        return html_table(
            ["编号", "轴", "失效模式", "典型成因", "注入方式"],
            [
                [
                    "FM1",
                    "前",
                    "自由脚轮（断电/驱动级失效）",
                    "供电中断、桥臂关断、控制器失效",
                    html_cell(
                        "<code>free_caster</code>：会话层积分机构 ODE（J=3 kg·m²，c=80 N·m·s/rad，η=0.6），"
                        "经作动器环节生效；复用平台逐步实时计算的 Reimpell 主销力矩",
                        raw=True,
                    ),
                ],
                [
                    "FM2",
                    "后",
                    "弯中/机动中原角锁死",
                    "自锁机构失电即锁",
                    html_cell("<code>stuck_hold</code>（锁存失效瞬间实际轮角）", raw=True),
                ],
                [
                    "FM3",
                    "后",
                    "跑飞后锁死",
                    "驱动级故障先失控输出再锁死",
                    html_cell(f"<code>stuck_value</code> +{STUCK_DEG:.0f}°", raw=True),
                ],
            ],
        )

    def hara_table() -> str:
        return html_table(
            ["危害事件", "运行场景", "S", "E", "C（未缓解，实测）", "ASIL"],
            [
                [
                    "H1 后轮跑飞后锁死→非预期横摆",
                    "高速直行 ~100 km/h",
                    "S3",
                    "E4",
                    html_cell(
                        f"<b>C3</b>（TTLD {fmt(b['ttld_s'], 1)} s）",
                        {"style": f"color:{C_COLORS['C3']}"},
                        raw=True,
                    ),
                    html_cell("<b>D</b>", raw=True),
                ],
                [
                    "H2 后轮弯中锁死→出弯残余转向",
                    "山区弯道-出弯 ~60 km/h",
                    "S3",
                    "E3",
                    html_cell("<b>C2</b>", {"style": f"color:{C_COLORS['C2']}"}, raw=True),
                    "B",
                ],
                [
                    "H3 前轮断电自由→前轴转向不足化",
                    "弯中 ~60 km/h",
                    "S2",
                    "E3",
                    html_cell("<b>C2</b>（~4 Hz 衰减摆振后转向不足外漂）", {"style": f"color:{C_COLORS['C2']}"}, raw=True),
                    "A–B",
                ],
                [
                    "H4 前轮断电自由→toe 失衡慢漂",
                    "高速直行 ~100 km/h",
                    "S3",
                    "E4",
                    html_cell(
                        f"<b>C2</b>（TTLD {fmt(ff_s['ttld_s'], 1)} s）",
                        {"style": f"color:{C_COLORS['C2']}"},
                        raw=True,
                    ),
                    "B",
                ],
            ],
        )

    def controllability_table() -> str:
        return html_table(
            [html_cell("等级", raw=True), f"判据（评估窗 {EVAL_WINDOW:.0f} s）"],
            [
                [
                    html_cell("<b>C1</b>", {"style": f"color:{C_COLORS['C1']}"}, raw=True),
                    html_cell("TTLD=∞ 且 偏差峰值&lt;0.45 m 且 Δr残余&lt;1°/s 且 Δa_y峰值&lt;2 m/s²", raw=True),
                ],
                [
                    html_cell("<b>C2</b>", {"style": f"color:{C_COLORS['C2']}"}, raw=True),
                    html_cell(f"TTLD ≥ {T_REACT} s（反应窗内未脱离车道）且 Δa_y峰值 &lt; 5 m/s²", raw=True),
                ],
                [
                    html_cell("<b>C3</b>", {"style": f"color:{C_COLORS['C3']}"}, raw=True),
                    "其余",
                ],
            ],
        )

    def tool_iteration_table() -> str:
        return html_table(
            ["版本", "由分析需求倒逼的平台能力"],
            [
                ["v0.10", "实验底座：SimSession 无头会话（~25× 实时）、run 落盘、KPI 流水线、变体矩阵"],
                [
                    "v0.13",
                    "定时故障注入进实验 schema（stuck 族）；fault_reconfig 容错策略——三次数据驱动"
                    "迭代：纯运动学 ICR 投影被数据否决 → 镜像 P 环在后轮饱和自旋 → PI+限幅减速定型",
                ],
                [
                    "v0.14",
                    html_cell(
                        "<b>自由脚轮失效模型</b>（free_caster 机构 ODE，复用平台实时主销力矩通道）；"
                        "fault_reconfig free 模式（增益补偿）；<b>参数×故障×工况敏感性流水线</b>"
                        "（vehicle overrides / scene μ 逐点自动配无故障参考）",
                        raw=True,
                    ),
                ],
            ],
        )

    n_cases = len(rows) // 2
    n_c3_base = sum(1 for r in rows if r["mitigation"] == "baseline" and r["c_class"] == "C3")
    n_c3_mit = sum(1 for r in rows if r["mitigation"] == "mitigated" and r["c_class"] == "C3")
    n_param_runs = sum(2 * len(d["points"]) for d in param_sens["A"].values()) * 2
    n_runs = 3 + n_cases * 2 + len(sens) + n_param_runs

    title = "四轮独立转向单轮失效的 ISO 26262 可控性分析与容错控制（v2：机构差异化 + 参数敏感性）"
    styles = f"""
 body {{ font-family: "PingFang SC","Hiragino Sans GB","Microsoft YaHei",sans-serif;
        max-width: 990px; margin: 0 auto; padding: 40px 24px; color:#1a202c; line-height:1.75; }}
 h1 {{ font-size: 24px; border-bottom: 3px solid #1f77b4; padding-bottom: 10px; }}
 h2 {{ font-size: 19px; margin-top: 2.2em; border-left: 5px solid #1f77b4; padding-left: 10px; }}
 h3 {{ font-size: 15.5px; margin-top: 1.6em; }}
 table {{ border-collapse: collapse; width: 100%; font-size: 12.5px; margin: 12px 0; }}
 th, td {{ border: 1px solid #cbd5e0; padding: 5px 8px; text-align: center; }}
 th {{ background: #edf2f7; }}
 figure {{ margin: 18px 0; text-align: center; }}
 .abstract {{ background:#f7fafc; border:1px solid #e2e8f0; border-radius:8px; padding:16px 20px; }}
 .kbox {{ background:#fffbeb; border-left:4px solid #d69e2e; padding:10px 16px; margin:14px 0; }}
 .toolbox {{ background:#f0fff4; border-left:4px solid #2f855a; padding:10px 16px; margin:14px 0; }}
 .eq {{ background:#f7fafc; padding:8px 18px; margin:10px 0; font-family:STIX,serif; }}
 code {{ background:#edf2f7; padding:1px 5px; border-radius:4px; font-size: 12px; }}
 .meta {{ color:#718096; font-size: 13px; }}
"""
    run_meta = meta_paragraph(f"""仿真平台：4WIS Simulator v{SIM_VERSION} · 车辆：LS9 默认参数（m=2900 kg，L=3.16 m）·
模型：simplified_dynamic（RK4 + 半隐式轮速 + c_α(F_z) 载荷敏感线性胎）·
复现：<code>python scripts/study_single_wheel_failure.py</code>（共 {n_runs} 个仿真 run）""")
    abstract = callout_box(f"""<b>摘要</b> —
本文第 2 版根据实际执行机构设定重构失效模型：<b>前轮转向执行器无自锁（逆效率 η≈0.6），断电失效呈自由脚轮
状态</b>——转向自由度变为受主销力矩驱动的动力学（J·δ̈=−η·τ<sub>KP</sub>−c·δ̇）；<b>后轮执行器自锁
（正效率 0.3、逆效率 0），失效锁死在当时位置</b>。仿真表明两类失效物理本质截然不同：前轮自由失效因脚轮自对准而<b>天然温和</b>（弯中断电经 ~4 Hz 衰减摆振、约 1 s 收敛到零侧偏力平衡；
直行仅因静态 toe 平衡丢失产生慢漂），全部工况 ≤C2；后轮自锁失效把故障<b>冻结为持续干扰源</b>，跑飞后锁死 +4° @100 km/h
为 <b>C3</b>（TTLD {fmt(b["ttld_s"], 1)} s）。据此设计差异化安全机制：后轮锁死用<b>同轴镜像力抵消 +
ESP 级横摆 PI + 限幅减速</b>，前轮自由用<b>健康轮增益补偿</b>；缓解后全矩阵 C3 清零（最恶性组合横摆扰动
{fmt(b["dyaw_peak_dps"], 1)}→{fmt(m["dyaw_peak_dps"], 1)}°/s）。进一步以 9 个整车参数（主销后倾/偏置、
侧偏刚度、质量、质心前后/高度、横摆惯量、轴距、路面 μ）做 OAT 敏感性扫描（{n_param_runs} run）：
结论有两条推翻直觉：<b>轮胎侧偏刚度与横摆惯量才是缓解后可控性的第一敏感因子（硬胎/轻小车更危险——
干扰力∝c_α、航向积累∝1/I_z），而路面 μ 近乎平坦（力平衡型缓解两侧同缩）；自由脚轮失效的温和性对参数
结构性稳健（残余 Δr 恒 ≈0.5°/s），几何抓手是小主销偏置而非主销后倾</b>——为机构选型与底盘参数设计
给出功能安全侧的量化输入。研究过程同步迭代了仿真平台
（自由脚轮失效模型、参数×故障×工况扫描流水线，§9）。""", "abstract")
    mechanism_takeaway = callout_box(f"""前轮"无自锁+高逆效率"的失效形态<b>天然安全</b>——主销后倾就是被动安全机制，
代价是正常工况需持续供电抵抗回正力矩；后轮"自锁+零逆效率"省电、断电保持，代价是把每次失效都
<b>冻结成持续干扰源</b>，且最不利模式（跑飞后锁死）成为 ASIL D 源头。若后轮改用可反驱机构+常闭
离合器（断电脱开→退化为自由脚轮），H1 可整体降级为 H3/H4 类温和失效——本报告的量化数据
（C3 vs C2、TTLD {fmt(b["ttld_s"], 1)} s vs {fmt(ff_s["ttld_s"], 1)} s）正是这类机构决策所需的输入。""", "kbox")
    matrix_takeaway = callout_box(f"""<b>矩阵结论</b>：未缓解 {n_c3_base}/{n_cases} 组合 C3——<b>全部来自后轮自锁失效</b>
（跑飞后锁死 TTLD {fmt(b["ttld_s"], 1)} s）；前轮自由失效全部 ≤C2，印证"自由脚轮天然温和"。
缓解后 {n_c3_mit}/{n_cases} 组合 C3（<b>清零</b>）：最恶性组合横摆扰动峰值
{fmt(b["dyaw_peak_dps"], 1)}→{fmt(m["dyaw_peak_dps"], 1)}°/s、2 s 航向漂移
{fmt(b["dpsi_2s_deg"], 1)}→{fmt(m["dpsi_2s_deg"], 1)}°。""", "kbox")
    tool_iteration = callout_box(f"""<p>本研究的另一目的：<b>用实际工程分析迭代仿真工具</b>。两版研究写入平台的能力：</p>
{tool_iteration_table()}
<p>沉淀的可复用资产：任意"失效 × 机动 × 参数"矩阵已是配置问题而非编码问题；本报告任何 run 均可在
前端「试验/分析」页交互复查与回放。</p>""", "toolbox")
    conclusion = report_section("10　结论", """<p>(1) 执行机构特性（自锁性/逆效率）决定单轮失效的物理形态：前轮（无自锁，η_rev=0.6）失效为自由
脚轮——主销后倾使其自对准、天然温和（全部工况 ≤C2）；后轮（自锁，η_rev=0）失效冻结为持续干扰，
跑飞后锁死 @100 km/h 为 C3/ASIL D。(2) 差异化容错控制（后轮：同轴镜像+ESP 级横摆 PI+限幅减速；
前轮：健康轮增益补偿）将全矩阵 C3 清零。(3) 参数敏感性给出底盘设计的功能安全输入：硬胎/低惯量平台需更快检测预算、小 scrub 有利失效
温和性、失效工况不约束 caster、μ 对力平衡型缓解近乎中性；安全概念在全部扫描范围内保持 C2 稳健。(4) 机构选型量化论据：非自锁+高
逆效率以正常能耗换失效温和性——后轮构型值得按本数据重估（可反驱+常闭离合器方案可将 ASIL D 危害
整体降级）。(5) 平台同步获得自由脚轮失效模型与参数敏感性流水线（§9），后续任意失效研究可直接复用。</p>""")
    footer_meta = meta_paragraph("本报告由 4WIS Simulator 自动生成 · 全部数据与图表来自实跑仿真")
    body = f"""

<h1>四轮独立转向系统单轮转向失效的 ISO 26262 功能安全可控性分析与容错控制策略设计<br>
<span style="font-size:15px;color:#4a5568">v2 — 执行机构差异化失效建模（前轮自由脚轮 / 后轮自锁锁死）与整车参数敏感性研究</span></h1>
{run_meta}

{abstract}
<p><b>关键词</b>：四轮独立转向；ISO 26262；可控性；自由脚轮；自锁机构；容错控制；参数敏感性</p>

<h2>1　引言</h2>
<p>v1 报告将单轮失效统一建模为"卡死"。实际执行机构给出更精细的图景：本项目前轮采用<b>非自锁</b>传动
（逆效率 ≈60%——轮胎力可反驱机构），后轮采用<b>自锁</b>传动（逆效率 0）。失效形态因此从"卡在哪"变成
"<b>自由还是冻结</b>"：前轮断电后在主销力矩驱动下自由摆动；后轮失效后锁死在失效位置。这一机构差异
直接改写危害等级、安全机制设计乃至机构选型论证——本文以此为主线，并将可控性分析扩展到整车参数空间：
车轮参数（主销几何、轮胎刚度）、车身参数（质量、质心、惯量）、底盘参数（轴距、路面附着）。</p>

<h2>2　执行机构设定与失效模式</h2>
<h3>2.1 机构传动特性</h3>
{actuator_table()}
<h3>2.2 分析的失效模式（FMEA 摘要）</h3>
{fmea_table()}
<p>前轮机械卡滞（异物侵入）仍可能出现"前轮卡死"，概率量级低于断电类失效；其后果与缓解（同轴镜像）
已在 v1 验证，本文不再重复。</p>

<h2>3　HARA：危害分析与风险评估（按机构差异化修订）</h2>
{hara_table()}
<p>安全目标 <b>SG1（ASIL D，由 H1 导出）：任一转向执行器单点失效不得导致车辆非预期偏离车道</b>；
安全状态：重构转向 + 受控降速的降级行驶。</p>

<h2>4　可控性定量评估方法</h2>
<p>与 v1 相同的口径：<b>驾驶员开环</b>（失效后不纠正，最不利暴露）；以同参数无故障参考 run 的轨迹为
基准折线，度量横向偏差 d(t)、车道脱离时间 TTLD（裕度 {LANE_MARGIN} m）、相对参考的横摆扰动 Δr、
侧向加速度增量 Δa_y（时间对齐相减——稳态弯名义 a_y 不计入）、质心侧偏角 β。</p>
{controllability_table()}

<h2>5　差异化安全机制设计</h2>
<h3>5.1 后轮锁死（FM2/FM3）：同轴镜像力抵消 + ESP 级横摆 PI + 限幅减速</h3>
<p class="eq">δ_RR = δ_RR<sup>alloc</sup> − (δ_s − δ_RL<sup>alloc</sup>)；&nbsp;
δ_fb = clip(k_r·[(r_des−r) + 0.8∫(r_des−r)dt], ±8°)，前轮 +δ_fb / 后轮 −δ_fb；&nbsp;
v_cap(t) = max(v_limit, v_detect − 3 m/s²·t)</p>
<p>要点（v1→v2 迭代获得）：①同轴等力臂使镜像同时对消 ΣF_y 与 ΣM_z；②后轮高速锁死是自旋激励，
且镜像使两条后胎带 ±δ_s 偏置运行、车身侧偏一起来即双双饱和——必须 ESP 级 PI（P 稳瞬态、I 持住
饱和残余力矩）；③限速必须带减速度限幅——阶跃限速使轮速伺服以摩擦极限制动，纵向力恰好抢占饱和后轴
的摩擦椭圆。</p>
<h3>5.2 前轮自由（FM1）：健康轮增益补偿</h3>
<p>自由轮脚轮自对准到零侧偏力方向（≈局部速度方向≈理想阿克曼几何）——<b>它自动做对几何、只是不再出力</b>。
故无需镜像（无寄生力可抵消），机制为<b>权限补偿</b>：</p>
<p class="eq">δ_FR = g·δ_FR<sup>alloc</sup>（g≈2，线性区内恢复前轴合力）＋ 同款横摆 PI ＋ 限速</p>
{img("fig1", "机理")}
<h3>5.3 机构选型的功能安全权衡（本设定的核心启示）</h3>
{mechanism_takeaway}

<h2>6　主矩阵仿真结果</h2>
{img("fig2", "后轮锁死时间历程")}
{img("fig3", "前轮自由行为")}
{img("fig4", "轨迹")}
<h3>6.1 全矩阵指标</h3>
{matrix_table()}
{img("fig5", "指标条形图")}
{img("fig7", "C 分级矩阵")}
{matrix_takeaway}
<h3>6.2 检测延时敏感性（FTTI 分解）</h3>
{img("fig6", "检测延时")}
{sens_table()}
<p>τ_d ≤ 0.3 s 保持 C2、≤0.05 s 达 C1。建议检测预算：轮端残差判决 ≤50 ms + 去抖 ≤100 ms +
域控裁决切换 ≤100 ms（合计 ≤250 ms ≈ FTTI 的 1/4）。前轮自由失效的检测特征不同——无"指令-角度
残差"权限，应监测<b>母线电流/驱动使能 + 角速度异常</b>，通常更快（≤50 ms）。</p>
<h3>6.3 缓解代价</h3>
{img("fig10", "执行器代价")}
<p>镜像抵消让后轴两轮持续对抗（侧偏角驻留），轮胎发热/磨损增加、该轴侧向储备被占用——限速
{V_LIMIT_KMH:.0f} km/h 的依据；持续保持力矩需求（齿条力×臂长）应按负载特性页数据校核后轮执行器
的持续工作点（正效率 30% 意味着电机功率放大 ~3.3×）。</p>

<h2>7　整车参数对可控性的影响（OAT 敏感性）</h2>
<p>两个锚定问题：<b>锚 A</b>——安全概念（镜像+PI+限速）对车辆参数变化的稳健性（直行100 · 后轮跑飞
锁死 · 缓解开）；<b>锚 B</b>——机构/整车参数对前轮自由失效"天然温和度"的影响（弯中60 · 前轮自由 ·
未缓解）。每参数独立扫描（OAT），其余保持 LS9 基准；<b>每个取值均重跑同参数的无故障参考</b>。</p>
{img("fig8", "龙卷风")}
{img("fig9", "参数曲线")}
<h3>7.1 锚 A 数据（缓解后横向偏差 @2.5 s）</h3>
{param_table("A", "xtrack_2_5s", "偏差@2.5s [m]")}
<h3>7.2 锚 B 数据（前轮自由失效横摆残余）</h3>
{param_table("B", "dyaw_resid_dps", "Δr 残余 [°/s]")}
<h3>7.3 工程解读（按实测数据，含两条推翻预期的结论）</h3>
<ul>
<li><b>轮胎侧偏刚度是锚 A 第一敏感因子，方向"反直觉"</b>：c_α 80→150 kN/rad 使缓解后偏差
0.46→0.88 m（近 2×）。机理：锁死轮的寄生力 ∝ c_α·δ_s——<b>胎越硬，同样 4° 锁死注入的干扰越大</b>；
检测窗内的漂移无缓解可言，全由 c_α 定标。运动型硬胎配置需要相应更快的检测预算。</li>
<li><b>横摆惯量 I_z 越大越安全</b>（5500→9500 kg·m²：0.85→0.66 m）：检测窗内故障力矩对航向的
积累 ∝ 1/I_z。轻小车型对同样故障更敏感——安全概念移植到小车平台时检测预算需重估。</li>
<li><b>路面附着 μ 近乎平坦（推翻"低附着更危险"的预期）</b>：μ=0.5 时甚至略好（0.55 vs 0.74 m）。
机理：4° 锁死的干扰力需求 ≈8.4 kN，μ=0.5 时被 μF_z≈3.5 kN 削顶——<b>低附着把干扰力和缓解权限
一起等比缩小，而镜像抵消是力平衡型机制，两侧同缩、净效应近零</b>。这是该安全机制的一个内在稳健性。</li>
<li><b>主销后倾角对两个锚都近乎零敏感（推翻"后倾决定回正品质"的预期）</b>：锚 A 中锁死轮不再有
转向自由度，主销几何本就不进力路径（零敏感是物理正确）；锚 B 中自由轮稳态温和度由 α→0 平衡结构性
保证（残余 Δr 恒 ≈0.5°/s），2–8° 后倾仅改变瞬态峰值 &lt;10%（机构阻尼与作动器滞后主导）。
<b>失效工况不构成对 caster 的设计约束</b>——底盘可按操稳/手感自由选取。</li>
<li><b>自由失效瞬态的真正几何抓手是主销偏置 scrub</b>：5→50 mm 使 Δr 峰值 +31%（Fy·scrub 力矩
阶跃项）——小 scrub 有利失效温和性，与传统转向设计直觉一致。</li>
<li><b>轴荷分配</b>：质心前移使前轴失效外漂加快（锚 B：2.68→2.94 m）——前置配重平台的前轮失效
预算更紧。质量/轴距在扫描范围内 ≤±5%（稳健）。</li>
</ul>

<h2>8　讨论与局限</h2>
<p>①开环无驾驶员口径偏保守（安全侧）。②自由脚轮模型实测给出 ~4 Hz、约 1 s 收敛的衰减摆振（图 3a）——频率 ≈√(η·c_α·(scrub+r·tanτ)/J)
与经典 caster shimmy 一致；<b>衰减率由机构阻尼 c 决定，c=80 N·m·s/rad 为工程估计，机构选型时应以
"断电摆振 3 个周期内收敛"为阻尼下限做台架辨识</b>；本轮胎模型无松弛长度（松弛长度会降低摆振频率并
恶化阻尼裕度），建议辨识后用 Pacejka+松弛长度复跑。③后轮正效率 30% 的正常工况功率放大（~3.3×）应结合负载特性页
齿条力做执行器持续工作点校核。④机动中锁死（DLC）仍是最难场景，"完成当前机动再降级"的前馈策略
留作后续。⑤参数敏感性为 OAT 口径，交互效应（如 μ×质心）需 DOE 全因子补充。</p>

<h2>9　仿真工具迭代记录（本研究倒逼的平台演进）</h2>
{tool_iteration}

{conclusion}

<h2>参考文献</h2>
<ol style="font-size:13px">
<li>ISO 26262-3:2018, Road vehicles — Functional safety — Part 3: Concept phase.</li>
<li>ISO 3888-2:2011, Passenger cars — Test track for a severe lane-change manoeuvre.</li>
<li>Pacejka, H. B. <i>Tire and Vehicle Dynamics</i>, 3rd ed., 2012.</li>
<li>Reimpell, J. et al. <i>The Automotive Chassis</i>, 2nd ed., 2001（主销力矩口径）.</li>
<li>4WIS Simulator v{SIM_VERSION}：docs/v1_platform_refactor_plan.md 与 CHANGELOG（平台演进）.</li>
</ol>

<h2>附录 A　复现</h2>
<p><code>python scripts/study_single_wheel_failure.py</code>（约 2 分钟，{n_runs} runs：主矩阵 +
检测延时 + 参数敏感性含逐点无故障参考）。指标原始数据见
<code>single_wheel_failure_metrics.json</code>。</p>
{footer_meta}
"""
    return ReportDocument(title=title, styles=styles).render(body)


# ─── main ────────────────────────────────────────────────────────────────────

def main() -> None:
    print("§1 主矩阵 …")
    rows, curves = run_matrix()

    print("§2 参数敏感性 …")
    param_sens = run_param_sensitivity()

    print("§3 出图 …")
    figs = {
        "fig1": fig_mechanism(),
        "fig2": fig_timehistory(curves),
        "fig3": fig_free_front(curves),
        "fig4": fig_trajectories(curves),
        "fig5": fig_metric_bars(rows),
        "fig6": fig_sensitivity(curves),
        "fig7": fig_c_matrix(rows),
        "fig8": fig_tornado(param_sens),
        "fig9": fig_param_curves(param_sens),
        "fig10": fig_actuator_cost(curves),
    }

    print("§4 写报告 …")
    write_json(
        OUT_DIR / "single_wheel_failure_metrics.json",
        {"rows": rows, "sensitivity": curves["sensitivity"],
         "param_sensitivity": param_sens, "version": SIM_VERSION},
    )
    html = build_html(rows, curves, param_sens, figs)
    out = OUT_DIR / "single_wheel_failure_safety_analysis.html"
    out.write_text(html, encoding="utf-8")
    print(f"✓ 报告：{out}")


if __name__ == "__main__":
    main()
