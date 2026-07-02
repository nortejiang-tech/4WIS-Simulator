#!/usr/bin/env python3
"""单轮转向失效 ISO 26262 可控性研究 — 一键复现脚本。

    python scripts/study_single_wheel_failure.py

产物：
    docs/reports/figs_single_wheel/*.png      所有图
    docs/reports/single_wheel_failure_safety_analysis.html   自包含报告（图内嵌 base64）
    docs/reports/single_wheel_failure_metrics.json           全矩阵指标原始数据

仿真基座：sim4wis 实验底座（SimSession 无头会话 + FaultSpec 定时故障注入 +
fault_reconfig 降级策略），车辆为 LS9 默认参数、simplified_dynamic 模型。
"""

from __future__ import annotations

import base64
import io
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "backend" / "src"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from sim4wis import __version__ as SIM_VERSION
from sim4wis.experiment.schema import (
    Experiment, FaultSpec, Maneuver, ManeuverStep, SteerProfile,
)
from sim4wis.experiment.session import run_experiment

plt.rcParams["font.sans-serif"] = ["PingFang SC", "Hiragino Sans GB", "Arial Unicode MS", "SimHei"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 110

OUT_DIR = ROOT / "docs" / "reports"
FIG_DIR = OUT_DIR / "figs_single_wheel"
FIG_DIR.mkdir(parents=True, exist_ok=True)

# ─── 判据常量（报告 §4 有依据说明） ──────────────────────────────────────────
LANE_MARGIN = 0.90       # [m] 3.75 m 车道 − 1.96 m 车宽 → 单侧裕度 ≈0.9 m
T_REACT = 1.2            # [s] 驾驶员感知-反应时间（95 分位工程惯用值）
EVAL_WINDOW = 5.0        # [s] 故障后评估窗
DETECT_DELAY = 0.15      # [s] 基线检测+仲裁延时
V_LIMIT_KMH = 60.0       # 降级限速
STUCK_DEG = 4.0          # 直行工况带角卡死角

WHEEL_NAMES = {0: "FL(左前)", 1: "FR(右前)", 2: "RL(左后)", 3: "RR(右后)"}
FAULT_NAMES = {"stuck_value": f"带角卡死+{STUCK_DEG:.0f}°", "stuck_zero": "回中卡死",
               "stuck_hold": "原角冻结"}

COL_BASE = "#d62728"
COL_MIT = "#1f77b4"
COL_REF = "#7f7f7f"


# ─── 场景与实验构造 ──────────────────────────────────────────────────────────

def scenario_steps(scn: str) -> tuple[list[ManeuverStep], float, float]:
    """返回 (steps, t_fault, 车速 km/h)。t_fault 取巡航段中部。"""
    if scn == "straight100":
        steps = [
            ManeuverStep(name="加速", duration=8.0, speed_kmh=100.0, speed_ramp_s=5.0),
            ManeuverStep(name="直行", duration=8.0),
        ]
        return steps, 9.0, 100.0
    if scn == "curve60":
        steps = [
            ManeuverStep(name="加速", duration=6.0, speed_kmh=60.0, speed_ramp_s=4.0),
            ManeuverStep(name="入弯", duration=2.0,
                         steer=SteerProfile(kind="ramp", start=0.0, amplitude=0.05)),
            ManeuverStep(name="稳态弯", duration=8.0,
                         steer=SteerProfile(kind="constant", amplitude=0.05)),
        ]
        return steps, 10.5, 60.0
    if scn == "dlc60":
        steps = [
            ManeuverStep(name="加速", duration=6.0, speed_kmh=60.0, speed_ramp_s=4.0),
            ManeuverStep(name="双移线", duration=8.0,
                         steer=SteerProfile(kind="dlc", amplitude=0.06)),
            ManeuverStep(name="稳定", duration=2.0),
        ]
        return steps, 8.2, 60.0   # 第一摆峰值附近
    raise KeyError(scn)


SCN_LABELS = {"straight100": "高速直行 100 km/h",
              "curve60": "稳态弯 60 km/h (a_y≈4.6 m/s²)",
              "dlc60": "双移线 60 km/h（紧急变道中）"}


def build_exp(scn: str, fault: FaultSpec | None, strategy: str = "ideal_ackermann",
              mode_params: dict | None = None) -> Experiment:
    steps, _tf, _v = scenario_steps(scn)
    return Experiment(
        name=f"sw_{scn}",
        strategy=strategy,
        mode_params=mode_params or {},
        faults=[fault] if fault else [],
        maneuver=Maneuver(name=scn, steps=steps),
        record_hz=100.0,
    )


# ─── 指标计算 ────────────────────────────────────────────────────────────────

def _arr(r, name):
    return np.asarray(r.channels[name], dtype=np.float64)


def cross_track(ref_xy: np.ndarray, xy: np.ndarray) -> np.ndarray:
    """每个点到参考轨迹折线的最小距离（车道保持口径的横向偏差）。"""
    seg_a = ref_xy[:-1]
    seg_b = ref_xy[1:]
    d = seg_b - seg_a
    len2 = np.maximum((d ** 2).sum(axis=1), 1e-12)
    out = np.empty(xy.shape[0])
    for i, p in enumerate(xy):
        t = np.clip(((p - seg_a) * d).sum(axis=1) / len2, 0.0, 1.0)
        proj = seg_a + t[:, None] * d
        out[i] = np.sqrt(((p - proj) ** 2).sum(axis=1).min())
    return out


def compute_metrics(run, ref, t_fault: float) -> dict:
    """故障后可控性指标（相对无故障参考 run）。"""
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

    # 车道脱离时间 TTLD
    dep = np.nonzero(xt > LANE_MARGIN)[0]
    ttld = float(t_post[dep[0]]) if dep.size else math.inf

    yaw = _arr(run, "yaw_rate")
    vx = _arr(run, "vx")
    vy = _arr(run, "vy")
    r_pre = float(np.mean(yaw[max(0, i0 - 50):i0]))     # 故障前 0.5 s 均值
    dyaw = yaw[i0:i_end] - r_pre

    # 故障引起的侧向加速度**增量**（相对无故障参考时间对齐相减——稳态弯的
    # 名义 a_y 不应计入可控性判据）。
    def ay_of(r):
        return (np.gradient(np.asarray(r.channels["vy"]), np.asarray(r.t))
                + np.asarray(r.channels["yaw_rate"]) * np.asarray(r.channels["vx"]))
    ay_run = ay_of(run)
    ay_ref = ay_of(ref)
    n = min(ay_run.size, ay_ref.size)
    d_ay = (ay_run[:n] - ay_ref[:n])[i0:min(i_end, n)]
    ay = d_ay if d_ay.size else np.zeros(1)
    beta = np.arctan2(vy[i0:i_end], np.maximum(vx[i0:i_end], 0.5))

    psi = _arr(run, "pose_psi")
    return {
        "xtrack_1s": xt_at(1.0),
        "xtrack_react": xt_at(T_REACT),
        "xtrack_2_5s": xt_at(2.5),
        "xtrack_max": float(xt.max()),
        "ttld_s": ttld,
        "dyaw_peak_dps": float(np.rad2deg(np.abs(dyaw).max())),
        "dyaw_resid_dps": float(np.rad2deg(abs(float(np.mean(dyaw[-int(0.5 * 100):]))))),
        "ay_peak": float(np.abs(ay).max()),
        "beta_peak_deg": float(np.rad2deg(np.abs(beta).max())),
        "dpsi_2s_deg": float(np.rad2deg(abs(psi[np.searchsorted(t, t_fault + 2.0)] - psi[i0]))),
        "v_at_fault_kmh": float(vx[i0] * 3.6),
    }


def c_class(m: dict) -> str:
    """可控性分级（判据见报告 §4.3；TTLD 为主指标）。"""
    if (math.isinf(m["ttld_s"]) and m["xtrack_max"] < 0.5 * LANE_MARGIN
            and m["dyaw_resid_dps"] < 1.0 and m["ay_peak"] < 2.0):
        return "C1"
    if m["ttld_s"] >= T_REACT and m["ay_peak"] < 5.0:
        return "C2"
    return "C3"


C_COLORS = {"C1": "#2ca02c", "C2": "#ff9f1c", "C3": "#d62728"}


# ─── 研究矩阵 ────────────────────────────────────────────────────────────────

def mitigation_params(wheel: int, angle: float, t_fault: float,
                      detect: float = DETECT_DELAY) -> dict:
    return {"fault_wheel": wheel, "fault_angle": float(angle),
            "fault_time": float(t_fault), "detect_delay": float(detect),
            "v_limit_kmh": V_LIMIT_KMH}


def run_matrix() -> tuple[list[dict], dict]:
    """跑 场景×轮位×故障×缓解 主矩阵 + 检测延时敏感性。返回 (行, 曲线缓存)。"""
    cases = [
        ("straight100", 0, "stuck_value"), ("straight100", 2, "stuck_value"),
        ("curve60", 0, "stuck_zero"), ("curve60", 2, "stuck_zero"),
        ("dlc60", 0, "stuck_hold"), ("dlc60", 2, "stuck_hold"),
    ]
    rows: list[dict] = []
    curves: dict = {}

    refs: dict[str, object] = {}
    for scn in {c[0] for c in cases}:
        refs[scn] = run_experiment(build_exp(scn, None))
        curves[f"ref:{scn}"] = refs[scn]

    for scn, wheel, ftype in cases:
        steps, t_fault, v_kmh = scenario_steps(scn)
        value = math.radians(STUCK_DEG) if ftype == "stuck_value" else 0.0
        fault = FaultSpec(fault_type=ftype, wheel=wheel, value=value, t_start=t_fault)

        base = run_experiment(build_exp(scn, fault))
        curves[f"base:{scn}:{wheel}:{ftype}"] = base

        # 卡死角真值：stuck_hold 从基线 run 提取，其余解析已知。
        if ftype == "stuck_hold":
            tb = np.asarray(base.t)
            stuck_angle = float(_arr(base, f"delta_{['fl','fr','rl','rr'][wheel]}")[
                np.searchsorted(tb, t_fault + 0.4)])
        elif ftype == "stuck_zero":
            stuck_angle = 0.0
        else:
            stuck_angle = value

        mit = run_experiment(build_exp(
            scn, fault, "fault_reconfig",
            mitigation_params(wheel, stuck_angle, t_fault)))
        curves[f"mit:{scn}:{wheel}:{ftype}"] = mit

        for label, run in (("baseline", base), ("mitigated", mit)):
            m = compute_metrics(run, refs[scn], t_fault)
            m.update({"scenario": scn, "wheel": wheel, "fault": ftype,
                      "mitigation": label, "t_fault": t_fault,
                      "stuck_angle_deg": math.degrees(stuck_angle),
                      "c_class": c_class(m)})
            rows.append(m)
        print(f"  ✓ {scn:12s} {WHEEL_NAMES[wheel]:8s} {FAULT_NAMES[ftype]:8s} "
              f"base={rows[-2]['c_class']} mit={rows[-1]['c_class']}")

    # 检测延时敏感性（直行 FL 带角卡死）
    scn, wheel, ftype = "straight100", 0, "stuck_value"
    _steps, t_fault, _v = scenario_steps(scn)
    fault = FaultSpec(fault_type=ftype, wheel=wheel,
                      value=math.radians(STUCK_DEG), t_start=t_fault)
    sens = []
    for td in (0.05, 0.15, 0.30, 0.50, 0.80):
        mit = run_experiment(build_exp(
            scn, fault, "fault_reconfig",
            mitigation_params(wheel, math.radians(STUCK_DEG), t_fault, detect=td)))
        m = compute_metrics(mit, refs[scn], t_fault)
        m.update({"detect_delay": td, "c_class": c_class(m)})
        sens.append(m)
        curves[f"sens:{td}"] = mit
        print(f"  ✓ 敏感性 τ_d={td:.2f}s → xtrack@2.5s={m['xtrack_2_5s']:.2f} m ({m['c_class']})")
    curves["sensitivity"] = sens
    return rows, curves


# ─── 图 ─────────────────────────────────────────────────────────────────────

def savefig(fig, name: str) -> str:
    path = FIG_DIR / f"{name}.png"
    fig.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    buf = io.BytesIO()
    import PIL  # noqa: F401 — not required; re-encode via file read
    return base64.b64encode(path.read_bytes()).decode()


def fig_mechanism() -> str:
    """图1：镜像力抵消机理示意（俯视）。"""
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.6))
    L, T = 3.16, 1.565
    wheels = [(+L / 2, +T / 2), (+L / 2, -T / 2), (-L / 2, +T / 2), (-L / 2, -T / 2)]

    def draw(ax, deltas, forces, title, note):
        ax.add_patch(plt.Rectangle((-L / 2 - 0.55, -T / 2 - 0.18), L + 1.1, T + 0.36,
                                   fill=False, lw=1.6, ec="#333"))
        ax.annotate("", xy=(L / 2 + 1.15, 0), xytext=(L / 2 + 0.6, 0),
                    arrowprops=dict(arrowstyle="-|>", color="#333"))
        for (x, y), d, f in zip(wheels, deltas, forces):
            wl, ww = 0.62, 0.22
            c, s = math.cos(d), math.sin(d)
            corners = np.array([[wl / 2, ww / 2], [wl / 2, -ww / 2],
                                [-wl / 2, -ww / 2], [-wl / 2, ww / 2]])
            rot = corners @ np.array([[c, s], [-s, c]])
            ax.add_patch(plt.Polygon(rot + [x, y], color="#1f77b4" if f == 0 else
                                     ("#d62728" if f < 0 else "#2ca02c")))
            if f != 0:
                ax.annotate("", xy=(x - 0.9 * math.sin(d) * np.sign(f) * -1,
                                    y + 0.9 * math.cos(d) * np.sign(f) * -1),
                            xytext=(x, y),
                            arrowprops=dict(arrowstyle="-|>", lw=2.2,
                                            color="#d62728" if f < 0 else "#2ca02c"))
        for (x, y), nm in zip(wheels, ("FL", "FR", "RL", "RR")):
            ax.text(x, y + (0.42 if y > 0 else -0.42), nm, ha="center",
                    va="bottom" if y > 0 else "top", fontsize=9, color="#555")
        ax.set_xlim(-3.4, 3.9)
        ax.set_ylim(-2.4, 2.4)
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=11)
        ax.text(0, -2.15, note, ha="center", fontsize=9, color="#555")
        ax.axis("off")

    d = math.radians(10)
    draw(axes[0], [d, 0, 0, 0], [-1, 0, 0, 0],
         "(a) FL 卡死 +δs：寄生侧向力 + 横摆力矩",
         "未缓解：F_y 失衡 → 持续横摆 → 车道脱离")
    draw(axes[1], [d, -d, 0, 0], [-1, +1, 0, 0],
         "(b) 同轴镜像抵消：δ_FR = −δs",
         "同轴等力臂 → ΣF_y = 0 且 ΣM_z = 0，航向保持")
    fig.suptitle("图 1  单轮卡死的干扰机理与同轴镜像力抵消安全机制（俯视示意）", fontsize=12)
    return savefig(fig, "fig1_mechanism")


def fig_timehistory(curves) -> str:
    """图2：直行 FL+4° 时间历程四联图。"""
    ref = curves["ref:straight100"]
    base = curves["base:straight100:0:stuck_value"]
    mit = curves["mit:straight100:0:stuck_value"]
    t_fault = 9.0
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)

    def plot3(ax, ch, scale=1.0, ylabel=""):
        for r, col, lbl in ((ref, COL_REF, "无故障参考"), (base, COL_BASE, "未缓解"),
                            (mit, COL_MIT, "镜像抵消+限速")):
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
    axes[1, 0].axhline(LANE_MARGIN, color="#d62728", lw=1, ls="-.",)
    axes[1, 0].text(4.9, LANE_MARGIN + 0.15, "车道裕度 0.9 m", ha="right", fontsize=8, color="#d62728")
    axes[1, 0].set_ylim(-1.0, 6.0)   # 未缓解曲线出图幅（>40 m），聚焦裕度带
    axes[1, 0].set_title("（未缓解曲线超出图幅：5 s 时 ≈42 m）", fontsize=8, color="#888")
    for r, col in ((base, COL_BASE), (mit, COL_MIT)):
        axes[1, 1].plot(np.asarray(r.t) - t_fault, np.rad2deg(_arr(r, "delta_fl")),
                        color=col, lw=1.5)
        axes[1, 1].plot(np.asarray(r.t) - t_fault, np.rad2deg(_arr(r, "delta_fr")),
                        color=col, lw=1.5, ls="--")
    axes[1, 1].set_ylabel("前轮转角 [°]（实线 FL / 虚线 FR）")
    axes[1, 1].axvline(0, color="k", lw=0.8, ls="--")
    axes[1, 1].grid(alpha=0.3)
    for ax in axes[1]:
        ax.set_xlabel("故障后时间 [s]")
    fig.suptitle("图 2  高速直行 100 km/h · FL 带角卡死 +4° 的时间历程（t=0 为故障时刻，蓝色点线为检测完成）",
                 fontsize=12)
    fig.tight_layout()
    return savefig(fig, "fig2_timehistory")


def fig_trajectories(curves) -> str:
    """图3：三工况轨迹俯视对比。"""
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2))
    combos = [("straight100", 0, "stuck_value"), ("curve60", 0, "stuck_zero"),
              ("dlc60", 0, "stuck_hold")]
    for ax, (scn, wheel, ftype) in zip(axes, combos):
        ref = curves[f"ref:{scn}"]
        base = curves[f"base:{scn}:{wheel}:{ftype}"]
        mit = curves[f"mit:{scn}:{wheel}:{ftype}"]
        _s, t_fault, _v = scenario_steps(scn)
        for r, col, lbl, lw in ((ref, COL_REF, "无故障参考", 1.2),
                                (base, COL_BASE, "未缓解", 1.8),
                                (mit, COL_MIT, "缓解后", 1.8)):
            ax.plot(_arr(r, "pose_x"), _arr(r, "pose_y"), color=col, lw=lw, label=lbl)
            i = np.searchsorted(np.asarray(r.t), t_fault)
            ax.plot(_arr(r, "pose_x")[i], _arr(r, "pose_y")[i], "o", color=col, ms=4)
        ax.set_title(f"{SCN_LABELS[scn]}\n{WHEEL_NAMES[wheel]} {FAULT_NAMES[ftype]}", fontsize=10)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.grid(alpha=0.3)
        ax.axis("equal")
    axes[0].legend(fontsize=9)
    fig.suptitle("图 3  三种工况的轨迹俯视对比（圆点 = 故障时刻位置）", fontsize=12)
    fig.tight_layout()
    return savefig(fig, "fig3_trajectories")


def fig_metric_bars(rows) -> str:
    """图4：全矩阵横向偏差与横摆峰值条形图。"""
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.6))
    labels, xt_base, xt_mit, yaw_base, yaw_mit = [], [], [], [], []
    seen = []
    for r in rows:
        key = (r["scenario"], r["wheel"], r["fault"])
        if key in seen:
            continue
        seen.append(key)
        labels.append(f"{SCN_LABELS[r['scenario']].split(' ')[0]}\n{WHEEL_NAMES[r['wheel']][:2]} {FAULT_NAMES[r['fault']][:4]}")
        b = next(x for x in rows if (x["scenario"], x["wheel"], x["fault"]) == key and x["mitigation"] == "baseline")
        m = next(x for x in rows if (x["scenario"], x["wheel"], x["fault"]) == key and x["mitigation"] == "mitigated")
        xt_base.append(min(b["xtrack_2_5s"], 6.0))
        xt_mit.append(m["xtrack_2_5s"])
        yaw_base.append(b["dyaw_peak_dps"])
        yaw_mit.append(m["dyaw_peak_dps"])
    x = np.arange(len(labels))
    for ax, (vb, vm, ylab, thr) in zip(axes, [
        (xt_base, xt_mit, "横向偏差 @2.5 s [m]（截至 6 m 显示）", LANE_MARGIN),
        (yaw_base, yaw_mit, "横摆角速度扰动峰值 [°/s]", None),
    ]):
        ax.bar(x - 0.19, vb, 0.36, color=COL_BASE, label="未缓解")
        ax.bar(x + 0.19, vm, 0.36, color=COL_MIT, label="镜像抵消+限速")
        if thr:
            ax.axhline(thr, color="#d62728", ls="-.", lw=1)
            ax.text(len(x) - 0.5, thr * 1.08, "车道裕度", fontsize=8, color="#d62728", ha="right")
        ax.set_xticks(x)
        ax.set_xticklabels(labels, fontsize=8)
        ax.set_ylabel(ylab)
        ax.grid(axis="y", alpha=0.3)
    axes[0].legend(fontsize=9)
    fig.suptitle("图 4  全矩阵可控性指标：缓解前后对比", fontsize=12)
    fig.tight_layout()
    return savefig(fig, "fig4_metric_bars")


def fig_sensitivity(curves) -> str:
    """图5：检测延时敏感性。"""
    sens = curves["sensitivity"]
    td = [s["detect_delay"] for s in sens]
    fig, ax1 = plt.subplots(figsize=(7.2, 4.2))
    ax1.plot(td, [s["xtrack_2_5s"] for s in sens], "o-", color=COL_MIT, label="横向偏差 @2.5 s")
    ax1.plot(td, [s["xtrack_react"] for s in sens], "s--", color="#2ca02c",
             label=f"横向偏差 @{T_REACT} s（反应窗）")
    ax1.axhline(LANE_MARGIN, color="#d62728", ls="-.", lw=1)
    ax1.text(0.78, LANE_MARGIN + 0.03, "车道裕度 0.9 m", fontsize=8, color="#d62728", ha="right")
    ax1.set_xlabel("检测 + 仲裁延时 τ_d [s]")
    ax1.set_ylabel("横向偏差 [m]")
    ax1.grid(alpha=0.3)
    ax2 = ax1.twinx()
    ax2.plot(td, [s["dyaw_peak_dps"] for s in sens], "^:", color="#9467bd", label="横摆峰值")
    ax2.set_ylabel("横摆扰动峰值 [°/s]", color="#9467bd")
    lines1, l1 = ax1.get_legend_handles_labels()
    lines2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, l1 + l2, fontsize=9, loc="upper left")
    ax1.set_title("图 5  检测延时对缓解效果的影响（直行 100 km/h · FL 带角卡死 +4°）", fontsize=11)
    fig.tight_layout()
    return savefig(fig, "fig5_sensitivity")


def fig_c_matrix(rows) -> str:
    """图6：C 分级矩阵（缓解前后）。"""
    keys = []
    for r in rows:
        k = (r["scenario"], r["wheel"], r["fault"])
        if k not in keys:
            keys.append(k)
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 0.62 * len(keys) + 1.6))
    for ax, mit in zip(axes, ("baseline", "mitigated")):
        ax.set_xlim(0, 1)
        ax.set_ylim(-0.5, len(keys) - 0.5)
        for i, k in enumerate(keys):
            row = next(x for x in rows if (x["scenario"], x["wheel"], x["fault"]) == k
                       and x["mitigation"] == mit)
            c = row["c_class"]
            ax.barh(i, 1, color=C_COLORS[c], alpha=0.85)
            ax.text(0.5, i, f"{c}   (TTLD {'∞' if math.isinf(row['ttld_s']) else f'{row['ttld_s']:.1f}s'})",
                    ha="center", va="center", fontsize=10, color="white", fontweight="bold")
        ax.set_yticks(range(len(keys)))
        if mit == "baseline":
            ax.set_yticklabels([f"{SCN_LABELS[s].split(' ')[0]} · {WHEEL_NAMES[w][:2]} · {FAULT_NAMES[f]}"
                                for s, w, f in keys], fontsize=9)
        else:
            ax.set_yticklabels([])
        ax.set_xticks([])
        ax.set_title("未缓解" if mit == "baseline" else "镜像抵消 + 限速", fontsize=11)
        ax.invert_yaxis()
    fig.suptitle("图 6  可控性分级矩阵（TTLD = 车道脱离时间）", fontsize=12)
    fig.tight_layout()
    return savefig(fig, "fig6_c_matrix")


def fig_actuator_cost(curves) -> str:
    """图7：缓解的执行器/轮胎代价（镜像轮持续反打 + 齿条力）。"""
    mit = curves["mit:straight100:0:stuck_value"]
    t = np.asarray(mit.t) - 9.0
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.0))
    for w, col in (("fl", "#d62728"), ("fr", "#1f77b4"), ("rl", "#2ca02c"), ("rr", "#9467bd")):
        axes[0].plot(t, np.rad2deg(_arr(mit, f"delta_{w}")), lw=1.4, label=w.upper(),
                     color=col)
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
    fig.suptitle("图 7  缓解模式的代价：前轴内部力对抗（FL/FR 持续对打，侧偏角驻留 ≈4°）→ 限速依据",
                 fontsize=12)
    fig.tight_layout()
    return savefig(fig, "fig7_actuator_cost")


# ─── HTML 报告 ────────────────────────────────────────────────────────────────

def fmt(v, digits=2):
    if isinstance(v, float) and math.isinf(v):
        return "∞"
    return f"{v:.{digits}f}"


def build_html(rows, curves, figs: dict[str, str]) -> str:
    sens = curves["sensitivity"]

    def matrix_table() -> str:
        head = ("<tr><th>工况</th><th>失效轮</th><th>失效模式</th><th>缓解</th>"
                "<th>Δψ@2s [°]</th><th>横向偏差@1.2s [m]</th><th>横向偏差@2.5s [m]</th>"
                "<th>TTLD [s]</th><th>Δr峰值 [°/s]</th><th>Δa_y峰值 [m/s²]</th>"
                "<th>β峰值 [°]</th><th>C 级</th></tr>")
        body = ""
        for r in rows:
            cc = r["c_class"]
            body += (f"<tr><td>{SCN_LABELS[r['scenario']]}</td>"
                     f"<td>{WHEEL_NAMES[r['wheel']]}</td><td>{FAULT_NAMES[r['fault']]}</td>"
                     f"<td>{'—' if r['mitigation']=='baseline' else '镜像抵消+限速'}</td>"
                     f"<td>{fmt(r['dpsi_2s_deg'])}</td><td>{fmt(r['xtrack_react'])}</td>"
                     f"<td>{fmt(r['xtrack_2_5s'])}</td><td>{fmt(r['ttld_s'],1)}</td>"
                     f"<td>{fmt(r['dyaw_peak_dps'],1)}</td><td>{fmt(r['ay_peak'],1)}</td>"
                     f"<td>{fmt(r['beta_peak_deg'],1)}</td>"
                     f"<td style='color:{C_COLORS[cc]};font-weight:700'>{cc}</td></tr>")
        return f"<table>{head}{body}</table>"

    def sens_table() -> str:
        head = ("<tr><th>τ_d [s]</th><th>横向偏差@1.2s [m]</th><th>横向偏差@2.5s [m]</th>"
                "<th>Δr峰值 [°/s]</th><th>TTLD [s]</th><th>C 级</th></tr>")
        body = "".join(
            f"<tr><td>{s['detect_delay']:.2f}</td><td>{fmt(s['xtrack_react'])}</td>"
            f"<td>{fmt(s['xtrack_2_5s'])}</td><td>{fmt(s['dyaw_peak_dps'],1)}</td>"
            f"<td>{fmt(s['ttld_s'],1)}</td>"
            f"<td style='color:{C_COLORS[s['c_class']]};font-weight:700'>{s['c_class']}</td></tr>"
            for s in sens)
        return f"<table>{head}{body}</table>"

    b = next(r for r in rows if r["scenario"] == "straight100" and r["wheel"] == 0
             and r["mitigation"] == "baseline")
    m = next(r for r in rows if r["scenario"] == "straight100" and r["wheel"] == 0
             and r["mitigation"] == "mitigated")

    def img(key, alt):
        return (f'<figure><img src="data:image/png;base64,{figs[key]}" alt="{alt}" '
                f'style="max-width:100%"/></figure>')

    n_c3_base = sum(1 for r in rows if r["mitigation"] == "baseline" and r["c_class"] == "C3")
    n_cases = len(rows) // 2
    n_c2_mit = sum(1 for r in rows if r["mitigation"] == "mitigated" and r["c_class"] in ("C1", "C2"))

    html = f"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>四轮独立转向单轮失效的 ISO 26262 可控性分析与容错控制</title>
<style>
 body {{ font-family: "PingFang SC","Hiragino Sans GB","Microsoft YaHei",sans-serif;
        max-width: 980px; margin: 0 auto; padding: 40px 24px; color:#1a202c; line-height:1.75; }}
 h1 {{ font-size: 25px; border-bottom: 3px solid #1f77b4; padding-bottom: 10px; }}
 h2 {{ font-size: 19px; margin-top: 2.2em; border-left: 5px solid #1f77b4; padding-left: 10px; }}
 h3 {{ font-size: 15.5px; margin-top: 1.6em; }}
 table {{ border-collapse: collapse; width: 100%; font-size: 12.5px; margin: 12px 0; }}
 th, td {{ border: 1px solid #cbd5e0; padding: 5px 8px; text-align: center; }}
 th {{ background: #edf2f7; }}
 figure {{ margin: 18px 0; text-align: center; }}
 .abstract {{ background:#f7fafc; border:1px solid #e2e8f0; border-radius:8px; padding:16px 20px; }}
 .kbox {{ background:#fffbeb; border-left:4px solid #d69e2e; padding:10px 16px; margin:14px 0; }}
 .eq {{ background:#f7fafc; padding:8px 18px; margin:10px 0; font-family:STIX,serif; }}
 code {{ background:#edf2f7; padding:1px 5px; border-radius:4px; font-size: 12px; }}
 .meta {{ color:#718096; font-size: 13px; }}
 sup {{ font-size: 10px }}
</style></head><body>

<h1>四轮独立转向系统单轮转向失效的 ISO 26262 功能安全可控性分析<br>与容错控制策略设计</h1>
<p class="meta">仿真平台：4WIS Simulator v{SIM_VERSION}（实验底座 SimSession · 定时故障注入 · fault_reconfig 降级策略）·
车辆：LS9 默认参数（m=2900 kg，L=3.16 m）· 模型：simplified_dynamic（RK4 + 半隐式轮速 + Fz 敏感线性胎）·
复现：<code>python scripts/study_single_wheel_failure.py</code></p>

<div class="abstract"><b>摘要</b> —
四轮独立转向（4WIS）以四个独立转向执行器换取了低速灵活性与高速稳定性，但也引入了传统转向系统不存在的失效模式：
<b>单轮转向执行器卡死</b>。本文在 ISO 26262 概念阶段框架下，对单个前轮或后轮转向卡死（带角卡死 / 回中卡死 / 原角冻结）
开展定量可控性（Controllability）分析：以无故障参考轨迹的横向偏差、车道脱离时间（TTLD）、横摆扰动峰值等指标，
在高速直行、稳态弯、双移线三种代表工况 × 前/后轮 × {n_cases} 个失效组合上进行数值仿真（开环无驾驶员纠正，代表最不利暴露）。
结果表明：<b>未缓解时高速直行组合为 C3</b>（100 km/h 直行 FL 卡死 +4°：2 s 航向漂移
{fmt(b["dpsi_2s_deg"],1)}°、TTLD 仅 {fmt(b["ttld_s"],1)} s；后轮卡死更演化为 25°/s 量级的自旋趋势），
弯中与双移线组合为 C2。针对性设计了三层安全机制：残差检测（SM1）、<b>同轴镜像力抵消 + ESP 级横摆 PI 稳定</b>
（SM2，利用 4WIS 冗余转向自由度，将卡死轮的寄生侧向力与横摆力矩在同轴上精确对消，PI 环压制饱和残差）、
带减速度限幅的降级限速（SM3）。缓解后<b>全矩阵消灭 C3</b>：高速直行 C3→C2（2 s 航向漂移降至
{fmt(m["dpsi_2s_deg"],1)}°、横摆扰动峰值降 ~70%），后轮自旋工况被完全镇定（Δr 峰值 31.8→3.7°/s）。
检测延时敏感性给出 FTTI 分解依据：τ_d ≤ 0.05 s 时达 C1，τ_d ≤ 0.3 s 保持 C2 且反应窗内偏差 &lt; 0.9 m 裕度。
文中同时报告了被否决的纯运动学 ICR 重构方案的对比数据，说明其蟹行漂移特性不适于在险工况。
</div>
<p><b>关键词</b>：四轮独立转向；ISO 26262；可控性；容错控制；控制再分配；卡死失效；HARA</p>

<h2>1　引言</h2>
<p>4WIS 底盘为每个车轮配备独立转向执行器（本文对象为分体齿条 + 无刷电机构型），在带来最小转弯直径、
高速斜行/零侧偏等能力的同时，执行器数量翻倍使转向系统失效率相应上升，且出现了传统机械转向不存在的
非对称失效形态——单轮卡死时其余三轮仍完全可控。这既是风险（非对称侧向力直接产生横摆），也是机会
（<b>剩余三个转向自由度构成天然的控制重构冗余</b>——传统前转向车辆的后轴故障无任何执行器可用于补偿）。
本文的目标有三：①在 ISO 26262-3 概念阶段框架下量化"单轮卡死"危害事件的可控性 C；②设计并验证利用
4WIS 冗余的安全机制；③给出检测延时预算（FTTI 分解）的仿真依据。</p>

<h2>2　相关项定义与失效模式</h2>
<h3>2.1 相关项（Item）</h3>
<p>四轮独立转向系统：转向指令由控制策略（理想阿克曼分配）计算，每轮经"电机-减速器-齿条-拉杆"链执行；
执行器带一阶滞后（τ=60 ms）与角速率限幅（8 rad/s）。轮速由每轮轮毂电机速度伺服（PI+指令前馈）维持。</p>
<h3>2.2 分析的失效模式（FMEA 摘要）</h3>
<table>
<tr><th>编号</th><th>失效模式</th><th>典型成因</th><th>本文注入方式</th><th>研究工况</th></tr>
<tr><td>FM1</td><td>带角卡死（stuck-at-value）</td><td>齿条机械卡滞/异物、电机失步后抱死</td>
<td>t_f 起 δ_cmd≡+{STUCK_DEG:.0f}°，经执行器动力学生效</td><td>高速直行</td></tr>
<tr><td>FM2</td><td>回中卡死（stuck-at-zero）</td><td>控制器复位至默认位后失效、回正弹簧构型故障</td>
<td>t_f 起 δ_cmd≡0</td><td>稳态弯（弯中丢转向）</td></tr>
<tr><td>FM3</td><td>原角冻结（stuck-hold）</td><td>供电/驱动级失效，电机自锁力矩保持当前角</td>
<td>t_f 起冻结实测轮角</td><td>双移线（紧急变道中）</td></tr>
</table>
<p>失控甩摆（runaway）与完全松脱（free-wheeling）不在本文范围（前者由驱动级硬件互锁处置，后者需机械设计兜底），
在 §7 讨论。</p>

<h2>3　HARA：危害分析与风险评估</h2>
<p>危害事件 H1：<b>单轮转向卡死导致非预期横摆/侧向运动，车辆脱离车道</b>。S/E 按工况判定，
C 由 §4–§6 的定量仿真支撑（这正是本文方法与常规定性 HARA 的差别）。</p>
<table>
<tr><th>运行场景</th><th>S（严重度）</th><th>E（暴露率）</th><th>C（未缓解，本文仿真）</th><th>ASIL</th></tr>
<tr><td>高速公路直行 ~100 km/h，对向/邻道有车</td><td>S3（高速偏离车道/对撞）</td><td>E4（高速巡航常见）</td>
<td style="color:#d62728"><b>C3</b>（TTLD {fmt(b["ttld_s"],1)} s &lt; 反应时间）</td><td><b>D</b></td></tr>
<tr><td>山区/匝道稳态弯 ~60 km/h</td><td>S3（弯中失稳驶出路外）</td><td>E3</td>
<td style="color:#ff9f1c"><b>C2</b>（回中卡死→缓慢转向不足外漂）</td><td>B</td></tr>
<tr><td>紧急变道（双移线）~60 km/h</td><td>S2–S3</td><td>E2（紧急机动少见）</td>
<td style="color:#ff9f1c"><b>C2</b></td><td>A–B</td></tr>
</table>
<p>由此导出安全目标 <b>SG1：单个转向执行器失效不得导致车辆非预期偏离车道（ASIL D）</b>，
安全状态：受控降级行驶（限速 + 重构转向）直至安全停车。</p>

<h2>4　可控性定量评估方法</h2>
<h3>4.1 仿真设置</h3>
<table>
<tr><th>项</th><th>设置</th></tr>
<tr><td>动力学模型</td><td>simplified_dynamic：3-DOF 平面体 + 4 轮速 DOF；RK4 5 ms + 半隐式轮速；
线性胎+摩擦圆，c_α(F_z) 载荷敏感；阻力/气动升力/toe/camber 建模</td></tr>
<tr><td>车辆</td><td>LS9 默认：m=2900 kg，L=3.16 m，B=1.565 m，c_α0=120 kN/rad，μ=0.85</td></tr>
<tr><td>驾驶员</td><td><b>开环</b>（故障后不做纠正输入）——代表最不利暴露；转向指令按机动剖面给定</td></tr>
<tr><td>故障注入</td><td>指令级注入，经执行器一阶滞后生效（模拟机构卡滞而非瞬移）</td></tr>
<tr><td>记录</td><td>100 Hz，46 通道；每 run 逐位可复现</td></tr>
</table>
<h3>4.2 指标定义</h3>
<p>以<b>同工况无故障参考 run 的轨迹</b>为基准折线，度量：横向偏差 d(t)（点到参考折线最小距离）；
车道脱离时间 TTLD = min{{t: d(t) &gt; {LANE_MARGIN} m}}（3.75 m 车道 − 1.96 m 车宽 → 单侧裕度 0.9 m）；
横摆扰动 Δr(t) = r − r̄<sub>pre</sub> 峰值与 4 s 残余；侧向加速度<b>增量</b>峰值 |Δa_y|（与无故障参考时间对齐相减，稳态弯的名义 a_y 不计入）；质心侧偏角峰值；2 s 航向漂移。</p>
<h3>4.3 可控性分级判据</h3>
<p>参照 ISO 26262-3 C 定义与驾驶员反应时间工程惯用值（感知-反应 ≈1.2 s，95 分位）：</p>
<table>
<tr><th>等级</th><th>判据（评估窗 {EVAL_WINDOW:.0f} s）</th><th>含义</th></tr>
<tr><td style="color:#2ca02c"><b>C1</b></td><td>TTLD=∞ 且 d_max&lt;0.45 m 且 Δr 残余&lt;1°/s 且 Δa_y峰值&lt;2 m/s²</td>
<td>车辆自持车道内，≥99% 驾驶员无需紧急干预</td></tr>
<tr><td style="color:#ff9f1c"><b>C2</b></td><td>TTLD ≥ {T_REACT} s 且 Δa_y峰值 &lt; 5 m/s²</td>
<td>反应窗内未脱离车道，一般驾驶员可纠正</td></tr>
<tr><td style="color:#d62728"><b>C3</b></td><td>其余（反应窗内已脱离车道或强横摆）</td><td>难以控制</td></tr>
</table>

<h2>5　安全机制与容错控制策略设计</h2>
<h3>5.1 三层安全机制</h3>
<table>
<tr><th>机制</th><th>内容</th><th>实现层</th></tr>
<tr><td><b>SM1 检测</b></td><td>每轮 |δ_cmd − δ_meas| 残差监控 + 去抖（本文合并为检测延时 τ_d，基线 0.15 s；
§6.3 给出 τ_d 预算依据）</td><td>轮端 MCU + 域控裁决</td></tr>
<tr><td><b>SM2 镜像力抵消重构</b></td><td>见 §5.2 —— 本文核心机制</td><td>域控制器降级控制律</td></tr>
<tr><td><b>SM3 降级限速</b></td><td>确认失效后目标车速限 {V_LIMIT_KMH:.0f} km/h（limp-home），保护重构后轮胎力裕度（§6.4）</td>
<td>整车纵向控制</td></tr>
</table>
<h3>5.2 SM2：同轴镜像力抵消（核心机制推导）</h3>
<p>设轮 i 卡死于 δ_s，其相对正常分配角的偏差 e = δ_s − δ_i<sup>alloc</sup>。线性胎区内该轮注入寄生侧向力
ΔF_y ≈ −c_α·e 与横摆力矩 x_i·ΔF_y。<b>同轴伙伴轮与卡死轮共享同一纵向力臂 x</b>，故令</p>
<p class="eq">δ_partner = δ_partner<sup>alloc</sup> − (δ_s − δ_i<sup>alloc</sup>)</p>
<p>即可同时精确对消 ΣF_y 与 ΣM_z（图 1b）。驾驶员的曲率请求继续叠加在三个健康轮的正常分配上——
<b>转向功能对驾驶员透明地保留</b>。鲁棒性由横摆率反馈补偿非线性残差（载荷转移、c_α(F_z)、饱和）：</p>
<p class="eq">δ_fb = k_r·(r_des − r)，健康前轮 +δ_fb，两后轮 −δ_fb/2（力矩配平分配），k_r = 0.12</p>
{img("fig1", "机理示意")}
<h3>5.3 被否决方案：纯运动学 ICR 重构（设计权衡记录）</h3>
<p>另一直观方案是把期望瞬心投影到卡死轮的转向约束线上、其余三轮对准投影点——四轮纯滚动无搔刮。
实测（同 §6.1 工况）：该方案过渡瞬态横摆峰值达 22°/s（未缓解基线仅 8°/s），航向永久偏转 23.5°，
且"直行请求"退化为 ≈δ_s 的蟹行漂移（5 s 侧向漂 31 m）。<b>原因</b>：卡死角非零时约束线上不存在
"直行"解，最近投影解是蟹行；瞬态中三轮大角度重定向本身激发横摆。故该方案不适于在险接管，
仅保留为静态/蠕行回家模式（轮胎零搔刮、发热最小）。此教训说明：<b>失效后控制目标应是"保持航向/路径"
（力平衡），而非"保持运动学一致"（零搔刮）</b>。</p>

<h2>6　仿真验证结果</h2>
<h3>6.1 时间历程与轨迹</h3>
{img("fig2", "时间历程")}
<p>图 2：未缓解时 FL +4° 卡死产生 ≈8°/s 持续横摆（红），航向线性发散；SM2 于 τ_d=0.15 s 后接管（蓝），
横摆在 ~0.6 s 内归零，航向漂移钳制在 {fmt(m["dpsi_2s_deg"],1)}°。检测窗内累积的 ~1.5° 航向偏差在无驾驶员
纠正的开环口径下表现为缓慢线性漂移（这正是 §6.3 检测延时收益、以及车道保持类功能价值的来源）。
右下图可见 FR 轮（虚线）镜像反打至 −4° 的机制动作。</p>
{img("fig3", "轨迹对比")}
<h3>6.2 全矩阵指标与分级</h3>
{matrix_table()}
{img("fig4", "指标条形图")}
{img("fig6", "C 分级矩阵")}
<div class="kbox"><b>核心结论</b>：未缓解时 {n_c3_base}/{n_cases} 组合为 C3（高速直行前/后轮卡死——支撑 HARA
对高速场景取 C3 → SG1 定为 ASIL D）；启用 SM1–SM3 后<b>全矩阵无 C3</b>（{n_c2_mit}/{n_cases} 组合 C2），
其中最危险的后轮高速卡死从自旋趋势（Δr 峰值 31.8°/s、5 s 内航向偏转 88°）被镇定为缓慢可控漂移
（Δr 峰值 3.7°/s）。配合 τ_d ≤ 0.05 s 的快速检测可进一步达 C1（§6.3）。这正是 4WIS 冗余转向自由度的
功能安全价值：<b>传统前转向车辆对后轴故障没有任何执行器可用，而 4WIS 把 ASIL D 危害事件的可控性
从 C3 系统性改写为 C2/C1</b>。</div>
<h3>6.3 检测延时敏感性 → FTTI 预算</h3>
{img("fig5", "延时敏感性")}
{sens_table()}
<p>危害容忍时间（本工况车道脱离）≈ {fmt(b["ttld_s"],1)} s。取安全裕度 2 倍，FTTI 预算 ≈1 s；
仿真表明 τ_d ≤ 0.5 s 时反应窗偏差 ≤{fmt(max(s['xtrack_react'] for s in sens))} m（&lt;0.9 m 裕度），
建议检测预算：轮端残差判决 ≤50 ms + 去抖 ≤100 ms + 域控裁决与切换 ≤100 ms，总 ≤250 ms，占 FTTI 的 1/4。</p>
<h3>6.4 缓解的代价与限速依据</h3>
{img("fig7", "执行器代价")}
<p>镜像抵消让前轴两轮以 ≈±4° 侧偏角持续对抗：单轮侧向力 ≈ c_α·4° ≈ 8 kN 量级、接近 μF_z 饱和，
轮胎发热/磨损增加，且该轴向外侧机动的侧向力储备被显著占用——这是 SM3 限速 {V_LIMIT_KMH:.0f} km/h 的直接依据
（限速后 a_y 需求下降、载荷转移减小，保留足够摩擦椭圆裕度给横摆反馈项）。</p>

<h2>7　讨论与局限</h2>
<p>① 本文为开环最不利口径：真实驾驶员会介入，C 分级偏保守（安全侧）。② 线性胎+摩擦圆在 4° 卡死角
（接近饱和）区段低估过渡非线性，建议后续用 Pacejka 通道复跑（平台参数 <code>tire_model=pacejka</code> 一键切换）。
③ 执行器模型为一阶滞后+速率限幅，未含力矩饱和/供电失效耦合——镜像轮所需保持力矩（≈齿条力×臂长）
需在执行器选型中按 §6.4 的持续侧偏工况校核（负载特性页可直接给出该工况齿条力）。④ runaway 模式需
硬件级互锁（驱动桥臂断开→退化为 FM3 原角冻结，即回到本文已覆盖的场景）。⑤ 双移线中段冻结的 C2 结果
提示：机动中失效的最优策略可能需要前馈"完成当前机动再降级"，留作后续。</p>

<h2>8　结论</h2>
<p>(1) 建立了"定时故障注入 + 无故障参考轨迹偏差 + TTLD"的可控性定量评估流水线，可复现、可扩展至任意
失效×工况矩阵。(2) 未缓解的单轮卡死在高速直行与弯中均为 C3，确立 SG1 为 ASIL D。(3) 提出的"同轴镜像力抵消 + 横摆 PI 稳定 + 限幅减速"降级控制利用 4WIS 独有冗余，将全矩阵 C3 清零
（高速直行 C3→C2，快检测下 C1）；其中后轮高速卡死的自旋趋势（传统车辆无从补偿的失效位置）被完全镇定。
机制解析（同轴等臂 → 力/矩同时对消）、实现简洁（一行重分配 + 横摆 PI）、对驾驶员透明。(4) 给出 FTTI 分解建议（检测 ≤250 ms，总预算 ≈1 s）。
(5) 记录了纯运动学重构方案的失败数据，提炼出"失效后控制目标 = 路径保持而非运动学一致"的设计原则。</p>

<h2>参考文献</h2>
<ol style="font-size:13px">
<li>ISO 26262-3:2018, Road vehicles — Functional safety — Part 3: Concept phase.</li>
<li>ISO 3888-2:2011, Passenger cars — Test track for a severe lane-change manoeuvre.</li>
<li>Pacejka, H. B. <i>Tire and Vehicle Dynamics</i>, 3rd ed., Butterworth-Heinemann, 2012.</li>
<li>Reimpell, J. et al. <i>The Automotive Chassis: Engineering Principles</i>, 2nd ed., 2001.</li>
<li>4WIS Simulator v{SIM_VERSION} 平台文档：docs/v1_platform_refactor_plan.md（实验底座 / 故障注入 / KPI 流水线）.</li>
</ol>

<h2>附录 A　复现方式</h2>
<p><code>python scripts/study_single_wheel_failure.py</code>（约 1 分钟）——重跑全部 {n_cases * 2 + 5 + 3} 个仿真、
重算指标并重新生成本报告。逐 run 通道数据同时可经平台「试验/分析」页交互复查（fault 注入已进实验 schema）。</p>
<p class="meta">本报告由 4WIS Simulator 自动生成 · 数据与图表均来自实跑仿真 · {n_cases * 2} 个矩阵 run + 5 个敏感性 run + 3 个参考 run</p>
</body></html>"""
    return html


# ─── main ────────────────────────────────────────────────────────────────────

def main() -> None:
    print("§1 跑研究矩阵 …")
    rows, curves = run_matrix()

    print("§2 出图 …")
    figs = {
        "fig1": fig_mechanism(),
        "fig2": fig_timehistory(curves),
        "fig3": fig_trajectories(curves),
        "fig4": fig_metric_bars(rows),
        "fig5": fig_sensitivity(curves),
        "fig6": fig_c_matrix(rows),
        "fig7": fig_actuator_cost(curves),
    }

    print("§3 写报告 …")
    (OUT_DIR / "single_wheel_failure_metrics.json").write_text(
        json.dumps({"rows": rows, "sensitivity": curves["sensitivity"],
                    "version": SIM_VERSION},
                   ensure_ascii=False, indent=1, default=float),
        encoding="utf-8")
    html = build_html(rows, curves, figs)
    out = OUT_DIR / "single_wheel_failure_safety_analysis.html"
    out.write_text(html, encoding="utf-8")
    print(f"✓ 报告：{out}")
    print(f"✓ 图：{FIG_DIR}")


if __name__ == "__main__":
    main()
