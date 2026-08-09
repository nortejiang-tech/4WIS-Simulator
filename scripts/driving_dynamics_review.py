#!/usr/bin/env python3
"""Driving-dynamics realism review — does the vehicle behave like a real car?

The unit tests answer "is the maths right". This answers a different question:
"would a driver recognise this as a car". Those are not the same — a model can
be internally consistent and still feel nothing like a vehicle.

Method
------
Standard objective manoeuvres from the vehicle-dynamics literature, chosen
because each one has a published subjective correlate: drivers reliably notice
these numbers changing.

    A1  full-throttle acceleration      ISO-style standing start
    A2  full-stop braking + pedal sweep
    A3  steady-state cornering          ISO 4138 (constant steer, speed sweep)
    A4  step steer                      ISO 7401 (transient response)
    A5  steering sensitivity            driver-facing yaw gain
    A6  straight-line stability

Each metric is scored against a **class-typical published range** for the
vehicle under test, not against measured data for this specific car. That
distinction matters: this is a plausibility review, the same category as the
project's existing `independent_reference: GAP`. Passing it means "nothing here
is obviously unlike a car"; it does not mean "validated".

Reference configuration
-----------------------
Realism is judged on the CONVENTIONAL configuration — front-axle-only steering
(`rear_wheel_steer` with rear_ratio = 0), multibody model (the only one with
roll and pitch as real DOF), torque longitudinal mode (a pedal, not a speed
request). The 4WIS strategies are the research subject; they are deliberately
*not* the realism baseline, because a zero-sideslip four-wheel-steer car is not
supposed to handle like a normal one.

Usage
-----
    backend/.venv/bin/python scripts/driving_dynamics_review.py
    backend/.venv/bin/python scripts/driving_dynamics_review.py --report docs/reports/driving_dynamics_review.md
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "backend" / "src"))

from sim4wis.controller.longitudinal import (  # noqa: E402
    apply_brake_command, apply_drive_command)
from sim4wis.controller.registry import make_strategy  # noqa: E402
from sim4wis.core.derived import update_derived_outputs  # noqa: E402
from sim4wis.core.state import (  # noqa: E402
    DriverInput, EnvironmentState, VehicleParams)
from sim4wis.vehicle.multibody import MultiBodyModel  # noqa: E402
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel  # noqa: E402

DT = 0.002          # 500 Hz — transient metrics need the resolution
G = 9.81
MU_DRY = 0.90       # the surface these reference ranges assume


# ─── metric bookkeeping ──────────────────────────────────────────────────────

@dataclass
class Metric:
    key: str
    label: str
    value: float
    unit: str
    lo: float
    hi: float
    why: str

    @property
    def ok(self) -> bool:
        return self.lo <= self.value <= self.hi

    @property
    def verdict(self) -> str:
        if self.ok:
            return "OK"
        return "LOW" if self.value < self.lo else "HIGH"


RESULTS: list[Metric] = []


def record(key, label, value, unit, lo, hi, why):
    m = Metric(key, label, float(value), unit, lo, hi, why)
    RESULTS.append(m)
    mark = "✓" if m.ok else "✗"
    print(f"  {mark} {label:38s} {value:9.3f} {unit:8s} "
          f"[{lo:g}–{hi:g}]  {m.verdict}")
    return m


# ─── rig ─────────────────────────────────────────────────────────────────────

class Rig:
    """One vehicle under test, driven step by step."""

    def __init__(self, model: str = "multibody", **over):
        base = dict(longitudinal_mode="torque")
        base.update(over)
        self.p: VehicleParams = replace(VehicleParams(), **base)
        cls = MultiBodyModel if model == "multibody" else SimplifiedDynamicModel
        self.model = cls(self.p)
        self.model.reset()
        # Front-axle-only steering: the conventional-car reference.
        self.strategy = make_strategy("rear_wheel_steer", self.p)
        self.driver = DriverInput(gear=1, mode_params={
            "rws_mode": "fixed_ratio", "rear_ratio": 0.0})
        self.env = EnvironmentState(mu=MU_DRY)

    # -- driver channels --
    def pedal(self, throttle=None, brake=None, gear=None):
        if throttle is not None:
            self.driver.throttle = float(throttle)
        if brake is not None:
            self.driver.brake = float(brake)
        if gear is not None:
            self.driver.gear = int(gear)

    def hold_speed(self, v_ms: float | None):
        """Use the pedal-independent speed channel to sit at a speed."""
        if v_ms is None:
            self.driver.mode_params.pop("speed_target_ms", None)
        else:
            self.driver.mode_params["speed_target_ms"] = float(v_ms)

    def steer_road(self, delta_rad: float | None):
        """Command a physical front-wheel angle, bypassing the feel layer."""
        if delta_rad is None:
            self.driver.mode_params.pop("steer_raw_rad", None)
        else:
            self.driver.mode_params["steer_raw_rad"] = float(delta_rad)

    def steer_wheel(self, norm: float):
        """Command through the driver feel layer (normalised steering axis)."""
        self.driver.mode_params.pop("steer_raw_rad", None)
        self.driver.steering = float(norm)

    def step(self):
        cmd = self.strategy.compute(self.driver, self.model.state, DT)
        apply_brake_command(cmd, self.driver, self.p)
        apply_drive_command(cmd, self.driver, self.p, self.model.state)
        self.model.step(DT, cmd, self.env)
        update_derived_outputs(self.model.state, self.p)
        return self.model.state

    def run(self, seconds: float):
        for _ in range(int(seconds / DT)):
            self.step()
        return self.model.state

    def settle(self, seconds: float = 6.0):
        return self.run(seconds)

    @property
    def s(self):
        return self.model.state

    def front_road_angle(self) -> float:
        return float(np.mean(self.s.delta[:2]))


# ─── A1. acceleration ────────────────────────────────────────────────────────

def a1_acceleration():
    print("\nA1  全油门加速 (torque mode, AWD 50/50)")
    r = Rig()
    r.pedal(throttle=1.0, gear=1)
    t = 0.0
    t100 = None
    t50 = t80 = None
    trace = []
    while t < 30.0:
        r.step(); t += DT
        v = r.s.vx * 3.6
        trace.append((t, v, r.s.ax))
        if t50 is None and v >= 50: t50 = t
        if t80 is None and v >= 80: t80 = t
        if t100 is None and v >= 100:
            t100 = t
            break
    record("accel_0_100", "0–100 km/h", t100 if t100 else 99.0, "s", 4.0, 8.0,
           "大型电动 SUV（~2.9 t, 400 kW）的量产区间")
    if t50 and t80:
        record("accel_50_80", "50–80 km/h 中段加速", t80 - t50, "s", 1.2, 3.5,
               "超车工况，主观上比 0–100 更能反映动力感")
    # Constant-power law: once the power bus binds, a_x·v must be constant.
    # (An earlier "low/high speed ratio" version of this was a bad metric — its
    # low-speed window sat in the torque build-up, not in a steady regime.)
    def a_at(v_target):
        near = [a for tt, v, a in trace if abs(v - v_target) < 3]
        return float(np.mean(near)) if near else 0.0
    a70, a95 = a_at(70), a_at(95)
    if a70 > 0 and a95 > 0:
        record("const_power_law", "恒功率律 (a·v 守恒)", (a95 * 95) / (a70 * 70), "-",
               0.88, 1.12,
               "功率上限生效后 a_x∝1/v；偏离 1 说明不是恒功率段或功率上限没接上")
    return r


# ─── A2. braking ─────────────────────────────────────────────────────────────

def a2_braking():
    print("\nA2  制动")
    r = Rig()
    r.hold_speed(100 / 3.6)
    r.settle(12.0)
    r.hold_speed(None)
    r.pedal(throttle=0.0, brake=1.0)
    v0, x0 = r.s.vx, r.s.x
    t = 0.0
    peak_pitch = 0.0
    while r.s.vx > 0.3 and t < 15.0:
        r.step(); t += DT
        peak_pitch = max(peak_pitch, abs(math.degrees(r.s.pitch)))
    dist = abs(r.s.x - x0)
    record("brake_100_0", "100–0 km/h 制动距离", dist, "m", 36.0, 50.0,
           f"干路面 μ={MU_DRY}；重型 SUV 量产区间（理论极限 {(v0**2)/(2*MU_DRY*G):.1f} m）")
    record("brake_pitch", "制动俯仰角峰值", peak_pitch, "deg", 0.4, 3.5,
           "点头量：过小=悬架过硬不真实，过大=晕船感")

    # pedal linearity — a driver expects deceleration ∝ pedal
    pedals, decels = [], []
    for pedal in (0.2, 0.4, 0.6, 0.8):
        rr = Rig()
        rr.hold_speed(80 / 3.6); rr.settle(10.0); rr.hold_speed(None)
        rr.pedal(throttle=0.0, brake=pedal)
        rr.run(0.6)
        acc = []
        for _ in range(int(1.0 / DT)):
            rr.step(); acc.append(rr.s.ax)
        pedals.append(pedal); decels.append(-float(np.mean(acc)))
    slope, icept = np.polyfit(pedals, decels, 1)
    pred = slope * np.array(pedals) + icept
    ss_res = float(np.sum((np.array(decels) - pred) ** 2))
    ss_tot = float(np.sum((np.array(decels) - np.mean(decels)) ** 2))
    r2 = 1.0 - ss_res / max(ss_tot, 1e-9)
    record("brake_linearity", "制动踏板线性度 R²", r2, "-", 0.95, 1.0,
           "减速度应正比于踏板；非线性=踏板难以调制")
    print(f"      踏板 {pedals} → 减速度 {[round(d,2) for d in decels]} m/s²")
    return r


# ─── A3. steady-state cornering (ISO 4138) ───────────────────────────────────

def a3_steady_cornering():
    print("\nA3  稳态圆周 (ISO 4138 定转角变车速)")
    delta = math.radians(2.0)       # fixed road-wheel angle
    rows = []
    for v_kmh in (30, 45, 60, 75, 90):
        r = Rig()
        r.steer_road(delta)
        r.hold_speed(v_kmh / 3.6)
        r.settle(14.0)
        st = r.s
        ay = abs(st.ay)
        if ay < 0.05:
            continue
        rows.append(dict(v=st.vx, ay=ay, delta=abs(r.front_road_angle()),
                         roll=abs(math.degrees(st.roll)),
                         beta=abs(math.degrees(math.atan2(st.vy, max(st.vx, 0.1))))))
    if len(rows) < 3:
        print("      ! 稳态点不足，跳过")
        return
    L = VehicleParams().wheelbase
    ay = np.array([x["ay"] for x in rows])
    dl = np.array([x["delta"] for x in rows])
    R = np.array([x["v"] ** 2 / x["ay"] for x in rows])
    # δ = L/R + K·a_y   ⇒   K = slope of (δ − L/R) vs a_y
    resid = dl - L / R
    K_rad_per_ms2 = float(np.polyfit(ay, resid, 1)[0])
    K_deg_per_g = K_rad_per_ms2 * (180.0 / math.pi) * G
    record("understeer_gradient", "不足转向梯度 K", K_deg_per_g, "deg/g", 0.5, 6.0,
           "乘用车按设计都是不足转向；<0 是过多转向（不稳定），>6 是发钝")
    roll = np.array([x["roll"] for x in rows])
    roll_grad = float(np.polyfit(ay / G, roll, 1)[0])
    record("roll_gradient", "侧倾梯度", roll_grad, "deg/g", 3.0, 8.5,
           "SUV 典型 5–8；跑车 3–4。过小=像卡丁车，过大=晕")
    for x in rows:
        print(f"      v={x['v']*3.6:5.1f} km/h  a_y={x['ay']:5.2f}  "
              f"δ={math.degrees(x['delta']):5.2f}°  roll={x['roll']:4.2f}°  β={x['beta']:4.2f}°")

    # limit cornering — max steady a_y
    best = 0.0
    rear_slip = 0.0
    front_slip = 0.0
    for d_deg in (4, 6, 8, 10, 13):
        r = Rig()
        r.steer_road(math.radians(d_deg))
        r.hold_speed(70 / 3.6)
        r.settle(14.0)
        if abs(r.s.ay) > best:
            best = abs(r.s.ay)
            sa = getattr(r.model, "slip_alpha", np.zeros(4))
            front_slip = abs(math.degrees(float(np.mean(sa[:2]))))
            rear_slip = abs(math.degrees(float(np.mean(sa[2:]))))
    record("max_lat_accel", "极限侧向加速度", best / G, "g", 0.65, 0.95,
           f"干路面 μ={MU_DRY} 的量产车区间")
    # Deliberately NOT body sideslip: β = α_r + l_r·a_y/v² puts the two terms in
    # opposition, and at 70 km/h they very nearly cancel — β reads ≈0.5° there
    # for an entirely healthy car, so it has no resolving power at that speed.
    # The rear axle's own slip angle is the unambiguous "is the back sliding"
    # number and does not depend on where the crossover happens to fall.
    record("limit_rear_slip", "极限后轴侧偏角", rear_slip, "deg", 1.0, 8.0,
           "后轴滑移量；过小=极限毫无预兆，过大=已经在甩")
    record("limit_slip_balance", "极限前/后轴侧偏比", front_slip / max(rear_slip, 1e-6),
           "-", 1.05, 3.0,
           "前轴滑得比后轴多 = 不足转向（推头先到）；<1 意味着极限时甩尾")


# ─── A4. step steer (ISO 7401) ───────────────────────────────────────────────

def a4_step_steer():
    print("\nA4  阶跃转向 (ISO 7401, 100 km/h)")
    r = Rig()
    r.hold_speed(100 / 3.6)
    r.settle(14.0)
    r.steer_road(math.radians(1.5))
    t, trace = 0.0, []
    while t < 3.0:
        r.step(); t += DT
        trace.append((t, abs(r.s.yaw_rate)))
    yaws = np.array([y for _, y in trace])
    ts = np.array([tt for tt, _ in trace])
    steady = float(np.mean(yaws[-int(0.5 / DT):]))
    peak = float(np.max(yaws))
    idx = np.argmax(yaws >= 0.9 * steady)
    t90 = float(ts[idx]) if steady > 1e-6 else 99.0
    record("yaw_t90", "横摆角速度 90% 响应时间", t90, "s", 0.10, 0.50,
           "ISO 7401；主观上就是「转向跟手程度」")
    record("yaw_overshoot", "横摆角速度超调", (peak / max(steady, 1e-9) - 1) * 100,
           "%", 0.0, 40.0, "0=过阻尼发闷，>40%=甩、需要修正")
    record("yaw_gain_steady", "稳态横摆增益 r/δ", math.degrees(steady) / 1.5,
           "(°/s)/°", 3.0, 9.0,
           "每度【前轮】角的横摆角速度。自行车模型 r/δ = v/(L+K·v²)："
           "中性车在 100 km/h 是 8.8，K=1.5 deg/g 的车约 5.3")


# ─── A5. steering sensitivity through the feel layer ─────────────────────────

def a5_steering_feel():
    print("\nA5  方向盘灵敏度（经手感层，驾驶员视角）")
    for v_kmh in (50, 100):
        r = Rig()
        r.hold_speed(v_kmh / 3.6)
        r.settle(12.0)
        r.steer_wheel(0.10)          # 10% of wheel travel
        r.settle(5.0)
        sw_deg = 0.10 * r.p.steer_wheel_range / 2.0
        gain = math.degrees(abs(r.s.yaw_rate)) / sw_deg
        record(f"sw_gain_{v_kmh}", f"方向盘横摆增益 @{v_kmh} km/h", gain,
               "(°/s)/°sw", 0.05, 0.35,
               "每度【方向盘】的横摆角速度；量产车 0.15–0.25。"
               "同时被传动比 i(v) 和不足转向梯度 K 决定")


# ─── A6. straight-line stability ─────────────────────────────────────────────

def a6_straight_line():
    print("\nA6  直线稳定性")
    r = Rig()
    r.hold_speed(120 / 3.6)
    r.settle(10.0)
    y0, psi0 = r.s.y, r.s.psi
    r.run(10.0)
    drift = abs(r.s.y - y0)
    yaw_drift = abs(math.degrees(r.s.psi - psi0))
    record("straight_drift", "120 km/h 直线 10 s 横向漂移", drift, "m", 0.0, 1.0,
           "无输入时应基本走直线；大漂移=对称性或数值问题")
    record("straight_yaw", "同上航向漂移", yaw_drift, "deg", 0.0, 2.0,
           "同上")


# ─── report ──────────────────────────────────────────────────────────────────

def write_report(path: Path):
    ok = sum(1 for m in RESULTS if m.ok)
    lines = [
        "# 驾驶动态真实性评审",
        "",
        f"> 自动生成：`scripts/driving_dynamics_review.py`　·　"
        f"{ok}/{len(RESULTS)} 项落在参考区间",
        "",
        "**这是可信性评审，不是验证。** 每项指标对照的是该车型级别的公开典型区间，",
        "不是这台车的实测数据——与项目里 `independent_reference: GAP` 属于同一范畴。",
        "通过只意味着「没有明显不像车的地方」。",
        "",
        "参考构型：前轮转向（`rear_wheel_steer`, rear_ratio=0）· 多体模型（唯一有侧倾/俯仰自由度）",
        f"· 扭矩纵向模式 · 干路面 μ={MU_DRY}。4WIS 策略是研究对象，不作真实性基准。",
        "",
        "| 指标 | 实测 | 参考区间 | 判定 | 依据 |",
        "|---|---|---|---|---|",
    ]
    for m in RESULTS:
        mark = "✓" if m.ok else f"**{m.verdict}**"
        lines.append(f"| {m.label} | {m.value:.3f} {m.unit} | "
                     f"{m.lo:g}–{m.hi:g} | {mark} | {m.why} |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\n报告已写入 {path}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", type=Path, default=None)
    args = ap.parse_args()

    print("=" * 74)
    print("驾驶动态真实性评审 — 前轮转向 / 多体 / 扭矩模式 / μ=%.2f" % MU_DRY)
    print("=" * 74)

    a1_acceleration()
    a2_braking()
    a3_steady_cornering()
    a4_step_steer()
    a5_steering_feel()
    a6_straight_line()

    bad = [m for m in RESULTS if not m.ok]
    print("\n" + "=" * 74)
    print(f"{len(RESULTS) - len(bad)}/{len(RESULTS)} 项落在参考区间")
    for m in bad:
        print(f"  ✗ {m.label}: {m.value:.3f} {m.unit} "
              f"(参考 {m.lo:g}–{m.hi:g}) — {m.why}")

    if args.report:
        write_report(args.report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
