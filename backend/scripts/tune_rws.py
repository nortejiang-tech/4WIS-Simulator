"""Headless RWS evaluation / gain-tuning harness.

Runs each rear-wheel-steering strategy through standard open-loop maneuvers at
representative speeds on the simplified-dynamic model and reports the metrics the
ScorePanel uses, so we can compare against the baselines (front-only Ackermann,
constant counter-phase) and tune default gains before release.

Usage:
    python scripts/tune_rws.py            # comparison table
    python scripts/tune_rws.py --sweep    # gain sweeps for ②③④
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from sim4wis.controller.registry import make_strategy
from sim4wis.core.state import DriverInput, EnvironmentState, VehicleParams
from sim4wis.environment.disturbance import Scene
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel

DT = 0.005


def _env() -> EnvironmentState:
    sc = Scene()
    return EnvironmentState(scene=sc, mu=sc.base_mu)


def run_step(strategy_name: str, speed_kmh: float, amp: float = 0.3,
             mode_params: dict | None = None, hold_s: float = 4.0) -> dict:
    """Warm up to `speed_kmh` straight, then apply a step steering input.

    Returns metrics over the maneuver window.
    """
    p = VehicleParams()
    m = SimplifiedDynamicModel(p)
    env = _env()
    strat = make_strategy(strategy_name, p)
    v = speed_kmh / 3.6
    throttle = max(-1.0, min(1.0, v / p.v_max))
    mp = dict(mode_params or {})

    # Warm up straight to target speed.
    for _ in range(int(4.0 / DT)):
        cmd = strat.compute(DriverInput(throttle=throttle, steering=0.0, mode_params=mp), m.state)
        m.step(DT, cmd, env)

    # Step maneuver.
    n = int(hold_s / DT)
    yaw = np.empty(n)
    beta = np.empty(n)
    vy = np.empty(n)
    for i in range(n):
        d = DriverInput(throttle=throttle, steering=amp, mode_params=mp)
        cmd = strat.compute(d, m.state)
        m.step(DT, cmd, env)
        s = m.state
        yaw[i] = s.yaw_rate
        vy[i] = s.vy
        beta[i] = math.degrees(math.atan2(s.vy, max(abs(s.vx), 0.1)))

    tail = slice(int(n * 0.7), n)  # last 30 % = steady state
    yaw_ss = float(np.mean(yaw[tail]))
    yaw_peak = float(np.max(np.abs(yaw)))
    overshoot = (yaw_peak / abs(yaw_ss) - 1.0) * 100 if abs(yaw_ss) > 1e-6 else 0.0
    return {
        "yaw_ss_dps": math.degrees(yaw_ss),
        "yaw_overshoot_pct": overshoot,
        "beta_ss_deg": float(np.mean(beta[tail])),
        "beta_peak_deg": float(np.max(np.abs(beta))),
        "vy_peak_kmh": float(np.max(np.abs(vy))) * 3.6,
        "stable": bool(np.all(np.isfinite(yaw)) and yaw_peak < 5.0),
    }


# All RWS modes are now the single unified strategy; mode picked via rws_mode.
RWS = "rear_wheel_steer"
STRATS = [
    ("ackermann", "前轮(基准)", {}),
    (RWS, "定比 -0.5", {"rws_mode": "fixed_ratio", "rear_ratio": -0.5}),
    (RWS, "①车速调度", {"rws_mode": "speed_schedule"}),
    (RWS, "③横摆反馈", {"rws_mode": "yaw_feedback"}),
    (RWS, "②稳态+瞬态", {"rws_mode": "transient"}),
    (RWS, "④模型跟踪", {"rws_mode": "model_following"}),
]


# Speed-appropriate step amplitude so the linear tyre model stays in its valid
# (sub-saturation) range — a 10° step at 120 km/h is well beyond grip.
def _amp_for(speed_kmh: float) -> float:
    if speed_kmh <= 40:
        return 0.30
    if speed_kmh <= 80:
        return 0.15
    return 0.06


def comparison() -> None:
    for spd in (30.0, 70.0, 120.0):
        amp = _amp_for(spd)
        print(f"\n=== 阶跃转向 amp={amp} @ {spd:.0f} km/h (simplified_dynamic) ===")
        print(f"{'策略':<14}{'横摆稳态°/s':>11}{'超调%':>8}{'侧偏稳态°':>10}"
              f"{'侧偏峰°':>9}{'vy峰km/h':>10}{'稳定':>6}")
        for name, label, mp in STRATS:
            r = run_step(name, spd, amp=amp, mode_params=mp)
            print(f"{label:<14}{r['yaw_ss_dps']:>11.2f}{r['yaw_overshoot_pct']:>8.1f}"
                  f"{r['beta_ss_deg']:>10.2f}{r['beta_peak_deg']:>9.2f}"
                  f"{r['vy_peak_kmh']:>10.2f}{'✓' if r['stable'] else '✗':>6}")


def sweep() -> None:
    SA = _amp_for(120.0)
    print("\n### ③ yaw_feedback — g2 扫描 @120km/h (目标: 降超调/侧偏, 稳定) ###")
    for g2 in (0.0, 0.2, 0.35, 0.5, 0.8):
        r = run_step(RWS, 120.0, amp=SA, mode_params={"rws_mode": "yaw_feedback", "g1": -0.10, "g2": g2})
        print(f"  g2={g2:>4}: yaw_ss={r['yaw_ss_dps']:6.2f}°/s overshoot={r['yaw_overshoot_pct']:6.1f}% "
              f"beta_ss={r['beta_ss_deg']:6.2f}° beta_peak={r['beta_peak_deg']:5.2f}° "
              f"{'✓' if r['stable'] else '✗'}")

    print("\n### ② transient — c 扫描 @120km/h (目标: 改善转入瞬态, 不失稳) ###")
    for c in (0.0, 0.06, 0.12, 0.2, 0.35):
        r = run_step(RWS, 120.0, amp=SA, mode_params={"rws_mode": "transient", "c_transient": c})
        print(f"  c={c:>5}: yaw_ss={r['yaw_ss_dps']:6.2f}°/s overshoot={r['yaw_overshoot_pct']:6.1f}% "
              f"beta_ss={r['beta_ss_deg']:6.2f}° beta_peak={r['beta_peak_deg']:5.2f}° "
              f"{'✓' if r['stable'] else '✗'}")

    print("\n### ④ model_following — g_yaw 扫描 @120km/h (目标: 跟踪参考横摆, β≈0) ###")
    for g in (0.0, 0.1, 0.2, 0.4, 0.7):
        r = run_step(RWS, 120.0, amp=SA, mode_params={"rws_mode": "model_following", "g_yaw": g})
        print(f"  g_yaw={g:>4}: yaw_ss={r['yaw_ss_dps']:6.2f}°/s overshoot={r['yaw_overshoot_pct']:6.1f}% "
              f"beta_ss={r['beta_ss_deg']:6.2f}° beta_peak={r['beta_peak_deg']:5.2f}° "
              f"{'✓' if r['stable'] else '✗'}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", action="store_true")
    args = ap.parse_args()
    comparison()
    if args.sweep:
        sweep()
