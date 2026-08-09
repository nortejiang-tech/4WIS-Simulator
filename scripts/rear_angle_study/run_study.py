"""What is each degree of rear-steer authority worth, and where?

A separate question from the architecture ladder (scripts/decoupling_study),
and deliberately kept separate: there the angle envelope is held FIXED so the
comparison is between steering systems; here the envelope IS the variable.

The study is built around one observation. The rear angle a zero-sideslip law
asks for is

    |delta_r| = |k(v)| * |delta_f|

so demand is a surface over two axes, not a single number. |k| is near 1.0 at a
walking pace and settles around 0.6-0.7 at motorway speed, but delta_f at
motorway speed is a couple of degrees while at parking speed it is at the stop.
The product therefore collapses at speed and explodes at parking — which is why
"how many degrees of rear steer do I need" has no answer until you say for what.

Three analyses:

  1. DEMAND. Run every manoeuvre with the rear authority set wide open and
     record the peak angle the law actually asks for. Authority beyond that
     number is dead weight for that manoeuvre.
  2. VALUE. Sweep the authority from 0 to 12 deg and measure each manoeuvre's
     own metric, so the knee — the angle past which the metric stops moving —
     is measured rather than argued.
  3. MARGINAL VALUE. The derivative of (2): what the NEXT degree buys, which is
     the number an actuator spec actually turns on.

    python scripts/rear_angle_study/run_study.py
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend" / "src"))
sys.path.insert(0, str(ROOT / "scripts" / "decoupling_study"))

import numpy as np  # noqa: E402

from architectures import BY_KEY, FRONT_LIMIT_DEG  # noqa: E402
from harness import DT, Run, step_steer, steady_state  # noqa: E402
from sim4wis.controller.rws_common import zero_sideslip_ratio  # noqa: E402
from sim4wis.core.state import VehicleParams  # noqa: E402

#: Authority levels swept. Dense at the low end because that is where every
#: knee in the data turns out to be.
LIMITS = (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, 12.0)
WIDE = 40.0                 # "unlimited" rear, for the demand measurement
LAW = {"rws_mode": "speed_schedule"}

#: The manoeuvres. `kind` selects the measurement; `metric` names the number
#: whose knee we are looking for, and `better` says which way is good.
CASES = (
    dict(key="park", label="泊车掉头", label_en="parking U-turn",
         kind="circle", v=10.0, delta=FRONT_LIMIT_DEG,
         metric="turning_circle", unit="m", better="lower",
         blurb="10 km/h 全锁。反相后轮把 ICR 前移并内收，直接缩小转弯直径。"),
    dict(key="alley", label="低速窄道", label_en="low-speed manoeuvring",
         kind="circle", v=20.0, delta=25.0,
         metric="turning_circle", unit="m", better="lower",
         blurb="20 km/h、25° 前轮 —— 掉头以外最常见的低速大转角工况，"
               "如窄路调头、停车场行进间转弯。"),
    dict(key="urban", label="城市变道", label_en="urban lane change",
         kind="step", v=60.0, ay=3.0,
         metric="beta_per_g", unit="°/g", better="lower",
         blurb="60 km/h、3 m/s² 阶跃。此速度接近同相/反相的过渡点，"
               "后轮需求最小 —— 这是个反例工况。"),
    dict(key="highway", label="高速变道", label_en="highway lane change",
         kind="step", v=120.0, ay=4.0,
         metric="beta_per_g", unit="°/g", better="lower",
         blurb="120 km/h、4 m/s² 阶跃。同相后轮压侧偏角，是高速稳定性的主战场。"),
    dict(key="evade", label="高速紧急规避", label_en="high-speed evasion",
         kind="step", v=120.0, ay=7.0,
         metric="beta_per_g", unit="°/g", better="lower",
         blurb="120 km/h、7 m/s² 阶跃 —— 逼近极限的规避动作。"
               "前轮转角比常规变道大得多，后轮需求随之放大。"),
    dict(key="cruise", label="高速稳态", label_en="high-speed steady cornering",
         kind="steady", v=120.0, delta=3.0,
         metric="beta_deg", unit="°", better="lower",
         blurb="120 km/h、定转角 3° 稳态回转，量高速巡航姿态。"),
)


def arch_with_limit(deg: float):
    """The L2 rung with its rear authority overridden.

    L2 is the carrier because it is the rung where the rear angle is the only
    thing in question: steer-by-wire front, and an actuator fast enough that
    bandwidth is not what limits the answer.
    """
    return replace(BY_KEY["L2"], rear_limit_deg=float(deg))


# ── measurement kinds ──────────────────────────────────────────────────────

def measure_circle(limit_deg, case, want_demand=False):
    arch = arch_with_limit(WIDE if want_demand else limit_deg)
    ss = steady_state(arch, LAW, case["v"], case["delta"], hold=12.0)
    p = VehicleParams()
    R = abs(ss["vx"] / ss["yaw"]) if abs(ss["yaw"]) > 1e-6 else float("inf")
    R_wall = math.hypot(R + p.track_front / 2.0, p.wheelbase / 2.0 + 0.9)
    return dict(turning_circle=2.0 * R_wall, R_cg=R,
                delta_r=abs(math.degrees(ss["delta_r"])),
                delta_f=abs(math.degrees(ss["delta_f"])),
                ay=abs(ss["ay"]))


def measure_steady(limit_deg, case, want_demand=False):
    arch = arch_with_limit(WIDE if want_demand else limit_deg)
    ss = steady_state(arch, LAW, case["v"], case["delta"], hold=10.0)
    b = math.degrees(ss["beta"])
    return dict(beta_deg=abs(b), beta_signed=b,
                delta_r=abs(math.degrees(ss["delta_r"])),
                delta_f=abs(math.degrees(ss["delta_f"])),
                ay=abs(ss["ay"]), yaw=abs(ss["yaw"]))


def _delta_for_ay(arch, v, ay_target):
    """Bisect the front angle onto a target lateral acceleration.

    Done per authority level, not once: clipping the rear changes the steering
    gain, so a single shared angle would compare different operating points.
    """
    lo, hi, best = 0.1, 20.0, (1.0, 0.0)
    for _ in range(11):
        mid = 0.5 * (lo + hi)
        ay = abs(steady_state(arch, LAW, v, mid, hold=6.0)["ay"])
        best = (mid, ay)
        if abs(ay - ay_target) < 0.03:
            break
        lo, hi = (mid, hi) if ay < ay_target else (lo, mid)
    return best


def measure_step(limit_deg, case, want_demand=False):
    arch = arch_with_limit(WIDE if want_demand else limit_deg)
    delta, ay = _delta_for_ay(arch, case["v"], case["ay"])
    r = step_steer(arch, LAW, case["v"], delta, secs=4.0, trace=True)
    dr = np.abs(r["trace"]["delta_r"])
    return dict(beta_per_g=abs(r["beta_per_g"]), beta_signed=r["beta_per_g"],
                yaw_t90=r["yaw_t90"],
                yaw_overshoot=r["yaw_overshoot"], beta_peak_deg=math.degrees(r["beta_peak"]),
                delta_r=float(np.max(dr)), delta_f=delta, ay=ay)


KIND = {"circle": measure_circle, "steady": measure_steady, "step": measure_step}


def job(args):
    ci, limit, demand = args
    case = CASES[ci]
    out = KIND[case["kind"]](limit, case, want_demand=demand)
    out.update(case=case["key"], limit=limit, demand=demand)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "docs" / "reports" /
                                         "rear_angle_study_data.json"))
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 4) - 1))
    args = ap.parse_args()
    t0 = time.time()
    log = lambda *a: print(*a, flush=True)  # noqa: E731
    pool = ProcessPoolExecutor(max_workers=args.jobs)
    stage_dir = Path(args.out).with_suffix("")
    stage_dir.mkdir(parents=True, exist_ok=True)

    # 1) Demand: how much rear angle does each manoeuvre actually ask for?
    log(f"[1/3] demand — rear authority open to {WIDE:.0f} deg")
    demand = list(pool.map(job, [(i, WIDE, True) for i in range(len(CASES))]))
    (stage_dir / "demand.json").write_text(json.dumps(demand))
    for c, r in zip(CASES, demand):
        log(f"      {c['label']:<10} v={c['v']:5.0f} km/h  δ_f={r['delta_f']:5.2f}°"
            f"  →  需求 δ_r = {r['delta_r']:6.2f}°")

    # 2) Value: sweep the authority.
    log(f"[2/3] value — sweeping rear authority {LIMITS[0]:.0f}..{LIMITS[-1]:.0f} deg "
        f"({len(CASES) * len(LIMITS)} runs)")
    sweep = list(pool.map(job, [(i, L, False)
                                for i in range(len(CASES)) for L in LIMITS]))
    (stage_dir / "sweep.json").write_text(json.dumps(sweep))
    pool.shutdown()

    # 3) Knee + marginal value.
    knees = {}
    for c in CASES:
        rows = sorted((r for r in sweep if r["case"] == c["key"]),
                      key=lambda r: r["limit"])
        vals = [r[c["metric"]] for r in rows]
        base, best = vals[0], (min(vals) if c["better"] == "lower" else max(vals))
        span = base - best if c["better"] == "lower" else best - base
        # Knee: the smallest authority reaching 95% of the total available gain.
        #
        # The metric for the sideslip cases is |beta|, deliberately. beta itself
        # crosses zero as authority grows — the law over-corrects past the
        # zero-sideslip point — so the raw signed value has a minimum in the
        # middle of the sweep and "more is better" is simply false for it. The
        # signed value is carried alongside so the report can show the crossing
        # rather than hide it behind an absolute value.
        knee = rows[-1]["limit"]
        if abs(span) > 1e-9:
            for r, v in zip(rows, vals):
                got = (base - v) if c["better"] == "lower" else (v - base)
                if got >= 0.95 * span:
                    knee = r["limit"]
                    break
        marg = []
        for i in range(1, len(rows)):
            d = rows[i]["limit"] - rows[i - 1]["limit"]
            gain = (vals[i - 1] - vals[i]) if c["better"] == "lower" else (vals[i] - vals[i - 1])
            marg.append(dict(deg=rows[i]["limit"], per_deg=gain / max(d, 1e-9)))
        knees[c["key"]] = dict(base=base, best=best, span=span, knee=knee,
                               pct_of_12=(abs(span) and
                                          100.0 * abs(base - vals[-1]) / abs(span)),
                               marginal=marg)
        log(f"      {c['label']:<10} {c['metric']:<12} "
            f"{base:8.3f} → {best:8.3f} {c['unit']:<4}  拐点 {knee:4.0f}°")

    try:
        commit = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"],
                                         cwd=ROOT, text=True).strip()
    except Exception:
        commit = "unknown"

    p = VehicleParams()
    kcurve = [dict(v=v, k=zero_sideslip_ratio(p, v / 3.6))
              for v in range(0, 165, 5)]

    out = dict(
        meta=dict(commit=commit, generated=time.strftime("%Y-%m-%d %H:%M:%S %Z"),
                  wall_seconds=round(time.time() - t0, 1),
                  dt=DT, model="multibody", carrier="L2 (SBW + RWS)",
                  front_limit_deg=FRONT_LIMIT_DEG, wide_deg=WIDE,
                  law="speed_schedule (analytic zero-sideslip ratio)",
                  vehicle=dict(mass=p.mass, wheelbase=p.wheelbase,
                               cg_to_front=p.cg_to_front,
                               track_front=p.track_front)),
        limits=list(LIMITS),
        cases=[{k: v for k, v in c.items()} for c in CASES],
        demand=demand, sweep=sweep, knees=knees, k_curve=kcurve,
    )
    dest = Path(args.out)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(out, indent=1))
    log(f"\nwrote {dest} ({dest.stat().st_size / 1024:.0f} kB, {time.time() - t0:.0f} s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
