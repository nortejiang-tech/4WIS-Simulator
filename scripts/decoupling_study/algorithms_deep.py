"""Deep characterisation of the six rear-steer control laws.

The main study measures the laws at one speed on one metric. That is enough to
rank them and not nearly enough to explain them, so this adds the four things a
reader has to see before believing a ranking:

  * **Speed dependence.** Every law's ratio is a function of speed, and two of
    them change SIGN across the sweep. A ranking taken at one speed can invert.
  * **Control effort.** How much rear angle a law spends, and how fast it moves
    it. Two laws reaching the same yaw response are not equivalent if one needs
    twice the actuator.
  * **Noise sensitivity — measured, not asserted.** The transient law
    differentiates the steering signal. The main report claimed that this
    amplifies steering-sensor noise and admitted the claim was untested in a
    noiseless simulation. This module injects noise on the steering channel and
    measures it.
  * **Linear theory vs the vehicle.** The zero-sideslip ratio and the reference
    yaw rate are closed from a constant-cornering-stiffness bicycle model. The
    report leans on that theory to explain the laws, so it has to show where the
    theory stops matching the multibody vehicle it is steering.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend" / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

from architectures import ALGORITHMS, ALGO_BY_KEY, BY_KEY  # noqa: E402
from harness import DT, Run, calibrate_steer_for_ay, step_steer  # noqa: E402
from sim4wis.controller.rws_common import (  # noqa: E402
    axle_cornering_stiffness, reference_yaw_rate, zero_sideslip_ratio,
)
from sim4wis.core.state import VehicleParams  # noqa: E402

CARRIER = "L2"                 # fixed hardware — the law is the only variable
SPEEDS = (40.0, 60.0, 100.0, 140.0)
AY = 3.0                       # m/s^2, reachable at all four speeds
ALGO_KEYS = [a.key for a in ALGORITHMS]

NOISE_SIGMA_DEG = 0.10         # steering-angle sensor noise, 1 sigma
NOISE_BW_HZ = 12.0             # band limit — above this a real column filters
NOISE_SECONDS = 12.0
NOISE_SEED = 20260810


def _mp(algo_key):
    return dict(ALGO_BY_KEY[algo_key].mode_params)


# ── 1. speed sweep ─────────────────────────────────────────────────────────

def job_speed(args):
    algo, v = args
    arch = BY_KEY[CARRIER]
    delta, ay = calibrate_steer_for_ay(arch, _mp(algo), v, AY, iters=10)
    r = step_steer(arch, _mp(algo), v, delta, secs=4.0, trace=True)
    tr = r["trace"]
    dr = np.array(tr["delta_r"])
    t = np.array(tr["t"])
    ddr = np.gradient(dr, t) if len(t) > 2 else np.zeros_like(dr)
    return dict(
        algo=algo, v=v, delta_deg=delta, ay_cal=ay,
        yaw_t90=r["yaw_t90"], yaw_overshoot=r["yaw_overshoot"],
        yaw_peak_time=r["yaw_peak_time"], yaw_gain=r["yaw_gain"],
        beta_per_g=r["beta_per_g"], ay_ss=r["ay_ss"], tb=r["tb"],
        # control effort
        dr_max=float(np.max(np.abs(dr))),
        dr_ss=float(np.mean(dr[-int(0.5 / DT):])),
        dr_rate_max=float(np.max(np.abs(ddr))),
        dr_over_df=float(np.mean(dr[-int(0.5 / DT):])) / max(delta, 1e-9),
    )


# ── 2. noise sensitivity ───────────────────────────────────────────────────

def _pink_ish(n, rng, bw_hz):
    """Band-limited noise: white, then a one-pole low pass at bw_hz.

    A steering-angle sensor's raw noise is broadband, but the column and the
    ECU's own anti-alias filter roll it off well before the control law sees
    it. Feeding unfiltered white noise would make any differentiating law look
    arbitrarily bad — the band limit is what makes the comparison fair.
    """
    w = rng.normal(0.0, 1.0, n)
    a = DT / (1.0 / (2 * math.pi * bw_hz) + DT)
    out = np.empty(n)
    acc = 0.0
    for i in range(n):
        acc += a * (w[i] - acc)
        out[i] = acc
    # restore unit variance after filtering
    s = float(np.std(out))
    return out / s if s > 1e-12 else out


def job_noise(args):
    (algo,) = args
    arch = BY_KEY[CARRIER]
    r = Run(arch, _mp(algo), 100.0)
    r.settle()
    n = int(NOISE_SECONDS / DT)
    rng = np.random.default_rng(NOISE_SEED)      # same sequence for every law
    noise = _pink_ish(n, rng, NOISE_BW_HZ) * math.radians(NOISE_SIGMA_DEG)
    dr, yaw, ay = [], [], []
    for i in range(n):
        r.steer(float(noise[i]))                  # straight ahead + sensor noise
        r.step()
        s = r.s
        dr.append(float(np.mean(s.delta[2:])))
        yaw.append(float(s.yaw_rate))
        ay.append(float(s.ay))
    dr = np.array(dr)
    ddr = np.gradient(dr, DT)
    return dict(
        algo=algo,
        in_rms_deg=float(np.std(np.degrees(noise))),
        dr_rms_deg=float(np.std(np.degrees(dr))),
        dr_rate_rms_dps=float(np.std(np.degrees(ddr))),
        dr_p2p_deg=float(np.degrees(np.max(dr) - np.min(dr))),
        yaw_rms_dps=float(np.std(np.degrees(yaw))),
        ay_rms=float(np.std(ay)),
        # the number that matters: how much the law multiplies the sensor noise
        # on its way to the rear actuator
        amplification=float(np.std(np.degrees(dr)) /
                            max(np.std(np.degrees(noise)), 1e-9)),
    )


# ── 3. sine with dwell, per law ────────────────────────────────────────────

def job_dwell(args):
    (algo,) = args
    arch = BY_KEY[CARRIER]
    r = Run(arch, _mp(algo), 80.0)
    r.settle()
    amp, freq = math.radians(4.0), 0.7
    w = 2 * math.pi * freq
    tq = 0.25 / freq
    yaw, beta, dr, ts = [], [], [], []
    t = 0.0

    def rec():
        s = r.s
        yaw.append(float(s.yaw_rate))
        beta.append(math.atan2(s.vy, max(abs(s.vx), 0.1)))
        dr.append(float(np.mean(s.delta[2:])))
        ts.append(t)

    while t < 3 * tq:
        r.steer(amp * math.sin(w * t)); r.step(); t += DT; rec()
    hold, th = amp * math.sin(w * 3 * tq), 0.0
    while th < 0.5:
        r.steer(hold); r.step(); t += DT; th += DT; rec()
    t2 = 3 * tq
    while t2 < 4 * tq:
        r.steer(amp * math.sin(w * t2)); r.step(); t2 += DT; t += DT; rec()
    t_end = t
    for _ in range(int(3.0 / DT)):
        r.steer(0.0); r.step(); t += DT; rec()

    yaw = np.degrees(np.array(yaw))
    ts = np.array(ts)
    peak = float(np.max(np.abs(yaw)))

    def at(dt_s):
        i = int(np.argmin(np.abs(ts - (t_end + dt_s))))
        return abs(float(yaw[i])) / max(peak, 1e-9)

    return dict(algo=algo, peak_yaw=peak,
                ratio_1s=at(1.0), ratio_175s=at(1.75),
                max_beta=float(np.degrees(np.max(np.abs(beta)))),
                dr_max=float(np.degrees(np.max(np.abs(dr)))))


# ── 4. linear theory vs the vehicle ────────────────────────────────────────

def theory_table(p: VehicleParams):
    """Closed-form bicycle-model predictions the laws are built on."""
    L, a = float(p.wheelbase), float(p.cg_to_front)
    b = L - a
    m = float(p.mass)
    # Current controller and vehicle use the same axle stiffness scales.
    cf, cr = axle_cornering_stiffness(p)
    sf, sr = float(p.tire_c_alpha_front_scale), float(p.tire_c_alpha_rear_scale)
    cf_true, cr_true = cf, cr

    def K_us(cff, crr):
        """Understeer gradient, deg/g. K = (m/L)*(b/Cf - a/Cr)."""
        return m / L * (b / cff - a / crr) * (180 / math.pi) * 9.80665

    def k_zs(u, cff, crr):
        return (-b + a * m * u * u / (crr * L)) / (a + b * m * u * u / (cff * L))

    rows = []
    for v in range(10, 165, 5):
        u = v / 3.6
        rows.append(dict(
            v=v,
            k=zero_sideslip_ratio(p, u),          # as the law computes it
            k_true=k_zs(u, cf_true, cr_true),     # as it should be
            r_ref_per_deg=math.degrees(reference_yaw_rate(p, u, math.radians(1.0))),
            yaw_gain_front_only=u / (L + (m * (b / cf_true - a / cr_true) / L) * u * u),
        ))
    return dict(K_law=K_us(cf, cr), K_true=K_us(cf_true, cr_true),
                v_crossover_law=math.sqrt(b * cr * L / (a * m)) * 3.6,
                v_crossover_true=math.sqrt(b * cr_true * L / (a * m)) * 3.6,
                cf=cf, cr=cr, cf_true=cf_true, cr_true=cr_true,
                scale_f=sf, scale_r=sr, a=a, b=b, L=L, m=m, rows=rows)


def job_corrected(args):
    """The decisive test of the plant-model hypothesis.

    If the zero-sideslip law overshoots because its k(v) is closed from
    Cf = Cr while the vehicle runs a 0.80/1.20 axle split, then feeding the
    SAME law a k-curve recomputed from the true stiffnesses must drive the
    steady sideslip to zero. Nothing else changes — same law, same hardware,
    same manoeuvre, one table of numbers swapped.
    """
    v, corrected = args
    p = VehicleParams()
    arch = BY_KEY[CARRIER]
    mp = {"rws_mode": "speed_schedule"}
    L, a, m = float(p.wheelbase), float(p.cg_to_front), float(p.mass)
    b = L - a
    cf, cr = axle_cornering_stiffness(p)
    if not corrected:
        # Explicit historical counterfactual; never double-apply scales to the
        # current helper's result or call the fixed current law "uncorrected".
        cf = cr = 2.0 * float(p.tire_c_alpha)
    mp["k_curve"] = [
        [s, (-b + a * m * (s / 3.6) ** 2 / (cr * L))
            / (a + b * m * (s / 3.6) ** 2 / (cf * L))]
        for s in (0, 20, 40, 60, 90, 130, 200)]
    delta, ay = calibrate_steer_for_ay(arch, mp, v, AY, iters=10)
    r = step_steer(arch, mp, v, delta, secs=4.0)
    return dict(v=v, corrected=corrected, delta_deg=delta,
                beta_per_g=r["beta_per_g"], yaw_t90=r["yaw_t90"],
                yaw_overshoot=r["yaw_overshoot"], yaw_gain=r["yaw_gain"])


def main() -> int:
    t0 = time.time()
    log = lambda *a: print(*a, flush=True)  # noqa: E731
    pool = ProcessPoolExecutor(max_workers=max(1, (os.cpu_count() or 4) - 1))

    log(f"[1/4] speed sweep — {len(ALGO_KEYS)} laws x {len(SPEEDS)} speeds "
        f"at a_y = {AY} m/s^2")
    speed = list(pool.map(job_speed, [(a, v) for a in ALGO_KEYS for v in SPEEDS]))
    for r in sorted(speed, key=lambda r: (r["algo"], r["v"])):
        log(f"      {r['algo']:<13} {r['v']:5.0f} km/h  δ_f {r['delta_deg']:5.2f}°"
            f"  T90 {r['yaw_t90']*1000:5.0f} ms  超调 {r['yaw_overshoot']:5.1f}%"
            f"  β/g {r['beta_per_g']:+6.2f}  δ_r/δ_f {r['dr_over_df']:+6.3f}")

    log(f"[2/4] noise sensitivity — {NOISE_SIGMA_DEG}° RMS on the steering "
        f"channel, {NOISE_BW_HZ:.0f} Hz band limit")
    noise = list(pool.map(job_noise, [(a,) for a in ALGO_KEYS]))
    for r in sorted(noise, key=lambda r: r["amplification"]):
        log(f"      {r['algo']:<13} 后轮 RMS {r['dr_rms_deg']:6.3f}°"
            f"  放大 {r['amplification']:6.2f}x"
            f"  后轮角速率 RMS {r['dr_rate_rms_dps']:7.2f} °/s"
            f"  横摆 RMS {r['yaw_rms_dps']:.4f} °/s")

    log("[3/4] sine with dwell per law")
    dwell = list(pool.map(job_dwell, [(a,) for a in ALGO_KEYS]))
    for r in dwell:
        log(f"      {r['algo']:<13} 峰值横摆 {r['peak_yaw']:6.2f} °/s"
            f"  1.0s {r['ratio_1s']:.3f}  1.75s {r['ratio_175s']:.3f}"
            f"  max β {r['max_beta']:5.2f}°  max δ_r {r['dr_max']:5.2f}°")
    pool.shutdown()

    log("[4/5] plant-model correction — does fixing k(v) fix the sideslip?")
    pool2 = ProcessPoolExecutor(max_workers=max(1, (os.cpu_count() or 4) - 1))
    corr = list(pool2.map(job_corrected,
                          [(v, c) for v in SPEEDS for c in (False, True)]))
    pool2.shutdown()
    for r in sorted(corr, key=lambda r: (r["v"], r["corrected"])):
        log(f"      {r['v']:5.0f} km/h  {'修正' if r['corrected'] else '默认'}"
            f"  δ_f {r['delta_deg']:5.2f}°  β/g {r['beta_per_g']:+6.2f}"
            f"  T90 {r['yaw_t90']*1000:5.0f} ms")

    log("[5/5] linear bicycle theory")
    th = theory_table(VehicleParams())
    log(f"      K（控制律假设 Cf=Cr）= {th['K_law']:.3f} deg/g")
    log(f"      K（车辆真实轴刚度）  = {th['K_true']:.3f} deg/g")
    log(f"      过渡速度 控制律 {th['v_crossover_law']:.1f} / "
        f"真实 {th['v_crossover_true']:.1f} km/h")

    out = dict(meta=dict(generated=time.strftime("%Y-%m-%d %H:%M:%S %Z"),
                         carrier=CARRIER, ay=AY, speeds=list(SPEEDS),
                         noise_sigma_deg=NOISE_SIGMA_DEG,
                         noise_bw_hz=NOISE_BW_HZ, noise_seconds=NOISE_SECONDS,
                         wall_seconds=round(time.time() - t0, 1)),
               speed=speed, noise=noise, dwell=dwell, corrected=corr, theory=th)
    dest = ROOT / "docs" / "reports" / "decoupling_algorithms_data.json"
    dest.write_text(json.dumps(out, indent=1))
    log(f"\nwrote {dest} ({dest.stat().st_size / 1024:.0f} kB, "
        f"{time.time() - t0:.0f} s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
