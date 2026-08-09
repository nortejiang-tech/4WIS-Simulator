"""Trajectory traces for the report's animations.

Three things the tables cannot show:

  * **Sine-with-dwell (FMVSS 126).** The manoeuvre that separates a car that
    settles from one that keeps yawing. Open loop, so no path controller sits
    between the architecture and the result.
  * **Steady cornering attitude.** Same path, different body attitude — the
    single clearest picture of what decoupling buys.
  * **Crab and zero-radius.** Manoeuvres with no Ackermann solution, so they
    are available to L3 and to nothing below it. Not a better score on a
    shared metric: a capability the other rungs do not have at all.

Output is a JSON of body poses plus per-wheel steer angles, which the report
animates directly.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend" / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

from architectures import BY_KEY, DEFAULT_LAW, LADDER  # noqa: E402
from harness import DT, Run  # noqa: E402
from sim4wis.controller.longitudinal import apply_brake_command, apply_drive_command  # noqa: E402
from sim4wis.controller.registry import make_strategy  # noqa: E402
from sim4wis.core.derived import update_derived_outputs  # noqa: E402
from sim4wis.core.state import DriverInput, EnvironmentState, VehicleParams  # noqa: E402
from sim4wis.vehicle.multibody import MultiBodyModel  # noqa: E402

STRIDE = 10          # keep every 10th sample -> 100 Hz -> 20 ms frames


def _mp(arch_key):
    from architectures import ALGO_BY_KEY
    return dict(ALGO_BY_KEY[DEFAULT_LAW[arch_key]].mode_params)


def _pose_row(r):
    s = r.s
    return dict(t=round(r.t, 4), x=round(s.x, 4), y=round(s.y, 4),
                psi=round(s.psi, 5),
                beta=round(math.atan2(s.vy, max(abs(s.vx), 0.1)), 5),
                yaw=round(s.yaw_rate, 5), ay=round(s.ay, 4),
                delta=[round(float(q), 5) for q in s.delta])


def sine_with_dwell(arch_key, v_kmh=80.0, amp_deg=4.0, freq=0.7, mu=0.90):
    """FMVSS 126: 0.7 Hz sine, held (dwell) 500 ms at the second peak, then
    completed. The dwell is what makes it a stability test rather than a
    frequency-response point — the car is asked to hold a large attitude and
    then to give it all back at once."""
    arch = BY_KEY[arch_key]
    r = Run(arch, _mp(arch_key), v_kmh, mu)
    r.settle()
    amp = math.radians(amp_deg)
    w = 2.0 * math.pi * freq
    t_quarter = 0.25 / freq
    t_dwell = 0.5
    rows = []
    t = 0.0
    # three-quarter sine, 500 ms dwell at the trough, then the last quarter
    phase1 = 3.0 * t_quarter
    while t < phase1:
        r.steer(amp * math.sin(w * t)); r.step(); rows.append(_pose_row(r)); t += DT
    hold = amp * math.sin(w * phase1)
    t_h = 0.0
    while t_h < t_dwell:
        r.steer(hold); r.step(); rows.append(_pose_row(r)); t_h += DT; t += DT
    t2 = phase1
    while t2 < 4.0 * t_quarter:
        r.steer(amp * math.sin(w * t2)); r.step(); rows.append(_pose_row(r))
        t2 += DT; t += DT
    # free run: does it settle?
    for _ in range(int(3.0 / DT)):
        r.steer(0.0); r.step(); rows.append(_pose_row(r))

    yaw = np.array([q["yaw"] for q in rows])
    tt = np.array([q["t"] for q in rows])
    t_end_input = tt[0] + phase1 + t_dwell + t_quarter
    after = yaw[tt >= t_end_input + 1.0]
    peak = float(np.max(np.abs(yaw)))
    # FMVSS 126 pass/fail is yaw rate 1.0 s and 1.75 s after the input ends,
    # as a fraction of the peak.
    def at(dt_s):
        i = int(np.argmin(np.abs(tt - (t_end_input + dt_s))))
        return abs(float(yaw[i])) / max(peak, 1e-9)
    return dict(arch=arch_key, rows=rows[::STRIDE],
                peak_yaw_deg=math.degrees(peak),
                ratio_1s=at(1.0), ratio_175s=at(1.75),
                residual=float(np.max(np.abs(after))) / max(peak, 1e-9)
                if len(after) else float("nan"),
                max_beta_deg=math.degrees(float(np.max(np.abs(
                    [q["beta"] for q in rows])))))


def steady_attitude(arch_key, v_kmh=90.0, ay_target=4.0, mu=0.90):
    """Same speed, steer calibrated to the same lateral acceleration, so the
    only thing that differs between rungs is how the body sits in the turn."""
    from harness import calibrate_steer_for_ay
    arch = BY_KEY[arch_key]
    delta, ay = calibrate_steer_for_ay(arch, _mp(arch_key), v_kmh, ay_target, mu)
    r = Run(arch, _mp(arch_key), v_kmh, mu)
    r.settle()
    r.steer(math.radians(delta))
    rows = []
    for _ in range(int(12.0 / DT)):
        r.step(); rows.append(_pose_row(r))
    tail = rows[-int(0.5 / DT):]
    yaw_ss = abs(float(np.mean([q["yaw"] for q in tail])))
    return dict(arch=arch_key, delta_deg=delta, ay=ay, rows=rows[::STRIDE],
                beta_deg=math.degrees(float(np.mean([q["beta"] for q in tail]))),
                yaw_ss_deg=math.degrees(yaw_ss),
                R=abs(float(r.s.vx)) / max(yaw_ss, 1e-9))


def free_strategy(name, v_kmh, steering, throttle, secs, mode_params=None):
    """Run one of the platform's non-Ackermann strategies open loop."""
    p = VehicleParams()
    m = MultiBodyModel(p); m.reset()
    strat = make_strategy(name, p)
    d = DriverInput(gear=1, steering=steering, throttle=throttle,
                    mode_params=mode_params or {})
    env = EnvironmentState(mu=0.90)
    rows = []
    t = 0.0
    for _ in range(int(secs / DT)):
        c = strat.compute(d, m.state, DT)
        apply_brake_command(c, d, p)
        apply_drive_command(c, d, p, m.state)
        m.step(DT, c, env)
        update_derived_outputs(m.state, p)
        t += DT
        s = m.state
        rows.append(dict(t=round(t, 4), x=round(s.x, 4), y=round(s.y, 4),
                         psi=round(s.psi, 5), yaw=round(s.yaw_rate, 5),
                         beta=round(math.atan2(s.vy, max(abs(s.vx), 0.1)), 5),
                         ay=round(s.ay, 4),
                         delta=[round(float(q), 5) for q in s.delta]))
    return dict(strategy=name, rows=rows[::STRIDE])


def main() -> int:
    t0 = time.time()
    log = lambda *a: print(*a, flush=True)  # noqa: E731
    out = {"meta": {"generated": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
                    "stride_hz": round(1.0 / (DT * STRIDE))}}

    log("sine with dwell (FMVSS 126)")
    out["sine_dwell"] = []
    for a in LADDER:
        r = sine_with_dwell(a.key)
        log(f"  {a.key}: peak yaw {r['peak_yaw_deg']:.2f} deg/s  "
            f"1.0s ratio {r['ratio_1s']:.3f}  1.75s ratio {r['ratio_175s']:.3f}  "
            f"max beta {r['max_beta_deg']:.2f} deg")
        out["sine_dwell"].append(r)

    log("steady cornering attitude at equal a_y")
    out["steady_attitude"] = []
    for a in LADDER:
        r = steady_attitude(a.key)
        log(f"  {a.key}: delta {r['delta_deg']:.2f} deg -> a_y {r['ay']:.2f}, "
            f"beta {r['beta_deg']:+.2f} deg")
        out["steady_attitude"].append(r)

    log("non-Ackermann modes (L3 only)")
    out["crab"] = free_strategy("crab", 0, 0.55, 0.35, 6.0)
    out["zero_radius"] = free_strategy("zero_radius", 0, 1.0, 0.30, 6.0)
    log(f"  crab: {len(out['crab']['rows'])} frames, "
        f"zero_radius: {len(out['zero_radius']['rows'])} frames")

    dest = ROOT / "docs" / "reports" / "decoupling_study_traces.json"
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(out, separators=(",", ":")))
    log(f"wrote {dest} ({dest.stat().st_size / 1024:.0f} kB, {time.time() - t0:.0f} s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
