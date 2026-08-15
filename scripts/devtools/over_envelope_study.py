"""Direction 6 — over-envelope study: how big a peak torque survives the
deep-slip blow-back?

A 5° step at 60 km/h commands ~0.8 g — beyond the tyre envelope. In deep
slip the aligning torque reverses: it pushes the wheel AWAY from centre
instead of restoring it, and the corner actuator meets the reversal as a
destabilising load. The 40 N·m actuator was pinned at its limit and the
wheel blown to the steer stops (validation-round finding); the 120 N·m
default was sized from full-lock parking (~104 N·m). This study answers the
question the sizing module cannot (it is quasi-static): at 5°@60 km/h, what
peak torque keeps the wheel within the architecture's own 0.05 rad
deviation limit — and what does the blow-back actually cost?

Sweeps `plant_peak_torque_nm` with the production controller shape
(pid_single + FF) and the open-loop reference, and reports per-peak:
max/end deviation, time past the 0.05 rad limit, and the reversal's peak
rack force. Deterministic; ~seconds per cell.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend" / "src"))

import numpy as np  # noqa: E402

from sim4wis.experiment.schema import Experiment  # noqa: E402
from sim4wis.experiment.session import SimSession  # noqa: E402

FF_STACK = {"ff": {"blocks": [
    {"type": "rack_force", "gain": 1.0, "lowpass_hz": 15},
    {"type": "velocity", "damping": 4.0, "inertia": 0.6},
    {"type": "friction", "friction_nm": 0.5},
]}}

CONTROLLERS = [("open_loop", {}), ("pid_single", FF_STACK)]
PEAKS_NM = [40.0, 60.0, 80.0, 100.0, 120.0]
#: 40 N·m anchors the historical finding (the original default was blown to
#: the steer stops); the sweep crosses the threshold where the blow-back
#: disappears.
DEVIATION_LIMIT = 0.05  # rad — the by-wire architecture's angle_deviation limit


def build_experiment(controller: str, kwargs: dict, peak_nm: float) -> Experiment:
    raw = {
        "name": f"over_envelope_{controller}_{int(peak_nm)}",
        "strategy": "ideal_ackermann",
        "record_hz": 200,
        "vehicle": {"overrides": {
            "steering_system": {
                "enabled": True,
                "architecture": "4wis",
                "angle_control": {
                    "enabled": True,
                    "controller": controller,
                    "plant_peak_torque_nm": peak_nm,
                    "sensor_quant_rad": 0.00017,
                    "sensor_delay_steps": 1,
                    "controller_kwargs": kwargs,
                },
            },
        }},
        "maneuver": {"steps": [
            {"duration": 2.0, "speed_kmh": 60,
             "steer": {"kind": "constant", "amplitude": 0.0}},
            # 5° front step at 60 km/h ≈ 0.8 g — deliberately over the envelope.
            {"duration": 4.0, "speed_kmh": 60,
             "steer": {"kind": "step", "amplitude": 5.0, "unit": "front_deg",
                       "t_step": 0.0}},
        ]},
    }
    return Experiment.model_validate(raw)


def run_one(controller: str, kwargs: dict, peak_nm: float) -> dict:
    exp = build_experiment(controller, kwargs, peak_nm)
    t0 = time.perf_counter()
    result = SimSession(exp).run()
    wall = time.perf_counter() - t0
    t = np.asarray(result.t, dtype=float)
    ch = {k: np.asarray(v, dtype=float) for k, v in result.channels.items()}
    cmd, act = ch["delta_cmd_fl"], ch["delta_fl"]
    dev = act - cmd
    # From the step onward only: the straight-ahead run-in has zero deviation.
    step_i = int(np.argmax(np.abs(np.diff(cmd)) > 1e-4)) + 1
    d = dev[step_i:]
    # The matrix semantics: a causal system necessarily lags a step by the
    # full amplitude during the rise — the blow-back is what happens AFTER
    # the response first reaches 90 % of the amplitude.
    amp = float(np.nanmax(np.abs(np.diff(cmd)[step_i - 1:])))
    reached = int(np.argmax(np.abs(act[step_i:]) >= 0.9 * amp)) + step_i
    d90 = dev[reached:] if reached < len(dev) else np.array([0.0])
    out = {
        "peak_nm": peak_nm,
        "wall_s": round(wall, 1),
        "max_dev_rad": float(np.nanmax(np.abs(d))),
        "peak_dev_after90_rad": float(np.nanmax(np.abs(d90))),
        "end_dev_rad": float(d[-1]),
        "violation_s": float(np.sum(np.abs(d) > DEVIATION_LIMIT)
                             * float(np.median(np.diff(t)))),
        "max_rack_force_n": float(np.nanmax(np.abs(ch["rack_force_fl"]))),
    }
    # The reversal: the most negative rack force against the step direction.
    signed = ch["rack_force_fl"][step_i:]
    out["peak_opposing_rack_n"] = float(np.nanmax(-signed)) if signed.size else 0.0
    # Per-wheel load torque demand = rack force × pinion radius (0.02 m).
    out["peak_load_torque_nm"] = out["max_rack_force_n"] * 0.02
    if "steer_corner_torque_cmd_fl" in ch:
        tc = np.abs(ch["steer_corner_torque_cmd_fl"][step_i:])
        out["pinned_fraction"] = float(np.mean(tc >= 0.99 * peak_nm))
    return out


def main() -> int:
    print(f"5° front step @ 60 km/h (~0.8 g) — deviation limit {DEVIATION_LIMIT} rad")
    print()
    rows: dict[str, list[dict]] = {}
    for controller, kwargs in CONTROLLERS:
        print(f"=== {controller} ===")
        hdr = (f"{'peak N·m':>8} {'max|dev|':>9} {'post90|dev|':>11} {'end dev':>9} "
               f"{'over0.05s':>10} {'load τ':>8} {'opp. F':>8} {'wall':>6}")
        print(hdr)
        rows[controller] = []
        for peak in PEAKS_NM:
            r = run_one(controller, kwargs, peak)
            rows[controller].append(r)
            print(f"{r['peak_nm']:>8.0f} {r['max_dev_rad']:>9.4f} "
                  f"{r['peak_dev_after90_rad']:>11.4f} {r['end_dev_rad']:>9.4f} "
                  f"{r['violation_s']:>10.2f} {r['peak_load_torque_nm']:>8.1f} "
                  f"{r['peak_opposing_rack_n']:>8.0f} {r['wall_s']:>5.1f}s")
        print()
    # The sizing answer: the smallest swept peak whose blow-back deviation
    # (post-90 %) stays inside the architecture limit.
    print("=== sizing answer ===")
    for controller, rs in rows.items():
        inside = [r for r in rs
                  if r["peak_dev_after90_rad"] <= DEVIATION_LIMIT
                  and abs(r["end_dev_rad"]) <= DEVIATION_LIMIT]
        if not inside:
            print(f"{controller:<12} no swept peak keeps the wheel inside "
                  f"{DEVIATION_LIMIT} rad — enlarge the sweep")
            continue
        best = inside[0]
        print(f"{controller:<12} peak >= {best['peak_nm']:.0f} N·m keeps the "
              f"blow-back inside {DEVIATION_LIMIT} rad "
              f"(post90 dev {best['peak_dev_after90_rad']:.4f}, "
              f"end {best['end_dev_rad']:.4f}); peak load demand "
              f"{best['peak_load_torque_nm']:.0f} N·m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
