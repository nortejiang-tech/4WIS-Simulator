"""Uniform step table for all 9 controllers at procedure conditions.

30 km/h, 1° front step, realistic sensor — the same condition as the step
procedure, extended to the advanced controllers for the report table.
Run: backend/.venv/bin/python scripts/devtools/step_table.py
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend" / "src"))

import numpy as np  # noqa: E402

from sim4wis.experiment.schema import Experiment  # noqa: E402
from sim4wis.experiment.session import SimSession  # noqa: E402
from sim4wis.study.tracking_step import _analyse_one  # noqa: E402

FF = {"ff": {"blocks": [
    {"type": "rack_force", "gain": 1.0},
    {"type": "velocity", "damping": 4.0, "inertia": 0.6},
    {"type": "friction", "friction_nm": 0.5},
]}}
CONTROLLERS = [
    ("open_loop", {}), ("pid_single", FF), ("pid_cascade", FF), ("lqr", FF),
    ("dob", {}), ("adrc", {}), ("smc", FF), ("mpc", FF), ("h_inf", FF),
]

print(f"{'controller':<12} {'rise_s':>7} {'overshoot%':>10} {'settle_s':>8} "
      f"{'ss_err_rad':>10} {'peak_dev':>9} {'rl_crosstalk':>12}")
for name, kw in CONTROLLERS:
    raw = {
        "name": "step_table", "strategy": "ideal_ackermann", "record_hz": 200,
        "vehicle": {"overrides": {"steering_system": {
            "enabled": True, "architecture": "4wis",
            "angle_control": {"enabled": True, "controller": name,
                              "sensor_quant_rad": 0.00017,
                              "sensor_delay_steps": 1,
                              "controller_kwargs": kw}}}},
        "maneuver": {"steps": [
            {"duration": 1.0, "speed_kmh": 30,
             "steer": {"kind": "constant", "amplitude": 0.0}},
            {"duration": 2.5, "speed_kmh": 30,
             "steer": {"kind": "step", "amplitude": 1.0, "unit": "front_deg",
                       "t_step": 0.0}},
        ]},
    }
    r = SimSession(Experiment.model_validate(raw)).run()
    t = np.asarray(r.t)
    ch = {k: np.asarray(v) for k, v in r.channels.items()}
    m = _analyse_one(ch["delta_cmd_fl"], ch["delta_fl"],
                     float(np.median(np.diff(t))))
    n0 = len(t) // 3
    rl_dev = float(np.max(np.abs(ch["delta_rl"][n0:] - ch["delta_cmd_rl"][n0:])))
    print(f"{name:<12} {m.rise_s:>7.3f} {m.overshoot_pct:>10.2f} "
          f"{m.settle_s:>8.3f} {m.ss_err_rad:>10.2e} {m.peak_dev_rad:>9.5f} "
          f"{rl_dev:>12.5f}", flush=True)
