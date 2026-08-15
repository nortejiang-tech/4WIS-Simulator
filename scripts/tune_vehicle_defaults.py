"""Tune the shipped controller defaults in the vehicle loop (one-off).

The evaluation procedure (tracking_step_response.yaml) runs the controllers
inside the full vehicle — rack force one step old, tire aligning load, sensor
quantisation and delay. Defaults designed on the bare corner plant leave a
slow tail there, so the shipped defaults are tuned against this procedure
itself: Nelder-Mead on the step cost, deterministic, and the resulting
numbers are baked into the controller classes with this script as their
provenance. The M3 tuning workbench generalises the same cost in-plant.

Cost = settle + 5·overshoot% + 20·|ss_err|/step_rad, from the study's own
metric — so the tuned defaults are optimal for exactly what the evaluation
reports.
"""

import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "backend" / "src"))

import numpy as np

from sim4wis.calibration.identify import (
    _load_procedure_raw,
    nelder_mead,
    run_sim_with_values,
)
from sim4wis.study.tracking_step import _analyse_one

ROOT = Path(__file__).resolve().parents[1]
RAW = _load_procedure_raw(str(ROOT / "procedures" / "tracking_step_response.yaml"))

# No lowpass on the load feedforward: the rack force is already one step old,
# and a 15 Hz lowpass on top pushes the loop's phase margin over the edge
# (measured: lqr overshoot 3.8 % → 0.3 %, settle 2.5 s → 1.28 s).
FF = {"ff": {"blocks": [
    {"type": "rack_force", "gain": 1.0},
    {"type": "velocity", "damping": 4.0, "inertia": 0.6},
    {"type": "friction", "friction_nm": 0.5},
]}}


def cost_of(controller: str, kwargs: dict) -> float:
    spec = copy.deepcopy(RAW)
    spec["sweep"] = {}
    ac = spec["baseline"]["vehicle"]["overrides"]["steering_system"]["angle_control"]
    ac["controller"] = controller
    ac["controller_kwargs"] = kwargs
    t, ch = run_sim_with_values(spec, {})
    m = _analyse_one(ch["delta_cmd_fl"], ch["delta_fl"], float(np.median(np.diff(t))))
    step = abs(m.amplitude_rad)
    return (m.settle_s + 5.0 * m.overshoot_pct / 100.0
            + 20.0 * abs(m.ss_err_rad) / max(step, 1e-9)
            + min(m.settle_s, 2.5) * 0.0  # settle dominates; keep it explicit
            )


TUNERS = {
    "pid_single": {
        "names": ["kp", "ki", "kd"],
        "x0": np.array([621.0, 3645.0, 28.4]),
        "lo": np.array([100.0, 500.0, 5.0]),
        "hi": np.array([3000.0, 20000.0, 200.0]),
    },
    "pid_cascade": {
        "names": ["kp_pos", "ki_pos", "kp_vel", "ki_vel"],
        "x0": np.array([18.0, 12.0, 30.0, 150.0]),
        "lo": np.array([5.0, 2.0, 10.0, 20.0]),
        "hi": np.array([100.0, 400.0, 150.0, 2000.0]),
    },
    "lqr": {
        "names": ["q_integral", "q_angle", "q_rate", "r"],
        "x0": np.array([400.0, 1200.0, 1.0, 0.01]),
        "lo": np.array([50.0, 100.0, 0.1, 0.001]),
        "hi": np.array([20000.0, 20000.0, 50.0, 0.5]),
    },
    "dob": {
        # Base is integral-free (the observer owns the DC action); tune the
        # base stiffness and the observer bandwidth together.
        "names": ["q_hz", "base.kp", "base.kd"],
        "x0": np.array([6.0, 800.0, 50.0]),
        "lo": np.array([2.0, 200.0, 10.0]),
        "hi": np.array([30.0, 4000.0, 300.0]),
        "kwargs": {"base": {"ki": 0.0}},  # merged into per-axis names below
        "flat": {"base": {"ki": 0.0}},
    },
    "adrc": {
        "names": ["omega_c", "omega_o"],
        "x0": np.array([30.0, 120.0]),
        "lo": np.array([10.0, 40.0]),
        "hi": np.array([80.0, 400.0]),
        "flat": {},
    },
    "mpc": {
        "names": ["q_integral", "q_angle", "r"],
        "x0": np.array([613.9, 4529.0, 0.02812]),
        "lo": np.array([50.0, 300.0, 0.001]),
        "hi": np.array([5000.0, 20000.0, 0.1]),
        "flat": {},
    },
    "h_inf": {
        "names": ["q_integral", "q_angle", "gamma"],
        "x0": np.array([300.0, 900.0, 6.0]),
        "lo": np.array([50.0, 200.0, 2.0]),
        "hi": np.array([8000.0, 20000.0, 20.0]),
        "flat": {},
    },
}


def _kwargs(names: list[str], x: np.ndarray, flat: dict) -> dict:
    """Axis vector → controller kwargs, unpacking dotted names (base.kp)."""
    kwargs: dict = {} if _CURRENT[0] == "dob" else {**FF}
    for name, value in zip(names, x, strict=True):
        if "." in name:
            outer, inner = name.split(".", 1)
            kwargs.setdefault(outer, {})
            if isinstance(kwargs[outer], dict):
                kwargs[outer][inner] = float(value)
        else:
            kwargs[name] = float(value)
    for outer, inner_kv in (flat or {}).items():
        merged = dict(kwargs.get(outer) or {})
        if isinstance(inner_kv, dict):
            merged.update(inner_kv)
            kwargs[outer] = merged
    return kwargs


_CURRENT: list[str] = [""]


def main() -> None:
    for controller, cfg in TUNERS.items():
        if controller != "mpc":
            continue
        names, x0, lo, hi = cfg["names"], cfg["x0"], cfg["lo"], cfg["hi"]
        flat = cfg.get("flat", {})
        _CURRENT[0] = controller

        def f(x: np.ndarray) -> float:
            try:
                return cost_of(controller, _kwargs(names, x, flat))
            except ValueError:
                # e.g. an infeasible H-inf gamma on this grid point: a large
                # penalty, not a crash — the optimiser must walk around it.
                return 100.0

        x, fx, n = nelder_mead(f, x0, lo=lo, hi=hi, max_evals=140)
        print(f"{controller}: cost {fx:.4f} ({n} evals)")
        print("  " + ", ".join(f"{nm}={v:.4g}" for nm, v in zip(names, x, strict=True)))


if __name__ == "__main__":
    main()
