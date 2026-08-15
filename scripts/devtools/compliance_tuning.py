"""Direction 3 — compliance tuning: the provenance of the compliant-plant gains.

Stage 1 (corner loop). The compliant transmission (k = 1200 N·m/rad,
~17.8 Hz resonance) makes every shipped default gain run away (900-1500 %
overshoot on a 1° step) — high-bandwidth wheel-rate feedback meeting an
unmodelled resonance. The tuning workbench's compliance channel
(resonance-safe analytic seed + black-box refinement with the rate-channel
low-pass in the axis) re-designs the tuneable v1 controllers on the corner
plant; the table is the honest before/after.

Stage 2 (vehicle loop). Corner gains do NOT transfer to the vehicle: the
production feedforward's 15 Hz rack-force lowpass destabilises the
compliant loop (642 % overshoot — the lesson-6 failure mode, re-measured in
the compliant dimension), and without it the vehicle retune lands. This
stage runs Nelder-Mead over the step procedure with the compliant plant,
the lowpass-free feedforward, and the rate filter in the axis, seeded from
the stage-1 gains.

Deterministic. The printed override block is the configuration the final
gains belong to. The rigid-tuned defaults stay the shipped behaviour — the
compliant plant and these gains are an opt-in evaluation dimension.
"""

from __future__ import annotations

import copy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend" / "src"))

import numpy as np  # noqa: E402

from sim4wis.calibration.identify import (  # noqa: E402
    _load_procedure_raw,
    nelder_mead,
    run_sim_with_values,
)
from sim4wis.steering.tracking.controller import make_controller  # noqa: E402
from sim4wis.steering.tracking.feedback import AngleSensor  # noqa: E402
from sim4wis.steering.tracking.plant import CornerActuatorPlant  # noqa: E402
from sim4wis.steering.tracking.tuning import (  # noqa: E402
    analytic_cascade_compliant,
    analytic_pid_compliant,
    transmission_resonance_hz,
    tune,
)
from sim4wis.study.tracking_step import _analyse_one  # noqa: E402

STIFFNESS = 1200.0  # N·m/rad
PLANT = {"transmission_stiffness_nms_per_rad": STIFFNESS,
         "peak_torque_nm": 120.0, "motor_inertia_fraction": 0.2}
CONTROLLERS = ["pid_single", "pid_cascade", "lqr"]
DT = 5.0e-4
T_END = 3.0
TARGET = 0.01745

# The production feedforward stack MINUS the rack-force lowpass: on the
# compliant loop the 15 Hz filter is a destabiliser (642 % overshoot at the
# vehicle level, 53 % without it) — the same phase-lag lesson the rigid LQR
# taught, re-measured in the compliant dimension.
FF = {"ff": {"blocks": [
    {"type": "rack_force", "gain": 1.0},
    {"type": "velocity", "damping": 4.0, "inertia": 0.6},
    {"type": "friction", "friction_nm": 0.5},
]}}

# Search boxes with PHYSICAL lower floors (zero): a gain at the floor is a
# legitimate "integral-free" design, not a bound artefact. The vehicle cost
# guards degeneracy (ss error is weighted 20×).
VEHICLE_AXES = {
    "pid_single": {
        "names": ["kp", "ki", "kd", "deriv_tau_s"],
        "lo": np.array([50.0, 0.0, 0.5, 0.005]),
        "hi": np.array([3000.0, 30000.0, 300.0, 0.2]),
    },
    "pid_cascade": {
        "names": ["kp_pos", "ki_pos", "kp_vel", "ki_vel", "deriv_tau_s"],
        "lo": np.array([2.0, 0.0, 5.0, 0.0, 0.005]),
        "hi": np.array([150.0, 500.0, 200.0, 3000.0, 0.2]),
    },
    "lqr": {
        "names": ["q_integral", "q_angle", "q_rate", "r", "deriv_tau_s"],
        "lo": np.array([0.0, 0.0, 0.0, 1.0e-6, 0.005]),
        "hi": np.array([20000.0, 20000.0, 50.0, 0.5, 0.2]),
    },
}

#: Warm-start seeds from the first vehicle passes (2026-08-16): the
#: vehicle-cost landscape is multimodal (two seeds, cost 0.43 vs 1.50 for
#: pid_single), so the recipe keeps every optimum it has ever found.
WARM_STARTS = {
    "pid_single": {"kp": 606.14, "ki": 46.117, "kd": 41.716,
                   "deriv_tau_s": 0.051491},
    "pid_cascade": {"kp_pos": 16.842, "ki_pos": 1.0, "kp_vel": 29.190,
                    "ki_vel": 10.0, "deriv_tau_s": 0.05001},
    "lqr": {"q_integral": 50.0, "q_angle": 2803.9, "q_rate": 0.01,
            "r": 0.040764, "deriv_tau_s": 0.04047},
}

RAW = _load_procedure_raw(
    str(ROOT / "procedures" / "tracking_step_response.yaml"))


def corner_step(controller: str, kwargs: dict) -> tuple[float, float, float]:
    """1° step on the compliant corner plant → (overshoot %, settle s, ss %)."""
    c = make_controller(controller, **kwargs)
    plant = CornerActuatorPlant(
        inertia_kgm2=0.6, damping_nms_per_rad=4.0, coulomb_friction_nm=0.5,
        peak_torque_nm=120.0, transmission_stiffness_nms_per_rad=STIFFNESS,
        motor_inertia_fraction=0.2)
    sensor = AngleSensor(quant_rad=0.00017, delay_steps=1)
    hist = []
    for _ in range(int(T_END / DT)):
        out = c.step(DT, target_angle=TARGET, target_rate=0.0,
                     feedback_angle=sensor.measure(plant.angle),
                     plant_angle=plant.angle, load_torque=3.0, speed_ms=0.0)
        plant.step(DT, out.torque_cmd, 3.0)
        hist.append(plant.angle)
    h = np.asarray(hist)
    overshoot = float((h.max() - TARGET) / TARGET * 100 if h.max() > TARGET else 0.0)
    err = np.abs(h - TARGET) / TARGET
    settle_idx = np.where(err > 0.02)[0]
    settle = float((settle_idx[-1] + 1) * DT if settle_idx.size else 0.0)
    ss_err = float(abs(h[-1] - TARGET) / TARGET * 100)
    return overshoot, settle, ss_err


def fmt(m: tuple[float, float, float]) -> str:
    return f"{m[0]:8.1f}%  {m[1]:5.2f}s  {m[2]:6.2f}%"


def _spec_for(controller: str, kwargs: dict) -> dict:
    spec = copy.deepcopy(RAW)
    spec["sweep"] = {}
    ac = spec["baseline"]["vehicle"]["overrides"]["steering_system"]["angle_control"]
    ac["controller"] = controller
    ac["controller_kwargs"] = {**kwargs, **FF}
    ac["plant_transmission_stiffness_nms_per_rad"] = STIFFNESS
    ac["plant_motor_inertia_fraction"] = 0.2
    return spec


def vehicle_metrics(controller: str, kwargs: dict) -> dict[str, float]:
    """Vehicle step procedure with the compliant plant → trk metrics."""
    t, ch = run_sim_with_values(_spec_for(controller, kwargs), {})
    m = _analyse_one(ch["delta_cmd_fl"], ch["delta_fl"],
                     float(np.median(np.diff(t))))
    mrl = _analyse_one(ch["delta_cmd_rl"], ch["delta_rl"],
                       float(np.median(np.diff(t))))
    return {"rise_s": m.rise_s, "overshoot_pct": m.overshoot_pct,
            "settle_s": m.settle_s, "ss_err_rad": m.ss_err_rad,
            "peak_dev_rad": m.peak_dev_rad,
            "rl_peak_dev_rad": mrl.peak_dev_rad}


def vehicle_cost(controller: str, kwargs: dict) -> float:
    m = vehicle_metrics(controller, kwargs)
    step = 0.01745
    return (m["settle_s"] + 5.0 * m["overshoot_pct"] / 100.0
            + 20.0 * abs(m["ss_err_rad"]) / max(step, 1e-9))


def main() -> None:
    f_res = transmission_resonance_hz(STIFFNESS, 0.6, 0.2)
    print(f"two-mass resonance: {f_res:.1f} Hz  (k = {STIFFNESS} N·m/rad, "
          "J_m:J_w = 0.12:0.48)")
    print()
    print("=== stage 1 — corner loop ===")
    print(f"{'controller':<12} {'config':<16} {'overshoot':<14} {'settle':<8} {'ss_err':<10}")
    print("-" * 64)
    corner_gains: dict[str, dict] = {}
    for controller in CONTROLLERS:
        before = corner_step(controller, {})
        result = tune(controller, max_evals=120, plant_kwargs=PLANT)
        after = corner_step(controller, result.tuned_params)
        print(f"{controller:<12} {'shipped default':<16} {fmt(before)}")
        print(f"{controller:<12} {'compliance-tuned':<16} {fmt(after)}")
        print(f"{controller:<12} {'  cost':<16} "
              f"analytic={result.cost_analytic:.4f} tuned={result.cost_tuned:.4f} "
              f"accepted={result.accepted}")
        print()
        corner_gains[controller] = {k: float(v)
                                    for k, v in result.tuned_params.items()}
    print("=== stage 2 — vehicle loop (Nelder-Mead over the step procedure) ===")
    final: dict[str, dict] = {}
    for controller in CONTROLLERS:
        cfg = VEHICLE_AXES[controller]
        names = cfg["names"]
        lo, hi = cfg["lo"], cfg["hi"]
        cache: dict[tuple[float, ...], float] = {}

        def f(x: np.ndarray) -> float:
            key = tuple(np.round(x, 9))
            if key in cache:
                return cache[key]
            kwargs = {n: float(v) for n, v in zip(names, x, strict=True)}
            try:
                val = vehicle_cost(controller, kwargs)
            except ValueError:
                val = 100.0
            cache[key] = val
            return val

        # Deterministic multi-start: the vehicle-cost landscape is multimodal
        # (measured: two seeds, two local optima, cost 0.43 vs 1.50). Seeds =
        # the stage-1 corner gains and the resonance-safe analytic design.
        seeds: list[np.ndarray] = [
            np.array([corner_gains[controller][n] for n in names]),
        ]
        if controller == "pid_single":
            alt = analytic_pid_compliant(0.6, 4.0, STIFFNESS)
        elif controller == "pid_cascade":
            alt = analytic_cascade_compliant(0.6, 4.0, STIFFNESS)
        else:
            alt = {"q_integral": 400.0, "q_angle": 1200.0, "q_rate": 1.0,
                   "r": 0.01, "deriv_tau_s": 0.05}
        seeds.append(np.array([alt[n] for n in names]))
        seeds.append(np.array([WARM_STARTS[controller][n] for n in names]))

        best: tuple[float, dict[str, float], int] | None = None
        for k, x0 in enumerate(seeds):
            x, fx, n = nelder_mead(f, x0, lo=lo, hi=hi, max_evals=120)
            gains = {n: float(v) for n, v in zip(names, x, strict=True)}
            if best is None or fx < best[0]:
                best = (fx, gains, n)
            print(f"   start {k}: cost={fx:.4f} ({n} evals)")
        fx, gains, n = best
        final[controller] = gains
        m = vehicle_metrics(controller, gains)
        print(f"{controller:<12} best cost={fx:.4f}")
        print("   " + ", ".join(f"{k}={v:.4g}" for k, v in gains.items()))
        print(f"   vehicle: rise={m['rise_s']:.2f}s os={m['overshoot_pct']:.1f}% "
              f"settle={m['settle_s']:.2f}s ss={m['ss_err_rad']:.5f} "
              f"peak_dev={m['peak_dev_rad']:.5f} rl_peak_dev={m['rl_peak_dev_rad']:.5f}")
        print()
    print("=== recommended vehicle override block (compliant plant) ===")
    print("vehicle:")
    print("  overrides:")
    print("    steering_system:")
    print("      enabled: true")
    print("      architecture: 4wis")
    print("      angle_control:")
    print("        enabled: true")
    print(f"        plant_transmission_stiffness_nms_per_rad: {STIFFNESS}")
    print("        plant_motor_inertia_fraction: 0.2")
    print("        sensor_quant_rad: 0.00017")
    print("        sensor_delay_steps: 1")
    print("        controller_kwargs:")
    print("          ff:")
    print("            blocks:")
    print("              - {type: rack_force, gain: 1.0}  # no lowpass — lesson 6")
    print("              - {type: velocity, damping: 4.0, inertia: 0.6}")
    print("              - {type: friction, friction_nm: 0.5}")
    for controller in CONTROLLERS:
        print(f"        # {controller}: {final[controller]}")
    print()
    print("NOTE: the rigid-tuned defaults stay the shipped behaviour — the")
    print("compliant plant and these gains are an opt-in evaluation dimension.")


if __name__ == "__main__":
    main()
