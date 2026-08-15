"""The tuning workbench (M3 · FR-9) — three channels, one acceptance bar.

    analytic    gains from the nominal plant model (pole placement, LQR) —
                the "given the model, the gains are given" channel
    relay       Åström–Hägglund relay identification on the plant, then the
                Ziegler–Nichols table — the semi-automatic channel
    black-box   Nelder-Mead on a step-response cost in the closed loop — the
                automatic channel, seeded from the analytic gains

**Compliance channel (direction 3).** ``step_cost`` / ``relay_identify`` /
``tune`` accept the two-mass transmission parameters; when a stiffness is
set, the analytic channel designs resonance-safe gains instead of the rigid
pole placement (the wheel-rate feedback paths excite the two-mass mode —
see ``analytic_pid_compliant``), and ``tune`` adds the rate-channel low-pass
(``deriv_tau_s``) to the search axis so the black-box can trade bandwidth
against filtering. The acceptance bar is unchanged and is measured on the
compliant plant itself.

`tune()` chains them: the analytic gains are the baseline; the black-box
search refines from them; the result is accepted only when its cost is at
most 110 % of the analytic baseline (the frozen acceptance criterion) — a
search that does not beat the analytic design is reported honestly, not
crowned. Everything is deterministic and every result carries its
provenance (method, evaluations, plant parameters, cost).

Scope note: this tunes against the **corner-plant loop**. The shipped
vehicle-loop defaults were tuned against the step procedure itself (see
scripts/tune_vehicle_defaults.py); both use the same step-cost shape, and
the plant-level channel exists so a controller can be re-tuned for a new
actuator without running the vehicle.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np

from sim4wis.calibration.identify import nelder_mead
from sim4wis.steering.tracking.controller import make_controller
from sim4wis.steering.tracking.feedback import AngleSensor
from sim4wis.steering.tracking.plant import CornerActuatorPlant

#: The frozen acceptance bar (requirements §4.5): the tuned cost must be at
#: most this factor of the analytic baseline.
ACCEPTANCE_RATIO = 1.10


@dataclass
class TuningResult:
    controller: str
    cost_analytic: float
    cost_tuned: float
    analytic_params: dict[str, float]
    tuned_params: dict[str, float]
    n_evals: int
    accepted: bool
    note: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "controller": self.controller,
            "cost_analytic": self.cost_analytic,
            "cost_tuned": self.cost_tuned,
            "accepted": self.accepted,
            "acceptance_ratio": ACCEPTANCE_RATIO,
            "n_evals": self.n_evals,
            "analytic_params": self.analytic_params,
            "tuned_params": self.tuned_params,
            "note": self.note,
        }


def analytic_pid(j: float, b: float, wn: float = 25.0,
                 zeta: float = 0.9) -> dict[str, float]:
    """PID gains from pole placement on the nominal plant."""
    from sim4wis.steering.tracking.controllers import pole_place_pid

    kp, ki, kd = pole_place_pid(j, b, wn, zeta)
    return {"kp": kp, "ki": ki, "kd": kd}


def analytic_cascade(j: float, b: float, *, wc_vel: float = 60.0,
                     wn_pos: float = 18.0, zeta_pos: float = 0.9) -> dict[str, float]:
    """Cascade gains: inner velocity loop bandwidth ωc, outer PI by pole
    placement on the (assumed) velocity servo."""
    kp_vel = j * wc_vel  # velocity loop: J·a ≈ kp_vel·(ω_ref − ω)
    ki_vel = b * wc_vel / 5.0  # small integral, scaled by the plant damping
    kp_pos = 2.0 * zeta_pos * wn_pos
    ki_pos = wn_pos * wn_pos / 4.0
    return {"kp_pos": kp_pos, "ki_pos": ki_pos,
            "kp_vel": kp_vel, "ki_vel": ki_vel}


def step_cost(
    controller: str,
    kwargs: dict[str, Any],
    *,
    inertia: float = 0.6, damping: float = 4.0,
    friction_nm: float = 0.5, peak_torque_nm: float = 40.0,
    transmission_stiffness_nms_per_rad: float | None = None,
    backlash_rad: float = 0.0,
    motor_inertia_fraction: float = 0.2,
    load_torque: float = 3.0,
    quant_rad: float = 0.00017, delay_steps: int = 1,
    target: float = 0.01745, t_end: float = 2.0,
    dt: float = 5.0e-4,
) -> float:
    """Closed-loop step cost on the corner plant: ITAE-normalised error plus
    a peak-torque penalty. Lower is better; deterministic. With a
    transmission stiffness set, the plant is the two-mass model — the cost
    then measures how the controller copes with the resonance and the
    backlash dead band (the compliance evaluation dimension)."""
    c = make_controller(controller, **kwargs)
    plant = CornerActuatorPlant(inertia_kgm2=inertia, damping_nms_per_rad=damping,
                                coulomb_friction_nm=friction_nm,
                                peak_torque_nm=peak_torque_nm,
                                transmission_stiffness_nms_per_rad=(
                                    transmission_stiffness_nms_per_rad),
                                backlash_rad=backlash_rad,
                                motor_inertia_fraction=motor_inertia_fraction)
    sensor = AngleSensor(quant_rad=quant_rad, delay_steps=delay_steps)
    n = max(int(t_end / dt), 10)
    t = 0.0
    itae = 0.0
    peak_u = 0.0
    for _ in range(n):
        out = c.step(dt, target_angle=target, target_rate=0.0,
                     feedback_angle=sensor.measure(plant.angle),
                     plant_angle=plant.angle, load_torque=load_torque,
                     speed_ms=0.0)
        if out.torque_cmd is not None:
            plant.step(dt, out.torque_cmd, load_torque)
            peak_u = max(peak_u, abs(out.torque_cmd))
        elif out.angle_set is not None:
            plant.angle = float(out.angle_set)
        t += dt
        itae += abs(target - plant.angle) * t * dt
    # Normalise by the step size, weigh the torque against the plant's peak.
    return itae / abs(target) + 0.05 * peak_u / max(peak_torque_nm, 1e-9)


def relay_identify(
    *,
    inertia: float = 0.6, damping: float = 4.0, friction_nm: float = 0.5,
    transmission_stiffness_nms_per_rad: float | None = None,
    backlash_rad: float = 0.0,
    motor_inertia_fraction: float = 0.2,
    relay_amplitude: float = 10.0, hysteresis: float = 0.01,
    t_max: float = 20.0, dt: float = 5.0e-4,
) -> tuple[float, float]:
    """Åström–Hägglund relay test on the corner plant → (Ku, Tu).

    The relay drives the plant into a limit cycle; the ultimate gain and
    period fall out of the oscillation: Ku = 4·d/(π·a), Tu = period. The
    relay carries a **hysteresis band** — without it a stiff plant switches
    every sample or two and the "period" measures the sampling rate, not the
    loop (measured: Tu = 1 ms, ZN gains in the billions). With the band the
    oscillation has a physically meaningful amplitude and period. Raises
    when no limit cycle develops.
    """
    plant = CornerActuatorPlant(inertia_kgm2=inertia, damping_nms_per_rad=damping,
                                coulomb_friction_nm=friction_nm,
                                peak_torque_nm=relay_amplitude * 2.0,
                                transmission_stiffness_nms_per_rad=(
                                    transmission_stiffness_nms_per_rad),
                                backlash_rad=backlash_rad,
                                motor_inertia_fraction=motor_inertia_fraction)
    n = int(t_max / dt)
    angles: list[float] = []
    u = relay_amplitude
    state_up = True  # u = +d, waiting for the angle to exceed +h
    crossings: list[int] = []
    for k in range(n):
        if state_up and plant.angle >= hysteresis:
            state_up, u = False, -relay_amplitude
            crossings.append(k)
        elif not state_up and plant.angle <= -hysteresis:
            state_up, u = True, relay_amplitude
            crossings.append(k)
        plant.step(dt, u, 0.0)
        angles.append(plant.angle)
    if len(crossings) < 6:
        raise RuntimeError("relay test did not reach a limit cycle")
    periods = np.diff(crossings[2:])
    period = float(np.median(periods)) * dt
    if period <= 4.0 * dt:  # still switching at the sample rate — meaningless
        raise RuntimeError(
            f"relay limit cycle collapsed to the sample rate ({period:.2e} s); "
            "raise the hysteresis or lower the relay amplitude")
    # Oscillation amplitude over the settled half-cycles.
    a1 = max(abs(a) for a in angles[crossings[1]:crossings[2]])
    a2 = max(abs(a) for a in angles[crossings[2]:crossings[3]])
    amp = 0.5 * (a1 + a2)
    ku = 4.0 * relay_amplitude / (math.pi * max(amp, 1e-12))
    return ku, period


def zn_pid(ku: float, tu: float) -> dict[str, float]:
    """The classic Ziegler–Nichols table (PID row) from (Ku, Tu)."""
    return {"kp": 0.6 * ku, "ki": 1.2 * ku / max(tu, 1e-9),
            "kd": 0.075 * ku * tu}


# ---------------------------------------------------------------------------
# Compliance channel (direction 3): resonance-safe analytic designs
# ---------------------------------------------------------------------------


def transmission_resonance_hz(k_trans: float, inertia: float = 0.6,
                               motor_fraction: float = 0.2) -> float:
    """The two-mass transmission resonance
    f = (1/2π)·sqrt(k·(J_m+J_w)/(J_m·J_w)) — the mode every wheel-rate
    feedback path meets head-on when the coupling is compliant."""
    j_m = float(inertia) * float(motor_fraction)
    j_w = float(inertia) * (1.0 - float(motor_fraction))
    omega = math.sqrt(float(k_trans) * (j_m + j_w) / max(j_m * j_w, 1e-12))
    return omega / (2.0 * math.pi)


def analytic_pid_compliant(j: float, b: float, k_trans: float, *,
                           frac: float = 0.2, zeta: float = 0.9,
                           wn_ratio: float = 8.0,
                           tau_ratio: float = 5.0) -> dict[str, float]:
    """PID gains for the compliant two-mass plant: pole placement on the
    lumped model with the bandwidth held under the resonance and a
    rate-channel low-pass cutting below it.

    The wheel-side rate feedback (the D term) is what excites the two-mass
    mode: with the shipped vehicle defaults (kp=2165 ⇒ wn≈60 rad/s ≈
    ω_res/1.9, τ = 0.01 s) the loop runs away at ~950 % overshoot on a 1°
    step. Grid-pinned on the k=1200 N·m/rad plant: wn = ω_res/8 and
    τ = 5/ω_res sit inside the stable basin with margin (7 % overshoot,
    neighbours 9-22 %). The constants are the rule; the black-box channel
    refines from here."""
    from sim4wis.steering.tracking.controllers import pole_place_pid

    w_res = 2.0 * math.pi * transmission_resonance_hz(k_trans, j, frac)
    wn = w_res / wn_ratio
    deriv_tau_s = tau_ratio / w_res
    kp, ki, kd = pole_place_pid(j, b, wn, zeta)
    return {"kp": kp, "ki": ki, "kd": kd, "deriv_tau_s": deriv_tau_s}


def analytic_cascade_compliant(j: float, b: float, k_trans: float, *,
                               frac: float = 0.2, zeta_pos: float = 0.9,
                               vel_ratio: float = 4.0,
                               tau_ratio: float = 4.0) -> dict[str, float]:
    """Cascade gains for the compliant plant: the inner velocity loop
    bandwidth and its rate filter are held below the resonance (the inner
    loop feeds back the wheel rate — the same excitation channel), the
    outer PI is pole-placed on the assumed velocity servo at a quarter of
    the inner bandwidth. Grid-pinned: wc_vel = ω_res/4, τ = 4/ω_res sits
    two grid steps inside the stable basin (~11 % overshoot)."""
    w_res = 2.0 * math.pi * transmission_resonance_hz(k_trans, j, frac)
    wc_vel = w_res / vel_ratio
    wn_pos = wc_vel / 2.5
    deriv_tau_s = tau_ratio / w_res
    kp_vel = j * wc_vel
    ki_vel = b * wc_vel / 5.0
    kp_pos = 2.0 * zeta_pos * wn_pos
    ki_pos = wn_pos * wn_pos / 4.0
    return {"kp_pos": kp_pos, "ki_pos": ki_pos,
            "kp_vel": kp_vel, "ki_vel": ki_vel,
            "deriv_tau_s": deriv_tau_s}


def lqr_compliant_weights() -> dict[str, float]:
    """LQR weight seed for the compliant plant.

    The rigid-model DARE weights alone excite the two-mass mode (measured:
    1475 % overshoot on the k=1200 plant); with the rate-channel low-pass
    raised the same weights sit inside the stable basin (measured:
    0 % overshoot, 10 % short at 3 s — the integral is closing, the
    black-box then tightens it against the compliant cost)."""
    return {"q_integral": 400.0, "q_angle": 1200.0, "q_rate": 1.0,
            "r": 0.01, "deriv_tau_s": 0.05}


#: ``deriv_tau_s`` search bounds [s] when the compliance channel adds the
#: rate-channel low-pass to the axis.
COMPLIANT_RATE_TAU_BOUNDS = (0.002, 0.2)

#: Lower-bound relaxations for the compliance channel: the rigid spaces were
#: sized around the rigid optimum, and the compliant optimum lives in the
#: low-gain region (measured: the rigid lower bounds clip it).
COMPLIANT_LO_RELAX = {
    "pid_single": {"ki": 10.0},
    "lqr": {"q_angle": 1.0, "q_rate": 0.01, "r": 0.0001},
}


def _compliant_plant(plant: dict[str, Any]) -> bool:
    return plant.get("transmission_stiffness_nms_per_rad") is not None


TUNE_SPACE: dict[str, tuple[tuple[str, ...], Callable[[dict[str, Any]],
                                                       dict[str, float]],
                             np.ndarray, np.ndarray]] = {}


def _seed_pid(plant: dict[str, Any]) -> dict[str, float]:
    if _compliant_plant(plant):
        return analytic_pid_compliant(
            float(plant.get("inertia", 0.6)), float(plant.get("damping", 4.0)),
            float(plant["transmission_stiffness_nms_per_rad"]),
            frac=float(plant.get("motor_inertia_fraction", 0.2)))
    return analytic_pid(0.6, 4.0)


def _seed_cascade(plant: dict[str, Any]) -> dict[str, float]:
    if _compliant_plant(plant):
        return analytic_cascade_compliant(
            float(plant.get("inertia", 0.6)), float(plant.get("damping", 4.0)),
            float(plant["transmission_stiffness_nms_per_rad"]),
            frac=float(plant.get("motor_inertia_fraction", 0.2)))
    return analytic_cascade(0.6, 4.0)


def _seed_lqr(plant: dict[str, Any]) -> dict[str, float]:
    if _compliant_plant(plant):
        return lqr_compliant_weights()
    return {"q_integral": 400.0, "q_angle": 1200.0, "q_rate": 1.0, "r": 0.01}


def _register_spaces() -> None:
    TUNE_SPACE["pid_single"] = (
        ("kp", "ki", "kd"),
        _seed_pid,
        np.array([100.0, 200.0, 5.0]),
        np.array([5000.0, 30000.0, 300.0]),
    )
    TUNE_SPACE["pid_cascade"] = (
        ("kp_pos", "ki_pos", "kp_vel", "ki_vel"),
        _seed_cascade,
        np.array([2.0, 1.0, 5.0, 10.0]),
        np.array([150.0, 500.0, 200.0, 3000.0]),
    )
    TUNE_SPACE["lqr"] = (
        ("q_integral", "q_angle", "q_rate", "r"),
        _seed_lqr,
        np.array([50.0, 100.0, 0.1, 0.001]),
        np.array([20000.0, 20000.0, 50.0, 0.5]),
    )


_register_spaces()


def tune(
    controller: str,
    *,
    max_evals: int = 120,
    plant_kwargs: dict[str, Any] | None = None,
) -> TuningResult:
    """Analytic baseline → black-box refinement → 110 % acceptance gate.

    With a compliant plant (``transmission_stiffness_nms_per_rad`` set) the
    analytic channel switches to the resonance-safe designs and the
    rate-channel low-pass ``deriv_tau_s`` joins the search axis."""
    if controller not in TUNE_SPACE:
        raise ValueError(
            f"controller {controller!r} has no tuning space; known: "
            f"{sorted(TUNE_SPACE)}")
    names, analytic_fn, lo, hi = TUNE_SPACE[controller]
    plant = plant_kwargs or {}
    analytic = analytic_fn(plant)
    if _compliant_plant(plant):
        if "deriv_tau_s" not in names:
            names = (*names, "deriv_tau_s")
            lo = np.append(lo, COMPLIANT_RATE_TAU_BOUNDS[0])
            hi = np.append(hi, COMPLIANT_RATE_TAU_BOUNDS[1])
            analytic.setdefault("deriv_tau_s", 0.01)
        lo = np.asarray(lo, dtype=float).copy()
        hi = np.asarray(hi, dtype=float).copy()
        for key, value in COMPLIANT_LO_RELAX.get(controller, {}).items():
            lo[names.index(key)] = min(float(value), lo[names.index(key)])
    cost_a = step_cost(controller, analytic, **plant)

    x0 = np.array([analytic[n] for n in names], dtype=float)
    cache: dict[tuple[float, ...], float] = {}

    def f(x: np.ndarray) -> float:
        key = tuple(np.round(x, 9))
        if key in cache:
            return cache[key]
        kwargs = {n: float(v) for n, v in zip(names, x, strict=True)}
        val = step_cost(controller, kwargs, **plant)
        cache[key] = val
        return val

    x_best, cost_t, n_evals = nelder_mead(f, x0, lo=lo, hi=hi, max_evals=max_evals)
    tuned = {n: float(v) for n, v in zip(names, x_best, strict=True)}
    accepted = cost_t <= ACCEPTANCE_RATIO * cost_a
    note = ""
    if not accepted:
        note = ("black-box search did not beat the analytic design by the "
                f"{ACCEPTANCE_RATIO:.0%} bar — the analytic gains stand, and "
                "this result is reported, not crowned")
        tuned, cost_t = analytic, cost_a
    return TuningResult(
        controller=controller, cost_analytic=cost_a, cost_tuned=cost_t,
        analytic_params=analytic, tuned_params=tuned, n_evals=n_evals,
        accepted=accepted, note=note,
    )
