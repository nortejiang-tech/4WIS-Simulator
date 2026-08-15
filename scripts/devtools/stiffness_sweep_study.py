"""Transmission-stiffness sweep study — how much bandwidth does k buy?

The motor-channel study showed the compliant wheel-position loop is bounded
by the transmission dynamics; this study quantifies the lever on the other
side: raising the coupling stiffness k (resonance and anti-resonance both
rise as sqrt(k)). Four questions, all deterministic:

    Q1  the k=1200 resonance-safe rule (wn = w_res/8, tau = 5/w_res) —
        does its quality transfer to stiffer transmissions?
    Q2  the stable-bandwidth ceiling vs k, with the plant's fixed viscous
        damping, and with damping scaled sqrt(k) (structural damping, the
        material-realistic case)
    Q3  how stiff must the transmission be for the compliant loop to match
        the rigid loop's corner quality?
    Q4  vehicle-level spot check of the rule gains at k = 1200/2400/4800

Findings (pinned): with the fixed viscous damping the ceiling stalls at
~2.5-3.1 Hz (the resonant mode loses damping as w_res rises); with
damping scaling sqrt(k) the ceiling scales with sqrt(k) up to the sampling
limit w_res*h <= ~0.2 (at the 2000 Hz inner rate, f_res <= ~60 Hz, i.e.
k <= ~35 000). Transmission spec takeaway: stiffness alone does not buy
bandwidth — it must come with damping that scales with it (real belts and
gearboxes carry structural damping, zeta ~ 0.01-0.05; the plant's fixed
viscous b=4 is the conservative case).
"""

from __future__ import annotations

import copy
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend" / "src"))

import numpy as np  # noqa: E402

from sim4wis.calibration.identify import (  # noqa: E402
    _load_procedure_raw,
    run_sim_with_values,
)
from sim4wis.steering.tracking.feedback import AngleSensor  # noqa: E402
from sim4wis.steering.tracking.plant import CornerActuatorPlant  # noqa: E402
from sim4wis.steering.tracking.controllers import pole_place_pid  # noqa: E402
from sim4wis.study.tracking_step import _analyse_one  # noqa: E402

KS = (1200.0, 2400.0, 4800.0, 9600.0, 19200.0, 38400.0)
DT = 5.0e-4
TARGET = 0.01745


class _RateLpf:
    """The controller's first-order filtered difference, reproduced."""

    def __init__(self, tau: float) -> None:
        self.tau, self.value, self.prev = tau, 0.0, None

    def update(self, v: float, dt: float) -> float:
        if self.prev is None:
            self.prev = float(v)
            return 0.0
        raw = (float(v) - self.prev) / dt
        self.prev = float(v)
        self.value += (raw - self.value) * dt / (self.tau + dt)
        return self.value


def _w_res(k: float) -> float:
    return math.sqrt(k * 0.6 / (0.12 * 0.48))


def corner_step(k: float, wn: float, tau: float, *, b_scale: float = 1.0,
                dt: float = DT, t_end: float = 3.0) -> tuple[float, float, float]:
    kp, ki, kd = pole_place_pid(0.6, 4.0 * b_scale, wn, 0.9)
    plant = CornerActuatorPlant(
        inertia_kgm2=0.6, damping_nms_per_rad=4.0 * b_scale,
        coulomb_friction_nm=0.5, peak_torque_nm=260.0,
        transmission_stiffness_nms_per_rad=k, motor_inertia_fraction=0.2)
    sensor = AngleSensor(quant_rad=0.00017, delay_steps=1)
    lpf = _RateLpf(tau)
    integral, hist = 0.0, []
    for _ in range(int(t_end / dt)):
        fb = sensor.measure(plant.angle)
        e = TARGET - fb
        integral = max(-2.0, min(2.0, integral + e * dt))
        plant.step(dt, kp * e + ki * integral - kd * lpf.update(fb, dt), 3.0)
        hist.append(plant.angle)
    h = np.asarray(hist)
    os_ = float((h.max() - TARGET) / TARGET * 100 if h.max() > TARGET else 0.0)
    err = np.abs(h - TARGET) / TARGET
    settle = float((np.where(err > 0.02)[0][-1] + 1) * dt
                   if (err > 0.02).any() else 0.0)
    ss = float(abs(h[-1] - TARGET) / TARGET * 100)
    return os_, settle, ss


def vehicle_step(k: float, wn: float, tau: float) -> tuple[float, float, float]:
    kp, ki, kd = pole_place_pid(0.6, 4.0, wn, 0.9)
    raw = _load_procedure_raw(
        str(ROOT / "procedures" / "tracking_step_response.yaml"))
    spec = copy.deepcopy(raw)
    spec["sweep"] = {}
    ac = spec["baseline"]["vehicle"]["overrides"]["steering_system"]["angle_control"]
    ac["controller"] = "pid_single"
    ac["controller_kwargs"] = {
        "ff": {"blocks": [
            {"type": "rack_force", "gain": 1.0},
            {"type": "velocity", "damping": 4.0, "inertia": 0.6},
            {"type": "friction", "friction_nm": 0.5}]},
        "kp": kp, "ki": ki, "kd": kd, "deriv_tau_s": tau}
    ac["plant_transmission_stiffness_nms_per_rad"] = k
    ac["plant_motor_inertia_fraction"] = 0.2
    t, ch = run_sim_with_values(spec, {})
    m = _analyse_one(ch["delta_cmd_fl"], ch["delta_fl"],
                     float(np.median(np.diff(t))))
    return m.rise_s, m.overshoot_pct, m.settle_s


def main() -> None:
    print("Q1: rule design (wn=w_res/8, tau=5/w_res) quality vs k, fixed damping")
    print(f"{'k':>8} {'f_res':>7} {'f_ar':>6} {'wn':>7} {'os %':>7} {'settle':>7}")
    for k in KS:
        w_res = _w_res(k)
        os_, settle, _ = corner_step(k, w_res / 8.0, 5.0 / w_res)
        print(f"{k:>8.0f} {w_res/(2*math.pi):>7.1f} "
              f"{math.sqrt(k/0.48)/(2*math.pi):>6.1f} {w_res/8.0:>7.1f} "
              f"{os_:>7.1f} {settle:>7.2f}")
    print()
    print("Q2: stable-bandwidth ceiling vs k (os<=25 %, ss<=5 %), two damping models")
    for label, scale in (("fixed viscous b", None), ("damping ~ sqrt(k)", "sqrt")):
        print(f"  [{label}]")
        for k in KS:
            w_res = _w_res(k)
            bs = math.sqrt(k / 1200.0) if scale else 1.0
            best = None
            for frac in (1/8, 1/7, 1/6, 1/5, 1/4):
                for tr in (2.0, 3.0, 4.0, 5.0):
                    os_, settle, ss = corner_step(k, w_res * frac, tr / w_res,
                                                  b_scale=bs)
                    if os_ <= 25.0 and ss <= 5.0 and (best is None
                                                      or w_res * frac > best[0]):
                        best = (w_res * frac, os_, settle, frac, tr)
            if best:
                print(f"    k={k:>6.0f}: wn={best[0]:5.1f} "
                      f"({best[0]/(2*math.pi):4.1f} Hz) os={best[1]:5.1f}% "
                      f"settle={best[2]:4.2f} frac={best[3]:.3f}")
            else:
                print(f"    k={k:>6.0f}: none stable in grid")
    print()
    print("Q2c: the sampling limit (k=19200, damping ~ sqrt(k), wn=w_res/8)")
    k, w_res = 19200.0, _w_res(19200.0)
    bs = math.sqrt(k / 1200.0)
    for dt in (5e-4, 2.5e-4, 1e-4):
        os_, settle, _ = corner_step(k, w_res / 8.0, 4.0 / w_res, b_scale=bs, dt=dt)
        print(f"    dt={dt:.0e} (w_res*h={w_res*dt:.3f}): os={os_:5.1f}% settle={settle:4.2f}s")
    print()
    print("Q3: rigid-parity corner reference (rigid plant, wn=25)")
    # the rigid plant with the standard analytic design
    plant = CornerActuatorPlant(inertia_kgm2=0.6, damping_nms_per_rad=4.0,
                                coulomb_friction_nm=0.5, peak_torque_nm=260.0)
    kp, ki, kd = pole_place_pid(0.6, 4.0, 25.0, 0.9)
    sensor = AngleSensor(quant_rad=0.00017, delay_steps=1)
    lpf = _RateLpf(0.01)
    integral, hist = 0.0, []
    for _ in range(6000):
        fb = sensor.measure(plant.angle)
        e = TARGET - fb
        integral = max(-2.0, min(2.0, integral + e * DT))
        plant.step(DT, kp * e + ki * integral - kd * lpf.update(fb, DT), 3.0)
        hist.append(plant.angle)
    h = np.asarray(hist)
    err = np.abs(h - TARGET) / TARGET
    settle = float((np.where(err > 0.02)[0][-1] + 1) * DT if (err > 0.02).any() else 0.0)
    print(f"    rigid: wn=25 -> settle {settle:.2f}s; parity needs wn>=25 with "
          "bounded overshoot (see Q2)")
    print()
    print("Q4: vehicle-level spot check, rule gains (wn=w_res/8), fixed b")
    for k in (1200.0, 2400.0, 4800.0):
        w_res = _w_res(k)
        for tr in (3.0, 4.0, 5.0):
            try:
                rise, os_, settle = vehicle_step(k, w_res / 8.0, tr / w_res)
                print(f"    k={k:>6.0f} tau={tr:.0f}/w_res: rise={rise:.2f}s "
                      f"os={os_:6.1f}% settle={settle:.2f}s")
            except Exception as e:  # noqa: BLE001 — report the honest refusal
                print(f"    k={k:>6.0f} tau={tr:.0f}/w_res: "
                      f"{type(e).__name__}: {str(e)[:60]}")
    print()
    print("transmission spec takeaway: stiffness alone does not buy bandwidth —")
    print("the fixed viscous damping leaves the resonance progressively")
    print("underdamped (ceiling stalls ~3 Hz); with damping scaling sqrt(k)")
    print("the ceiling scales sqrt(k) until the sampling limit w_res*h <= ~0.2")
    print("(f_res <= ~60 Hz at the 2000 Hz inner rate). Real belts/gearboxes")
    print("carry structural damping (zeta ~ 0.01-0.05) — that is the missing")
    print("ingredient, not k itself.")


if __name__ == "__main__":
    main()
