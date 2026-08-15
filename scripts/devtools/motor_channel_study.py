"""Motor-side sensor channel study — the colocated-feedback question.

Direction 3's compliant-plant work found the wheel-rate feedback paths
excite the two-mass resonance, and asked whether a motor-side sensor
(the resolver a real corner module carries) is the fundamental fix. This
script answers it with three structures on the k=1200 N·m/rad plant:

    1. motor-rate damping   PI position on the wheel + D on the motor rate
    2. motor-angle feedback PID position on the MOTOR angle (colocated)
    3. colocated cascade    wheel-position outer + motor-rate inner

Verdict (grid-pinned, deterministic): the wheel-position loop crosses the
transmission spring in every structure, so the outer bandwidth is bounded
by the anti-resonance dynamics (~2-3 Hz at k=1200) — the shipped
wheel-side design (wn ~ 20 at the vehicle level, rate low-pass) is already
near that ceiling. The colocated structures add ~10-20 % corner bandwidth
at best and are WORSE across backlash (the position loop still crosses the
dead zone; motor-angle feedback cannot even see it — 14-129 % steady wheel
error). The honest levers are a stiffer transmission or a dual-loop with a
slow wheel trim — both flagged, neither is control-layer plumbing. No
protocol change is shipped on the strength of these numbers.

Deterministic; ~30 s.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend" / "src"))

import numpy as np  # noqa: E402

from sim4wis.steering.tracking.feedback import AngleSensor  # noqa: E402
from sim4wis.steering.tracking.plant import CornerActuatorPlant  # noqa: E402
from sim4wis.steering.tracking.controllers import pole_place_pid  # noqa: E402

K = 1200.0
TARGET = 0.01745
W_RES = math.sqrt(K * 0.6 / (0.12 * 0.48))


def _plant(backlash: float) -> CornerActuatorPlant:
    return CornerActuatorPlant(
        inertia_kgm2=0.6, damping_nms_per_rad=4.0, coulomb_friction_nm=0.5,
        peak_torque_nm=260.0, transmission_stiffness_nms_per_rad=K,
        motor_inertia_fraction=0.2, backlash_rad=backlash)


def _metrics(hist: list[float]) -> tuple[float, float]:
    h = np.asarray(hist)
    os_ = float((h.max() - TARGET) / TARGET * 100 if h.max() > TARGET else 0.0)
    ss = float(abs(h[-1] - TARGET) / TARGET * 100)
    return os_, ss


def motor_rate_damping(wn: float, motor_damp: float,
                       backlash: float = 0.0) -> tuple[float, float]:
    kp, ki, _ = pole_place_pid(0.6, 4.0, wn, 0.9)
    plant = _plant(backlash)
    sensor = AngleSensor(quant_rad=0.00017, delay_steps=1)
    integral, hist = 0.0, []
    for _ in range(6000):
        e = TARGET - sensor.measure(plant.angle)
        integral = max(-2.0, min(2.0, integral + e * 5e-4))
        plant.step(5e-4, kp * e + ki * integral - motor_damp * plant._omega_m, 3.0)
        hist.append(plant.angle)
    return _metrics(hist)


def motor_angle_feedback(wn: float, backlash: float = 0.0) -> tuple[float, float]:
    kp, ki, kd = pole_place_pid(0.6, 4.0, wn, 0.9)
    plant = _plant(backlash)
    sensor = AngleSensor(quant_rad=0.00017, delay_steps=1)
    integral, hist = 0.0, []
    for _ in range(6000):
        e = TARGET - sensor.measure(plant._theta_m)
        integral = max(-2.0, min(2.0, integral + e * 5e-4))
        plant.step(5e-4, kp * e + ki * integral - kd * plant._omega_m, 3.0)
        hist.append(plant.angle)
    return _metrics(hist)


def colocated_cascade(wn_pos: float, wc_vel: float,
                      backlash: float = 0.0) -> tuple[float, float]:
    kp_p, ki_p = 2.0 * 0.9 * wn_pos, wn_pos * wn_pos / 4.0
    kp_v, ki_v = 0.6 * wc_vel, 4.0 * wc_vel / 5.0
    plant = _plant(backlash)
    sensor = AngleSensor(quant_rad=0.00017, delay_steps=1)
    i_p, i_v, hist = 0.0, 0.0, []
    for _ in range(6000):
        e = TARGET - sensor.measure(plant.angle)
        i_p = max(-2.0, min(2.0, i_p + e * 5e-4))
        e_v = kp_p * e + ki_p * i_p - plant._omega_m
        i_v = max(-2.0, min(2.0, i_v + e_v * 5e-4))
        plant.step(5e-4, kp_v * e_v + ki_v * i_v, 3.0)
        hist.append(plant.angle)
    return _metrics(hist)


def main() -> None:
    print(f"two-mass plant: k={K} N·m/rad, resonance {W_RES/(2*math.pi):.1f} Hz, "
          f"anti-resonance {math.sqrt(K/0.48)/(2*math.pi):.1f} Hz")
    print()
    print("1. motor-rate damping — best stable (wn, damping) per target wn:")
    for wn in (15.0, 20.0, 25.0, 30.0):
        best = None
        for md in (20.0, 30.0, 40.0, 60.0, 80.0):
            os_, ss = motor_rate_damping(wn, md)
            if ss < 5.0 and (best is None or os_ < best[0]):
                best = (os_, ss, md)
        if best:
            print(f"   wn={wn:4.0f} md={best[2]:4.0f} -> os {best[0]:6.1f}%  ss {best[1]:5.2f}%")
        else:
            print(f"   wn={wn:4.0f} no stable pair")
    print("2. motor-angle position feedback (wheel error = twist + backlash):")
    for wn in (20.0, 40.0):
        for bl in (0.0, 0.01, 0.02):
            os_, ss = motor_angle_feedback(wn, bl)
            print(f"   wn={wn:4.0f} bl={bl:5.3f} -> os {os_:6.1f}%  wheel ss {ss:6.2f}%")
    print("3. colocated cascade — best inner loop per outer wn:")
    for wn_pos in (15.0, 20.0, 25.0, 30.0):
        best = None
        for wc in (30.0, 50.0, 80.0):
            os_, ss = colocated_cascade(wn_pos, wc)
            if ss < 5.0 and (best is None or os_ < best[0]):
                best = (os_, ss, wc)
        if best:
            print(f"   wn_pos={wn_pos:4.0f} wc_vel={best[2]:4.0f} -> os {best[0]:6.1f}%  ss {best[1]:5.2f}%")
        else:
            print(f"   wn_pos={wn_pos:4.0f} no stable pair")
    print("4. backlash tolerance, best corner designs (wn=20):")
    for bl in (0.0, 0.005, 0.01, 0.02):
        os_, ss = colocated_cascade(20.0, 50.0, bl)
        print(f"   cascade bl={bl:5.3f} -> os {os_:7.1f}%  ss {ss:6.2f}%")
    print()
    print("verdict: the wheel-position loop crosses the spring in every")
    print("structure; the outer bandwidth is bounded by the anti-resonance")
    print("(~2-3 Hz) and the shipped wheel-side design is already near it.")
    print("Colocated structures add ~10-20 % at best and are worse across")
    print("backlash. Levers: stiffer transmission or a slow-trim dual loop —")
    print("neither is shipped as control plumbing on these numbers.")


if __name__ == "__main__":
    main()
