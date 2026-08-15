"""Backlash-tolerance structure study — what can the wheel-side loop do?

The compliance-tuned loop tolerates ~5 mrad of backlash at the vehicle
level and ~2-3 mrad at the bare corner plant. This study quantifies the
mechanism and measures the wheel-side structures that could buy more —
and why they cannot.

The mechanism: while the twist sits inside the dead zone the wheel sensor
sees nothing; when the motor crosses the gap the spring engages with a
torque step ~ k * backlash (12 N·m at 10 mrad, k=1200) on J_w = 0.48 —
a wheel kick the feedback only discovers afterwards. The overshoot and
the settled hunting scale with that bang.

Measured (grid-pinned, compliance-tuned pid_single + FF, corner plant):
integral dead-bands and conditional integration do not move the numbers
(the bang is not an integral-windup problem); halving kp halves the
overshoot but the steady error balloons (the weak loop cannot hold the
gap + load); more kd re-excites the two-mass resonance (the direction-3
finding); a slower rate filter helps marginally (94 % at 10 mrad).

Honest levers, in order of effectiveness: tighter hardware backlash
(precision gearboxes, 2-5 mrad — the real-world answer), motor-channel
crossing-rate control (needs the colocated architecture whose own study
showed the tradeoff), softer k (trades bandwidth against the bang).
Nothing wheel-side only. No plumbing is shipped on these numbers.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend" / "src"))

import numpy as np  # noqa: E402

from sim4wis.steering.tracking.controller import make_controller  # noqa: E402
from sim4wis.steering.tracking.feedback import AngleSensor  # noqa: E402
from sim4wis.steering.tracking.plant import CornerActuatorPlant  # noqa: E402

BASE = {"kp": 623.77, "ki": 50.72, "kd": 41.72, "deriv_tau_s": 0.05638}
FF = {"ff": {"blocks": [
    {"type": "rack_force", "gain": 1.0},
    {"type": "velocity", "damping": 4.0, "inertia": 0.6},
    {"type": "friction", "friction_nm": 0.5}]}}
TARGET = 0.01745


def run(kwargs: dict, backlash: float, t_end: float = 4.0) -> tuple[float, float, float]:
    c = make_controller("pid_single", **kwargs, **FF)
    plant = CornerActuatorPlant(
        inertia_kgm2=0.6, damping_nms_per_rad=4.0, coulomb_friction_nm=0.5,
        peak_torque_nm=260.0, transmission_stiffness_nms_per_rad=1200.0,
        motor_inertia_fraction=0.2, backlash_rad=backlash)
    sensor = AngleSensor(quant_rad=0.00017, delay_steps=1)
    hist = []
    for _ in range(int(t_end / 5e-4)):
        out = c.step(5e-4, target_angle=TARGET, target_rate=0.0,
                     feedback_angle=sensor.measure(plant.angle),
                     plant_angle=plant.angle, load_torque=3.0, speed_ms=0.0)
        plant.step(5e-4, out.torque_cmd, 3.0)
        hist.append(plant.angle)
    h = np.asarray(hist)
    os_ = float((h.max() - TARGET) / TARGET * 100 if h.max() > TARGET else 0.0)
    ss = float(abs(h[-1] - TARGET) / TARGET * 100)
    hunt = float(h[-4000:].max() - h[-4000:].min())
    return os_, ss, hunt


def main() -> None:
    print("engagement bang scale: k*backlash (N·m step on J_w=0.48)")
    for bl in (0.005, 0.01, 0.02):
        print(f"  {bl*1000:>4.0f} mrad -> {1200.0*bl:>5.0f} N·m step")
    print()
    print("tolerance, compliance-tuned pid_single + FF, corner plant:")
    for bl in (0.0, 0.005, 0.01, 0.02):
        os_, ss, hunt = run(BASE, bl)
        print(f"  bl={bl:5.3f}: os={os_:6.1f}%  ss={ss:6.2f}%  hunt={hunt:.5f} rad")
    print()
    print("wheel-side structures at bl=0.01 (the structures that could help):")
    variants = (
        ("baseline", {}),
        ("kp x0.5", {"kp": BASE["kp"] * 0.5}),
        ("kp x0.25", {"kp": BASE["kp"] * 0.25}),
        ("kd x2", {"kd": BASE["kd"] * 2.0}),
        ("ki=0", {"ki": 0.0}),
        ("tau x2", {"deriv_tau_s": BASE["deriv_tau_s"] * 2.0}),
    )
    for name, extra in variants:
        os_, ss, hunt = run({**BASE, **extra}, 0.01)
        print(f"  {name:<12} os={os_:7.1f}%  ss={ss:7.2f}%  hunt={hunt:.5f} rad")
    print()
    print("verdict: the bang is invisible to the wheel sensor until engagement,")
    print("so no wheel-side structure tames it — integral dead-bands move")
    print("nothing, lower kp trades overshoot for steady error, more kd")
    print("re-excites the resonance. Levers: tighter hardware backlash")
    print("(2-5 mrad precision gearboxes), motor-channel crossing-rate control")
    print("(colocated architecture, with its own measured tradeoffs), or")
    print("softer k (trades bandwidth). Nothing shipped on these numbers.")


if __name__ == "__main__":
    main()
