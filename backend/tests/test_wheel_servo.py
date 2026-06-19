"""Unit tests — per-wheel PI speed servo."""

from __future__ import annotations

from sim4wis.vehicle.wheel_servo import WheelSpeedServo


def test_proportional_response() -> None:
    s = WheelSpeedServo(kp=200.0, ki=0.0, torque_limit=2000.0)
    t = s.update(omega_actual=0.0, omega_cmd=1.0, dt=0.005)
    assert t > 0.0
    assert t <= 2000.0


def test_output_saturation() -> None:
    s = WheelSpeedServo(torque_limit=100.0)
    t = s.update(omega_actual=0.0, omega_cmd=50.0, dt=0.005)
    assert t == 100.0
    t = s.update(omega_actual=50.0, omega_cmd=0.0, dt=0.005)
    assert t == -100.0


def test_integral_accumulates_and_resets() -> None:
    s = WheelSpeedServo(kp=0.0, ki=50.0, torque_limit=2000.0)
    for _ in range(100):
        s.update(omega_actual=0.0, omega_cmd=1.0, dt=0.01)
    assert s.integral > 0.0
    t_before = s.update(0.0, 1.0, 0.01)
    assert t_before > 0.0
    s.reset()
    assert s.integral == 0.0
    # After reset with zero error → zero torque (no inherited integral kick).
    assert s.update(omega_actual=1.0, omega_cmd=1.0, dt=0.01) == 0.0


def test_anti_windup_clamps_integral() -> None:
    s = WheelSpeedServo(integral_limit=50.0)
    for _ in range(10_000):
        s.update(omega_actual=0.0, omega_cmd=100.0, dt=0.01)
    assert abs(s.integral) <= 50.0
