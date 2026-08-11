"""Tests — the steering system plant.

The centrepiece is `test_assist_reduces_effort_at_parking`. It is a regression
guard for a defect that shipped in this package once: assist without a damping
term multiplies the effective stiffness by the boost ratio, the loop rings, and
assist *raises* parking effort instead of lowering it. That failure looked like
a plausible model right up until someone read the number.
"""

from __future__ import annotations

import math

import pytest

from sim4wis.steering.assist import DEFAULT_SPEED_BP, DEFAULT_TAU_BP, AssistMap, AssistMapError, get
from sim4wis.steering.motor import Motor
from sim4wis.steering.params import SteeringSystemParams
from sim4wis.steering.plant import SteeringPlant

PARK_RACK_N = 10_000.0


def _plant(**over) -> SteeringPlant:
    p = SteeringSystemParams(enabled=True)
    for k, v in over.items():
        setattr(p, k, v)
    return SteeringPlant(params=p, assist_map=get("default"))


def _sweep(plant: SteeringPlant, *, rack_n: float, speed_ms: float,
           amp_deg: float, freq: float = 0.2, dt: float = 0.005, cycles: float = 2.0,
           assist: bool = True):
    """Impose a sinusoidal hand-wheel angle against an angle-proportional load."""
    plant.reset()
    n = int(round(cycles / freq / dt))
    amp = math.radians(amp_deg)
    out = []
    for i in range(n):
        t = i * dt
        theta = amp * math.sin(2.0 * math.pi * freq * t)
        omega = amp * 2.0 * math.pi * freq * math.cos(2.0 * math.pi * freq * t)
        load = rack_n * (plant.state.pinion_angle / amp)
        s = plant.step(dt, hand_angle=theta, hand_rate=omega,
                       rack_force=load, speed_ms=speed_ms, assist_enabled=assist)
        out.append((math.degrees(theta), s))
    return out


class TestAssistMap:
    def test_deadband_then_progressive_rise(self):
        m = get("default")
        assert m.assist_torque(0.1, 0.0) == 0.0
        rising = [m.assist_torque(t, 0.0) for t in (0.5, 1.5, 2.5, 3.5)]
        assert all(b > a for a, b in zip(rising, rising[1:], strict=False))

    def test_sign_follows_the_driver(self):
        m = get("default")
        assert m.assist_torque(-2.0, 0.0) == pytest.approx(-m.assist_torque(2.0, 0.0))

    def test_assist_falls_with_speed(self):
        m = get("default")
        speeds = [m.assist_torque(2.5, v / 3.6) for v in (0, 20, 60, 120)]
        assert all(b < a for a, b in zip(speeds, speeds[1:], strict=False))

    def test_parking_effort_is_anchored_to_the_platforms_load_model(self):
        """Not to a guess.

        `sweep_load_analysis` puts full-lock parking at mu = 0.9 at 20.7 kN on
        the rack for the default vehicle. The first version of this map was
        sized from an assumed 10.4 kN and left 163 N.m of the real load on the
        driver; the sizing pass found it. The two models must agree.
        """
        from sim4wis.core.state import VehicleParams
        from sim4wis.vehicle.load_analysis import sweep_load_analysis

        p = VehicleParams()
        row = sweep_load_analysis(
            p, speeds=[0.0], angles=[p.steer_limit], wheel_index=0, mu=0.9,
        )["rows"][0]
        needed = 2.0 * abs(row["rack_force"]) * p.pinion_radius
        delivered = 3.5 + get("default").assist_torque(3.5, 0.0)
        # Within a hand-wheel effort of each other, which is the calibration.
        assert delivered == pytest.approx(needed, rel=0.2)

    def test_the_unassisted_baseline_gives_nothing(self):
        assert get("none").assist_torque(8.0, 0.0) == 0.0

    def test_a_curve_that_dips_is_refused(self):
        # More driver effort buying less help is felt as a catch — a defect.
        with pytest.raises(AssistMapError, match="catch"):
            AssistMap(
                name="dip",
                table=tuple(
                    (0.0, 0.0, 25.0, 75.0, 60.0, 205.0, 235.0, 250.0) if i == 0 else row
                    for i, row in enumerate(AssistMap().table)
                ),
            )

    @pytest.mark.parametrize("kw,msg", [
        ({"tau_bp": (0.5, 1.0, 2.0)}, "expected"),
        ({"table": ((0.0, 1.0),)}, "expected"),
    ])
    def test_malformed_tables_are_refused(self, kw, msg):
        with pytest.raises(AssistMapError, match=msg):
            AssistMap(**kw)

    def test_scaling_is_the_sweep_handle(self):
        base, half = get("default"), get("default").scaled(0.5)
        assert half.assist_torque(2.5, 0.0) == pytest.approx(base.assist_torque(2.5, 0.0) / 2)

    def test_round_trips_through_a_dict(self):
        m = get("default")
        assert AssistMap.from_dict(m.to_dict()).assist_torque(2.5, 5.0) == pytest.approx(
            m.assist_torque(2.5, 5.0)
        )

    def test_breakpoints_cover_the_calibration_range(self):
        assert DEFAULT_TAU_BP[0] == 0.0 and DEFAULT_TAU_BP[-1] >= 8.0
        assert DEFAULT_SPEED_BP[-1] >= 120.0


class TestMotor:
    def test_torque_is_capped_at_peak(self):
        m = Motor(SteeringSystemParams().motor)
        for _ in range(200):
            s = m.step(0.001, 50.0, 0.0)
        assert s.torque == pytest.approx(m.p.peak_torque, rel=1e-3)
        assert s.saturated

    def test_available_torque_falls_to_zero_at_no_load_speed(self):
        m = Motor(SteeringSystemParams().motor)
        w_max = m.p.no_load_speed_rpm * 2 * math.pi / 60
        assert m.available_torque(0.0, 0.0) == pytest.approx(m.p.peak_torque)
        assert m.available_torque(w_max, 0.0) == pytest.approx(0.0, abs=1e-9)
        assert 0 < m.available_torque(w_max / 2, 0.0) < m.p.peak_torque

    def test_sustained_overload_derates(self):
        m = Motor(SteeringSystemParams().motor)
        cold = m.available_torque(0.0, 0.0)
        for _ in range(60_000):                       # 60 s at 1 ms
            m.step(0.001, m.p.peak_torque, 0.0)
        assert m.state.heat > 1.0
        assert m.available_torque(0.0, m.state.heat) < cold

    def test_torque_follows_the_command_at_its_bandwidth(self):
        m = Motor(SteeringSystemParams().motor)
        tau = 1.0 / (2 * math.pi * m.p.bandwidth_hz)
        for _ in range(int(tau / 1e-4)):
            s = m.step(1e-4, 3.0, 0.0)
        assert s.torque == pytest.approx(3.0 * (1 - math.exp(-1.0)), rel=0.05)

    def test_current_tracks_torque_through_kt(self):
        m = Motor(SteeringSystemParams().motor)
        s = m.step(0.01, 2.0, 0.0)
        assert s.current == pytest.approx(s.torque / m.p.torque_constant)


class TestPlant:
    def test_assist_reduces_effort_at_parking(self):
        """The regression guard. Assist must never make steering heavier.

        Shipped once as 96.5 N.m unassisted against 174.3 N.m assisted, because
        the assist loop had no damping and rang.
        """
        with_assist = _sweep(_plant(), rack_n=PARK_RACK_N, speed_ms=0.0,
                             amp_deg=90.0, assist=True)
        without = _sweep(_plant(), rack_n=PARK_RACK_N, speed_ms=0.0,
                         amp_deg=90.0, assist=False)
        peak_on = max(abs(s.hand_torque) for _, s in with_assist)
        peak_off = max(abs(s.hand_torque) for _, s in without)
        assert peak_on < peak_off / 5.0, f"assist {peak_on:.1f} vs unassisted {peak_off:.1f} N.m"
        assert peak_on < 10.0, f"{peak_on:.1f} N.m is not a power-assisted car"

    def test_an_undamped_assist_loop_diverges(self):
        """Pins *why* the damping term exists, not just that it is there."""
        undamped = _sweep(_plant(assist_damping_ratio=0.0), rack_n=PARK_RACK_N,
                          speed_ms=0.0, amp_deg=90.0)
        damped = _sweep(_plant(), rack_n=PARK_RACK_N, speed_ms=0.0, amp_deg=90.0)
        assert (max(abs(s.hand_torque) for _, s in undamped)
                > 5.0 * max(abs(s.hand_torque) for _, s in damped))

    def test_the_motor_stays_inside_its_envelope_at_parking(self):
        run = _sweep(_plant(), rack_n=PARK_RACK_N, speed_ms=0.0, amp_deg=90.0)
        peak = max(abs(s.motor_torque) for _, s in run)
        assert peak <= SteeringSystemParams().motor.peak_torque + 1e-6
        assert peak > 1.0, "a 10 kN parking load should be working the motor"

    def test_on_centre_effort_is_small_at_speed(self):
        run = _sweep(_plant(), rack_n=2000.0, speed_ms=100 / 3.6, amp_deg=10.0)
        assert max(abs(s.hand_torque) for _, s in run) < 4.0

    def test_friction_produces_a_hysteresis_loop(self):
        """The mechanism ISO 13674 measures — and what mu*sign(omega) cannot do."""
        run = _sweep(_plant(), rack_n=1500.0, speed_ms=100 / 3.6, amp_deg=15.0)
        tail = run[len(run) // 2:]
        area = 0.0
        for (a_deg, a), (b_deg, b) in zip(tail, tail[1:], strict=False):
            area += 0.5 * (a.hand_torque + b.hand_torque) * (b_deg - a_deg)
        assert abs(area) > 1.0, "no hysteresis means no on-centre feel to measure"

    def test_the_pinion_sticks_under_a_torque_friction_can_hold(self):
        p = _plant()
        p.reset()
        for _ in range(50):
            s = p.step(0.005, hand_angle=1e-4, hand_rate=0.0,
                       rack_force=0.0, speed_ms=0.0, assist_enabled=False)
        assert s.stuck and abs(s.pinion_angle) < 1e-6

    def test_substepping_matches_a_finer_outer_step(self):
        coarse = _sweep(_plant(), rack_n=4000.0, speed_ms=10.0, amp_deg=30.0, dt=0.005)
        fine = _sweep(_plant(), rack_n=4000.0, speed_ms=10.0, amp_deg=30.0, dt=0.001)
        pc = max(abs(s.hand_torque) for _, s in coarse)
        pf = max(abs(s.hand_torque) for _, s in fine)
        assert pc == pytest.approx(pf, rel=0.15), f"{pc:.2f} vs {pf:.2f}"

    def test_assist_raises_the_mode_the_step_has_to_carry(self):
        p = _plant()
        boost = get("default").boost_ratio(3.5, 0.0)
        assert p.assisted_mode_hz(boost) > 5.0 * p.natural_frequency_hz()

    def test_channels_carry_the_signal_that_did_not_exist_before(self):
        run = _sweep(_plant(), rack_n=4000.0, speed_ms=10.0, amp_deg=30.0)
        ch = run[-1][1].to_channels()
        assert "steer_hand_torque" in ch and "steer_torque_sensor" in ch
        assert all(isinstance(v, float) for v in ch.values())


def test_the_plant_is_off_by_default():
    """The whole compatibility promise in one assertion."""
    assert SteeringSystemParams().enabled is False
    assert SteeringSystemParams().describe()["enabled"] is False
