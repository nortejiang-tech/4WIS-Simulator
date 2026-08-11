"""Tests — the by-wire front axle.

`test_kickback_is_filtered` is the one that matters. Cutting the mechanical
path is only worth doing if the road information survives and the impacts do
not; a road-feel design that passes rack force straight through has thrown away
the reason for the architecture while keeping all of its cost.
"""

from __future__ import annotations

import math

import pytest

from sim4wis.steering.bywire import ByWireParams, ByWirePlant


def _drive(plant, *, dt=0.005, n=600, delta_deg=3.0, rack_n=3000.0,
           speed_ms=25.0, rack_fn=None, hold_hand=True):
    plant.reset()
    out = []
    ratio = 7.71
    for i in range(n):
        cmd = 0.0 if i * dt < 0.2 else math.radians(delta_deg)
        hand = cmd * ratio
        prev = out[-1][0] if out else 0.0
        rate = (hand - prev) / dt
        force = rack_fn(i * dt) if rack_fn else rack_n
        s = plant.step(dt, hand_angle=hand if hold_hand else 0.0,
                       hand_rate=rate, delta_cmd=cmd,
                       rack_force=force, speed_ms=speed_ms)
        out.append((hand, s))
    return out


class TestAngleTracking:
    def test_the_wheel_reaches_the_commanded_angle(self):
        run = _drive(ByWirePlant())
        assert run[-1][1].road_wheel_angle == pytest.approx(math.radians(3.0), rel=1e-3)
        assert abs(run[-1][1].angle_deviation) < 1e-3

    def test_tracking_is_bandwidth_limited_not_instant(self):
        run = _drive(ByWirePlant(), n=60)
        mid = run[45][1]
        assert 0.0 < abs(mid.road_wheel_angle) < math.radians(3.0)

    def test_a_fast_input_hits_the_rate_limit(self):
        p = ByWireParams(rate_limit=0.2)
        run = _drive(ByWirePlant(p), delta_deg=30.0, n=120)
        assert any(s.rate_limited for _, s in run)
        assert max(abs(s.road_wheel_rate) for _, s in run) <= p.rate_limit + 1e-9

    def test_deviation_is_flagged_when_the_actuator_cannot_keep_up(self):
        # A safety-relevant signal on a by-wire car: it is how you learn the
        # actuator saturated, stalled or lost a phase.
        run = _drive(ByWirePlant(ByWireParams(rate_limit=0.05)),
                     delta_deg=30.0, n=60)
        assert any(s.deviation_flag for _, s in run)


class TestRoadFeel:
    def test_hand_torque_grows_with_rack_force(self):
        light = _drive(ByWirePlant(), rack_n=1000.0)[-1][1]
        heavy = _drive(ByWirePlant(), rack_n=6000.0)[-1][1]
        assert abs(heavy.road_feel_torque) > abs(light.road_feel_torque)
        assert abs(heavy.hand_torque) > abs(light.hand_torque)

    def test_road_feel_gain_rises_with_speed(self):
        p = ByWirePlant()
        assert p.road_feel_gain(30.0) > p.road_feel_gain(0.0)

    def test_a_by_wire_car_still_returns_to_centre(self):
        # No caster torque reaches the hand wheel, so without a synthesised
        # centring spring the wheel simply stays where it was left.
        run = _drive(ByWirePlant(), rack_n=0.0)
        assert abs(run[-1][1].centring_torque) > 0.1

    def test_centring_grows_with_speed(self):
        p = ByWirePlant()
        assert p.centring_gain(40.0) > p.centring_gain(0.0)

    def test_feedback_torque_is_capped_by_its_motor(self):
        run = _drive(ByWirePlant(), rack_n=200_000.0)
        peak = max(abs(s.hand_torque) for _, s in run)
        assert peak <= ByWireParams().feedback_peak_nm + 1e-6
        assert any(s.feedback_saturated for _, s in run)

    def test_kickback_is_filtered(self):
        """The reason to go by-wire, stated as a number.

        A 20 Hz rack-force disturbance is an impact, not information. It must
        reach the driver's hands attenuated relative to a slow input of the same
        amplitude — which a mechanical column cannot do at all, because it has
        no way to tell the two apart.

        The bound is 3x rather than something more dramatic because a
        first-order filter is all this models: 20 Hz sits 3.33 octaves-ish above
        a 6 Hz corner, so -20 dB/decade delivers 1/3.4 and no more. Production
        road-feel designs use steeper filtering, and if this model is ever asked
        a question about real impact rejection that is the first thing that has
        to change. Recording the limit here is the point.
        """
        amp = 4000.0

        def fast(t):
            return amp * math.sin(2 * math.pi * 20.0 * t)

        def slow(t):
            return amp * math.sin(2 * math.pi * 0.2 * t)

        def swing(rack_fn):
            run = _drive(ByWirePlant(), rack_fn=rack_fn, n=1200, hold_hand=False)
            tail = [s.road_feel_torque for _, s in run[len(run) // 2:]]
            return max(tail) - min(tail)

        assert swing(fast) < swing(slow) / 3.0

    def test_the_filter_is_what_does_it(self):
        # Same test with the low-pass opened right up: the attenuation goes.
        amp = 4000.0

        def fast(t):
            return amp * math.sin(2 * math.pi * 20.0 * t)

        wide = ByWirePlant(ByWireParams(road_feel_lowpass_hz=400.0))
        run = _drive(wide, rack_fn=fast, n=1200, hold_hand=False)
        tail = [s.road_feel_torque for _, s in run[len(run) // 2:]]
        assert (max(tail) - min(tail)) > 0.5


def test_channels_do_not_claim_a_torque_sensor():
    """There is no torsion bar, so there is no torque-sensor signal to report."""
    ch = ByWirePlant().step(0.005, hand_angle=0.1, hand_rate=0.0,
                            delta_cmd=0.01, rack_force=1000.0,
                            speed_ms=20.0).to_channels()
    assert "steer_torque_sensor" not in ch
    assert "steer_hand_torque" in ch and "steer_angle_deviation" in ch
