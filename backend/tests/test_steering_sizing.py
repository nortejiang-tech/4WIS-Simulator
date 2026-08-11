"""Tests — actuator sizing.

Two of these are about refusing to answer. A sizing pass drives past actuator
capability on purpose, and the plant is only faithful *within* capability, so
the module has to know the difference between "this motor is too small" (which
it can say) and "here is by how much" (which, past saturation, it cannot).
"""

from __future__ import annotations

import dataclasses

import pytest

from sim4wis.core.state import VehicleParams
from sim4wis.steering.sizing import (
    DEFAULT_SCENARIOS,
    SATURATION_TRUST_LIMIT,
    run_scenario,
    size_actuator,
)


def _params(**motor):
    p = VehicleParams()
    st = dataclasses.replace(p.steering_system, enabled=True)
    if motor:
        st = dataclasses.replace(st, motor=dataclasses.replace(st.motor, **motor))
    return dataclasses.replace(p, steering_system=st)


LOW_SPEED = next(s for s in DEFAULT_SCENARIOS if s.id == "low_speed_manoeuvre")


class TestScenarioLibrary:
    def test_each_scenario_says_what_it_exposes(self):
        # A case nobody can explain the purpose of is a case nobody maintains.
        assert all(s.exposes for s in DEFAULT_SCENARIOS)
        assert {s.exposes for s in DEFAULT_SCENARIOS} != {DEFAULT_SCENARIOS[0].exposes}

    def test_the_library_covers_torque_speed_and_thermal(self):
        ids = {s.id for s in DEFAULT_SCENARIOS}
        assert {"parking_full_lock", "evasive_at_speed", "parking_repeat"} <= ids
        assert next(s for s in DEFAULT_SCENARIOS if s.id == "parking_repeat").cycles > 1


class TestWithinCapability:
    def test_a_manoeuvre_the_motor_covers_produces_usable_numbers(self):
        r = run_scenario(_params(), LOW_SPEED)
        assert not r.beyond_capability
        assert 0.5 < r.peak_motor_torque < _params().steering_system.motor.peak_torque
        assert 0.0 < r.peak_hand_torque < 20.0, r.peak_hand_torque
        assert r.rms_motor_torque <= r.peak_motor_torque

    def test_load_comes_from_the_platform_not_from_a_constant(self):
        # Halving grip must reduce the rack force the actuator works against,
        # because the load is read from the same quasi-static model the load
        # page uses rather than assumed.
        high = run_scenario(_params(), LOW_SPEED)
        low = run_scenario(_params(), dataclasses.replace(LOW_SPEED, mu=0.4))
        assert low.peak_rack_force < high.peak_rack_force

    def test_a_bigger_motor_needs_less_of_the_driver(self):
        # Both sizes must be *within* capability or the comparison is being
        # made against an oscillation — the module's own rule, and this test
        # broke it on the first attempt by reaching for a 4 N.m motor.
        small = run_scenario(_params(peak_torque=8.0), LOW_SPEED)
        big = run_scenario(_params(peak_torque=16.0), LOW_SPEED)
        assert not small.beyond_capability and not big.beyond_capability
        assert big.peak_hand_torque < small.peak_hand_torque


class TestBeyondCapability:
    def test_an_undersized_motor_is_flagged_rather_than_quantified(self):
        """The honest refusal.

        Past saturation the plant rings instead of going heavy, so its peak
        numbers stop meaning anything. The module may still say "too small" —
        that conclusion survives — but it must mark the run so nobody reads a
        margin off an oscillation.
        """
        r = run_scenario(_params(peak_torque=1.0), LOW_SPEED)
        assert r.saturated_fraction > SATURATION_TRUST_LIMIT
        assert r.beyond_capability
        assert "不可信" in r.to_dict()["note"]

    def test_the_verdict_fails_and_says_why(self):
        req = size_actuator(_params(peak_torque=1.0))
        assert req.verdict["pass"] is False
        assert req.verdict["beyond_capability_in"]
        assert "更大的电机" in req.verdict["note"]

    def test_a_capable_run_carries_no_note(self):
        assert "note" not in run_scenario(_params(), LOW_SPEED).to_dict()


class TestRequirement:
    def test_it_names_which_manoeuvre_drove_each_number(self):
        # "6.9 N.m" is not a requirement; "6.9 N.m, from full-lock parking" is.
        req = size_actuator(_params())
        assert set(req.driven_by) == {"peak_torque", "peak_speed", "peak_power", "thermal"}
        assert all(v in {s.id for s in DEFAULT_SCENARIOS} for v in req.driven_by.values())

    def test_rms_is_duty_weighted_not_a_plain_average(self):
        # Averaging the test cases would weight parking like motorway driving.
        req = size_actuator(_params(), scenarios=(LOW_SPEED,))
        only = req.scenarios[0]
        assert req.rms_torque == pytest.approx(only.rms_motor_torque, rel=1e-9)

    def test_speed_is_checked_against_the_no_load_rating(self):
        req = size_actuator(_params(), scenarios=(LOW_SPEED,))
        check = req.verdict["checks"]["speed"]
        assert check["available"] == pytest.approx(
            _params().steering_system.motor.no_load_speed_rpm * 2 * 3.141592653589793 / 60,
            rel=1e-6,
        )

    def test_a_comfortable_motor_passes_every_headline_check(self):
        req = size_actuator(_params(peak_torque=20.0, continuous_torque=10.0,
                                    no_load_speed_rpm=20_000.0),
                            scenarios=(LOW_SPEED,))
        assert all(c["pass"] for c in req.verdict["checks"].values())
