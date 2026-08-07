"""Tests — `unit: front_deg` steer decoupling (work-package B3).

The point of B3 is that a validation experiment commands a physical front-wheel
angle and bypasses the driver feel layer, so re-tuning the feel layer cannot
disturb a regression baseline. These guard the traps that makes possible:

  * a strategy that ignores `steer_raw_rad` must fail loudly, not silently run
    the experiment with zero steering and call the straight line a baseline,
  * the bypass must not destroy experiment-level `mode_params`,
  * `amplitude` must be bounded against its declared unit,
  * a `normalized` step after a `front_deg` step must go back through the layer.
"""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from sim4wis.experiment.schema import (
    Experiment,
    Maneuver,
    ManeuverStep,
    SteerProfile,
)
from sim4wis.experiment.session import FRONT_DEG_STRATEGIES, SimSession


def _exp(strategy: str, steer: SteerProfile, mode_params: dict | None = None,
         model: str = "kinematic") -> Experiment:
    return Experiment(
        name="t",
        dt=0.005,
        record_hz=50,
        model_type=model,
        strategy=strategy,
        mode_params=mode_params or {},
        maneuver=Maneuver(steps=[
            ManeuverStep(name="go", duration=0.5, speed_kmh=30.0, steer=steer),
        ]),
    )


# ── 1. unsupported strategies must refuse, not fake a baseline ───────────────

def test_front_deg_on_unsupported_strategy_raises():
    exp = _exp("crab", SteerProfile(kind="constant", amplitude=2.0, unit="front_deg"))
    with pytest.raises(ValueError, match="does not support steer unit 'front_deg'"):
        SimSession(exp).run()


@pytest.mark.parametrize("strategy", sorted(FRONT_DEG_STRATEGIES))
def test_supported_strategies_actually_steer(strategy):
    """Every strategy on the allow-list must produce real steering, otherwise
    the allow-list is lying and the guard is useless."""
    exp = _exp(strategy, SteerProfile(kind="constant", amplitude=3.0, unit="front_deg"))
    r = SimSession(exp).run()
    peak = max(abs(v) for v in r.channels["delta_cmd_fl"])
    assert math.degrees(peak) > 0.5, f"{strategy} ignored steer_raw_rad"


def test_normalized_still_works_on_every_strategy():
    exp = _exp("crab", SteerProfile(kind="constant", amplitude=0.3))
    SimSession(exp).run()   # must not raise


# ── 2. the bypass must not clobber experiment-level mode_params ──────────────

def test_front_deg_preserves_experiment_mode_params():
    """Regression guard: the bypass rebuilt `driver.mode_params` from the STEP
    every tick, dropping the experiment-level dict. fault_reconfig reads
    fault_wheel / fault_time / detect_delay / v_limit_kmh from there, so it
    silently rearmed with defaults (fault at t=0) and the run read as a huge
    cross-track error rather than as a configuration failure."""
    seen: list[dict] = []
    exp = _exp(
        "fault_reconfig",
        SteerProfile(kind="constant", amplitude=1.5, unit="front_deg"),
        mode_params={"fault_wheel": 2, "fault_time": 99.0, "v_limit_kmh": 42.0},
    )
    session = SimSession(exp)
    original = session.strategy.compute

    def spy(driver, state, dt=0.0):
        seen.append(dict(driver.mode_params))
        return original(driver, state, dt)

    session.strategy.compute = spy
    session.run()

    assert seen, "strategy was never called"
    for mp in seen:
        assert mp.get("fault_wheel") == 2
        assert mp.get("fault_time") == 99.0
        assert mp.get("v_limit_kmh") == 42.0
        assert "steer_raw_rad" in mp


def test_normalized_step_clears_a_stale_raw_angle():
    """A front_deg step followed by a normalized one must go back through the
    feel layer, not keep bypassing it with the last raw angle."""
    seen: list[dict] = []
    exp = Experiment(
        name="t", dt=0.005, record_hz=50, model_type="kinematic",
        strategy="ideal_ackermann",
        maneuver=Maneuver(steps=[
            ManeuverStep(name="raw", duration=0.2, speed_kmh=30.0,
                         steer=SteerProfile(kind="constant", amplitude=2.0,
                                            unit="front_deg")),
            ManeuverStep(name="norm", duration=0.2, speed_kmh=30.0,
                         steer=SteerProfile(kind="constant", amplitude=0.1)),
        ]),
    )
    session = SimSession(exp)
    original = session.strategy.compute

    def spy(driver, state, dt=0.0):
        seen.append(("steer_raw_rad" in driver.mode_params, float(driver.steering)))
        return original(driver, state, dt)

    session.strategy.compute = spy
    session.run()

    assert seen[0][0] is True, "first step should bypass"
    assert seen[-1][0] is False, "stale steer_raw_rad leaked into the normalized step"
    assert seen[-1][1] == pytest.approx(0.1)


# ── 3. amplitude is bounded against its unit ─────────────────────────────────

def test_normalized_amplitude_is_bounded_to_unit_range():
    """Widening the field bound to allow degrees must not let a normalized
    profile carry 50 — that is permanent full lock, silently."""
    with pytest.raises(ValidationError):
        SteerProfile(kind="constant", amplitude=50.0, unit="normalized")
    with pytest.raises(ValidationError):
        SteerProfile(kind="ramp", start=5.0, amplitude=0.5, unit="normalized")


def test_front_deg_amplitude_allows_degrees():
    SteerProfile(kind="constant", amplitude=35.0, unit="front_deg")
    with pytest.raises(ValidationError):
        SteerProfile(kind="constant", amplitude=200.0, unit="front_deg")


def test_normalized_amplitude_within_range_is_fine():
    SteerProfile(kind="constant", amplitude=1.0, unit="normalized")
    SteerProfile(kind="ramp", start=-1.0, amplitude=0.25, unit="normalized")


# ── 4. the bypass measures the vehicle, not the driver mapping ───────────────

def test_front_deg_is_immune_to_feel_layer_tuning():
    """The whole reason B3 exists: changing the feel-layer knobs must not move
    a front_deg baseline by even a float ulp."""
    steer = SteerProfile(kind="constant", amplitude=2.0, unit="front_deg")
    base = _exp("ideal_ackermann", steer)
    tuned = _exp("ideal_ackermann", steer)
    tuned.vehicle.overrides = {"steer_wheel_range": 900.0, "steer_ratio_low": 2.0}

    a = SimSession(base).run().channels["delta_cmd_fl"]
    b = SimSession(tuned).run().channels["delta_cmd_fl"]
    assert a == b


def test_normalized_is_not_immune():
    """Sanity check on the test above — with the normalised unit the same
    override MUST move the result, or the bypass proves nothing."""
    steer = SteerProfile(kind="constant", amplitude=0.5)
    base = _exp("ideal_ackermann", steer)
    tuned = _exp("ideal_ackermann", steer)
    tuned.vehicle.overrides = {"steer_wheel_range": 900.0, "steer_ratio_low": 2.0}

    a = SimSession(base).run().channels["delta_cmd_fl"]
    b = SimSession(tuned).run().channels["delta_cmd_fl"]
    assert a != b
