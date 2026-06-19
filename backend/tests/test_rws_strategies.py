"""Tests for the unified rear-wheel-steering strategy (rear_wheel_steer).

Covered:
  * registry exposes the unified strategy
  * zero_sideslip_ratio: counter-phase low speed, in-phase high speed, crossover
  * each mode produces the expected rear-phase behaviour
  * yaw-feedback is STABLE on the kinematic model (regression for the ±limit
    algebraic-loop wobble fixed by the low-pass filter)
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from sim4wis.controller.registry import available_strategies, make_strategy
from sim4wis.controller.rws_common import zero_sideslip_ratio
from sim4wis.core.state import DriverInput, EnvironmentState, VehicleParams, VehicleState
from sim4wis.environment.disturbance import Scene
from sim4wis.vehicle.kinematic import KinematicModel

DT = 0.005


@pytest.fixture
def params() -> VehicleParams:
    return VehicleParams()


def _df(p: VehicleParams, steering: float) -> float:
    return p.steer_limit * steering


def _mp(mode: str, **kw) -> dict:
    return {"rws_mode": mode, **kw}


def test_registry_lists_unified_rws() -> None:
    assert "rear_wheel_steer" in available_strategies()


def test_zero_sideslip_ratio_sign_and_crossover(params: VehicleParams) -> None:
    assert zero_sideslip_ratio(params, 0.0) < 0.0
    assert zero_sideslip_ratio(params, 50.0) > 0.0
    lo, hi = 0.0, 50.0
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        (lo, hi) = (mid, hi) if zero_sideslip_ratio(params, mid) < 0 else (lo, mid)
    cross_kmh = 0.5 * (lo + hi) * 3.6
    assert 45.0 < cross_kmh < 75.0


def test_fixed_ratio_counter_phase(params: VehicleParams) -> None:
    strat = make_strategy("rear_wheel_steer", params)
    df = _df(params, 0.4)
    cmd = strat.compute(
        DriverInput(throttle=0.3, steering=0.4, mode_params=_mp("fixed_ratio", rear_ratio=-0.5)),
        VehicleState(vx=5.0),
    )
    assert np.sign(cmd.delta_cmd[2]) == -np.sign(df)


def test_speed_schedule_phase_flip(params: VehicleParams) -> None:
    strat = make_strategy("rear_wheel_steer", params)
    df = _df(params, 0.4)
    lo = strat.compute(DriverInput(throttle=0.3, steering=0.4, mode_params=_mp("speed_schedule")),
                       VehicleState(vx=0.5))
    hi = strat.compute(DriverInput(throttle=0.5, steering=0.4, mode_params=_mp("speed_schedule")),
                       VehicleState(vx=45.0))
    assert np.sign(lo.delta_cmd[2]) == -np.sign(df)   # low speed → counter
    assert np.sign(hi.delta_cmd[2]) == np.sign(df)    # high speed → in-phase


def test_speed_schedule_custom_curve(params: VehicleParams) -> None:
    strat = make_strategy("rear_wheel_steer", params)
    cmd = strat.compute(
        DriverInput(throttle=0.3, steering=0.3,
                    mode_params=_mp("speed_schedule", k_curve=[[0.0, 0.5], [200.0, 0.5]])),
        VehicleState(vx=0.5),
    )
    assert np.sign(cmd.delta_cmd[2]) == np.sign(_df(params, 0.3))  # in-phase even at low v


def test_unknown_mode_falls_back(params: VehicleParams) -> None:
    strat = make_strategy("rear_wheel_steer", params)
    cmd = strat.compute(
        DriverInput(throttle=0.3, steering=0.3, mode_params=_mp("nonsense")),
        VehicleState(vx=30.0),
    )
    assert np.all(np.isfinite(cmd.delta_cmd))


def _kinematic_jitter(mode: str, steer: float = 0.15, speed_kmh: float = 80.0) -> float:
    """Steady-state rear-angle std (deg) driving on the kinematic model."""
    p = VehicleParams()
    m = KinematicModel(p)
    sc = Scene()
    env = EnvironmentState(scene=sc, mu=sc.base_mu)
    strat = make_strategy("rear_wheel_steer", p)
    thr = (speed_kmh / 3.6) / p.v_max
    for _ in range(int(4.0 / DT)):
        m.step(DT, strat.compute(DriverInput(throttle=thr, steering=0.0, mode_params=_mp(mode)), m.state), env)
    rr = []
    for _ in range(int(6.0 / DT)):
        cmd = strat.compute(DriverInput(throttle=thr, steering=steer, mode_params=_mp(mode)), m.state)
        m.step(DT, cmd, env)
        rr.append(float(cmd.delta_cmd[2]))
    rr = np.array(rr[-int(3.0 / DT):])
    return math.degrees(rr.std())


def test_yaw_feedback_stable_on_kinematic() -> None:
    """Regression: raw yaw feedback oscillated ±limit (~33°) on kinematic;
    the low-pass filter must keep it steady."""
    assert _kinematic_jitter("yaw_feedback") < 0.5


def test_all_modes_stable_on_kinematic() -> None:
    for mode in ("fixed_ratio", "speed_schedule", "yaw_feedback", "transient", "model_following"):
        assert _kinematic_jitter(mode) < 0.5, f"{mode} jitters on kinematic"
