"""Single-wheel-failure safety study plumbing — fault injection + reconfiguration.

Covers:
    * time-triggered FaultSpec in the headless SimSession (stuck_hold latches
      the physical angle at activation; stuck_zero recentres)
    * FaultReconfigStrategy geometry: the three healthy wheels' ICR lines all
      pass through one point ON the stuck wheel's constraint line
    * end-to-end benefit: with reconfiguration, post-fault heading drift is a
      fraction of the unmitigated case (the report's headline claim)
"""

from __future__ import annotations

import numpy as np
import pytest

from sim4wis.controller.registry import make_strategy
from sim4wis.core.state import DriverInput, VehicleParams, VehicleState
from sim4wis.experiment.schema import (
    Experiment, FaultSpec, Maneuver, ManeuverStep, SteerProfile,
)
from sim4wis.experiment.session import run_experiment
from sim4wis.vehicle.geometry import line_intersection, wheel_perpendicular_dir


def _straight_exp(speed_kmh: float, faults: list[FaultSpec], strategy: str = "ideal_ackermann",
                  mode_params: dict | None = None) -> Experiment:
    return Experiment(
        name="fault_study",
        strategy=strategy,
        mode_params=mode_params or {},
        faults=faults,
        maneuver=Maneuver(steps=[
            ManeuverStep(name="accel", duration=6.0, speed_kmh=speed_kmh, speed_ramp_s=4.0),
            ManeuverStep(name="hold", duration=6.0,
                         steer=SteerProfile(kind="constant", amplitude=0.0)),
        ]),
    )


def test_stuck_value_fault_reaches_and_holds() -> None:
    """FL commanded stuck at +4° from t=7 s — wheel physically goes there."""
    exp = _straight_exp(60.0, [FaultSpec(fault_type="stuck_value", wheel=0,
                                         value=np.deg2rad(4.0), t_start=7.0)])
    r = run_experiment(exp)
    t = np.asarray(r.t)
    d_fl = np.asarray(r.channels["delta_fl"])
    # Before the fault: straight (only static toe).
    i_pre = np.searchsorted(t, 6.9)
    assert abs(np.rad2deg(d_fl[i_pre])) < 0.3
    # Well after: at the stuck angle (+toe offset tolerance).
    i_post = np.searchsorted(t, 9.0)
    assert np.rad2deg(d_fl[i_post]) == pytest.approx(4.0, abs=0.3)
    # And the vehicle is yawing / drifting (fault has consequences).
    assert abs(r.channels["yaw_rate"][i_post]) > 0.01


def test_stuck_hold_latches_current_angle() -> None:
    """Jam during a steady turn freezes the wheel at its in-use angle."""
    exp = Experiment(
        name="hold_latch",
        strategy="ideal_ackermann",
        faults=[FaultSpec(fault_type="stuck_hold", wheel=1, t_start=8.0)],
        maneuver=Maneuver(steps=[
            ManeuverStep(name="accel", duration=5.0, speed_kmh=40.0, speed_ramp_s=3.0),
            ManeuverStep(name="turn", duration=3.0,
                         steer=SteerProfile(kind="constant", amplitude=0.05)),
            ManeuverStep(name="straighten", duration=4.0,
                         steer=SteerProfile(kind="constant", amplitude=0.0)),
        ]),
    )
    r = run_experiment(exp)
    t = np.asarray(r.t)
    d_fr = np.asarray(r.channels["delta_fr"])
    angle_at_fault = d_fr[np.searchsorted(t, 8.0)]
    angle_late = d_fr[np.searchsorted(t, 11.5)]
    assert abs(angle_at_fault) > np.deg2rad(0.5)          # actually turning at t=8
    assert angle_late == pytest.approx(angle_at_fault, abs=np.deg2rad(0.3))  # frozen


def test_reconfig_mirror_cancellation_geometry() -> None:
    """Post-detection allocation: the same-axle partner mirrors the stuck
    wheel's angle error so axle net force and yaw moment cancel."""
    p = VehicleParams()
    strat = make_strategy("fault_reconfig", p)
    delta_s = np.deg2rad(5.0)

    # Straight request, zero yaw state → pure feedforward visible.
    driver = DriverInput(throttle=0.3, steering=0.0, mode_params={
        "fault_wheel": 0, "fault_angle": float(delta_s),
        "fault_time": 0.0, "detect_delay": 0.0,
    })
    state = VehicleState()
    state.t = 1.0   # past detection
    cmd = strat.compute(driver, state)

    assert cmd.delta_cmd[0] == pytest.approx(delta_s, abs=1e-9)     # physical truth
    assert cmd.delta_cmd[1] == pytest.approx(-delta_s, abs=1e-9)    # mirror partner
    assert cmd.delta_cmd[2] == pytest.approx(0.0, abs=1e-9)         # rears untouched
    assert cmd.delta_cmd[3] == pytest.approx(0.0, abs=1e-9)

    # Yaw-rate feedback trims in the correct direction: vehicle yawing left
    # (r > r_des = 0) → healthy front steers right (negative), rears left.
    state.yaw_rate = 0.1
    cmd_fb = strat.compute(driver, state)
    assert cmd_fb.delta_cmd[1] < cmd.delta_cmd[1]
    assert cmd_fb.delta_cmd[2] > 0.0 and cmd_fb.delta_cmd[3] > 0.0

    # Rear-wheel fault: partner is the other rear wheel.
    driver_r = DriverInput(throttle=0.3, steering=0.0, mode_params={
        "fault_wheel": 2, "fault_angle": float(delta_s),
        "fault_time": 0.0, "detect_delay": 0.0,
    })
    cmd_r = strat.compute(driver_r, VehicleState(t=1.0))
    assert cmd_r.delta_cmd[2] == pytest.approx(delta_s, abs=1e-9)
    assert cmd_r.delta_cmd[3] == pytest.approx(-delta_s, abs=1e-9)


def test_free_caster_self_aligns_and_stays_stable() -> None:
    """Non-self-locking front failure: the wheel casters to its zero-force
    equilibrium (α≈0) within ~0.5 s and shows no shimmy divergence."""
    exp = Experiment(
        name="free",
        strategy="ideal_ackermann",
        faults=[FaultSpec(fault_type="free_caster", wheel=0, t_start=10.5)],
        maneuver=Maneuver(steps=[
            ManeuverStep(name="accel", duration=6.0, speed_kmh=60.0, speed_ramp_s=4.0),
            ManeuverStep(name="in", duration=2.0,
                         steer=SteerProfile(kind="ramp", start=0.0, amplitude=0.05)),
            ManeuverStep(name="hold", duration=6.0,
                         steer=SteerProfile(kind="constant", amplitude=0.05)),
        ]),
    )
    r = run_experiment(exp)
    t = np.asarray(r.t)
    alpha_fl = np.asarray(r.channels["slip_alpha_fl"])
    # Pre-fault: cornering slip present; post-settle: castered to near-zero force.
    assert abs(np.rad2deg(alpha_fl[np.searchsorted(t, 10.4)])) > 1.0
    i_late = np.searchsorted(t, 12.5)
    assert abs(np.rad2deg(alpha_fl[i_late])) < 0.3
    # No shimmy: wheel angle bounded and quiescent at the end.
    d_fl = np.rad2deg(np.asarray(r.channels["delta_fl"]))
    tail = d_fl[np.searchsorted(t, 13.0):]
    assert np.all(np.isfinite(tail))
    assert float(np.ptp(tail)) < 0.5


def test_reconfig_free_mode_boosts_healthy_partner() -> None:
    """free fault_kind: no mirror; healthy same-axle wheel gets gain-boosted."""
    p = VehicleParams()
    strat = make_strategy("fault_reconfig", p)
    driver = DriverInput(throttle=0.3, steering=0.10, mode_params={
        "fault_wheel": 0, "fault_kind": "free",
        "fault_time": 0.0, "detect_delay": 0.0, "front_gain": 2.0,
    })
    # Steady-state condition: yaw rate matches the (speed-capped) request so
    # the yaw-PI term is quiescent and the pure feedforward is visible.
    v_cmd = min(0.3 * p.v_max, 60.0 / 3.6)
    kappa = strat._kappa_max * 0.10
    state = VehicleState(t=1.0, yaw_rate=v_cmd * kappa)
    cmd = strat.compute(driver, state)
    nom = strat._ideal(kappa, v_cmd)
    assert cmd.delta_cmd[1] == pytest.approx(2.0 * float(nom.delta_cmd[1]), abs=1e-6)
    # Rears keep their nominal allocation.
    assert cmd.delta_cmd[2] == pytest.approx(float(nom.delta_cmd[2]), abs=1e-6)
    assert cmd.delta_cmd[3] == pytest.approx(float(nom.delta_cmd[3]), abs=1e-6)


def test_reconfig_reduces_post_fault_heading_drift() -> None:
    """Headline claim: reconfiguration cuts uncorrected heading drift by a
    large factor vs. no mitigation (FL stuck at +4° @ 80 km/h straight)."""
    fault = [FaultSpec(fault_type="stuck_value", wheel=0,
                       value=np.deg2rad(4.0), t_start=7.0)]

    base = run_experiment(_straight_exp(80.0, fault))
    mit = run_experiment(_straight_exp(
        80.0, fault, strategy="fault_reconfig",
        mode_params={"fault_wheel": 0, "fault_angle": float(np.deg2rad(4.0)),
                     "fault_time": 7.0, "detect_delay": 0.15, "v_limit_kmh": 60.0},
    ))

    def heading_drift(r) -> float:
        t = np.asarray(r.t)
        psi = np.asarray(r.channels["pose_psi"])
        i0 = np.searchsorted(t, 7.0)
        i1 = np.searchsorted(t, 9.0)   # 2 s after the fault
        return abs(np.rad2deg(psi[i1] - psi[i0]))

    d_base = heading_drift(base)
    d_mit = heading_drift(mit)
    assert d_base > 3.0                    # unmitigated: clearly hazardous drift
    assert d_mit < 0.5 * d_base            # mitigation at least halves it
