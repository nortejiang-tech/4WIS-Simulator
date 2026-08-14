"""Tests — the per-corner angle-tracking control layer (M1).

Two contracts carry the whole layer, and both are pinned here:

1. **The disabled layer is bit-identical** — ``angle_control.enabled``
   defaults to False and then nothing the layer adds runs.
2. **open_loop reproduces the legacy by-wire update bit-exactly** — enabling
   the layer with the default controller must not move a single number the
   legacy path produced.

On top of that: the actuator plant's 2nd-order response is pinned against
its analytic solution, stiction holds under Coulomb friction, the sensor
quantises/delays deterministically, inner-rate controllers get interpolated
targets, and 4WIS corners follow their own commands.
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest

from sim4wis.core.state import (
    N_WHEELS,
    ControlCommand,
    EnvironmentState,
    VehicleParams,
)
from sim4wis.steering import architecture as arch
from sim4wis.steering.bywire import ByWirePlant
from sim4wis.steering.tracking import (
    AngleSensor,
    CornerActuatorPlant,
    OpenLoopController,
    make_controller,
)
from sim4wis.steering.tracking.controller import (
    AngleTrackingController,
    TrackingOutput,
)
from sim4wis.steering.tracking.coupling import (
    controlled_corners,
    corner_tracking_step,
    make_corner_trackers,
)
from sim4wis.vehicle.model_registry import make_vehicle_model


def _sbw_params(*, enable_layer: bool, controller: str = "open_loop",
                per_wheel_arch: bool = False) -> VehicleParams:
    p = VehicleParams()
    p.steering_system.enabled = True
    p.steering_system.architecture = "4wis" if per_wheel_arch else "sbw"
    p.steering_system.angle_control = replace(
        p.steering_system.angle_control, enabled=enable_layer, controller=controller)
    return p


def _cmd(*angles: float) -> ControlCommand:
    delta = np.zeros(N_WHEELS)
    for i, a in enumerate(angles):
        delta[i] = float(a)
    cmd = ControlCommand.zero()
    cmd.delta_cmd = delta
    return cmd


# ---------------------------------------------------------------------------
# Contract 1 + 2: bit-exactness
# ---------------------------------------------------------------------------


def test_open_loop_reproduces_legacy_bywire_bit_exact():
    rng = np.random.default_rng(7)
    legacy = ByWirePlant()
    tracker = OpenLoopController()
    angle_legacy, angle_tracker = 0.0, 0.0
    for _ in range(200):
        dt = 0.005
        cmd = float(rng.uniform(-0.5, 0.5))
        legacy.step(dt, hand_angle=cmd, hand_rate=0.0, delta_cmd=cmd,
                    rack_force=0.0, speed_ms=20.0)
        out = tracker.step(dt, target_angle=cmd, target_rate=0.0,
                           feedback_angle=angle_tracker, plant_angle=angle_tracker,
                           load_torque=0.0, speed_ms=20.0)
        angle_legacy = legacy.state.road_wheel_angle
        angle_tracker = float(out.angle_set)
        assert angle_tracker == angle_legacy
        assert out.diagnostics["deviation"] == legacy.state.angle_deviation


def test_make_corner_trackers_is_none_when_disabled():
    # Default params: the layer is off — no trackers, legacy paths untouched.
    assert make_corner_trackers(VehicleParams()) is None
    # Steering enabled, layer still off.
    p = VehicleParams()
    p.steering_system.enabled = True
    p.steering_system.architecture = "sbw"
    assert make_corner_trackers(p) is None
    # Steering off, layer on: the layer lives under the steering system.
    p2 = VehicleParams()
    p2.steering_system.angle_control = replace(
        p2.steering_system.angle_control, enabled=True)
    assert make_corner_trackers(p2) is None


def test_mechanical_architecture_has_no_corner_trackers():
    p = VehicleParams()
    p.steering_system.enabled = True
    p.steering_system.architecture = "r_eps"
    p.steering_system.angle_control = replace(
        p.steering_system.angle_control, enabled=True)
    assert make_corner_trackers(p) is None  # no corner actuators at all
    p.steering_system.architecture = "eps_rws"
    trackers, per_wheel = make_corner_trackers(p)
    assert set(trackers) == {2, 3}  # rear corners only
    assert per_wheel is False


def test_controlled_corners_follow_the_architecture():
    assert controlled_corners(arch.get("sbw")) == (0, 1)
    assert controlled_corners(arch.get("sbw_rws")) == (0, 1, 2, 3)
    assert controlled_corners(arch.get("4wis")) == (0, 1, 2, 3)
    assert controlled_corners(arch.get("r_eps")) == ()


# ---------------------------------------------------------------------------
# Plant: analytic pinning
# ---------------------------------------------------------------------------


def test_plant_pure_inertia_matches_the_analytic_step_response():
    plant = CornerActuatorPlant(inertia_kgm2=1.0, damping_nms_per_rad=0.0,
                                coulomb_friction_nm=0.0, peak_torque_nm=1.0)
    dt = 0.0005
    t = 0.0
    for _ in range(2000):
        plant.step(dt, 1.0, 0.0)
        t += dt
    # θ(t) = ½·τ/J·t² with the semi-implicit rate update: rate_k = k·dt/J·τ,
    # θ_k = Σ rate_i·dt = τ/J·dt²·k(k+1)/2.
    n = 2000
    expected = 0.5 * n * (n + 1) * dt * dt
    assert plant.angle == pytest.approx(expected, rel=1e-12)
    assert plant.rate == pytest.approx(n * dt, rel=1e-12)


def test_plant_stiction_holds_under_friction():
    plant = CornerActuatorPlant(inertia_kgm2=1.0, damping_nms_per_rad=0.0,
                                coulomb_friction_nm=0.5, peak_torque_nm=10.0)
    for _ in range(10):
        plant.step(0.0005, 0.3, 0.0)  # inside the friction band
    assert plant.angle == 0.0
    assert plant.rate == 0.0
    plant.step(0.0005, 0.7, 0.0)  # outside — it moves
    assert plant.angle > 0.0


def test_plant_peak_torque_saturates():
    plant = CornerActuatorPlant(inertia_kgm2=1.0, damping_nms_per_rad=0.0,
                                coulomb_friction_nm=0.0, peak_torque_nm=1.0)
    plant.step(0.0005, 99.0, 0.0)
    assert plant.rate == pytest.approx(1.0 * 0.0005)


# ---------------------------------------------------------------------------
# Sensor
# ---------------------------------------------------------------------------


def test_sensor_quantises_delays_and_is_deterministic():
    s1 = AngleSensor(quant_rad=0.01, delay_steps=1, noise_std_rad=0.001, seed=5)
    s2 = AngleSensor(quant_rad=0.01, delay_steps=1, noise_std_rad=0.001, seed=5)
    seq1, seq2 = [], []
    for i in range(20):
        truth = 0.05 + 0.001 * i  # quantises to 0.05 throughout
        seq1.append(s1.measure(truth))
        seq2.append(s2.measure(truth))
    assert seq1 == seq2  # seeded: identical sequences
    # Quantisation: every returned value is a multiple of the step.
    assert all(v == round(v / 0.01) * 0.01 for v in seq1)
    # Delay: a discrete sensor is one sample behind even at delay_steps = 0,
    # so delay_steps = 1 reads are delayed by two periods — the first two
    # readings drain the pipeline's zeros, the third carries the first truth.
    assert seq1[0] == 0.0 and seq1[1] == 0.0
    assert seq1[2] == pytest.approx(0.05, abs=0.02)


def test_sensor_noise_off_by_default_is_exact():
    s = AngleSensor(quant_rad=0.0, delay_steps=0)
    # One-sample pipeline delay: each reading is the previous measurement.
    assert s.measure(0.1) == 0.0
    assert s.measure(0.2) == 0.1
    assert s.measure(0.3) == 0.2


# ---------------------------------------------------------------------------
# Coupling: inner-rate interpolation and target selection
# ---------------------------------------------------------------------------


class _TargetAccumulator(AngleTrackingController):
    """Test controller: echoes the interpolated target back as angle_set."""

    name = "test_accumulator"
    inner_rate_hz = 2000.0

    def __init__(self) -> None:
        self.targets: list[tuple[float, float]] = []

    def step(self, dt, *, target_angle, target_rate, feedback_angle,
             plant_angle, load_torque, speed_ms):
        self.targets.append((dt, target_angle))
        return TrackingOutput(angle_set=target_angle)


def test_inner_rate_controller_gets_interpolated_targets():
    p = _sbw_params(enable_layer=True)
    trackers, per_wheel = make_corner_trackers(p)
    assert trackers is not None and per_wheel is False
    acc = _TargetAccumulator()
    trackers[0].controller = acc
    trackers[1].controller = _TargetAccumulator()
    delta = np.zeros(N_WHEELS)
    delta[0] = delta[1] = 0.5
    rack = np.zeros(N_WHEELS)
    corner_tracking_step(trackers, per_wheel, delta, 0.005,
                         rack_forces=rack, speed_ms=20.0, pinion_radius=0.02)
    assert len(acc.targets) == 10  # 5 ms at 2000 Hz
    assert acc.targets[-1][1] == pytest.approx(0.5)  # lands on the final target
    # Ramping: the first inner step saw the target ~10 % of the way along.
    assert acc.targets[0][1] == pytest.approx(0.05, abs=1e-9)


def test_non_per_wheel_corners_follow_the_averaged_command():
    p = _sbw_params(enable_layer=True)  # sbw: not per-wheel
    trackers, per_wheel = make_corner_trackers(p)
    delta = np.zeros(N_WHEELS)
    delta[0], delta[1] = 0.4, 0.6
    tracked, _ = corner_tracking_step(
        trackers, per_wheel, delta, 0.5, rack_forces=np.zeros(N_WHEELS),
        speed_ms=0.0, pinion_radius=0.02)
    # One 0.5 s step at 10 Hz bandwidth ≈ fully converged onto the average.
    assert tracked[0] == pytest.approx(0.5, abs=0.02)
    assert tracked[1] == pytest.approx(0.5, abs=0.02)


# ---------------------------------------------------------------------------
# Vehicle integration
# ---------------------------------------------------------------------------


def _run_model(params: VehicleParams, steps: int = 200, *,
               delta_fl: float = 0.1, delta_fr: float = 0.1,
               delta_rl: float = 0.0, delta_rr: float = 0.0) -> list[float]:
    model = make_vehicle_model(params, "simplified_dynamic")
    env = EnvironmentState(mu=0.9)
    out = []
    for _ in range(steps):
        model.step(0.005, _cmd(delta_fl, delta_fr, delta_rl, delta_rr), env)
        out.append(float(model._delta_act[0]))
    return out


def test_sbw_with_layer_enabled_matches_legacy_sbw_bit_exact():
    legacy = _run_model(_sbw_params(enable_layer=False))
    layered = _run_model(_sbw_params(enable_layer=True))
    assert legacy == layered  # open_loop must be indistinguishable


def test_4wis_corners_follow_their_own_commands():
    p = _sbw_params(enable_layer=True, per_wheel_arch=True)  # 4wis
    model = make_vehicle_model(p, "simplified_dynamic")
    env = EnvironmentState(mu=0.9)
    for _ in range(400):
        model.step(0.005, _cmd(0.3, -0.2, 0.1, -0.05), env)
    d = model._delta_act
    assert d[0] == pytest.approx(0.3, abs=0.01)
    assert d[1] == pytest.approx(-0.2, abs=0.01)  # no averaging
    assert d[2] == pytest.approx(0.1, abs=0.01)
    assert d[3] == pytest.approx(-0.05, abs=0.01)
    assert d[0] != d[1]
    # Corner deviation channels carry the layer's own signal.
    assert not math.isnan(model.steering_channels["steer_corner_deviation_fl"])


def test_eps_rws_rear_corners_track_through_the_layer():
    p = VehicleParams()
    p.steering_system.enabled = True
    p.steering_system.architecture = "eps_rws"
    p.steering_system.angle_control = replace(
        p.steering_system.angle_control, enabled=True)
    model = make_vehicle_model(p, "simplified_dynamic")
    env = EnvironmentState(mu=0.9)
    for _ in range(400):
        model.step(0.005, _cmd(0.05, 0.05, 0.08, 0.08), env)
    d = model._delta_act
    # Front axle stays mechanical (plant), rear follows the trackers.
    assert d[2] == pytest.approx(0.08, abs=0.01)
    assert d[3] == pytest.approx(0.08, abs=0.01)
    assert not math.isnan(model.steering_channels["steer_corner_deviation_rl"])
    # Front corners have no trackers — their deviation channel stays NaN.
    assert math.isnan(model.steering_channels["steer_corner_deviation_fl"])


def test_multibody_4wis_corners_follow_their_own_commands():
    p = _sbw_params(enable_layer=True, per_wheel_arch=True)  # 4wis
    model = make_vehicle_model(p, "multibody")
    env = EnvironmentState(mu=0.9)
    for _ in range(300):
        model.step(0.005, _cmd(0.25, -0.15, 0.1, -0.05), env)
    d = model._delta_act
    assert d[0] == pytest.approx(0.25, abs=0.01)
    assert d[1] == pytest.approx(-0.15, abs=0.01)
    assert d[2] == pytest.approx(0.1, abs=0.01)
    assert d[3] == pytest.approx(-0.05, abs=0.01)
    assert not math.isnan(model.steering_channels["steer_corner_deviation_fl"])


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------


def test_unknown_controller_name_is_refused():
    with pytest.raises(ValueError, match="unknown angle-tracking controller"):
        make_controller("not_a_controller")
