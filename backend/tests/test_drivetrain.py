"""Tests — drive form (FWD / RWD / AWD with a continuously variable split).

The backbone here is the closed-form traction-limited launch acceleration.
With mass m, wheelbase L, CG at distance `a` from the front axle (so `b = L−a`
from the rear) and CG height h, steady-state force balance plus longitudinal
load transfer ΔFz = m·ax·h/L gives:

    rear drive   N_r = m·g·a/L + m·ax·h/L,  μ·N_r = m·ax
                 ⇒  ax = μ·g·a / (L − μ·h)      self-reinforcing
    front drive  N_f = m·g·b/L − m·ax·h/L,  μ·N_f = m·ax
                 ⇒  ax = μ·g·b / (L + μ·h)      self-limiting
    all-wheel    ⇒  ax = μ·g

The sign of the h term is the whole story of why a rear-drive car launches
harder: accelerating loads its driven axle, while a front-drive car unloads
its own. None of that is coded anywhere — it has to fall out of the load
transfer model and the friction circle, which is exactly what these check.
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest

from sim4wis.controller.longitudinal import apply_brake_command, apply_drive_command
from sim4wis.controller.registry import make_strategy
from sim4wis.core.derived import update_derived_outputs
from sim4wis.core.simulator import Simulator
from sim4wis.core.state import DriverInput, EnvironmentState, VehicleParams, VehicleState
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel
from sim4wis.vehicle.powertrain import axle_split, drive_torques

DT = 0.005
G = 9.81


def _params(**over) -> VehicleParams:
    """Torque mode, and free of the resistances the closed form ignores.

    Crr / Cd are zeroed so the measured limit can be compared against the
    analytic one directly rather than through a fudge factor; the power cap is
    lifted so the *traction* limit is what binds, which is what is under test.
    """
    base = dict(longitudinal_mode="torque", rolling_resistance_coeff=0.0,
                drag_coeff_cd=0.0, aero_lift_coeff_front=0.0,
                aero_lift_coeff_rear=0.0, motor_power_max=0.0,
                motor_torque_max=20_000.0)
    base.update(over)
    return replace(VehicleParams(), **base)


def analytic_launch_accel(p: VehicleParams, mu: float, front_ratio: float) -> float:
    """Closed-form traction-limited launch acceleration [m/s²]."""
    L, a, h = p.wheelbase, p.cg_to_front, p.cg_height
    b = L - a
    if front_ratio >= 0.999:
        return mu * G * b / (L + mu * h)
    if front_ratio <= 0.001:
        return mu * G * a / (L - mu * h)
    return mu * G


def _launch(p: VehicleParams, mu: float, steps: int = 500) -> float:
    """Full-throttle standing start; return the settled ax.

    Long enough (2.5 s) for the load transfer to reach equilibrium — the
    closed form is a steady-state balance, and sampling during the torque
    build-up reads ~20% low.

    Expect the measured value to sit a few percent *under* the closed form:
    static toe and camber put a small α on every wheel, and that lateral force
    takes a bite out of the same friction circle the drive force is spending
    from. The closed form has no alignment in it.
    """
    sim = Simulator(dt_sim=DT, dt_push=0.05)
    sim.reset()
    sim.params = p
    sim.model = SimplifiedDynamicModel(p)
    sim.model.reset()
    sim.strategy = make_strategy("ideal_ackermann", p)
    env = EnvironmentState(mu=mu)
    sim.set_driver(throttle=1.0, gear=1)
    ax = []
    for k in range(steps):
        cmd = sim.strategy.compute(sim.driver, sim.model.state, DT)
        apply_brake_command(cmd, sim.driver, sim.params)
        apply_drive_command(cmd, sim.driver, sim.params, sim.model.state)
        sim.model.step(DT, cmd, env)
        update_derived_outputs(sim.model.state, sim.params)
        if k > steps // 2:          # skip the torque build-up transient
            ax.append(sim.model.state.ax)
    return float(np.mean(ax))


# ── 1. the closed-form launch limits ─────────────────────────────────────────

@pytest.mark.parametrize("mu", [0.85, 0.50])
@pytest.mark.parametrize("front_ratio,name", [(0.0, "RWD"), (1.0, "FWD"), (0.5, "AWD")])
def test_launch_matches_closed_form(mu, front_ratio, name):
    p = _params(drive_front_ratio=front_ratio)
    expected = analytic_launch_accel(p, mu, front_ratio)
    measured = _launch(p, mu)
    # Alignment (toe/camber) spends a little of the same friction circle, so
    # the vehicle lands a few percent under the idealised balance — never over.
    assert measured <= expected * 1.02, (
        f"{name} @ μ={mu}: {measured:.3f} exceeds the traction limit {expected:.3f}"
    )
    assert measured == pytest.approx(expected, rel=0.10), (
        f"{name} @ μ={mu}: measured {measured:.3f} vs closed form {expected:.3f}"
    )


@pytest.mark.parametrize("mu", [0.85, 0.50])
def test_rear_drive_launches_harder_than_front(mu):
    """The sign of the load-transfer term, measured. This is the single most
    recognisable difference between the two drive forms."""
    rwd = _launch(_params(drive_front_ratio=0.0), mu)
    fwd = _launch(_params(drive_front_ratio=1.0), mu)
    assert rwd > fwd * 1.15
    # and both are bracketed by the all-wheel case
    awd = _launch(_params(drive_front_ratio=0.5), mu)
    assert fwd < awd
    assert rwd < awd * 1.02


def test_all_wheel_drive_reaches_mu_g():
    p = _params(drive_front_ratio=0.5)
    assert _launch(p, 0.85) == pytest.approx(0.85 * G, rel=0.08)


def test_load_transfer_matches_the_analytic_term():
    """ΔFz = m·ax·h/L per axle — the mechanism the launch asymmetry rests on."""
    p = _params(drive_front_ratio=0.5)
    sim = Simulator(dt_sim=DT, dt_push=0.05)
    sim.reset()
    sim.params = p
    sim.model = SimplifiedDynamicModel(p)
    sim.model.reset()
    sim.strategy = make_strategy("ideal_ackermann", p)
    env = EnvironmentState(mu=0.85)
    sim.set_driver(throttle=1.0, gear=1)
    for _ in range(100):
        cmd = sim.strategy.compute(sim.driver, sim.model.state, DT)
        apply_brake_command(cmd, sim.driver, sim.params)
        apply_drive_command(cmd, sim.driver, sim.params, sim.model.state)
        sim.model.step(DT, cmd, env)
    st = sim.model.state
    front = st.fz[0] + st.fz[1]
    rear = st.fz[2] + st.fz[3]
    static_front = p.mass * G * (p.wheelbase - p.cg_to_front) / p.wheelbase
    shifted = p.mass * st.ax * p.cg_height / p.wheelbase
    assert front == pytest.approx(static_front - shifted, rel=0.02)
    assert front + rear == pytest.approx(p.mass * G, rel=0.02)


# ── 2. the split is continuous, not three modes ──────────────────────────────

def test_axle_split_is_continuous_and_normalised():
    for lam in (0.0, 0.25, 0.5, 0.75, 1.0):
        f, r = axle_split(replace(VehicleParams(), drive_front_ratio=lam))
        assert f == pytest.approx(lam)
        assert f + r == pytest.approx(1.0)


def test_split_moves_torque_between_axles():
    p = _params(motor_power_max=0.0)
    st = VehicleState()
    drv = DriverInput(throttle=0.5, gear=1)
    for lam, front_bigger in ((1.0, True), (0.0, False)):
        t = drive_torques(replace(p, drive_front_ratio=lam), drv, st)
        f, r = float(t[0] + t[1]), float(t[2] + t[3])
        assert (f > r) == front_bigger


def test_torque_distribution_is_continuous_in_the_split():
    """λ is a real dial, not three modes: the axle torques must move smoothly
    across the whole range with no step at the endpoints."""
    p = _params(motor_power_max=0.0, motor_torque_max=1e9)   # no cap: pure split
    st = VehicleState()
    drv = DriverInput(throttle=0.5, gear=1)
    lams = np.linspace(0.0, 1.0, 21)
    fronts = []
    for lam in lams:
        t = drive_torques(replace(p, drive_front_ratio=float(lam)), drv, st)
        fronts.append(float(t[0] + t[1]))
    diffs = np.diff(fronts)
    assert np.all(diffs > 0.0), "front share must rise monotonically with λ"
    assert np.ptp(diffs) < 1e-6, "and do so linearly — no step at the ends"


def test_any_mixed_split_reaches_mu_g_when_torque_is_unlimited():
    """A physical subtlety worth pinning down: once BOTH axles are driven hard
    enough to saturate, the whole tyre contact reaches μ·m·g and the launch
    limit is μ·g regardless of how the torque was split. The split only rations
    grip when torque is the binding constraint — which is why the closed-form
    single-axle formulas apply at λ = 0 and λ = 1 only.

    This is why launch acceleration is deliberately NOT tested as monotonic in
    λ: with unlimited torque it is flat across the interior and drops only at
    the two endpoints.
    """
    mu = 0.85
    interior = [_launch(_params(drive_front_ratio=lam), mu) for lam in (0.25, 0.5, 0.75)]
    for a in interior:
        assert a == pytest.approx(mu * G, rel=0.10)
    ends = [_launch(_params(drive_front_ratio=lam), mu) for lam in (0.0, 1.0)]
    for a in ends:
        assert a < min(interior) * 0.98


# ── 3. powertrain envelope ───────────────────────────────────────────────────

def test_per_wheel_torque_cap_binds_at_launch():
    p = _params(motor_torque_max=1500.0, motor_power_max=0.0, drive_front_ratio=0.5)
    t = drive_torques(p, DriverInput(throttle=1.0, gear=1), VehicleState())
    assert np.all(np.abs(t) <= 1500.0 + 1e-9)
    assert np.allclose(np.abs(t), 1500.0)


def test_front_drive_has_half_the_launch_torque_of_all_wheel():
    """Two motors instead of four — an honest hardware difference, not an
    artefact of the split arithmetic."""
    p = _params(motor_power_max=0.0)
    st = VehicleState()
    drv = DriverInput(throttle=1.0, gear=1)
    awd = np.sum(np.abs(drive_torques(replace(p, drive_front_ratio=0.5), drv, st)))
    fwd = np.sum(np.abs(drive_torques(replace(p, drive_front_ratio=1.0), drv, st)))
    assert fwd == pytest.approx(awd / 2, rel=1e-9)


def test_power_cap_binds_at_speed_and_equalises_total_torque():
    """Above the base speed the shared bus limits total torque to P/ω whatever
    the split is — which is what makes a λ sweep at speed a clean experiment on
    distribution alone."""
    p = _params(motor_power_max=200_000.0, motor_torque_max=20_000.0)
    st = VehicleState()
    st.wheel_omega = np.full(4, 60.0)          # ≈ 85 km/h on the default tyre
    drv = DriverInput(throttle=1.0, gear=1)
    totals = [float(np.sum(np.abs(drive_torques(replace(p, drive_front_ratio=lam), drv, st))))
              for lam in (0.0, 0.5, 1.0)]
    for t in totals:
        assert t == pytest.approx(totals[0], rel=1e-9)
    # and it really is the power budget
    assert totals[0] * 60.0 == pytest.approx(200_000.0, rel=1e-9)


def test_power_cap_is_applied_after_the_torque_cap():
    """Order matters: capping power first would let one wheel exceed its motor
    rating whenever the others were idle."""
    p = _params(motor_torque_max=1000.0, motor_power_max=1_000_000.0,
                drive_front_ratio=1.0)
    st = VehicleState()
    st.wheel_omega = np.full(4, 10.0)
    t = drive_torques(p, DriverInput(throttle=1.0, gear=1), st)
    assert np.all(np.abs(t) <= 1000.0 + 1e-9)


# ── 4. differential ──────────────────────────────────────────────────────────

def test_independent_diff_lets_each_wheel_take_its_own_torque():
    p = _params(diff_type="independent", drive_front_ratio=0.5, motor_power_max=0.0)
    st = VehicleState()
    st.grip_valid = True
    st.grip_capacity = np.array([200.0, 6000.0, 6000.0, 6000.0])   # FL on ice
    t = drive_torques(p, DriverInput(throttle=0.2, gear=1), st)
    assert t[0] == pytest.approx(t[1])          # 4WID: no coupling, equal demand


def test_open_diff_axle_is_limited_by_its_weaker_wheel():
    """The failure mode this option exists to show: one wheel on ice takes the
    whole axle down with it, because an open gearset cannot hold a torque
    difference across itself."""
    p = _params(diff_type="open", drive_front_ratio=1.0, motor_power_max=0.0)
    st = VehicleState()
    st.grip_valid = True
    st.grip_capacity = np.array([200.0, 6000.0, 6000.0, 6000.0])   # FL on ice
    t = drive_torques(p, DriverInput(throttle=1.0, gear=1), st)
    ceiling = p.tire_radius * 200.0
    assert t[0] == pytest.approx(ceiling, rel=1e-9)
    assert t[1] == pytest.approx(ceiling, rel=1e-9), "both sides must match"

    indep = drive_torques(replace(p, diff_type="independent"),
                          DriverInput(throttle=1.0, gear=1), st)
    assert np.sum(indep[:2]) > np.sum(t[:2]) * 5, "independent must do far better"


def test_open_diff_without_tyre_data_falls_back_cleanly():
    p = _params(diff_type="open", drive_front_ratio=0.5, motor_power_max=0.0)
    t = drive_torques(p, DriverInput(throttle=0.5, gear=1), VehicleState())
    assert np.all(np.isfinite(t))
    assert np.all(t > 0.0)


# ── 5. authority and defaults ────────────────────────────────────────────────

def test_default_split_is_balanced_all_wheel():
    assert VehicleParams().drive_front_ratio == 0.5
    assert VehicleParams().diff_type == "independent"


@pytest.mark.parametrize("drv", [
    DriverInput(throttle=1.0, brake=1.0, gear=1),
    DriverInput(throttle=1.0, handbrake=1, gear=1),
    DriverInput(throttle=1.0, gear=0),
])
def test_brake_and_neutral_still_outrank_the_drivetrain(drv):
    p = _params()
    assert np.allclose(drive_torques(p, drv, VehicleState()), 0.0)


def test_reverse_gear_reverses_every_wheel():
    p = _params()
    t = drive_torques(p, DriverInput(throttle=0.5, gear=-1), VehicleState())
    assert np.all(t < 0.0)


def test_speed_limit_rescales_the_pedal_in_speed_servo_mode():
    """The limit gives the whole pedal to the range being driven, rather than
    clipping the top and leaving most of the travel dead — which is the actual
    complaint behind 'hard to hold a speed by hand'."""
    from sim4wis.controller.longitudinal import speed_command
    p = replace(VehicleParams(), driver_speed_limit=10.0)
    drv = DriverInput(throttle=1.0, gear=1)
    assert speed_command(p, drv, 0.0, DT) == pytest.approx(10.0)
    drv = DriverInput(throttle=0.5, gear=1)
    assert speed_command(p, drv, 0.0, DT) == pytest.approx(5.0), "half pedal = half the limit"


def test_speed_limit_does_not_touch_the_validation_channel():
    """An experiment states the speed it wants; a driver aid must not veto it,
    or every golden would move the moment someone set a limit."""
    from sim4wis.controller.longitudinal import speed_command
    p = replace(VehicleParams(), driver_speed_limit=5.0)
    drv = DriverInput(throttle=0.0, gear=1, mode_params={"speed_target_ms": 25.0})
    assert speed_command(p, drv, 0.0, DT) == pytest.approx(25.0)


def test_speed_limit_tapers_drive_torque_in_torque_mode():
    """Torque mode has no speed target to rescale, so the limit fades the
    torque out over a band below it — a hard cut would surge and hunt."""
    p = _params(driver_speed_limit=20.0, motor_power_max=0.0)
    drv = DriverInput(throttle=1.0, gear=1)
    st = VehicleState()
    st.vx = 5.0
    full = float(np.max(np.abs(drive_torques(p, drv, st))))
    st.vx = 19.0
    near = float(np.max(np.abs(drive_torques(p, drv, st))))
    st.vx = 21.0
    over = float(np.max(np.abs(drive_torques(p, drv, st))))
    assert full > near > over
    assert over == pytest.approx(0.0, abs=1e-9)


def test_drivetrain_is_inert_in_speed_servo_mode():
    """The default path must not see any of this — that is what keeps every
    promoted baseline valid."""
    from sim4wis.core.state import ControlCommand
    p = replace(_params(), longitudinal_mode="speed_servo")
    cmd = ControlCommand.zero()
    apply_drive_command(cmd, DriverInput(throttle=1.0, gear=1), p, VehicleState())
    assert np.allclose(cmd.drive_torque_cmd, 0.0)


# ── 6. emergent handling: the driven axle spends its own friction budget ─────

def test_driven_axle_loses_lateral_capability_under_power():
    """Not coded anywhere — it follows from Fx and Fy sharing one circle.
    Front drive eats the front axle's lateral margin (understeer on power),
    rear drive eats the rear's (oversteer)."""
    def margins(front_ratio: float) -> tuple[float, float]:
        p = _params(drive_front_ratio=front_ratio, motor_power_max=300_000.0)
        sim = Simulator(dt_sim=DT, dt_push=0.05)
        sim.reset()
        sim.params = p
        sim.model = SimplifiedDynamicModel(p)
        sim.model.reset()
        sim.strategy = make_strategy("ideal_ackermann", p)
        env = EnvironmentState(mu=0.85)
        sim.set_driver(throttle=0.25, steering=0.25, gear=1)
        for k in range(2400):
            if k == 1800:
                sim.set_driver(throttle=1.0)      # power on, mid-corner
            cmd = sim.strategy.compute(sim.driver, sim.model.state, DT)
            apply_brake_command(cmd, sim.driver, sim.params)
            apply_drive_command(cmd, sim.driver, sim.params, sim.model.state)
            sim.model.step(DT, cmd, env)
            update_derived_outputs(sim.model.state, sim.params)
        st = sim.model.state
        front = float(np.mean(st.grip_margin_lat[:2]))
        rear = float(np.mean(st.grip_margin_lat[2:]))
        return front, rear

    f_front, f_rear = margins(1.0)      # front drive
    r_front, r_rear = margins(0.0)      # rear drive
    # front drive spends more of the FRONT axle's lateral budget than rear
    # drive does, and vice versa
    assert f_front - f_rear < r_front - r_rear


# ── 7. handling balance (driving-dynamics realism) ───────────────────────────

def test_understeer_gradient_is_production_like():
    """K = W_f/C_f − W_r/C_r must land in the production band (1–4 deg/g).

    A car set up neutral (K ≈ 0) has a yaw gain v/(L + K·v²) that grows without
    bound with speed, and at the limit gives no warning before the rear leaves.
    Regression guard for the chassis tuning: one cornering stiffness for all
    four wheels put this vehicle at 0.07 deg/g.
    """
    p = VehicleParams()
    L = p.wheelbase
    w_f = p.mass * G * (L - p.cg_to_front) / L
    w_r = p.mass * G * p.cg_to_front / L
    c_axle = 2 * p.tire_c_alpha
    k_rad_per_g = (w_f / (c_axle * p.tire_c_alpha_front_scale)
                   - w_r / (c_axle * p.tire_c_alpha_rear_scale))
    assert 1.0 <= math.degrees(k_rad_per_g) <= 4.0


def test_front_axle_slips_first_at_the_limit():
    """Understeer where it matters: the nose washes wide before the tail goes."""
    p = replace(VehicleParams(), longitudinal_mode="torque")
    sim = Simulator(dt_sim=DT, dt_push=0.05)
    sim.reset()
    sim.params = p
    sim.model = SimplifiedDynamicModel(p)
    sim.model.reset()
    sim.strategy = make_strategy("rear_wheel_steer", p)
    env = EnvironmentState(mu=0.9)
    sim.driver.mode_params.update({"rws_mode": "fixed_ratio", "rear_ratio": 0.0,
                                   "steer_raw_rad": math.radians(8.0),
                                   "speed_target_ms": 70 / 3.6})
    sim.driver.gear = 1
    for _ in range(4000):
        cmd = sim.strategy.compute(sim.driver, sim.model.state, DT)
        apply_brake_command(cmd, sim.driver, sim.params)
        apply_drive_command(cmd, sim.driver, sim.params, sim.model.state)
        sim.model.step(DT, cmd, env)
        update_derived_outputs(sim.model.state, sim.params)
    sa = np.abs(sim.model.slip_alpha)
    assert np.mean(sa[:2]) > np.mean(sa[2:]) * 1.05


def test_roll_couple_distribution_is_reachable_and_reversible():
    """The bar must actually move load between the axles — and setting the
    fraction to 0 must restore the legacy behaviour exactly."""
    from sim4wis.vehicle.load_transfer import vertical_loads
    base = VehicleParams()
    front_heavy = replace(base, suspension=replace(base.suspension,
                                                   roll_stiffness_front_frac=0.75))
    rear_heavy = replace(base, suspension=replace(base.suspension,
                                                  roll_stiffness_front_frac=0.25))
    fz_f = vertical_loads(front_heavy, ax=0.0, ay=5.0)
    fz_r = vertical_loads(rear_heavy, ax=0.0, ay=5.0)
    spread_front = abs(fz_f[1] - fz_f[0])
    spread_rear = abs(fz_r[1] - fz_r[0])
    assert spread_front > spread_rear * 1.5, "front bias must load the front pair more"
    # total roll couple conserved either way
    for fz in (fz_f, fz_r):
        assert float(np.sum(fz)) == pytest.approx(base.mass * G, rel=1e-9)
