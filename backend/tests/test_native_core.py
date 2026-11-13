"""Native/reference differential checks plus independent physics and ABI checks."""

import math
from dataclasses import replace

import numpy as np
import pytest

from sim4wis import native
from sim4wis.core.derived import update_derived_outputs
from sim4wis.core.state import ControlCommand, EnvironmentState, VehicleParams
from sim4wis.experiment.schema import Experiment
from sim4wis.experiment.session import run_experiment
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel
from sim4wis.vehicle.geometry import steering_linkage_metrics
from sim4wis.vehicle.kc import AxleKC, ComplianceCoeffs, KinematicCurve, VehicleKC
from sim4wis.vehicle.model_core import semi_implicit_wheel_spin
from sim4wis.vehicle.multibody import MultiBodyModel
from sim4wis.vehicle.tire import LinearTireModel, PacejkaTireModel

pytestmark = pytest.mark.skipif(native._native is None, reason="native build required")


def example(model="simplified_dynamic", tire="linear", case="staggered", duration=3.0):
    simultaneous = [
        dict(id="fl", fault_type="stuck_hold", wheel=0, t_start=0.713, t_end=1.713),
        dict(id="fr", fault_type="stuck_hold", wheel=1, t_start=0.713, t_end=1.713),
    ]
    staggered = [
        simultaneous[0],
        dict(id="rr", fault_type="free_caster", wheel=3, t_start=1.123, t_end=2.123),
        dict(id="bias", fault_type="sensor_bias", wheel=1, value=0.02, t_start=1.321),
        dict(id="noise", fault_type="sensor_noise", wheel=1, value=0.003, t_start=1.321),
        dict(id="drop", fault_type="sensor_dropout", wheel=0, t_start=1.543, t_end=2.321),
    ]
    return Experiment.model_validate(
        dict(
            name="native_parity",
            model_type=model,
            strategy="ideal_ackermann",
            dt=0.005,
            record_hz=200,
            fault_seed=20261114,
            vehicle={"overrides": {"tire_model": tire}},
            faults={"baseline": [], "simultaneous": simultaneous, "staggered": staggered}[case],
            maneuver={
                "steps": [
                    {
                        "duration": duration,
                        "speed_kmh": 40,
                        "speed_ramp_s": 1,
                        "steer": {
                            "kind": "sine",
                            "amplitude": 2,
                            "unit": "front_deg",
                            "freq_hz": 0.2,
                        },
                    }
                ]
            },
        )
    )


def channel_tolerance(name):
    # Near the friction-circle boundary, sqrt(cap^2-F^2) magnifies ulp errors.
    # 1e-5 N is < 2e-9 of a typical corner load. Positions/angles use 1e-9 SI.
    if name.startswith(("fz_", "rack_force_", "motor_torque_", "torque_steer_", "grip_margin_")):
        return 1e-5
    return 1e-9


@pytest.mark.parametrize("model", ["simplified_dynamic", "multibody"])
@pytest.mark.parametrize("tire", ["linear", "pacejka"])
@pytest.mark.parametrize("case", ["baseline", "simultaneous", "staggered"])
def test_complete_recorded_trajectories_and_exact_fault_timeline(model, tire, case):
    exp = example(model, tire, case)
    with native.use_engine("python"):
        reference = run_experiment(exp)
    with native.use_engine("native"):
        actual = run_experiment(exp)
    assert actual.meta["engine"]["body_core"] == "cpp17"
    assert actual.t == reference.t
    assert actual.meta["fault_timeline"] == reference.meta["fault_timeline"]
    assert actual.n_steps == reference.n_steps
    for name in reference.channels:
        np.testing.assert_allclose(
            actual.channels[name],
            reference.channels[name],
            rtol=1e-9,
            atol=channel_tolerance(name),
            equal_nan=True,
            err_msg=name,
        )


def _trace(cls, tire, reverse, detailed):
    curve = KinematicCurve((-0.08, -0.01, 0.0, 0.015, 0.08), (0.009, 0.003, 0.0, -0.002, -0.008))
    axle = AxleKC(
        toe=curve,
        camber=curve,
        compliance=ComplianceCoeffs(
            toe_per_fy=2e-7,
            toe_per_fx=-1e-7,
            toe_per_mz=4e-6,
            camber_per_fy=2e-7,
            camber_per_fz=-1e-7,
        ),
    )
    p = replace(
        VehicleParams(),
        tire_model=tire,
        cg_to_front=1.12,
        tire_load_sensitivity_time_domain=True,
        tire_mu_load_sensitivity=0.14,
        tire_relax_length=0.4,
        aero_lift_coeff_front=0.1,
        aero_lift_coeff_rear=-0.07,
        kc=VehicleKC(axle, axle),
        longitudinal_mode="torque",
    )
    p = replace(p, suspension=replace(p.suspension, roll_stiffness_front_frac=0.63))
    p.steering_system.enabled = detailed
    m = cls(p)
    m.state.vx = -8.0 if reverse else 8.0
    m.state.wheel_omega[:] = m.state.vx / p.tire_radius
    m.state.vy = 0.03
    m.state.yaw_rate = -0.02
    out = []
    for k in range(240):
        cmd = ControlCommand.zero()
        cmd.delta_cmd[:] = np.array([1, 0.9, -0.1, -0.15]) * 0.03 * math.sin(k / 50)
        cmd.drive_torque_cmd[:] = (-1 if reverse else 1) * 100.0
        cmd.brake_cmd[:] = 0.4 if k > 120 else 0.0
        m.step(0.00125, cmd, EnvironmentState(mu=0.63))
        update_derived_outputs(m.state, p)
        s = m.state
        out.append(
            np.r_[
                s.x,
                s.y,
                s.psi,
                s.vx,
                s.vy,
                s.yaw_rate,
                s.ax,
                s.ay,
                s.z,
                s.roll,
                s.pitch,
                s.delta,
                s.wheel_omega,
                s.fz,
                m.tire_fx,
                m.tire_fy,
                m.tire_mz,
                m.kc_toe,
                m.kc_camber,
                s.torque_steer,
                s.rack_force,
            ]
        )
    assert m._body_core == ("python" if native.requested_engine() == "python" else "cpp17")
    return out


@pytest.mark.parametrize("cls", [SimplifiedDynamicModel, MultiBodyModel])
@pytest.mark.parametrize("tire", ["linear", "pacejka"])
@pytest.mark.parametrize("reverse,detailed", [(False, False), (True, False), (False, True)])
def test_kc_relaxation_load_sensitivity_reverse_brake_and_steering(cls, tire, reverse, detailed):
    with native.use_engine("python"):
        expected = _trace(cls, tire, reverse, detailed)
    with native.use_engine("native"):
        actual = _trace(cls, tire, reverse, detailed)
    np.testing.assert_allclose(actual, expected, rtol=1e-9, atol=1e-8)


@pytest.mark.parametrize(
    "tire",
    [LinearTireModel(fz_nom=7000, load_exp=0.8), PacejkaTireModel(fz_nom=7000, load_exp=0.8)],
)
def test_native_tyre_friction_and_implicit_angular_impulse(tire):
    rng = np.random.default_rng(430)
    with native.use_engine("native"):
        for _ in range(100):
            # Include reverse and unloaded contacts; no brake clamp in the
            # angular impulse assertion (a clamped brake adds a stop impulse).
            speed = rng.uniform(-25, 25, 4)
            omega = speed / 0.395 + rng.uniform(-8, 8, 4)
            alpha = rng.uniform(-0.4, 0.4, 4)
            fz = rng.uniform(0, 14000, 4)
            fz[0] = 0
            mu = rng.uniform(0.05, 1.2, 4)
            torque = rng.uniform(-1600, 1600, 4)
            new, forces, k = semi_implicit_wheel_spin(
                dt=0.00125,
                wheel_omega=omega,
                vx_wheel=speed,
                alpha=alpha,
                fz=fz,
                mu=mu,
                torque=torque,
                tire=tire,
                tire_radius=0.395,
                wheel_inertia=2.5,
            )
            assert np.all(np.hypot(forces.fx, forces.fy) <= mu * fz * (1 + 1e-12) + 1e-10)
            np.testing.assert_allclose(
                2.5 * (new - omega), 0.00125 * (torque - 0.395 * forces.fx), atol=3e-8, rtol=1e-8
            )
            expected = np.array([tire.forces(alpha[i], k[i], fz[i], mu[i]) for i in range(4)])
            np.testing.assert_allclose(
                np.column_stack((forces.fx, forces.fy, forces.mz)), expected, atol=1e-8, rtol=1e-11
            )


def test_native_linkage_virtual_work_and_singular_stroke_flags():
    p = VehicleParams()
    rng = np.random.default_rng(20)
    # Obtain the actual geometry field without freezing a separate test model.
    g = p.steering_geometry
    for _ in range(150):
        delta = rng.uniform(-1.2, 1.2, 4)
        with native.use_engine("python"):
            expected = steering_linkage_metrics(delta, g)
        with native.use_engine("native"):
            actual = steering_linkage_metrics(delta, g)
        for name in expected:
            np.testing.assert_allclose(actual[name], expected[name], atol=1e-11, rtol=1e-10)
    delta = np.full(4, 0.02)
    h = 1e-6
    with native.use_engine("native"):
        m = steering_linkage_metrics(delta, g)
        jac = (
            steering_linkage_metrics(delta + h, g)["rack_travel"]
            - steering_linkage_metrics(delta - h, g)["rack_travel"]
        ) / (2 * h)
    np.testing.assert_allclose(m["rack_travel_derivative"], jac, atol=1e-8)


def test_custom_python_force_law_is_not_replaced(monkeypatch):
    class Custom(LinearTireModel):
        def forces(self, *args):
            return 0.0, 0.0, 0.0

    m = SimplifiedDynamicModel(VehicleParams(), Custom())
    with native.use_engine("native"):
        m.step(0.00125, ControlCommand.zero(), EnvironmentState())
        assert m._body_core == "python"
        assert "custom Python tyre" in m._body_fallback_reason
        assert np.array_equal(m.tire_fx, np.zeros(4))
    monkeypatch.setattr(LinearTireModel, "c_alpha_eff", lambda self, fz: 12.)
    assert native.tire_parameters(LinearTireModel()) is None


def test_overridden_hook_remains_observable():
    m = MultiBodyModel(VehicleParams())
    calls = []
    original = m._derivatives

    def observe(*args):
        calls.append(1)
        return original(*args)

    m._derivatives = observe
    with native.use_engine("native"):
        m.step(0.00125, ControlCommand.zero(), EnvironmentState())
    assert len(calls) == 5 and m._body_core == "python"


def test_invalid_native_buffers_fail_without_mutating_outputs():
    lib = native._native
    t = native.tire_parameters(LinearTireModel())
    q = np.array([0.001, 0.395, 2.5, 0.5])
    data = np.zeros((4, 7))
    out = np.full((4, 5), 123.0)
    invalid = [
        np.zeros(27),
        np.zeros((4, 7), dtype=np.float32),
        np.zeros((4, 14))[:, ::2],
        np.full((4, 7), np.nan),
        np.zeros((4, 7), dtype=">f8"),
    ]
    for bad in invalid:
        with pytest.raises((ValueError, BufferError)):
            lib.spin(t, q, bad, out)
        assert np.all(out == 123.0)
    with pytest.raises(ValueError):
        lib.spin(t, q, data, data.ravel()[:20])
    out.setflags(write=False)
    with pytest.raises((ValueError, BufferError)):
        lib.spin(t, q, data, out)


def test_explicit_missing_native_core_fails_and_auto_is_reported(monkeypatch):
    monkeypatch.setattr(native, "_native", None)
    with pytest.raises(RuntimeError, match="unavailable"):
        with native.use_engine("native"):
            pass
    with native.use_engine("auto"):
        assert native.engine_info()["numerical_core"] == "python"


def test_native_body_obeys_independent_offcentre_force_and_moment_balance():
    rng = np.random.default_rng(905)
    for _ in range(60):
        p = replace(
            VehicleParams(),
            cg_to_front=float(rng.uniform(0.8, 2.1)),
            drag_coeff_cd=0.0,
            rolling_resistance_coeff=0.0,
            camber_thrust_coeff=0.0,
            tire_relax_length=0.0,
            tire_mu_load_sensitivity=0.0,
        )
        m = SimplifiedDynamicModel(p, LinearTireModel())
        y0 = rng.uniform([2.0, -0.5, -0.3], [25.0, 0.5, 0.3])
        delta = rng.uniform(-0.2, 0.2, 4)
        m.state.wheel_omega[:] = rng.uniform(5.0, 60.0, 4)
        mus = rng.uniform(0.1, 1.0, 4)
        with native.use_engine("native"):
            accepted, derivative = native.integrate(
                m, 0, y0, 0.00125, delta, mus, np.zeros(4), fz_offset=np.zeros(4)
            )
        u, v, r = accepted
        fx, fy, mz = [], [], []
        for i, (xw, yw) in enumerate(p.wheel_positions_body()):
            c, s = math.cos(delta[i]), math.sin(delta[i])
            vx, vy = u - r * yw, v + r * xw
            vxw, vyw = c * vx + s * vy, -s * vx + c * vy
            den = max(abs(vxw), 0.5)
            force = m.tire.forces(
                m._alpha_scale[i] * math.atan2(vyw, den),
                (p.tire_radius * m.state.wheel_omega[i] - vxw) / den,
                m._fz_base[i],
                mus[i],
            )
            fx.append(c * force[0] - s * force[1])
            fy.append(s * force[0] + c * force[1])
            mz.append(force[2])
        e = p.wheelbase / 2 - p.cg_to_front
        xw, yw = p.wheel_positions_body().T
        force_balance = [sum(fx), sum(fy), sum((xw - e) * fy - yw * fx + mz)]
        from_derivative = [
            p.mass * (derivative[0] - r * v - e * r * r),
            p.mass * (derivative[1] + r * u + e * derivative[2]),
            p.inertia_z * derivative[2],
        ]
        np.testing.assert_allclose(from_derivative, force_balance, atol=1e-8, rtol=1e-12)
