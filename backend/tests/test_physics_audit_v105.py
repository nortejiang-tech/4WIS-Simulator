"""Manufactured-force, momentum, support and friction oracles for v0.105."""
from dataclasses import replace
import math
import numpy as np
import pytest
from sim4wis.core.state import VehicleParams, EnvironmentState
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel
from sim4wis.vehicle.multibody import MultiBodyModel
from sim4wis.vehicle.tire import LinearTireModel, PacejkaTireModel
from sim4wis.vehicle.load_transfer import vertical_loads
from sim4wis.vehicle.wheel_dynamics import implicit_spin


def test_vehicle_substeps_preserve_outer_handwheel_ramp_and_inner_rate():
    from sim4wis.core.state import ControlCommand, DriverInput
    from sim4wis.core.step import advance_model
    from sim4wis.fault import FaultInjector
    from sim4wis.fault.schema import FaultSpec
    p = VehicleParams()
    p.steering_system.enabled = True
    model = SimplifiedDynamicModel(p)
    cmd = ControlCommand.zero()
    cmd.delta_cmd[:2] = .01
    samples = []
    original = model._steering.step

    def observe(dt, **kw):
        samples.append((dt, kw["hand_angle"], kw["hand_rate"]))
        return original(dt, **kw)

    model._steering.step = observe
    # An off-grid sensor event creates unequal substeps; the physical input
    # still has the same constant slope throughout the entire outer frame.
    fi = FaultInjector([FaultSpec(fault_type="sensor_bias", wheel=0, t_start=.0017)])
    advance_model(model, p, DriverInput(), cmd, .005, EnvironmentState(), fi)
    elapsed = 0.
    rate = .01 * model._mech_ratio / .005
    for dt, angle, actual_rate in samples:
        elapsed += dt
        assert dt <= .0005 + 1e-12
        assert actual_rate == pytest.approx(rate)
        assert angle == pytest.approx(rate * elapsed)
    assert elapsed == pytest.approx(.005)


class KnownForce(LinearTireModel):
    def forces(self, alpha, kappa, fz, mu):
        return 120., 50., -3.


@pytest.mark.parametrize("cls", [SimplifiedDynamicModel, MultiBodyModel])
def test_full_newton_euler_force_and_moment_balance_at_offcentre_origin(cls):
    rng = np.random.default_rng(902)
    for _ in range(30):
        p = replace(VehicleParams(), cg_to_front=float(rng.uniform(.8, 2.1)),
                    drag_coeff_cd=0, rolling_resistance_coeff=0, static_toe_front=0,
                    static_toe_rear=0, bump_steer_coeff=0)
        model = cls(p, KnownForce())
        u, v, r = rng.uniform([2, -.5, -.2], [20, .5, .2])
        delta = rng.uniform(-.3, .3, 4)
        if cls is SimplifiedDynamicModel:
            der = model._derivatives(np.array([u,v,r]), np.full(4, u/p.tire_radius), delta,
                np.full(4,.9), np.zeros(4), 0, EnvironmentState())
        else:
            y = np.zeros(24); y[10:13] = [u,v,r]; y[20:24] = u/p.tire_radius
            der = model._derivatives(y, delta, np.full(4,.9), np.zeros(4))[10:13]
        # Independent wheel-to-body rotation and lever arms about G.
        fx = 120*np.cos(delta) - 50*np.sin(delta)
        fy = 120*np.sin(delta) + 50*np.cos(delta)
        e = p.wheelbase/2-p.cg_to_front
        x, y = p.wheel_positions_body().T
        ax_g = der[0] - r*v - e*r*r
        ay_g = der[1] + r*u + e*der[2]
        np.testing.assert_allclose([p.mass*ax_g, p.mass*ay_g, p.inertia_z*der[2]],
                                  [fx.sum(), fy.sum(), ((x-e)*fy-y*fx-3).sum()], atol=1e-8)


def test_support_loads_conserve_weight_and_unsaturated_pitch_roll_moments():
    p = VehicleParams()
    for ax in [-1.5, 0, 1.5]:
        for ay in [-2, 0, 2]:
            fz = vertical_loads(p, ax, ay)
            x,y = p.wheel_positions_body().T
            e = p.wheelbase/2-p.cg_to_front
            assert sum(fz) == pytest.approx(p.mass*9.81)
            assert sum((x-e)*fz) == pytest.approx(-p.mass*ax*p.cg_height, abs=1e-8)
            assert sum(y*fz) == pytest.approx(-p.mass*ay*p.cg_height, abs=1e-8)
    for ax, ay in [(50,50),(-50,-50),(0,100)]:
        fz = vertical_loads(p,ax,ay)
        assert min(fz) >= 0
        assert sum(fz) == pytest.approx(p.mass*9.81)


@pytest.mark.parametrize("tire", [LinearTireModel(), PacejkaTireModel()])
def test_combined_slip_friction_circle_and_wheel_angular_impulse(tire):
    rng = np.random.default_rng(20261113)
    for _ in range(100):
        alpha,kappa = rng.uniform(-.5,.5,2)
        fz,mu = rng.uniform(100,12000),rng.uniform(.05,1.2)
        fx,fy,_ = tire.forces(alpha,kappa,fz,mu)
        assert math.hypot(fx,fy) <= mu*fz*(1+1e-10)
        speed = float(rng.choice([-1,1])*rng.uniform(.5,20))
        radius,inertia,dt = .395,2.5,.005
        old = speed/radius * (1+kappa)
        torque = rng.uniform(-1000,1000)
        new = implicit_spin(omega=old,vx=speed,alpha=alpha,fz=fz,mu=mu,torque=torque,
            dt=dt,radius=radius,inertia=inertia,denom=max(abs(speed),.5),tire=tire)
        fx_new = tire.forces(alpha,(radius*new-speed)/max(abs(speed),.5),fz,mu)[0]
        assert inertia*(new-old) == pytest.approx(dt*(torque-radius*fx_new),abs=1e-7)
