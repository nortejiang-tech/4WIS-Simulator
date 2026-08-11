"""Tests — the steering plant coupled to the vehicle models.

The first test is the guard the whole integration was built behind: with the
plant disabled, both time-domain models must be **bit-identical** to what they
were. Everything else here is only allowed to exist because that one holds.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from sim4wis.core.state import ControlCommand, EnvironmentState, VehicleParams
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel
from sim4wis.vehicle.multibody import MultiBodyModel

MODELS = (SimplifiedDynamicModel, MultiBodyModel)


def _params(**steering):
    p = VehicleParams()
    if steering:
        return dataclasses.replace(
            p, steering_system=dataclasses.replace(p.steering_system, **steering)
        )
    return p


def _run(cls, params, *, n=500, dt=0.005, delta_deg=3.0, speed_kmh=60.0):
    m = cls(params)
    m.reset()
    env = EnvironmentState(mu=0.9)
    trace = []
    for i in range(n):
        cmd = ControlCommand.zero()
        cmd.delta_cmd[:2] = 0.0 if i * dt < 0.5 else np.radians(delta_deg)
        cmd.wheel_speed_cmd[:] = speed_kmh / 3.6 / params.tire_radius
        m.step(dt, cmd, env)
        trace.append((m.state.yaw_rate, float(m.state.delta[0]), m.state.vy))
    return m, trace


@pytest.mark.parametrize("cls", MODELS)
def test_disabled_plant_changes_nothing(cls):
    """The compatibility promise, asserted rather than assumed.

    Not 'close to' — identical. A subsystem that is switched off must not be
    able to move a golden baseline by a rounding error either.
    """
    _, off = _run(cls, _params())
    _, again = _run(cls, _params(enabled=False))
    assert off == again


@pytest.mark.parametrize("cls", MODELS)
def test_a_by_wire_architecture_does_not_take_the_mechanical_path(cls):
    """SBW steers, but never through a column it does not have.

    Producing a torque-sensor signal for hardware without a torsion bar would
    be worse than producing nothing: a plausible number in a report about a car
    that cannot generate it. So the by-wire axle gets its own plant, and that
    plant has no torque-sensor channel at all.
    """
    from sim4wis.steering.bywire import ByWirePlant

    model, _ = _run(cls, _params(enabled=True, architecture="sbw"))
    assert isinstance(model._steering, ByWirePlant)
    assert not hasattr(model._steering.state, "torque_sensor")
    assert "steer_torque_sensor" not in model._steering.state.to_channels()


@pytest.mark.parametrize("cls", MODELS)
def test_an_enabled_mechanical_axle_still_reaches_the_commanded_angle(cls):
    """Compliance and friction shift the steady state, but only slightly.

    A steering system that could not deliver the angle it was asked for would
    show up here as a large error, not a small one.
    """
    _, off = _run(cls, _params())
    _, on = _run(cls, _params(enabled=True, architecture="r_eps"))
    d_off, d_on = off[-1][1], on[-1][1]
    assert d_on == pytest.approx(d_off, rel=0.03), f"{d_on:.5f} vs {d_off:.5f}"
    assert on[-1][0] == pytest.approx(off[-1][0], rel=0.03)


def test_the_plant_reports_hand_torque_once_coupled():
    """The signal that did not exist before this work, measured in the loop."""
    model, _ = _run(SimplifiedDynamicModel, _params(enabled=True, architecture="r_eps"))
    st = model._steering.state
    assert 0.05 < abs(st.hand_torque) < 20.0, st.hand_torque
    assert abs(st.torque_sensor) > 0.0
    assert abs(st.assist_torque) > 0.0


def test_architecture_selects_the_reduction_ratio():
    col, _ = _run(SimplifiedDynamicModel, _params(enabled=True, architecture="c_eps"))
    rack, _ = _run(SimplifiedDynamicModel, _params(enabled=True, architecture="r_eps"))
    assert col._steering.motor_gear_ratio == 20.0
    assert rack._steering.motor_gear_ratio == 63.0


def test_steering_params_survive_a_profile_round_trip():
    from sim4wis.project.params_codec import params_from_dict, params_to_dict

    p = _params(enabled=True, architecture="eps_rws", assist_map="none")
    back = params_from_dict(params_to_dict(p))
    assert back.steering_system.enabled is True
    assert back.steering_system.architecture == "eps_rws"
    assert back.steering_system.assist_map == "none"
