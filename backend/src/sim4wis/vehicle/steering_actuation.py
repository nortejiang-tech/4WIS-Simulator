"""Shared steering advance; passive/jammed corners bypass powered servo dynamics."""

from __future__ import annotations

import numpy as np

from sim4wis.steering.plant import SteeringPlant
from sim4wis.steering.tracking.coupling import corner_tracking_step
from sim4wis.vehicle.geometry import steer_actuator
from sim4wis.vehicle.steering_link import front_axle_step


def ramp_start(model, cmd):
    """Previous targets for plants that interpolate across an outer frame.

    Splitting a vehicle frame must not compress the existing steering ramp
    into its first substep. Legacy held-command actuators stay held.
    """
    start = cmd.delta_cmd.copy()
    if isinstance(getattr(model, "_steering", None), SteeringPlant):
        start[:2] = model._prev_hand / model._mech_ratio
    tracking = getattr(model, "_tracking", None)
    if tracking is not None:
        for i, tracker in tracking[0].items():
            if tracker.controller.inner_rate_hz is not None:
                start[i] = tracker.prev_target
    return start


def advance_steering(model, cmd, dt):
    p, s = model.params, model.state
    previous = model._delta_act.copy()
    override = np.isfinite(cmd.steer_override)
    if model._steering is None:
        model._delta_act = steer_actuator(
            previous, cmd.delta_cmd, dt, p.steer_tau, p.steer_rate_max
        )
    else:
        model._delta_act[2:] = steer_actuator(
            previous[2:], cmd.delta_cmd[2:], dt, p.steer_tau, p.steer_rate_max
        )
        delta_f, model._prev_hand, model.steering_channels = front_axle_step(
            model._steering,
            model._mech_ratio,
            cmd.delta_cmd,
            dt,
            prev_hand=model._prev_hand,
            rack_force=float(np.sum(s.rack_force[:2])),
            speed_ms=float(s.vx),
        )
        model._delta_act[0] = model._delta_act[1] = delta_f
    if model._tracking is not None:
        trackers, per_wheel = model._tracking
        healthy = {i: tr for i, tr in trackers.items() if not override[i]}
        tracked, channels = corner_tracking_step(
            healthy,
            per_wheel,
            cmd.delta_cmd,
            dt,
            rack_forces=s.rack_force,
            speed_ms=float(s.vx),
            pinion_radius=float(p.pinion_radius),
        )
        for i in healthy:
            model._delta_act[i] = tracked[i]
        model.steering_channels.update(channels)
        model._prev_delta_cmd = cmd.delta_cmd.copy()
    model._delta_act[override] = cmd.steer_override[override]
    model._steer_rate = (model._delta_act - previous) / dt
    model._steer_rate[override] = cmd.steer_override_rate[override]
    s.steer_rate[:] = model._steer_rate
    if model._tracking is not None:
        for i, tr in model._tracking[0].items():
            if override[i]:
                # No hidden powered-integrator windup while the mechanism is disconnected.
                tr.reset(float(model._delta_act[i]))
                for key in list(model.steering_channels):
                    if key.endswith(("_fl", "_fr", "_rl", "_rr")[i]):
                        model.steering_channels[key] = float("nan")
    return model._delta_act
