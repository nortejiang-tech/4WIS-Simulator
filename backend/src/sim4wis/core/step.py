"""Shared accepted-step force chain for live, scripted and Agent execution."""

import math
from dataclasses import replace

import numpy as np

from sim4wis.controller.longitudinal import apply_brake_command, apply_drive_command
from sim4wis.core.derived import update_derived_outputs
from sim4wis.vehicle.steering_actuation import ramp_start

# The body/wheel/steering coupling is partitioned and only first order overall.
# A 5 ms outer control step is not a 5 ms integration accuracy guarantee.
# Internal 1.25 ms substeps bound the observed launch/trajectory splitting error
# while retaining the public control and recording cadence.
DYNAMIC_MAX_STEP = 0.00125


def advance_model(model, params, driver, cmd, dt, env, faults=None):
    """Hold control across dt, split at fault edges, then sample accepted states.

    Outer step/sample counts remain stable. Errors propagate; never reset here.
    """
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and positive")
    apply_brake_command(cmd, driver, params)
    apply_drive_command(cmd, driver, params, model.state)
    start_time = float(model.state.t)
    target = start_time + dt
    start_delta = ramp_start(model, cmd)
    ramp = not np.array_equal(start_delta, cmd.delta_cmd)
    while model.state.t < target - 1e-12:
        end = faults.next_boundary(model.state.t, target) if faults is not None else target
        if hasattr(model, "tire"):
            end = min(end, model.state.t + DYNAMIC_MAX_STEP)
        h = end - model.state.t
        frame_cmd = (
            replace(
                cmd,
                delta_cmd=start_delta + (cmd.delta_cmd - start_delta) * ((end - start_time) / dt),
            )
            if ramp
            else cmd
        )
        effective = faults.command(frame_cmd, model, h) if faults is not None else frame_cmd
        model.step(h, effective, env)
        s = model.state
        if not np.isfinite(
            np.r_[s.x, s.y, s.psi, s.vx, s.vy, s.yaw_rate, s.delta, s.fz, s.wheel_omega]
        ).all():
            raise ArithmeticError("non-finite physical state")
        if np.any(s.fz < 0):
            raise ArithmeticError("negative tyre support load")
        update_derived_outputs(model.state, params)
        if faults is not None:
            faults.sample(model)
        else:
            model.state.delta_reported[:] = model.state.delta
            model.state.fault_mode[:] = 0
            model.state.sensor_fault_active[:] = 0
        model.last_effective_cmd = effective
    return model.state
