"""Shared accepted-step force chain for live, scripted and Agent execution."""
from sim4wis.controller.longitudinal import apply_brake_command, apply_drive_command
from sim4wis.core.derived import update_derived_outputs


def advance_model(model, params, driver, cmd, dt, env):
    """Advance once; errors propagate to the execution host, never reset here."""
    apply_brake_command(cmd, driver, params)
    apply_drive_command(cmd, driver, params, model.state)
    model.step(dt, cmd, env)
    update_derived_outputs(model.state, params)
    return model.state
