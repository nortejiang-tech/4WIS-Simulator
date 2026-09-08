"""Longitudinal driver model — shared by every controller strategy.

Before work-package A, each strategy did its own

    v_cmd = p.v_max * float(driver.throttle)

with throttle ∈ [-1, +1] (negative = brake/reverse). That conflated four
distinct physical concerns into one signed scalar:

    * how hard to drive forward
    * how hard to brake
    * which direction to face (gear)
    * the brake's μ-dependence (a real brake loses stopping power on ice;
      the old "negative throttle" reversed the motors and braked just as
      hard on ice as on dry asphalt)

This module turns the new (throttle, brake, gear) driver channels into a
signed target body-frame longitudinal speed `v_cmd`, shared by *all*
strategies and *all* vehicle models:

    * Drive:   v_target = gear_sign · throttle · (v_max if D else v_max_reverse)
    * Coast:   brake=0, throttle=0 → v_target = 0 (only drag/rolling resistance)
    * Brake:   v_target is pulled toward 0 at a finite, μ-aware deceleration,
               so the *kinematic* model (which has no force concept) still
               brakes at a believable rate on ice vs asphalt. The dynamic
               model also applies a real friction-brake torque downstream
               (see vehicle/dynamic.py); both layers seeing the same μ is what
               makes the behaviour consistent across models.

The output is a single `v_cmd` the strategies use exactly where they used to
write `p.v_max * throttle`. Keep that substitution a one-liner so every
strategy stays in sync.
"""

from __future__ import annotations

import numpy as np

from sim4wis.core.state import (
    N_WHEELS,
    ControlCommand,
    DriverInput,
    VehicleParams,
    VehicleState,
)

# A brake/launch only counts once the vehicle is slower than this; above it a
# brake command still brakes (toward 0) but a direction change cannot engage.
# Matches the front-end direction-intent state machine threshold.
_GEAR_CHANGE_SPEED = 0.3   # m/s


def cornering_accel_limit(params: VehicleParams, driver: DriverInput,
                          state: VehicleState) -> float | None:
    """Grip envelope for interactive speed references; explicit tests bypass."""
    if (driver.mode_params or {}).get("speed_target_ms") is not None:
        return None
    return max(float(state.mu_avg), 0.0) * 9.81


def speed_command(params: VehicleParams, driver: DriverInput,
                  v_actual: float, dt: float, mu_eff: float = 0.85) -> float:
    """(throttle, brake, gear) → signed longitudinal speed command [m/s].

    Parameters
    ----------
    params      vehicle params (v_max, v_max_reverse)
    driver      the DriverInput for this tick
    v_actual    current body-frame longitudinal speed [m/s] (state.vx)
    dt          timestep [s] (used only to clamp the brake deceleration)
    mu_eff      effective friction coefficient the brake is allowed to use
                (four-wheel mean). Pass `state.mu_avg` — the models keep it
                updated from the actual per-wheel surface μ, so the kinematic
                model's brake rate drops on ice just like the dynamic one's.
                The 0.85 default is dry asphalt, for callers without a state.

    Authority order — brake first, then gear, then throttle:

      * The brake outranks the throttle. Pressing both (left-foot braking, or
        a wheel whose pedals overlap) must not launch the car; the pedal that
        removes energy wins.
      * The brake also outranks the gear selector, including Neutral. A real
        car's service brake does not care what gear you are in.
      * Only with the brake released does the gear decide: N coasts (target
        holds current speed — drag and rolling resistance still act in the
        dynamic models), D/R drive toward their own speed cap.

    The parking brake is folded in as a floor on the brake pedal so a handbrake
    alone still commands a stop.
    """
    throttle = float(driver.throttle)
    gear = int(driver.gear)
    # Parking brake acts as a brake demand of its own (rear axle only in the
    # torque model, but from the speed-command layer's view it is a stop).
    brake = max(float(driver.brake),
                float(1.0 - float(getattr(params, "brake_bias_front", 0.65)))
                if driver.handbrake else 0.0)

    if brake > 1e-3:
        # Pull v toward 0 at a finite, μ-limited deceleration. This is what
        # gives the *kinematic* model a believable stopping distance (it has no
        # notion of force, so without this the wheels would stop instantly).
        # The dynamic models apply a real friction torque in addition — both
        # layers seeing the same μ is what keeps them consistent.
        dv = brake * float(mu_eff) * 9.81 * dt    # most v can change this step
        if v_actual > 0.0:
            return max(0.0, v_actual - dv)
        if v_actual < 0.0:
            return min(0.0, v_actual + dv)
        return 0.0

    # Validation bypass — the longitudinal twin of `steer_raw_rad`.
    #
    # An experiment commands a speed in m/s directly, skipping the pedal
    # mapping entirely, so a regression baseline measures the *vehicle* and not
    # the driver-input model. Without it the experiment layer has to round-trip
    # its target through the pedal (`throttle = v/v_max`), which only cancels
    # while that mapping stays exactly linear over exactly [0, v_max] — every
    # future pedal curve, speed limiter or powertrain envelope would silently
    # re-baseline every golden. Handled here rather than per-strategy because
    # every strategy already routes through this one function.
    target = driver.mode_params.get("speed_target_ms") if driver.mode_params else None
    if target is not None:
        return float(target)

    if gear == 0:
        # Neutral: no drive, no engine brake — target holds current speed so
        # the vehicle coasts (drag/rolling resistance still act in the model).
        return float(v_actual)

    gear_sign = -1.0 if gear < 0 else 1.0
    v_cap = float(params.v_max) if gear > 0 else float(getattr(params, "v_max_reverse", 5.0))
    # Driver speed limit rescales the pedal's range instead of clipping its
    # top: full travel then means the limit, so the whole pedal is spent on the
    # speeds actually being driven. Clipping would leave most of the travel
    # dead and make the control problem worse, not better.
    limit = float(getattr(params, "driver_speed_limit", 0.0))
    if limit > 0.0:
        v_cap = min(v_cap, limit)
    return gear_sign * throttle * v_cap


def apply_brake_command(cmd: ControlCommand, driver: DriverInput,
                        params: VehicleParams) -> ControlCommand:
    """Fill the friction-brake actuator channels on `cmd` from the driver.

    Strategies reason about motion targets; braking is a vehicle-actuator
    concern, so the brake pedal is turned into per-wheel commands here, after
    the strategy has run. Both the live simulator loop and the offline
    experiment session call this — they must not drift apart, which is why it
    lives in one place rather than being open-coded in each.

    Each wheel gets its axle's share of the pedal (`brake_bias_front` forward,
    the remainder aft). The parking brake applies that same rear-axle share on
    its own, so pulling it does not out-brake a full pedal on the rear axle.
    """
    bias = float(getattr(params, "brake_bias_front", 0.65))
    front = bias * float(driver.brake)
    rear = (1.0 - bias) * float(driver.brake)
    if driver.handbrake:
        rear = max(rear, 1.0 - bias)
    cmd.brake_cmd = np.array([front, front, rear, rear], dtype=np.float64)
    cmd.handbrake = int(driver.handbrake)
    return cmd


def apply_drive_command(cmd: ControlCommand, driver: DriverInput,
                        params: VehicleParams,
                        state: VehicleState | None = None) -> ControlCommand:
    """Fill the drive-torque actuator channels on `cmd` from the driver.

    Only meaningful when `longitudinal_mode == "torque"`; in the default
    "speed_servo" mode the channel stays zero and the wheel-speed servos do the
    work from `wheel_speed_cmd`.

    The split itself lives in `vehicle/powertrain.py` — this is just the seam
    where the loop and the experiment session agree on where drive torque comes
    from, mirroring `apply_brake_command`.

    `state` supplies wheel speed (for the power bus) and last step's friction
    capacity (for the open differential). Without it the drivetrain falls back
    to the torque cap alone, which is what a caller with no state can honestly
    ask for.
    """
    if str(getattr(params, "longitudinal_mode", "speed_servo")) != "torque":
        cmd.drive_torque_cmd = np.zeros(N_WHEELS)
        return cmd

    from sim4wis.vehicle.powertrain import drive_torques  # local: avoids a cycle

    if state is None:
        state = VehicleState()
    cmd.drive_torque_cmd = drive_torques(params, driver, state)
    return cmd
