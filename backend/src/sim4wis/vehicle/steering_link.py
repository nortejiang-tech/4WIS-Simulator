"""Coupling between the vehicle models and the steering plant.

Kept in one place because two time-domain models need identical behaviour here,
and because the coupling carries three decisions that are easy to get subtly
wrong and hard to notice afterwards.

**Who decides what.** The strategy still commands a wheel angle; the plant
decides how the mechanism gets there. That preserves every existing control
law — a 4WIS allocation, a zero-sideslip rear law, a trajectory follower all
keep producing `delta_cmd` and none of them need to know a steering system
appeared underneath. What changes is that reaching the commanded angle now
costs assist, fights friction, and can run into a motor limit.

**Two different plants, one interface.** A mechanical axle maps a hand-wheel
angle onto a wheel angle through a torsion bar and assist. A by-wire axle
tracks a commanded wheel angle with one actuator and synthesises hand torque
with another, and the two are connected only by software. `front_axle_step`
hides that difference from the vehicle models, which only ever want a front
wheel angle back — but nothing else is shared, and in particular a by-wire
configuration never produces a torque-sensor signal, because the hardware has
no torsion bar to produce one with.

**The rack force is one step old.** `update_derived_outputs` fills
`state.rack_force` *after* `model.step()` returns, so what the plant sees is
the previous step's load. That is ordinary coupling lag at 5 ms, not a defect,
and it is written down here because the next person to read it will otherwise
spend an afternoon deciding whether it is one. On the first step it is zero and
the plant starts unloaded.

**The steering layer is multi-rate.** The vehicle outer loop steps at 5 ms, but
the column mode (~10 Hz, and 5x+ once assist closes the loop) cannot be resolved
by holding the commanded angle across that whole span: the peak hand torque came
out a third low because the peak fell between outer samples (D5). The mechanical
path therefore advances the plant at `STEERING_INNER_DT` = 0.5 ms with the
command interpolated across the span — a ramp to the inner loop, not a staircase
— while the outer-loop quantities (rack force, vehicle speed) stay held, and the
outputs (δ, hand torque) are read at the 5 ms boundary so nothing downstream
changes shape. 0.5 ms vs a 0.25 ms reference agree to <2% on the peak.
"""

from __future__ import annotations

import math

import numpy as np

from sim4wis.core.state import VehicleParams
from sim4wis.steering import architecture as arch
from sim4wis.steering.assist import get as get_assist_map
from sim4wis.steering.bywire import ByWirePlant
from sim4wis.steering.plant import STEERING_INNER_DT, SteeringPlant


def make_steering_plant(
    params: VehicleParams,
) -> tuple[SteeringPlant | ByWirePlant | None, float]:
    """Build the plant this vehicle's architecture calls for.

    Returns (plant, mechanical ratio). The ratio is returned alongside so
    callers do not re-derive it; it is meaningless for a by-wire axle, where
    the ratio is a software choice rather than a gear.
    """
    from sim4wis.controller.steering_feel import low_speed_gear_ratio

    ratio = max(float(low_speed_gear_ratio(params)), 1e-6)
    sys_params = getattr(params, "steering_system", None)
    if sys_params is None or not sys_params.enabled:
        return None, ratio

    try:
        architecture = arch.get(sys_params.architecture)
    except arch.ArchitectureError:
        return None, ratio
    if architecture.front_path != "mechanical":
        return ByWirePlant(), ratio

    plant = SteeringPlant(
        params=sys_params,
        assist_map=get_assist_map(sys_params.assist_map),
        pinion_radius=float(params.pinion_radius),
        motor_gear_ratio=float(architecture.motor_gear_ratio),
    )
    plant.reset()
    return plant, ratio


#: What a run records from the front axle, whatever kind of axle it is.
#:
#: One fixed set across architectures, with **NaN for signals this hardware does
#: not have** — a by-wire car has no torsion bar, so recording 0.0 in
#: `steer_torque_sensor` would read as "the sensor said zero" rather than "there
#: is no sensor", and every metric downstream would believe it. NaN propagates
#: instead, and the study layer already treats a NaN metric as untrusted rather
#: than as a measurement.
STEERING_CHANNELS: tuple[str, ...] = (
    "steer_hand_torque",       # what the driver's hands feel [N·m]
    "steer_hand_angle",        # steering wheel angle [rad] — the x-axis of every
                               # on-centre plot, so it is not optional
    "steer_torque_sensor",     # torsion-bar reading; mechanical axles only
    "steer_assist_torque",     # assist referred to the pinion [N·m]
    "steer_motor_torque",      # assist motor, or the by-wire feedback motor
    "steer_motor_speed",       # [rad/s] at the motor shaft
    "steer_angle_deviation",   # commanded − actual road wheel; by-wire only
    "steer_plant_active",      # 1.0 when the plant produced this sample
    # Per-corner tracking-layer deviations [rad] — NaN when the layer is off
    # or the corner has no actuator. The control-quality signal per wheel.
    "steer_corner_deviation_fl",
    "steer_corner_deviation_fr",
    "steer_corner_deviation_rl",
    "steer_corner_deviation_rr",
)

_ABSENT = {name: math.nan for name in STEERING_CHANNELS}


def idle_channels() -> dict[str, float]:
    """What a model without a steering plant reports.

    Not zeros. A run with no plant has no hand torque — it does not have a hand
    torque of zero — and the difference is what stops a target from being
    scored MET against a channel nothing simulated.
    """
    return {**_ABSENT, "steer_plant_active": 0.0}


def _channels(plant: SteeringPlant | ByWirePlant) -> dict[str, float]:
    out = dict(_ABSENT)
    out["steer_plant_active"] = 1.0
    s = plant.state
    if isinstance(plant, ByWirePlant):
        out["steer_hand_torque"] = s.hand_torque
        out["steer_motor_torque"] = s.hand_torque
        out["steer_angle_deviation"] = s.angle_deviation
        # `steer_hand_angle` is filled by the caller: on a by-wire axle the hand
        # wheel is an independent input the plant does not own.
        return out
    out["steer_hand_torque"] = s.hand_torque
    out["steer_hand_angle"] = s.hand_angle
    out["steer_torque_sensor"] = s.torque_sensor
    out["steer_assist_torque"] = s.assist_torque
    out["steer_motor_torque"] = s.motor_torque
    out["steer_motor_speed"] = s.motor_speed
    return out


def front_axle_step(
    plant: SteeringPlant | ByWirePlant,
    mech_ratio: float,
    delta_cmd: np.ndarray,
    dt: float,
    *,
    prev_hand: float,
    rack_force: float,
    speed_ms: float,
) -> tuple[float, float, dict[str, float]]:
    """One front-axle step, whichever kind of plant this is."""
    if isinstance(plant, ByWirePlant):
        # The hand wheel is an independent input. Until a real hand-wheel
        # signal is plumbed through, it follows the commanded angle through the
        # nominal ratio — which is what a driver holding that angle would have
        # done, and keeps the feel calibration exercised.
        delta_f_cmd = 0.5 * (float(delta_cmd[0]) + float(delta_cmd[1]))
        hand = delta_f_cmd * mech_ratio
        hand_rate = (hand - prev_hand) / max(dt, 1e-9)
        state = plant.step(
            dt,
            hand_angle=hand,
            hand_rate=hand_rate,
            delta_cmd=delta_f_cmd,
            rack_force=rack_force,
            speed_ms=speed_ms,
        )
        chans = _channels(plant)
        chans["steer_hand_angle"] = hand
        return state.road_wheel_angle, hand, chans
    angle, hand = plant_front_angle(
        plant, mech_ratio, delta_cmd, dt,
        prev_hand=prev_hand, rack_force=rack_force, speed_ms=speed_ms,
    )
    return angle, hand, _channels(plant)


def plant_front_angle(
    plant: SteeringPlant,
    mech_ratio: float,
    delta_cmd: np.ndarray,
    dt: float,
    *,
    prev_hand: float,
    rack_force: float,
    speed_ms: float,
    inner_dt: float = STEERING_INNER_DT,
) -> tuple[float, float]:
    """Advance the plant one vehicle step; return (front wheel angle, hand angle).

    The commanded front angle is converted to a hand-wheel angle through the
    mechanism's ratio, and the resulting pinion angle back again, so the ratio
    cancels in steady state and the plant contributes dynamics rather than a
    second gear stage.

    Multi-rate: the plant is advanced at the layer's inner rate (0.5 ms by
    default) with the commanded hand angle linearly interpolated across the
    span, instead of being held for the whole 5 ms outer step. That gives the
    inner loop the continuous excitation the column mode needs — the old
    zero-order hold under-resolved the peak hand torque by ~1/3 (D5). The
    outer-loop quantities (`rack_force`, `speed_ms`) stay held: the vehicle
    has nothing finer to offer for them, and they are the slow dynamics. The
    returned state is read at the outer boundary, so downstream consumers see
    exactly the same step shape as before.
    """
    delta_f_cmd = 0.5 * (float(delta_cmd[0]) + float(delta_cmd[1]))
    hand = delta_f_cmd * mech_ratio
    n_inner = max(1, int(round(dt / max(float(inner_dt), 1e-9))))
    h = dt / n_inner
    hand_rate = (hand - prev_hand) / max(dt, 1e-9)
    state = plant.state
    for k in range(n_inner):
        frac = (k + 1) / n_inner
        hand_k = prev_hand + (hand - prev_hand) * frac
        state = plant.step(
            h,
            hand_angle=hand_k,
            hand_rate=hand_rate,
            rack_force=rack_force,
            speed_ms=speed_ms,
        )
    return state.pinion_angle / mech_ratio, hand
