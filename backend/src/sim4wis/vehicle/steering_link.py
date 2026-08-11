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
"""

from __future__ import annotations

import numpy as np

from sim4wis.core.state import VehicleParams
from sim4wis.steering import architecture as arch
from sim4wis.steering.assist import get as get_assist_map
from sim4wis.steering.bywire import ByWirePlant
from sim4wis.steering.plant import SteeringPlant


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


def front_axle_step(
    plant: SteeringPlant | ByWirePlant,
    mech_ratio: float,
    delta_cmd: np.ndarray,
    dt: float,
    *,
    prev_hand: float,
    rack_force: float,
    speed_ms: float,
) -> tuple[float, float]:
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
        return state.road_wheel_angle, hand
    return plant_front_angle(
        plant, mech_ratio, delta_cmd, dt,
        prev_hand=prev_hand, rack_force=rack_force, speed_ms=speed_ms,
    )


def plant_front_angle(
    plant: SteeringPlant,
    mech_ratio: float,
    delta_cmd: np.ndarray,
    dt: float,
    *,
    prev_hand: float,
    rack_force: float,
    speed_ms: float,
) -> tuple[float, float]:
    """Advance the plant one step; return (front wheel angle, hand angle).

    The commanded front angle is converted to a hand-wheel angle through the
    mechanism's ratio, and the resulting pinion angle back again, so the ratio
    cancels in steady state and the plant contributes dynamics rather than a
    second gear stage.
    """
    delta_f_cmd = 0.5 * (float(delta_cmd[0]) + float(delta_cmd[1]))
    hand = delta_f_cmd * mech_ratio
    hand_rate = (hand - prev_hand) / max(dt, 1e-9)
    state = plant.step(
        dt,
        hand_angle=hand,
        hand_rate=hand_rate,
        rack_force=rack_force,
        speed_ms=speed_ms,
    )
    return state.pinion_angle / mech_ratio, hand
