"""Post-step derived outputs shared by the realtime loop and headless sessions.

After a model step the platform derives, from the raw VehicleState:
    * per-wheel steering-centre projection vs the vehicle ICR,
    * the split-rack force chain (kingpin τ → rack force → motor torque)
      through the hardpoint linkage.

Both the realtime `Simulator._loop` and the batch `SimSession` must produce
identical derived channels, so the computation lives here rather than being
inlined in either host.
"""

from __future__ import annotations

from sim4wis.core.state import VehicleParams, VehicleState
from sim4wis.vehicle.geometry import wheel_icr_projection, wheel_rack_force_from_linkage


def update_derived_outputs(state: VehicleState, params: VehicleParams) -> None:
    """Fill the derived fields of `state` in place from its raw fields."""
    state.wheel_icr_body, state.wheel_icr_dev = wheel_icr_projection(
        state.wheel_pos_body, state.delta, state.vehicle_icr_body
    )
    rack, motor, linkage = wheel_rack_force_from_linkage(
        state.torque_steer,
        state.delta,
        params.steering_geometry,
        params.steering_arm_length,
        params.pinion_radius,
        params.rack_mech_efficiency,
        params.motor_gear_ratio,
    )
    state.rack_force = rack
    state.motor_torque_demand = motor
    state.linkage_arm_tie_angle = linkage["arm_tie_angle"]
    state.linkage_tie_rack_angle = linkage["tie_rack_angle"]
    state.linkage_efficiency = linkage["efficiency"]
