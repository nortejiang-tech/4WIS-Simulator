"""Vehicle / suspension parameter endpoints (with physical-bounds validation)."""

from __future__ import annotations

import dataclasses
from typing import Any, Literal

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from sim4wis.core.simulator import get_simulator
from sim4wis.core.state import VehicleParams

router = APIRouter()


@router.get("/params")
async def get_params() -> dict[str, Any]:
    """Full vehicle + suspension parameter set (everything POST /params accepts)."""
    sim = get_simulator()
    return dataclasses.asdict(sim.params)


class ParamsUpdate(BaseModel):
    """Partial vehicle-parameter update. All fields optional; bounds reject
    physically meaningless values (pydantic returns 422 with details)."""

    wheelbase: float | None = Field(None, gt=0.5, le=10.0)
    track_front: float | None = Field(None, gt=0.3, le=5.0)
    track_rear: float | None = Field(None, gt=0.3, le=5.0)
    mass: float | None = Field(None, gt=100.0, le=50_000.0)
    inertia_z: float | None = Field(None, gt=10.0, le=1e6)
    cg_to_front: float | None = Field(None, gt=0.1, le=10.0)
    tire_radius: float | None = Field(None, gt=0.05, le=1.0)
    tire_width: float | None = Field(None, gt=0.05, le=0.8)
    contact_patch_radius: float | None = Field(None, gt=0.01, le=0.5)
    steer_limit: float | None = Field(None, gt=0.0, le=1.5708)
    v_max: float | None = Field(None, gt=0.0, le=120.0)
    cg_height: float | None = Field(None, gt=0.0, le=2.0)
    wheel_inertia: float | None = Field(None, gt=0.0, le=50.0)
    motor_torque_max: float | None = Field(None, gt=0.0, le=20_000.0)
    bump_steer_coeff: float | None = Field(None, ge=-10.0, le=10.0)
    unsprung_mass: float | None = Field(None, gt=0.0, le=500.0)
    tire_vertical_stiffness: float | None = Field(None, gt=1e4, le=2e6)
    steer_tau: float | None = Field(None, gt=0.005, le=1.0)
    steer_rate_max: float | None = Field(None, gt=0.1, le=50.0)
    tire_model: Literal["linear", "pacejka"] | None = None
    tire_c_alpha: float | None = Field(None, gt=1e3, le=1e6)
    tire_c_kappa: float | None = Field(None, gt=1e3, le=1e6)
    tire_t_pneumatic: float | None = Field(None, ge=0.0, le=0.2)
    tire_load_sensitivity_exp: float | None = Field(None, ge=0.0, le=2.0)
    parking_scrub_coeff: float | None = Field(None, ge=0.0, le=3.0)
    parking_lateral_coeff: float | None = Field(None, ge=0.0, le=3.0)
    parking_torque_coeff: float | None = Field(None, ge=0.0, le=3.0)
    low_speed_blend_ms: float | None = Field(None, gt=0.05, le=20.0)
    static_tire_deflection_deg: float | None = Field(None, gt=0.5, le=45.0)
    rolling_resistance_coeff: float | None = Field(None, ge=0.0, le=0.1)
    drag_coeff_cd: float | None = Field(None, ge=0.0, le=2.0)
    frontal_area: float | None = Field(None, gt=0.5, le=10.0)
    air_density: float | None = Field(None, gt=0.1, le=2.0)
    aero_lift_coeff_front: float | None = Field(None, ge=-1.0, le=2.0)
    aero_lift_coeff_rear: float | None = Field(None, ge=-1.0, le=2.0)
    camber_thrust_coeff: float | None = Field(None, ge=0.0, le=5.0)
    static_toe_front: float | None = Field(None, ge=-0.05, le=0.05)
    static_toe_rear: float | None = Field(None, ge=-0.05, le=0.05)
    tire_cx: float | None = Field(None, gt=1.0, le=2.5)
    tire_cy: float | None = Field(None, gt=1.0, le=2.5)
    tire_ex: float | None = Field(None, ge=-5.0, le=1.0)
    tire_ey: float | None = Field(None, ge=-5.0, le=1.0)
    servo_kp: float | None = Field(None, gt=0.0, le=5000.0)
    servo_ki: float | None = Field(None, ge=0.0, le=2000.0)
    # Split-rack transmission geometry
    steering_arm_length: float | None = Field(None, gt=0.05, le=0.5)
    pinion_radius: float | None = Field(None, gt=0.005, le=0.1)
    tie_rod_angle_deg: float | None = Field(None, ge=0.0, le=30.0)
    rack_mech_efficiency: float | None = Field(None, gt=0.5, le=1.0)
    motor_gear_ratio: float | None = Field(None, gt=1.0, le=100.0)

    class Suspension(BaseModel):
        caster_angle: float | None = Field(None, ge=-0.5, le=0.5)
        kingpin_inclination: float | None = Field(None, ge=-0.5, le=0.5)
        camber: float | None = Field(None, ge=-0.5, le=0.5)
        scrub_radius: float | None = Field(None, ge=0.0, le=0.2)
        spring_rate: float | None = Field(None, gt=0.0, le=1e6)
        damper_rate: float | None = Field(None, ge=0.0, le=1e5)
        anti_roll_rate: float | None = Field(None, ge=0.0, le=1e6)
        kingpin_mu: float | None = Field(None, ge=0.0, le=2.0)

    suspension: Suspension | None = None

    class SteeringGeometry(BaseModel):
        front_outer_x: float | None = Field(None, ge=-1.0, le=1.0)
        front_outer_y: float | None = Field(None, ge=-1.0, le=1.0)
        front_inner_x: float | None = Field(None, ge=-1.0, le=1.0)
        front_inner_y: float | None = Field(None, ge=-1.5, le=0.0)
        front_rack_axis_deg: float | None = Field(None, ge=-180.0, le=180.0)
        front_rack_travel_limit: float | None = Field(None, gt=0.0, le=0.5)
        rear_outer_x: float | None = Field(None, ge=-1.0, le=1.0)
        rear_outer_y: float | None = Field(None, ge=-1.0, le=1.0)
        rear_inner_x: float | None = Field(None, ge=-1.0, le=1.0)
        rear_inner_y: float | None = Field(None, ge=-1.5, le=0.0)
        rear_rack_axis_deg: float | None = Field(None, ge=-180.0, le=180.0)
        rear_rack_travel_limit: float | None = Field(None, gt=0.0, le=0.5)

    steering_geometry: SteeringGeometry | None = None

    def merged(self, current: VehicleParams) -> VehicleParams:
        """Overlay the non-None fields onto `current` (frozen dataclass)."""
        data = self.model_dump(exclude_none=True)
        susp_data = data.pop("suspension", None)
        steering_data = data.pop("steering_geometry", None)
        susp = current.suspension
        if susp_data:
            susp = dataclasses.replace(susp, **susp_data)
        steering_geometry = current.steering_geometry
        if steering_data:
            steering_geometry = dataclasses.replace(steering_geometry, **steering_data)
        return dataclasses.replace(
            current,
            suspension=susp,
            steering_geometry=steering_geometry,
            **data,
        )


@router.post("/params")
async def set_params(body: ParamsUpdate) -> dict[str, Any]:
    sim = get_simulator()
    new_params = body.merged(sim.params)
    if new_params.cg_to_front >= new_params.wheelbase:
        raise HTTPException(
            status_code=422,
            detail=f"cg_to_front ({new_params.cg_to_front}) must be < wheelbase ({new_params.wheelbase})",
        )
    sim.set_params(new_params)
    return await get_params()
