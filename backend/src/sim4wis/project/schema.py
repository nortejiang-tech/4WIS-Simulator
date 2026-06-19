"""Project file schema (pydantic v2) — YAML on disk, dict in memory.

The on-disk format matches `projects/default.yaml`. Loading round-trips:
    YAML → ProjectFile.model_validate(...) → ProjectFile.to_runtime() → applied
    to the running Simulator.

Phase 1 only persists vehicle / suspension / controller / recording fields.
Scene / disturbances are kept in the schema as forward-compatibility but the
runtime ignores them until Phase 2.
"""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, ConfigDict, Field

from sim4wis.core.state import SteeringGeometryParams, SuspensionParams, VehicleParams


class ProjectMeta(BaseModel):
    model_config = ConfigDict(extra="ignore")
    name: str = "default"
    description: str = ""


class VehicleSection(BaseModel):
    # Defaults calibrated to 智己 LS9 — see core.state.VehicleParams for sources.
    model_config = ConfigDict(extra="ignore")
    # Primary product models: "kinematic" | "simplified_dynamic".
    # "multibody" is still loadable for research/regression projects.
    model: str = "kinematic"
    wheelbase: float = 3.160
    track_front: float = 1.565172
    track_rear: float = 1.565172
    mass: float = 2900.0
    inertia_z: float = 7500.0
    cg_to_front: float = 1.550
    cg_height: float = 0.620
    tire_radius: float = 0.395
    tire_width: float = 0.265
    contact_patch_radius: float = 0.110
    steer_limit: float = 0.6109
    v_max: float = 55.6
    wheel_inertia: float = 2.5
    motor_torque_max: float = 3000.0
    bump_steer_coeff: float = 0.0
    unsprung_mass: float = 55.0
    tire_vertical_stiffness: float = 280_000.0
    steer_tau: float = 0.06
    steer_rate_max: float = 8.0
    # Tire model selection + parameters (shared linear/pacejka stiffness).
    tire_model: str = "linear"
    tire_c_alpha: float = 120_000.0
    tire_c_kappa: float = 100_000.0
    tire_t_pneumatic: float = 0.03
    tire_load_sensitivity_exp: float = 0.8
    parking_scrub_coeff: float = 0.80
    parking_lateral_coeff: float = 0.80
    parking_torque_coeff: float = 0.80
    low_speed_blend_ms: float = 1.50
    static_tire_deflection_deg: float = 8.0
    rolling_resistance_coeff: float = 0.012
    drag_coeff_cd: float = 0.30
    frontal_area: float = 2.80
    air_density: float = 1.225
    aero_lift_coeff_front: float = 0.30
    aero_lift_coeff_rear: float = 0.15
    camber_thrust_coeff: float = 1.0
    static_toe_front: float = 0.001745
    static_toe_rear: float = 0.0
    tire_cx: float = 1.65
    tire_cy: float = 1.30
    tire_ex: float = -0.5
    tire_ey: float = -1.0
    # Wheel-speed servo PI gains.
    servo_kp: float = 200.0
    servo_ki: float = 50.0
    # Split-rack transmission geometry.
    steering_arm_length: float = 0.146451
    pinion_radius: float = 0.020
    tie_rod_angle_deg: float = 6.67
    rack_mech_efficiency: float = 0.92
    motor_gear_ratio: float = 10.0


class SuspensionSection(BaseModel):
    model_config = ConfigDict(extra="ignore")
    caster_angle: float = 0.1047
    kingpin_inclination: float = 0.2094
    camber: float = -0.0131
    scrub_radius: float = 0.015
    spring_rate: float = 70_000.0
    damper_rate: float = 4_000.0
    anti_roll_rate: float = 30_000.0
    kingpin_mu: float = 0.6


class SteeringGeometrySection(BaseModel):
    model_config = ConfigDict(extra="ignore")
    front_outer_x: float = -0.145971235
    front_outer_y: float = -0.011844575
    front_inner_x: float = -0.176324
    front_inner_y: float = -0.362586
    front_rack_axis_deg: float = 90.0
    front_rack_travel_limit: float = 0.085
    rear_outer_x: float = 0.145971235
    rear_outer_y: float = -0.011844575
    rear_inner_x: float = 0.176324
    rear_inner_y: float = -0.362586
    rear_rack_axis_deg: float = 90.0
    rear_rack_travel_limit: float = 0.085


class ControllerSection(BaseModel):
    model_config = ConfigDict(extra="ignore")
    type: str = "ideal_ackermann"
    params: dict[str, Any] = Field(default_factory=dict)


class SceneSection(BaseModel):
    model_config = ConfigDict(extra="ignore")
    surface: str = "flat"
    base_mu: float = 0.85   # dry concrete default
    disturbances: list[dict[str, Any]] = Field(default_factory=list)

    def to_scene(self) -> "Scene":  # noqa: F821 — forward ref string for clarity
        """Build a runtime Scene from this section."""
        from sim4wis.environment.disturbance import Scene
        return Scene.from_dict({
            "surface": self.surface,
            "base_mu": self.base_mu,
            "disturbances": self.disturbances,
        })


class RecordingSection(BaseModel):
    model_config = ConfigDict(extra="ignore")
    enabled: bool = True
    channels: list[str] = Field(default_factory=lambda: [
        "vx", "vy", "yaw_rate",
        "wheel_angles", "wheel_torques", "icr_position",
    ])


class ProjectFile(BaseModel):
    """Top-level project file."""
    model_config = ConfigDict(extra="ignore")
    project: ProjectMeta = Field(default_factory=ProjectMeta)
    vehicle: VehicleSection = Field(default_factory=VehicleSection)
    suspension: SuspensionSection = Field(default_factory=SuspensionSection)
    steering_geometry: SteeringGeometrySection = Field(default_factory=SteeringGeometrySection)
    controller: ControllerSection = Field(default_factory=ControllerSection)
    scene: SceneSection = Field(default_factory=SceneSection)
    recording: RecordingSection = Field(default_factory=RecordingSection)

    # ---- conversion helpers ----

    def vehicle_params(self) -> VehicleParams:
        v = self.vehicle
        s = self.suspension
        sg = self.steering_geometry
        return VehicleParams(
            wheelbase=v.wheelbase,
            track_front=v.track_front,
            track_rear=v.track_rear,
            mass=v.mass,
            inertia_z=v.inertia_z,
            cg_to_front=v.cg_to_front,
            tire_radius=v.tire_radius,
            tire_width=v.tire_width,
            contact_patch_radius=v.contact_patch_radius,
            steer_limit=v.steer_limit,
            v_max=v.v_max,
            cg_height=v.cg_height,
            wheel_inertia=v.wheel_inertia,
            motor_torque_max=v.motor_torque_max,
            bump_steer_coeff=v.bump_steer_coeff,
            unsprung_mass=v.unsprung_mass,
            tire_vertical_stiffness=v.tire_vertical_stiffness,
            steer_tau=v.steer_tau,
            steer_rate_max=v.steer_rate_max,
            tire_model=v.tire_model,
            tire_c_alpha=v.tire_c_alpha,
            tire_c_kappa=v.tire_c_kappa,
            tire_t_pneumatic=v.tire_t_pneumatic,
            tire_load_sensitivity_exp=v.tire_load_sensitivity_exp,
            parking_scrub_coeff=v.parking_scrub_coeff,
            parking_lateral_coeff=v.parking_lateral_coeff,
            parking_torque_coeff=v.parking_torque_coeff,
            low_speed_blend_ms=v.low_speed_blend_ms,
            static_tire_deflection_deg=v.static_tire_deflection_deg,
            rolling_resistance_coeff=v.rolling_resistance_coeff,
            drag_coeff_cd=v.drag_coeff_cd,
            frontal_area=v.frontal_area,
            air_density=v.air_density,
            aero_lift_coeff_front=v.aero_lift_coeff_front,
            aero_lift_coeff_rear=v.aero_lift_coeff_rear,
            camber_thrust_coeff=v.camber_thrust_coeff,
            static_toe_front=v.static_toe_front,
            static_toe_rear=v.static_toe_rear,
            tire_cx=v.tire_cx,
            tire_cy=v.tire_cy,
            tire_ex=v.tire_ex,
            tire_ey=v.tire_ey,
            servo_kp=v.servo_kp,
            servo_ki=v.servo_ki,
            steering_arm_length=v.steering_arm_length,
            pinion_radius=v.pinion_radius,
            tie_rod_angle_deg=v.tie_rod_angle_deg,
            rack_mech_efficiency=v.rack_mech_efficiency,
            motor_gear_ratio=v.motor_gear_ratio,
            suspension=SuspensionParams(
                caster_angle=s.caster_angle,
                kingpin_inclination=s.kingpin_inclination,
                camber=s.camber,
                scrub_radius=s.scrub_radius,
                spring_rate=s.spring_rate,
                damper_rate=s.damper_rate,
                anti_roll_rate=s.anti_roll_rate,
                kingpin_mu=s.kingpin_mu,
            ),
            steering_geometry=SteeringGeometryParams(
                front_outer_x=sg.front_outer_x,
                front_outer_y=sg.front_outer_y,
                front_inner_x=sg.front_inner_x,
                front_inner_y=sg.front_inner_y,
                front_rack_axis_deg=sg.front_rack_axis_deg,
                front_rack_travel_limit=sg.front_rack_travel_limit,
                rear_outer_x=sg.rear_outer_x,
                rear_outer_y=sg.rear_outer_y,
                rear_inner_x=sg.rear_inner_x,
                rear_inner_y=sg.rear_inner_y,
                rear_rack_axis_deg=sg.rear_rack_axis_deg,
                rear_rack_travel_limit=sg.rear_rack_travel_limit,
            ),
        )

    @classmethod
    def from_runtime(
        cls,
        name: str,
        description: str,
        params: VehicleParams,
        strategy_name: str,
        strategy_params: dict[str, Any] | None = None,
        model_type: str = "kinematic",
        scene: SceneSection | None = None,
    ) -> ProjectFile:
        return cls(
            project=ProjectMeta(name=name, description=description),
            vehicle=VehicleSection(
                model=model_type,
                wheelbase=params.wheelbase,
                track_front=params.track_front,
                track_rear=params.track_rear,
                mass=params.mass,
                inertia_z=params.inertia_z,
                cg_to_front=params.cg_to_front,
                cg_height=params.cg_height,
                tire_radius=params.tire_radius,
                tire_width=params.tire_width,
                contact_patch_radius=params.contact_patch_radius,
                steer_limit=params.steer_limit,
                v_max=params.v_max,
                wheel_inertia=params.wheel_inertia,
                motor_torque_max=params.motor_torque_max,
                bump_steer_coeff=params.bump_steer_coeff,
                unsprung_mass=params.unsprung_mass,
                tire_vertical_stiffness=params.tire_vertical_stiffness,
                steer_tau=params.steer_tau,
                steer_rate_max=params.steer_rate_max,
                tire_model=params.tire_model,
                tire_c_alpha=params.tire_c_alpha,
                tire_c_kappa=params.tire_c_kappa,
                tire_t_pneumatic=params.tire_t_pneumatic,
                tire_load_sensitivity_exp=params.tire_load_sensitivity_exp,
                parking_scrub_coeff=params.parking_scrub_coeff,
                parking_lateral_coeff=params.parking_lateral_coeff,
                parking_torque_coeff=params.parking_torque_coeff,
                low_speed_blend_ms=params.low_speed_blend_ms,
                static_tire_deflection_deg=params.static_tire_deflection_deg,
                rolling_resistance_coeff=params.rolling_resistance_coeff,
                drag_coeff_cd=params.drag_coeff_cd,
                frontal_area=params.frontal_area,
                air_density=params.air_density,
                aero_lift_coeff_front=params.aero_lift_coeff_front,
                aero_lift_coeff_rear=params.aero_lift_coeff_rear,
                camber_thrust_coeff=params.camber_thrust_coeff,
                static_toe_front=params.static_toe_front,
                static_toe_rear=params.static_toe_rear,
                tire_cx=params.tire_cx,
                tire_cy=params.tire_cy,
                tire_ex=params.tire_ex,
                tire_ey=params.tire_ey,
                servo_kp=params.servo_kp,
                servo_ki=params.servo_ki,
                steering_arm_length=params.steering_arm_length,
                pinion_radius=params.pinion_radius,
                tie_rod_angle_deg=params.tie_rod_angle_deg,
                rack_mech_efficiency=params.rack_mech_efficiency,
                motor_gear_ratio=params.motor_gear_ratio,
            ),
            suspension=SuspensionSection(
                caster_angle=params.suspension.caster_angle,
                kingpin_inclination=params.suspension.kingpin_inclination,
                camber=params.suspension.camber,
                scrub_radius=params.suspension.scrub_radius,
                spring_rate=params.suspension.spring_rate,
                damper_rate=params.suspension.damper_rate,
                anti_roll_rate=params.suspension.anti_roll_rate,
                kingpin_mu=params.suspension.kingpin_mu,
            ),
            steering_geometry=SteeringGeometrySection(
                front_outer_x=params.steering_geometry.front_outer_x,
                front_outer_y=params.steering_geometry.front_outer_y,
                front_inner_x=params.steering_geometry.front_inner_x,
                front_inner_y=params.steering_geometry.front_inner_y,
                front_rack_axis_deg=params.steering_geometry.front_rack_axis_deg,
                front_rack_travel_limit=params.steering_geometry.front_rack_travel_limit,
                rear_outer_x=params.steering_geometry.rear_outer_x,
                rear_outer_y=params.steering_geometry.rear_outer_y,
                rear_inner_x=params.steering_geometry.rear_inner_x,
                rear_inner_y=params.steering_geometry.rear_inner_y,
                rear_rack_axis_deg=params.steering_geometry.rear_rack_axis_deg,
                rear_rack_travel_limit=params.steering_geometry.rear_rack_travel_limit,
            ),
            controller=ControllerSection(
                type=strategy_name,
                params=strategy_params or {},
            ),
            scene=scene if scene is not None else SceneSection(),
        )
