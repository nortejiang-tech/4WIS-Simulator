"""Core state types used by the simulation loop, models, controllers, and bus.

Frame conventions (see docs/design.md §2):
    * World frame: X-forward (east), Y-left (north), Z-up (right-handed).
    * Body frame:  origin at vehicle geometric center (midpoint of axles),
                   X forward, Y left, Z up.
    * Yaw ψ: rotation of body X about world Z, CCW positive.

Wheel numbering (see WheelIndex):
    0 = FL (front-left)
    1 = FR (front-right)
    2 = RL (rear-left)
    3 = RR (rear-right)

Units throughout: SI (m, kg, rad, s, N, N·m).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any

import numpy as np


N_WHEELS = 4


class WheelIndex(IntEnum):
    FL = 0
    FR = 1
    RL = 2
    RR = 3


# ---------------------------------------------------------------------------
# Vehicle parameters (geometry only in Phase 1; suspension passthrough for
# future use)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SuspensionParams:
    """Per-wheel suspension geometry parameters.

    Defaults calibrated to 智己 LS9 (IM LS9) — a flagship full-size SUV with
    4-wheel steering. These specific angles are NOT published by the OEM, so
    they are reasonable engineering estimates for a large air-sprung SUV.
    """

    caster_angle: float = 0.1047        # 主销后倾角 ≈6° [rad]（估算）
    kingpin_inclination: float = 0.2094  # 主销内倾角 ≈12° [rad]（估算）
    camber: float = -0.0131             # 外倾角 ≈-0.75° [rad]（估算）
    scrub_radius: float = 0.015         # 主销偏置量 [m]（估算）
    # Multibody (step 19) suspension parameters — per corner unless noted.
    spring_rate: float = 70_000.0       # 悬架弹簧刚度 [N/m]（估算，~1.6Hz ride）
    damper_rate: float = 4_000.0        # 减振器阻尼 [N·s/m]（估算，ζ≈0.3）
    anti_roll_rate: float = 30_000.0    # 防倾杆等效横摆刚度 [N·m/rad]（估算，整车）
    # Kinematic model's placeholder kingpin friction coefficient (the dynamic
    # models compute the real Reimpell/Pacejka moment instead).
    kingpin_mu: float = 0.6


@dataclass(frozen=True)
class SteeringGeometryParams:
    """Symmetric 2D rack/tie-rod hardpoints for front and rear axles.

    Coordinates are local to the wheel kingpin/wheel-centre at straight ahead:
    X forward, Y left, metres. The fields describe the *left* wheel. Right-side
    hardpoints are mirrored across local Y=0. The outer ball joint rotates with
    wheel steer angle; the inner joint slides along the rack axis while keeping
    the zero-position tie-rod length.
    """

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


@dataclass(frozen=True)
class VehicleParams:
    """Static vehicle parameters.

    Defaults calibrated to **智己 LS9 (IM LS9)** — a flagship full-size 6-seat
    SUV with 4-wheel steering, which maps naturally onto this 4WIS simulator.
    Published specs (轴距/质量/轮胎/尺寸) use real values; quantities the OEM
    does not publish (yaw inertia, wheel inertia, per-wheel motor torque, CG
    height, suspension angles) are reasonable engineering estimates and are
    marked 估算 below. Replaced from the project YAML at load time.
    """

    wheelbase: float = 3.160            # L 轴距 [m]（LS9 ≈3160 mm）
    track_front: float = 1.565172       # 前主销距/等效轮距 [m]（LS9 转向几何表）
    track_rear: float = 1.565172        # 后主销距/等效轮距 [m]（按前轴几何对称估算）
    mass: float = 2900.0                # 整备质量 [kg]（LS9 ≈2.9 t）
    inertia_z: float = 7500.0           # 横摆转动惯量 [kg·m²]（估算 ≈ m·a·b）
    cg_to_front: float = 1.550          # 质心到前轴距离 [m]（估算，略偏前 ~49/51）
    tire_radius: float = 0.395          # 车轮滚动半径 [m]（285/45 R21 ≈0.395）
    tire_width: float = 0.265           # 轮胎名义宽度 [m]（LS9 21/22"常见 265 宽）
    contact_patch_radius: float = 0.110 # 等效接地印迹扭转半径 [m]（工程估算）
    steer_limit: float = 0.6109         # ±35° 单轮最大转角 [rad]
    v_max: float = 55.6                 # 最大车速 [m/s]（≈200 km/h 顶速）
    # Dynamic-model parameters (used only by SimplifiedDynamicModel)
    cg_height: float = 0.620            # 质心高度 [m]（估算，大型 SUV）
    wheel_inertia: float = 2.5          # 单轮转动惯量 [kg·m²]（估算，21" 轮）
    motor_torque_max: float = 3000.0    # 单轮电机扭矩上限 [N·m]（估算，高性能 EV）
    # Multibody (step 19) — unsprung mass + vertical tyre stiffness per corner.
    unsprung_mass: float = 55.0         # 单角簧下质量 [kg]（估算，轮+制动+轮毂）
    tire_vertical_stiffness: float = 280_000.0  # 轮胎垂向刚度 [N/m]（估算）
    # Steering actuator (P2) — first-order lag + rate limit on δ toward δ_cmd.
    # Used by the dynamic / multibody models (kinematic stays instantaneous).
    steer_tau: float = 0.06             # 转向作动器一阶时间常数 [s]
    steer_rate_max: float = 8.0         # 转向最大角速率 [rad/s]（≈458°/s）
    # Bump-steer approximation: per-wheel toe perturbation per metre of vertical
    # wheel travel [rad/m]. 0 = off. The planar model has no suspension DOF, so
    # this is an engineering stand-in that makes a wheel toe as it rides over a
    # SpeedBump (left wheels toe +, right wheels toe −), producing a visible
    # steering twitch instead of nothing. Real bump-steer needs the multibody
    # model (Phase 3 step 19).
    bump_steer_coeff: float = 0.0
    # Tire model selection + shared parameters (used by dynamic / multibody).
    # Stiffnesses are shared between linear and pacejka so the small-slip
    # behaviour (and wheel-servo tuning) is identical across models.
    tire_model: str = "linear"          # "linear" | "pacejka"
    tire_c_alpha: float = 120_000.0     # 侧偏刚度 [N/rad]（额定 Fz 处）
    tire_c_kappa: float = 100_000.0     # 纵滑刚度 [N]
    tire_t_pneumatic: float = 0.03      # 气胎拖距 [m]
    # Load sensitivity: c_alpha(Fz) = c_alpha0·(Fz/Fz_nom)^p
    # Typical p ≈ 0.7–0.9 for radial tyres; 0 = no load sensitivity.
    # Important for the load-analysis page: high speed → aero lift drops Fz →
    # the linear-region slope of the τ-δ curve softens, not just the saturation
    # plateau height.
    tire_load_sensitivity_exp: float = 0.8
    parking_scrub_coeff: float = 0.80   # 低速/原地轮胎扭转阻力系数 [-]（旧字段，向后兼容）
    parking_lateral_coeff: float = 0.80  # 静态接地斑侧向力系数（A 块拆分自 parking_scrub_coeff）
    parking_torque_coeff: float = 0.80   # 静态接地斑阻力矩系数（A 块拆分自 parking_scrub_coeff）
    low_speed_blend_ms: float = 1.50    # 低速补偿衰减速度 [m/s]
    static_tire_deflection_deg: float = 8.0  # 停车扭转 tanh 饱和角 [deg]（原写死，现可调）
    # Driving-force / aero balance — enable speed-dependent δ_eq drift (A1).
    rolling_resistance_coeff: float = 0.012  # Crr 滚动阻力系数 [-]
    drag_coeff_cd: float = 0.30          # Cd 风阻系数 [-]
    frontal_area: float = 2.80           # A 迎风面积 [m²]（LS9 ~2.8）
    air_density: float = 1.225           # ρ 空气密度 [kg/m³]
    # Aero lift per axle (v0.7.4) — gives Fz a v² speed dependence so the
    # tire-saturation plateau lowers with speed and τ-δ curves separate
    # continuously across the speed slider. Positive = lift (reduces Fz).
    # Defaults are deliberately on the high side of the SUV range so the
    # plateau-vs-speed effect is *visible* in the τ-δ curve. Set both to 0
    # to recover the v0.7.3 behaviour.
    aero_lift_coeff_front: float = 0.30  # Cl_F 前轴升力系数（0~0.4 工程量级）
    aero_lift_coeff_rear: float = 0.15   # Cl_R 后轴升力系数
    # Camber thrust (A2) — Fy_camber = Cγ · γ_per_wheel · Fz at α=0.
    camber_thrust_coeff: float = 1.0     # Cγ [1/rad]，典型 0.8~1.5
    # Static toe (A3) — per-axle 偏置，per-wheel 镜像。+ = toe-in.
    static_toe_front: float = 0.001745   # 前轴静态 toe-in ≈ 0.1° [rad]
    static_toe_rear: float = 0.0         # 后轴静态 toe-in [rad]，4WIS 常用 0
    tire_cx: float = 1.65               # Pacejka 纵向形状因子
    tire_cy: float = 1.30               # Pacejka 侧向形状因子
    tire_ex: float = -0.5               # Pacejka 纵向曲率因子
    tire_ey: float = -1.0               # Pacejka 侧向曲率因子
    # Wheel-speed servo PI gains (previously hard-coded in wheel_servo.py).
    servo_kp: float = 200.0             # [N·m / (rad/s)]
    servo_ki: float = 50.0              # [N·m / rad]
    suspension: SuspensionParams = field(default_factory=SuspensionParams)
    # Split-rack transmission geometry (分体齿条传动参数).
    # Used to convert kingpin torques → rack forces → motor torque demands.
    steering_arm_length: float = 0.146451  # 转向臂长 L_arm [m]（LS9 梯形臂长）
    pinion_radius: float = 0.020         # 齿轮节圆半径 r_p [m]（典型 ~20 mm）
    tie_rod_angle_deg: float = 6.67      # 直行混合拉杆角 β [deg]（LS9 表）
    rack_mech_efficiency: float = 0.92   # 齿条正效率 η（典型 0.85~0.95）
    motor_gear_ratio: float = 10.0       # 电机到齿条减速比 i
    steering_geometry: SteeringGeometryParams = field(default_factory=SteeringGeometryParams)

    def wheel_positions_body(self) -> np.ndarray:
        """Return a (4, 2) array of wheel center positions in body frame.

        Body origin = midpoint of front and rear axles. So front axles are at
        x = +L/2, rear axles at x = -L/2. Left wheels at y = +track/2.
        """
        L = self.wheelbase
        tf = self.track_front / 2.0
        tr = self.track_rear / 2.0
        return np.array(
            [
                [+L / 2.0, +tf],   # FL
                [+L / 2.0, -tf],   # FR
                [-L / 2.0, +tr],   # RL
                [-L / 2.0, -tr],   # RR
            ],
            dtype=np.float64,
        )


# ---------------------------------------------------------------------------
# Driver input → Controller → ControlCommand → VehicleModel → VehicleState
# ---------------------------------------------------------------------------


@dataclass
class DriverInput:
    """Normalised driver input — produced by keyboard/script/joystick layers.

    Convention:
        throttle  ∈ [-1, +1]   negative = brake/reverse, positive = forward
        steering  ∈ [-1, +1]   +1 = full left, -1 = full right
        handbrake ∈ {0, 1}
        mode_params: strategy-specific knobs (e.g., crab angle, rear ratio)
    """

    throttle: float = 0.0
    steering: float = 0.0
    handbrake: int = 0
    mode_params: dict[str, Any] = field(default_factory=dict)


@dataclass
class ControlCommand:
    """Output of a ControllerStrategy → input to the VehicleModel.

    Fields:
        delta_cmd:        (4,) commanded steer angles per wheel [rad]
        wheel_speed_cmd:  (4,) commanded wheel angular velocity [rad/s]
                          (Phase 1 kinematic model uses these directly as
                          ground-truth — Phase 2 dynamic model will treat
                          them as a reference for the torque controller.)
        icr_target_body:  (2,) commanded ICR in body frame, or NaN if the
                          strategy does not work in ICR space (e.g. crab).
                          Stored only for visualisation / diagnostics.
    """

    delta_cmd: np.ndarray
    wheel_speed_cmd: np.ndarray
    icr_target_body: np.ndarray  # shape (2,) — may contain NaN/Inf

    @staticmethod
    def zero() -> ControlCommand:
        return ControlCommand(
            delta_cmd=np.zeros(N_WHEELS),
            wheel_speed_cmd=np.zeros(N_WHEELS),
            icr_target_body=np.array([np.nan, np.nan]),
        )


# ---------------------------------------------------------------------------
# Vehicle / environment state
# ---------------------------------------------------------------------------


@dataclass
class VehicleState:
    """Full vehicle state at one instant.

    Pose (world frame):
        x, y    [m]    body origin in world
        psi     [rad]  yaw angle of body X axis vs. world X axis (CCW +)

    Body-frame velocities at body origin:
        vx, vy  [m/s]
        yaw_rate [rad/s]   = dψ/dt

    Per-wheel state (arrays of shape (4,)):
        delta        [rad]    actual steer angle  (might lag delta_cmd in Phase 2)
        wheel_omega  [rad/s]  wheel angular velocity (spin)
        fz           [N]      vertical load (Phase 2; Phase 1 just static = mg/4)
        torque_steer [N·m]    resistance moment around kingpin

    Derived geometry (computed by the model after each step):
        vehicle_icr_body (2,)  whole-vehicle ICR in body frame, derived from
                                (vx, vy, yaw_rate). Holds NaN when the motion
                                is straight-line (|yaw_rate| ≈ 0).
        wheel_pos_body  (4, 2) wheel positions in body frame (constant, kept
                                here for convenience in the streamer).

        Per-wheel "ICR" is a *line* (perpendicular to the wheel's rolling
        direction through the wheel centre), not a point — the frontend draws
        these lines from (wheel_pos_body[i], delta[i]). The unit test verifies
        that for the ideal-Ackermann strategy all four lines meet at one point.

    t: simulation time [s].
    """

    t: float = 0.0

    # Pose
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0

    # Vertical / attitude DOF (used by MultiBodyModel — step 19; 0 for the
    # planar kinematic / simplified-dynamic models).
    z: float = 0.0          # body CG heave above static ride height [m]
    roll: float = 0.0       # roll angle about body X [rad] (+ = right side down)
    pitch: float = 0.0      # pitch angle about body Y [rad] (+ = nose up)

    # Velocities (body frame, at body origin)
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0

    # Per-wheel (length-4 arrays)
    delta: np.ndarray = field(default_factory=lambda: np.zeros(N_WHEELS))
    wheel_omega: np.ndarray = field(default_factory=lambda: np.zeros(N_WHEELS))
    fz: np.ndarray = field(default_factory=lambda: np.zeros(N_WHEELS))
    torque_steer: np.ndarray = field(default_factory=lambda: np.zeros(N_WHEELS))
    susp_defl: np.ndarray = field(default_factory=lambda: np.zeros(N_WHEELS))  # suspension compression vs static [m]

    # Derived geometry
    vehicle_icr_body: np.ndarray = field(default_factory=lambda: np.full(2, np.nan))
    wheel_pos_body: np.ndarray = field(default_factory=lambda: np.zeros((N_WHEELS, 2)))
    # Per-wheel steering centre: vehicle ICR projected onto each wheel's
    # perpendicular line, plus the signed distance (see geometry.wheel_icr_projection).
    wheel_icr_body: np.ndarray = field(default_factory=lambda: np.full((N_WHEELS, 2), np.nan))
    wheel_icr_dev: np.ndarray = field(default_factory=lambda: np.full(N_WHEELS, np.nan))
    # Split-rack force chain (分体齿条力链): derived from torque_steer each step.
    rack_force: np.ndarray = field(default_factory=lambda: np.zeros(N_WHEELS))
    motor_torque_demand: np.ndarray = field(default_factory=lambda: np.zeros(N_WHEELS))
    linkage_arm_tie_angle: np.ndarray = field(default_factory=lambda: np.full(N_WHEELS, np.nan))
    linkage_tie_rack_angle: np.ndarray = field(default_factory=lambda: np.full(N_WHEELS, np.nan))
    linkage_efficiency: np.ndarray = field(default_factory=lambda: np.zeros(N_WHEELS))

    def copy(self) -> VehicleState:
        return VehicleState(
            t=self.t,
            x=self.x, y=self.y, psi=self.psi,
            z=self.z, roll=self.roll, pitch=self.pitch,
            vx=self.vx, vy=self.vy, yaw_rate=self.yaw_rate,
            delta=self.delta.copy(),
            wheel_omega=self.wheel_omega.copy(),
            fz=self.fz.copy(),
            torque_steer=self.torque_steer.copy(),
            susp_defl=self.susp_defl.copy(),
            vehicle_icr_body=self.vehicle_icr_body.copy(),
            wheel_pos_body=self.wheel_pos_body.copy(),
            wheel_icr_body=self.wheel_icr_body.copy(),
            wheel_icr_dev=self.wheel_icr_dev.copy(),
            rack_force=self.rack_force.copy(),
            motor_torque_demand=self.motor_torque_demand.copy(),
            linkage_arm_tie_angle=self.linkage_arm_tie_angle.copy(),
            linkage_tie_rack_angle=self.linkage_tie_rack_angle.copy(),
            linkage_efficiency=self.linkage_efficiency.copy(),
        )


@dataclass
class EnvironmentState:
    """Road / environment context delivered to the model each step.

    Phase 2a adds an optional `scene` field carrying a list of disturbance
    regions. KinematicModel still ignores them (its purpose is geometric).
    SimplifiedDynamicModel (step 10) will query `scene.wheel_env(...)` per
    wheel to get local mu / fz_offset / ground_z.
    """

    mu: float = 1.0                            # base mu (used if scene is None)
    surface_z: float = 0.0                     # placeholder
    # Late-bound to avoid an import cycle. `Scene | None` semantically.
    scene: object | None = None
