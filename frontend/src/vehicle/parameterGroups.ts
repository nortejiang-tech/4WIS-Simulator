export type FieldSpec = [string, string, number, number?];

export interface ParameterGroup {
  title: string;
  tier: "core" | "advanced";
  defaultOpen?: boolean;
  fields: FieldSpec[];
}

export const MM = 1000;
export const DEG = 180 / Math.PI;

export const PARAMETER_GROUPS: ParameterGroup[] = [
  {
    title: "核心整车",
    tier: "core",
    defaultOpen: true,
    fields: [
      ["wheelbase", "轴距 L (mm)", 10, MM],
      ["track_front", "前主销距/轮距 (mm)", 1, MM],
      ["track_rear", "后主销距/轮距 (mm)", 1, MM],
      ["mass", "整备质量 (kg)", 10],
      ["inertia_z", "横摆惯量 (kg·m²)", 100],
      ["cg_to_front", "质心到前轴 (mm)", 10, MM],
      ["cg_height", "质心高度 (mm)", 10, MM],
      ["v_max", "最大车速 (km/h)", 1, 3.6],
    ],
  },
  {
    title: "核心轮胎 / 转向",
    tier: "core",
    defaultOpen: true,
    fields: [
      ["tire_radius", "轮胎半径 (mm)", 1, MM],
      ["tire_width", "轮胎宽度 (mm)", 1, MM],
      ["tire_c_alpha", "侧偏刚度 Cα (N/rad)", 5000],
      ["tire_c_kappa", "纵滑刚度 Cκ (N)", 5000],
      ["tire_t_pneumatic", "气胎拖距 (mm)", 1, MM],
      ["steer_limit", "最大转角 (°)", 0.5, DEG],
      ["steer_tau", "作动时间常数 (s)", 0.01],
      ["steer_rate_max", "最大角速率 (rad/s)", 0.5],
      ["steer_wheel_range", "方向盘总转角 (°)", 30],
      ["steer_ratio_low", "低速传动比 (0=按盘径自动)", 0.5],
      ["steer_ratio_high", "高速传动比 (0=低速比×3.5)", 0.5],
    ],
  },
  {
    title: "核心主销 / 齿条",
    tier: "core",
    defaultOpen: true,
    fields: [
      ["suspension.caster_angle", "主销后倾 (°)", 0.2, DEG],
      ["suspension.kingpin_inclination", "主销内倾 (°)", 0.2, DEG],
      ["suspension.camber", "外倾角 (°)", 0.1, DEG],
      ["suspension.scrub_radius", "主销偏置 (mm)", 0.5, MM],
      ["steering_arm_length", "梯形臂长 r (mm)", 0.5, MM],
      ["pinion_radius", "齿轮节圆半径 (mm)", 0.5, MM],
      ["rack_mech_efficiency", "齿条效率 η", 0.01],
      ["motor_gear_ratio", "电机减速比", 0.5],
    ],
  },
  {
    title: "对齐 / 偏置标定",
    tier: "advanced",
    fields: [
      ["camber_thrust_coeff", "Camber thrust Cγ (1/rad)", 0.1],
      ["static_toe_front", "前轴 toe-in (°)", 0.02, DEG],
      ["static_toe_rear", "后轴 toe-in (°)", 0.02, DEG],
    ],
  },
  {
    title: "驱动 / 气动",
    tier: "advanced",
    fields: [
      ["rolling_resistance_coeff", "滚阻系数 Crr", 0.001],
      ["drag_coeff_cd", "风阻 Cd", 0.01],
      ["frontal_area", "迎风面积 A (m²)", 0.05],
      ["air_density", "空气密度 ρ (kg/m³)", 0.01],
      ["aero_lift_coeff_front", "前轴升力 Cl_F", 0.01],
      ["aero_lift_coeff_rear", "后轴升力 Cl_R", 0.01],
      ["v_max_reverse", "倒车限速 (m/s)", 0.5],
    ],
  },
  {
    title: "制动系统",
    tier: "advanced",
    defaultOpen: false,
    fields: [
      ["brake_torque_max", "最大制动力矩 (N·m)", 500],
      ["brake_bias_front", "前轴制动分配", 0.05],
      ["brake_tau", "制动响应时间常数 (s)", 0.01],
    ],
  },
  {
    title: "低速 / 停车补偿",
    tier: "advanced",
    fields: [
      ["contact_patch_radius", "接地扭转半径 (mm)", 1, MM],
      ["parking_lateral_coeff", "静态接地侧向系数", 0.05],
      ["parking_torque_coeff", "静态接地扭矩系数", 0.05],
      ["parking_scrub_coeff", "旧停车补偿系数", 0.05],
      ["static_tire_deflection_deg", "停车扭转饱和角 (°)", 0.5],
      ["low_speed_blend_ms", "低速衰减 (m/s)", 0.1],
    ],
  },
  {
    title: "Pacejka / 垂向轮胎",
    tier: "advanced",
    fields: [
      ["tire_load_sensitivity_exp", "载荷敏感指数 p", 0.05],
      ["tire_vertical_stiffness", "轮胎垂向刚度 (N/m)", 10000],
      ["tire_cx", "Pacejka Cx", 0.05],
      ["tire_cy", "Pacejka Cy", 0.05],
      ["tire_ex", "Pacejka Ex", 0.1],
      ["tire_ey", "Pacejka Ey", 0.1],
    ],
  },
  {
    title: "作动器 / 多体研究参数",
    tier: "advanced",
    fields: [
      ["wheel_inertia", "单轮惯量 (kg·m²)", 0.1],
      ["unsprung_mass", "簧下质量 (kg)", 1],
      ["motor_torque_max", "电机扭矩上限 (N·m)", 50],
      ["bump_steer_coeff", "bump-steer (rad/m)", 0.1],
      ["servo_kp", "轮速伺服 Kp", 10],
      ["servo_ki", "轮速伺服 Ki", 5],
      ["suspension.spring_rate", "弹簧刚度 (N/m)", 1000],
      ["suspension.damper_rate", "阻尼 (N·s/m)", 100],
      ["suspension.anti_roll_rate", "防倾杆刚度 (N·m/rad)", 1000],
      ["suspension.kingpin_mu", "运动学主销摩擦系数", 0.05],
    ],
  },
  {
    title: "齿条硬点",
    tier: "advanced",
    fields: [
      ["steering_geometry.front_outer_x", "前外球头 x (mm)", 0.5, MM],
      ["steering_geometry.front_outer_y", "前外球头 y (mm)", 0.5, MM],
      ["steering_geometry.front_inner_x", "前内球头 x (mm)", 0.5, MM],
      ["steering_geometry.front_inner_y", "前内球头 y (mm)", 0.5, MM],
      ["steering_geometry.front_rack_axis_deg", "前齿条轴角 (°)", 0.5],
      ["steering_geometry.front_rack_travel_limit", "前齿条半行程 (mm)", 1, MM],
      ["steering_geometry.rear_outer_x", "后外球头 x (mm)", 0.5, MM],
      ["steering_geometry.rear_outer_y", "后外球头 y (mm)", 0.5, MM],
      ["steering_geometry.rear_inner_x", "后内球头 x (mm)", 0.5, MM],
      ["steering_geometry.rear_inner_y", "后内球头 y (mm)", 0.5, MM],
      ["steering_geometry.rear_rack_axis_deg", "后齿条轴角 (°)", 0.5],
      ["steering_geometry.rear_rack_travel_limit", "后齿条半行程 (mm)", 1, MM],
    ],
  },
];

export const CORE_PARAMETER_GROUPS = PARAMETER_GROUPS.filter((group) => group.tier === "core");
export const ADVANCED_PARAMETER_GROUPS = PARAMETER_GROUPS.filter((group) => group.tier === "advanced");
