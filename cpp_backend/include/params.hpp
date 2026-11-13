// Generated from core/state.py by tools/export_contract.py. Do not edit
// defaults by hand.
#pragma once
#include "common.hpp"
namespace fourwis {
struct SuspensionParams {
  double caster_angle = 0.1047;
  double kingpin_inclination = 0.2094;
  double camber = {};
  double scrub_radius = 0.015;
  double spring_rate = 70000.0;
  double damper_rate = 4000.0;
  double anti_roll_rate = 30000.0;
  double roll_stiffness_front_frac = 0.6;
  double kingpin_mu = 0.6;
};
inline void to_json(json &j, const SuspensionParams &p) {
  j = json::object();
  j["caster_angle"] = p.caster_angle;
  j["kingpin_inclination"] = p.kingpin_inclination;
  j["camber"] = p.camber;
  j["scrub_radius"] = p.scrub_radius;
  j["spring_rate"] = p.spring_rate;
  j["damper_rate"] = p.damper_rate;
  j["anti_roll_rate"] = p.anti_roll_rate;
  j["roll_stiffness_front_frac"] = p.roll_stiffness_front_frac;
  j["kingpin_mu"] = p.kingpin_mu;
}
inline void from_json(const json &j, SuspensionParams &p) {
  if (j.contains("caster_angle"))
    j.at("caster_angle").get_to(p.caster_angle);
  if (j.contains("kingpin_inclination"))
    j.at("kingpin_inclination").get_to(p.kingpin_inclination);
  if (j.contains("camber"))
    j.at("camber").get_to(p.camber);
  if (j.contains("scrub_radius"))
    j.at("scrub_radius").get_to(p.scrub_radius);
  if (j.contains("spring_rate"))
    j.at("spring_rate").get_to(p.spring_rate);
  if (j.contains("damper_rate"))
    j.at("damper_rate").get_to(p.damper_rate);
  if (j.contains("anti_roll_rate"))
    j.at("anti_roll_rate").get_to(p.anti_roll_rate);
  if (j.contains("roll_stiffness_front_frac"))
    j.at("roll_stiffness_front_frac").get_to(p.roll_stiffness_front_frac);
  if (j.contains("kingpin_mu"))
    j.at("kingpin_mu").get_to(p.kingpin_mu);
}
struct SteeringGeometryParams {
  double front_outer_x = {};
  double front_outer_y = {};
  double front_inner_x = {};
  double front_inner_y = {};
  double front_rack_axis_deg = 90.0;
  double front_rack_travel_limit = 0.085;
  double rear_outer_x = 0.145971235;
  double rear_outer_y = {};
  double rear_inner_x = 0.176324;
  double rear_inner_y = {};
  double rear_rack_axis_deg = 90.0;
  double rear_rack_travel_limit = 0.085;
};
inline void to_json(json &j, const SteeringGeometryParams &p) {
  j = json::object();
  j["front_outer_x"] = p.front_outer_x;
  j["front_outer_y"] = p.front_outer_y;
  j["front_inner_x"] = p.front_inner_x;
  j["front_inner_y"] = p.front_inner_y;
  j["front_rack_axis_deg"] = p.front_rack_axis_deg;
  j["front_rack_travel_limit"] = p.front_rack_travel_limit;
  j["rear_outer_x"] = p.rear_outer_x;
  j["rear_outer_y"] = p.rear_outer_y;
  j["rear_inner_x"] = p.rear_inner_x;
  j["rear_inner_y"] = p.rear_inner_y;
  j["rear_rack_axis_deg"] = p.rear_rack_axis_deg;
  j["rear_rack_travel_limit"] = p.rear_rack_travel_limit;
}
inline void from_json(const json &j, SteeringGeometryParams &p) {
  if (j.contains("front_outer_x"))
    j.at("front_outer_x").get_to(p.front_outer_x);
  if (j.contains("front_outer_y"))
    j.at("front_outer_y").get_to(p.front_outer_y);
  if (j.contains("front_inner_x"))
    j.at("front_inner_x").get_to(p.front_inner_x);
  if (j.contains("front_inner_y"))
    j.at("front_inner_y").get_to(p.front_inner_y);
  if (j.contains("front_rack_axis_deg"))
    j.at("front_rack_axis_deg").get_to(p.front_rack_axis_deg);
  if (j.contains("front_rack_travel_limit"))
    j.at("front_rack_travel_limit").get_to(p.front_rack_travel_limit);
  if (j.contains("rear_outer_x"))
    j.at("rear_outer_x").get_to(p.rear_outer_x);
  if (j.contains("rear_outer_y"))
    j.at("rear_outer_y").get_to(p.rear_outer_y);
  if (j.contains("rear_inner_x"))
    j.at("rear_inner_x").get_to(p.rear_inner_x);
  if (j.contains("rear_inner_y"))
    j.at("rear_inner_y").get_to(p.rear_inner_y);
  if (j.contains("rear_rack_axis_deg"))
    j.at("rear_rack_axis_deg").get_to(p.rear_rack_axis_deg);
  if (j.contains("rear_rack_travel_limit"))
    j.at("rear_rack_travel_limit").get_to(p.rear_rack_travel_limit);
}
struct VehicleParams {
  double wheelbase = 3.16;
  double track_front = 1.565172;
  double track_rear = 1.565172;
  double mass = 2900.0;
  double inertia_z = 7500.0;
  double cg_to_front = 1.55;
  double tire_radius = 0.395;
  double tire_width = 0.265;
  double contact_patch_radius = 0.11;
  double steer_limit = 0.6109;
  double v_max = 55.6;
  double cg_height = 0.62;
  double wheel_inertia = 2.5;
  double motor_torque_max = 3000.0;
  double unsprung_mass = 55.0;
  double tire_vertical_stiffness = 280000.0;
  double steer_tau = 0.06;
  double steer_rate_max = 8.0;
  double bump_steer_coeff = 0.0;
  json kc = nullptr;
  std::string tire_model = "linear";
  double tire_c_alpha = 120000.0;
  double tire_relax_length = 0.0;
  double tire_relax_length_long = 0.0;
  double tire_mu_load_sensitivity = 0.0;
  double tire_c_alpha_front_scale = 0.8;
  double tire_c_alpha_rear_scale = 1.2;
  double tire_c_kappa = 100000.0;
  double tire_t_pneumatic = 0.03;
  double tire_load_sensitivity_exp = 0.8;
  bool tire_load_sensitivity_time_domain = true;
  double parking_scrub_coeff = 0.8;
  double parking_lateral_coeff = 0.8;
  double parking_torque_coeff = 0.8;
  double low_speed_blend_ms = 1.5;
  double static_tire_deflection_deg = 8.0;
  double rolling_resistance_coeff = 0.012;
  double drag_coeff_cd = 0.3;
  double frontal_area = 2.8;
  double air_density = 1.225;
  double aero_lift_coeff_front = 0.3;
  double aero_lift_coeff_rear = 0.15;
  double camber_thrust_coeff = 1.0;
  double static_toe_front = 0.001745;
  double static_toe_rear = 0.0;
  double tire_cx = 1.65;
  double tire_cy = 1.3;
  double tire_ex = {};
  double tire_ey = {};
  double servo_kp = 200.0;
  double servo_ki = 50.0;
  std::string longitudinal_mode = "speed_servo";
  double drive_front_ratio = 0.5;
  double motor_power_max = 400000.0;
  std::string diff_type = "independent";
  double driver_speed_limit = 0.0;
  double brake_torque_max = 12000.0;
  double brake_bias_front = 0.65;
  double brake_tau = 0.08;
  double v_max_reverse = 5.0;
  double steer_wheel_range = 540.0;
  double steer_ratio_low = 0.0;
  double steer_ratio_high = 0.0;
  double steer_ratio_v_ref = 22.0;
  double steer_ay_ref_frac = 0.9;
  SuspensionParams suspension = {};
  double steering_arm_length = 0.146451;
  double pinion_radius = 0.02;
  double tie_rod_angle_deg = 6.67;
  double rack_mech_efficiency = 0.92;
  double motor_gear_ratio = 10.0;
  SteeringGeometryParams steering_geometry = {};
  json steering_system = {};
};
inline void to_json(json &j, const VehicleParams &p) {
  j = json::object();
  j["wheelbase"] = p.wheelbase;
  j["track_front"] = p.track_front;
  j["track_rear"] = p.track_rear;
  j["mass"] = p.mass;
  j["inertia_z"] = p.inertia_z;
  j["cg_to_front"] = p.cg_to_front;
  j["tire_radius"] = p.tire_radius;
  j["tire_width"] = p.tire_width;
  j["contact_patch_radius"] = p.contact_patch_radius;
  j["steer_limit"] = p.steer_limit;
  j["v_max"] = p.v_max;
  j["cg_height"] = p.cg_height;
  j["wheel_inertia"] = p.wheel_inertia;
  j["motor_torque_max"] = p.motor_torque_max;
  j["unsprung_mass"] = p.unsprung_mass;
  j["tire_vertical_stiffness"] = p.tire_vertical_stiffness;
  j["steer_tau"] = p.steer_tau;
  j["steer_rate_max"] = p.steer_rate_max;
  j["bump_steer_coeff"] = p.bump_steer_coeff;
  j["kc"] = p.kc;
  j["tire_model"] = p.tire_model;
  j["tire_c_alpha"] = p.tire_c_alpha;
  j["tire_relax_length"] = p.tire_relax_length;
  j["tire_relax_length_long"] = p.tire_relax_length_long;
  j["tire_mu_load_sensitivity"] = p.tire_mu_load_sensitivity;
  j["tire_c_alpha_front_scale"] = p.tire_c_alpha_front_scale;
  j["tire_c_alpha_rear_scale"] = p.tire_c_alpha_rear_scale;
  j["tire_c_kappa"] = p.tire_c_kappa;
  j["tire_t_pneumatic"] = p.tire_t_pneumatic;
  j["tire_load_sensitivity_exp"] = p.tire_load_sensitivity_exp;
  j["tire_load_sensitivity_time_domain"] = p.tire_load_sensitivity_time_domain;
  j["parking_scrub_coeff"] = p.parking_scrub_coeff;
  j["parking_lateral_coeff"] = p.parking_lateral_coeff;
  j["parking_torque_coeff"] = p.parking_torque_coeff;
  j["low_speed_blend_ms"] = p.low_speed_blend_ms;
  j["static_tire_deflection_deg"] = p.static_tire_deflection_deg;
  j["rolling_resistance_coeff"] = p.rolling_resistance_coeff;
  j["drag_coeff_cd"] = p.drag_coeff_cd;
  j["frontal_area"] = p.frontal_area;
  j["air_density"] = p.air_density;
  j["aero_lift_coeff_front"] = p.aero_lift_coeff_front;
  j["aero_lift_coeff_rear"] = p.aero_lift_coeff_rear;
  j["camber_thrust_coeff"] = p.camber_thrust_coeff;
  j["static_toe_front"] = p.static_toe_front;
  j["static_toe_rear"] = p.static_toe_rear;
  j["tire_cx"] = p.tire_cx;
  j["tire_cy"] = p.tire_cy;
  j["tire_ex"] = p.tire_ex;
  j["tire_ey"] = p.tire_ey;
  j["servo_kp"] = p.servo_kp;
  j["servo_ki"] = p.servo_ki;
  j["longitudinal_mode"] = p.longitudinal_mode;
  j["drive_front_ratio"] = p.drive_front_ratio;
  j["motor_power_max"] = p.motor_power_max;
  j["diff_type"] = p.diff_type;
  j["driver_speed_limit"] = p.driver_speed_limit;
  j["brake_torque_max"] = p.brake_torque_max;
  j["brake_bias_front"] = p.brake_bias_front;
  j["brake_tau"] = p.brake_tau;
  j["v_max_reverse"] = p.v_max_reverse;
  j["steer_wheel_range"] = p.steer_wheel_range;
  j["steer_ratio_low"] = p.steer_ratio_low;
  j["steer_ratio_high"] = p.steer_ratio_high;
  j["steer_ratio_v_ref"] = p.steer_ratio_v_ref;
  j["steer_ay_ref_frac"] = p.steer_ay_ref_frac;
  j["suspension"] = p.suspension;
  j["steering_arm_length"] = p.steering_arm_length;
  j["pinion_radius"] = p.pinion_radius;
  j["tie_rod_angle_deg"] = p.tie_rod_angle_deg;
  j["rack_mech_efficiency"] = p.rack_mech_efficiency;
  j["motor_gear_ratio"] = p.motor_gear_ratio;
  j["steering_geometry"] = p.steering_geometry;
  j["steering_system"] = p.steering_system;
}
inline void from_json(const json &j, VehicleParams &p) {
  if (j.contains("wheelbase"))
    j.at("wheelbase").get_to(p.wheelbase);
  if (j.contains("track_front"))
    j.at("track_front").get_to(p.track_front);
  if (j.contains("track_rear"))
    j.at("track_rear").get_to(p.track_rear);
  if (j.contains("mass"))
    j.at("mass").get_to(p.mass);
  if (j.contains("inertia_z"))
    j.at("inertia_z").get_to(p.inertia_z);
  if (j.contains("cg_to_front"))
    j.at("cg_to_front").get_to(p.cg_to_front);
  if (j.contains("tire_radius"))
    j.at("tire_radius").get_to(p.tire_radius);
  if (j.contains("tire_width"))
    j.at("tire_width").get_to(p.tire_width);
  if (j.contains("contact_patch_radius"))
    j.at("contact_patch_radius").get_to(p.contact_patch_radius);
  if (j.contains("steer_limit"))
    j.at("steer_limit").get_to(p.steer_limit);
  if (j.contains("v_max"))
    j.at("v_max").get_to(p.v_max);
  if (j.contains("cg_height"))
    j.at("cg_height").get_to(p.cg_height);
  if (j.contains("wheel_inertia"))
    j.at("wheel_inertia").get_to(p.wheel_inertia);
  if (j.contains("motor_torque_max"))
    j.at("motor_torque_max").get_to(p.motor_torque_max);
  if (j.contains("unsprung_mass"))
    j.at("unsprung_mass").get_to(p.unsprung_mass);
  if (j.contains("tire_vertical_stiffness"))
    j.at("tire_vertical_stiffness").get_to(p.tire_vertical_stiffness);
  if (j.contains("steer_tau"))
    j.at("steer_tau").get_to(p.steer_tau);
  if (j.contains("steer_rate_max"))
    j.at("steer_rate_max").get_to(p.steer_rate_max);
  if (j.contains("bump_steer_coeff"))
    j.at("bump_steer_coeff").get_to(p.bump_steer_coeff);
  if (j.contains("kc"))
    j.at("kc").get_to(p.kc);
  if (j.contains("tire_model"))
    j.at("tire_model").get_to(p.tire_model);
  if (j.contains("tire_c_alpha"))
    j.at("tire_c_alpha").get_to(p.tire_c_alpha);
  if (j.contains("tire_relax_length"))
    j.at("tire_relax_length").get_to(p.tire_relax_length);
  if (j.contains("tire_relax_length_long"))
    j.at("tire_relax_length_long").get_to(p.tire_relax_length_long);
  if (j.contains("tire_mu_load_sensitivity"))
    j.at("tire_mu_load_sensitivity").get_to(p.tire_mu_load_sensitivity);
  if (j.contains("tire_c_alpha_front_scale"))
    j.at("tire_c_alpha_front_scale").get_to(p.tire_c_alpha_front_scale);
  if (j.contains("tire_c_alpha_rear_scale"))
    j.at("tire_c_alpha_rear_scale").get_to(p.tire_c_alpha_rear_scale);
  if (j.contains("tire_c_kappa"))
    j.at("tire_c_kappa").get_to(p.tire_c_kappa);
  if (j.contains("tire_t_pneumatic"))
    j.at("tire_t_pneumatic").get_to(p.tire_t_pneumatic);
  if (j.contains("tire_load_sensitivity_exp"))
    j.at("tire_load_sensitivity_exp").get_to(p.tire_load_sensitivity_exp);
  if (j.contains("tire_load_sensitivity_time_domain"))
    j.at("tire_load_sensitivity_time_domain")
        .get_to(p.tire_load_sensitivity_time_domain);
  if (j.contains("parking_scrub_coeff"))
    j.at("parking_scrub_coeff").get_to(p.parking_scrub_coeff);
  if (j.contains("parking_lateral_coeff"))
    j.at("parking_lateral_coeff").get_to(p.parking_lateral_coeff);
  if (j.contains("parking_torque_coeff"))
    j.at("parking_torque_coeff").get_to(p.parking_torque_coeff);
  if (j.contains("low_speed_blend_ms"))
    j.at("low_speed_blend_ms").get_to(p.low_speed_blend_ms);
  if (j.contains("static_tire_deflection_deg"))
    j.at("static_tire_deflection_deg").get_to(p.static_tire_deflection_deg);
  if (j.contains("rolling_resistance_coeff"))
    j.at("rolling_resistance_coeff").get_to(p.rolling_resistance_coeff);
  if (j.contains("drag_coeff_cd"))
    j.at("drag_coeff_cd").get_to(p.drag_coeff_cd);
  if (j.contains("frontal_area"))
    j.at("frontal_area").get_to(p.frontal_area);
  if (j.contains("air_density"))
    j.at("air_density").get_to(p.air_density);
  if (j.contains("aero_lift_coeff_front"))
    j.at("aero_lift_coeff_front").get_to(p.aero_lift_coeff_front);
  if (j.contains("aero_lift_coeff_rear"))
    j.at("aero_lift_coeff_rear").get_to(p.aero_lift_coeff_rear);
  if (j.contains("camber_thrust_coeff"))
    j.at("camber_thrust_coeff").get_to(p.camber_thrust_coeff);
  if (j.contains("static_toe_front"))
    j.at("static_toe_front").get_to(p.static_toe_front);
  if (j.contains("static_toe_rear"))
    j.at("static_toe_rear").get_to(p.static_toe_rear);
  if (j.contains("tire_cx"))
    j.at("tire_cx").get_to(p.tire_cx);
  if (j.contains("tire_cy"))
    j.at("tire_cy").get_to(p.tire_cy);
  if (j.contains("tire_ex"))
    j.at("tire_ex").get_to(p.tire_ex);
  if (j.contains("tire_ey"))
    j.at("tire_ey").get_to(p.tire_ey);
  if (j.contains("servo_kp"))
    j.at("servo_kp").get_to(p.servo_kp);
  if (j.contains("servo_ki"))
    j.at("servo_ki").get_to(p.servo_ki);
  if (j.contains("longitudinal_mode"))
    j.at("longitudinal_mode").get_to(p.longitudinal_mode);
  if (j.contains("drive_front_ratio"))
    j.at("drive_front_ratio").get_to(p.drive_front_ratio);
  if (j.contains("motor_power_max"))
    j.at("motor_power_max").get_to(p.motor_power_max);
  if (j.contains("diff_type"))
    j.at("diff_type").get_to(p.diff_type);
  if (j.contains("driver_speed_limit"))
    j.at("driver_speed_limit").get_to(p.driver_speed_limit);
  if (j.contains("brake_torque_max"))
    j.at("brake_torque_max").get_to(p.brake_torque_max);
  if (j.contains("brake_bias_front"))
    j.at("brake_bias_front").get_to(p.brake_bias_front);
  if (j.contains("brake_tau"))
    j.at("brake_tau").get_to(p.brake_tau);
  if (j.contains("v_max_reverse"))
    j.at("v_max_reverse").get_to(p.v_max_reverse);
  if (j.contains("steer_wheel_range"))
    j.at("steer_wheel_range").get_to(p.steer_wheel_range);
  if (j.contains("steer_ratio_low"))
    j.at("steer_ratio_low").get_to(p.steer_ratio_low);
  if (j.contains("steer_ratio_high"))
    j.at("steer_ratio_high").get_to(p.steer_ratio_high);
  if (j.contains("steer_ratio_v_ref"))
    j.at("steer_ratio_v_ref").get_to(p.steer_ratio_v_ref);
  if (j.contains("steer_ay_ref_frac"))
    j.at("steer_ay_ref_frac").get_to(p.steer_ay_ref_frac);
  if (j.contains("suspension"))
    j.at("suspension").get_to(p.suspension);
  if (j.contains("steering_arm_length"))
    j.at("steering_arm_length").get_to(p.steering_arm_length);
  if (j.contains("pinion_radius"))
    j.at("pinion_radius").get_to(p.pinion_radius);
  if (j.contains("tie_rod_angle_deg"))
    j.at("tie_rod_angle_deg").get_to(p.tie_rod_angle_deg);
  if (j.contains("rack_mech_efficiency"))
    j.at("rack_mech_efficiency").get_to(p.rack_mech_efficiency);
  if (j.contains("motor_gear_ratio"))
    j.at("motor_gear_ratio").get_to(p.motor_gear_ratio);
  if (j.contains("steering_geometry"))
    j.at("steering_geometry").get_to(p.steering_geometry);
  if (j.contains("steering_system"))
    j.at("steering_system").get_to(p.steering_system);
}
} // namespace fourwis
