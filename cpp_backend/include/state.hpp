#pragma once
#include "params.hpp"
namespace fourwis {
inline Wheels wheels(const VehicleParams &p) {
  return {{{p.wheelbase / 2, p.track_front / 2},
           {p.wheelbase / 2, -p.track_front / 2},
           {-p.wheelbase / 2, p.track_rear / 2},
           {-p.wheelbase / 2, -p.track_rear / 2}}};
}
struct Driver {
  double throttle = 0, brake = 0, steering = 0;
  int gear = 1, handbrake = 0;
  json mode_params = json::object();
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Driver, throttle, brake,
                                                steering, gear, handbrake,
                                                mode_params)
struct Command {
  V4 delta_cmd{}, wheel_speed_cmd{}, brake_cmd{}, drive_torque_cmd{},
      steer_override{nan, nan, nan, nan}, steer_override_rate{};
  V2 icr_target_body{nan, nan};
  int handbrake = 0;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(
    Command, delta_cmd, wheel_speed_cmd, brake_cmd, drive_torque_cmd,
    steer_override, steer_override_rate, icr_target_body, handbrake)
#define WHEEL_FIELDS(X)                                                        \
  X(delta_reported)                                                            \
  X(fault_mode)                                                                \
  X(sensor_fault_active) X(steer_rate) X(delta) X(wheel_omega) X(fz)           \
      X(torque_steer) X(susp_defl) X(wheel_locked) X(grip_capacity)            \
          X(grip_util) X(grip_margin_lat) X(grip_margin_long)                  \
              X(grip_alpha_peak) X(grip_kappa_peak) X(grip_beyond_peak_lat)    \
                  X(grip_beyond_peak_long) X(wheel_icr_dev) X(rack_force)      \
                      X(motor_torque_demand) X(linkage_arm_tie_angle)          \
                          X(linkage_tie_rack_angle) X(linkage_efficiency)
#define SCALAR_FIELDS(X)                                                       \
  X(t)                                                                         \
  X(x) X(y) X(psi) X(z) X(roll) X(pitch) X(vx) X(vy) X(yaw_rate) X(ax) X(ay)
struct State {
#define DECLARE_S(n) double n = 0;
  SCALAR_FIELDS(DECLARE_S)
#undef DECLARE_S
#define DECLARE_W(n) V4 n{};
  WHEEL_FIELDS(DECLARE_W)
#undef DECLARE_W
  double mu_avg = .85;
  bool grip_valid = false;
  V2 vehicle_icr_body{nan, nan};
  Wheels wheel_pos_body{}, wheel_icr_body{};
  State() {
    wheel_icr_dev.fill(nan);
    linkage_arm_tie_angle.fill(nan);
    linkage_tie_rack_angle.fill(nan);
    for (auto &x : wheel_icr_body)
      x.fill(nan);
  }
};
inline void to_json(json &j, const State &s) {
  j = json::object();
#define SAVE(n) j[#n] = s.n;
  SCALAR_FIELDS(SAVE)
  WHEEL_FIELDS(SAVE)
#undef SAVE
  j["mu_avg"] = s.mu_avg;
  j["grip_valid"] = s.grip_valid;
  j["vehicle_icr_body"] = s.vehicle_icr_body;
  j["wheel_pos_body"] = s.wheel_pos_body;
  j["wheel_icr_body"] = s.wheel_icr_body;
}
inline void from_json(const json &j, State &s) {
#define READ(n)                                                                \
  if (j.contains(#n) && !j.at(#n).is_null())                                   \
    j.at(#n).get_to(s.n);
  SCALAR_FIELDS(READ)
  WHEEL_FIELDS(READ)
#undef READ
  if (j.contains("mu_avg"))
    j.at("mu_avg").get_to(s.mu_avg);
}
struct Scene {
  json definition = {
      {"base_mu", .85}, {"surface", "flat"}, {"disturbances", json::array()}};
  struct Local {
    double mu, fz = 0, z = 0;
  };
  Local at(double x, double y) const {
    Local q{num(definition, "base_mu", .85)};
    bool overridden = false;
    for (const auto &d : definition.value("disturbances", json::array())) {
      double h = num(d, "heading"), c = std::cos(h), s = std::sin(h),
             dx = x - num(d, "x"), dy = y - num(d, "y"), u = c * dx + s * dy,
             v = -s * dx + c * dy;
      double length = num(d, "length"), width = num(d, "width");
      if (std::abs(u) > length / 2 || std::abs(v) > width / 2)
        continue;
      auto type = str(d, "type");
      if (type == "ice_patch" || type == "split_mu") {
        double mu =
            type == "ice_patch"
                ? num(d, "mu", .3)
                : num(d, v >= 0 ? "mu_left" : "mu_right", v >= 0 ? .9 : .3);
        q.mu = overridden ? std::min(q.mu, mu) : mu;
        overridden = true;
      } else if (type == "speed_bump") {
        double z = num(d, "height", .05) * .5 *
                   (1 + std::cos(pi * 2 * u / std::max(length, 1e-6)));
        q.z += z;
        q.fz += num(d, "stiffness", 1e5) * z;
      } else if (type == "slope")
        q.z += (u + length / 2) * std::tan(num(d, "angle"));
      else
        throw Error(422, "unknown disturbance type: " + type);
    }
    return q;
  }
};
} // namespace fourwis
