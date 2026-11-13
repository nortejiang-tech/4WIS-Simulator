#include "simulation.hpp"
namespace fourwis {
json normalize_experiment(json e) {
  json d = {{"name", "untitled"},
            {"description", ""},
            {"vehicle", {{"profile", nullptr}, {"overrides", json::object()}}},
            {"model_type", "simplified_dynamic"},
            {"strategy", "ideal_ackermann"},
            {"mode_params", json::object()},
            {"scene", nullptr},
            {"path", nullptr},
            {"faults", json::array()},
            {"fault_seed", 0},
            {"maneuver", {{"name", "untitled"}, {"steps", json::array()}}},
            {"dt", .005},
            {"record_hz", 50.},
            {"kpis", json::array()}};
  merge(d, e);
  auto &steps = d["maneuver"]["steps"];
  for (auto &step : steps) {
    json s = {{"name", ""},
              {"duration", 0.},
              {"steer",
               {{"kind", "constant"},
                {"amplitude", 0.},
                {"unit", "normalized"},
                {"freq_hz", .5},
                {"f0_hz", .1},
                {"f1_hz", 2.},
                {"t_step", .5},
                {"start", 0.}}},
              {"speed_kmh", nullptr},
              {"speed_ramp_s", 0.},
              {"mode_params", json::object()}};
    merge(s, step);
    step = s;
  }
  return d;
}
std::map<std::string, double> sample(const Model &m, const Command &c,
                                     const Driver &d, bool all) {
  auto &s = m.s;
  std::map<std::string, double> out = {{"pose_x", s.x},
                                       {"pose_y", s.y},
                                       {"pose_psi", s.psi},
                                       {"vx", s.vx},
                                       {"vy", s.vy},
                                       {"yaw_rate", s.yaw_rate},
                                       {"driver_steering", d.steering},
                                       {"driver_throttle", d.throttle},
                                       {"ax", s.ax},
                                       {"ay", s.ay}};
  for (const char *n :
       {"steer_hand_torque", "steer_hand_angle", "steer_torque_sensor",
        "steer_assist_torque", "steer_motor_torque", "steer_motor_speed",
        "steer_angle_deviation", "steer_plant_active",
        "steer_corner_deviation_fl", "steer_corner_deviation_fr",
        "steer_corner_deviation_rl", "steer_corner_deviation_rr"})
    out[n] = num(m.steering_channels, n,
                 std::string(n) == "steer_plant_active" ? 0 : nan);
  for (int i = 0; i < 4; i++) {
    auto add = [&](const char *n, double v) {
      out[std::string(n) + "_" + wheel_names[i]] = v;
    };
    add("delta", s.delta[i]);
    add("delta_cmd", c.delta_cmd[i]);
    add("delta_reported", s.delta_reported[i]);
    add("fault_mode", s.fault_mode[i]);
    add("sensor_fault_active", s.sensor_fault_active[i]);
    add("steer_rate", s.steer_rate[i]);
    add("omega", s.wheel_omega[i]);
    add("fz", s.fz[i]);
    add("torque_steer", s.torque_steer[i]);
    add("rack_force", s.rack_force[i]);
    add("motor_torque", s.motor_torque_demand[i]);
    add("slip_alpha", m.alpha[i]);
    add("slip_kappa", m.kappa[i]);
    add("icr_dev", s.wheel_icr_dev[i]);
    add("grip_util", s.grip_util[i]);
    add("grip_margin_lat", s.grip_margin_lat[i]);
    add("grip_margin_long", s.grip_margin_long[i]);
    if (all) {
      for (auto q : std::vector<std::pair<std::string, const V4 *>>{
               {"susp_defl", &s.susp_defl},
               {"wheel_locked", &s.wheel_locked},
               {"grip_capacity", &s.grip_capacity},
               {"grip_alpha_peak", &s.grip_alpha_peak},
               {"grip_kappa_peak", &s.grip_kappa_peak},
               {"grip_beyond_peak_lat", &s.grip_beyond_peak_lat},
               {"grip_beyond_peak_long", &s.grip_beyond_peak_long},
               {"linkage_arm_tie_angle", &s.linkage_arm_tie_angle},
               {"linkage_tie_rack_angle", &s.linkage_tie_rack_angle},
               {"linkage_efficiency", &s.linkage_efficiency}})
        add(q.first.c_str(), (*q.second)[i]);
      add("deltacmd", c.delta_cmd[i]);
      add("drive_torque_cmd", c.drive_torque_cmd[i]);
      add("brake_cmd", c.brake_cmd[i]);
      add("wheel_speed_cmd", c.wheel_speed_cmd[i]);
      add("tire_fx", m.kind == "kinematic" ? nan : m.fx[i]);
      add("tire_fy", m.kind == "kinematic" ? nan : m.fy[i]);
      add("icr_x", s.wheel_icr_body[i][0]);
      add("icr_y", s.wheel_icr_body[i][1]);
      if (m.kind == "kinematic") {
        add("slip_alpha", nan);
        add("slip_kappa", nan);
      }
    }
  }
  if (all) {
    out["pose_z"] = s.z;
    out["roll"] = s.roll;
    out["pitch"] = s.pitch;
    out["grip_valid"] = s.grip_valid;
    out["mu_avg"] = s.mu_avg;
    out["driver_brake"] = d.brake;
    out["driver_gear"] = d.gear;
    out["driver_handbrake"] = d.handbrake;
    out["icr_vehicle_body_x"] = s.vehicle_icr_body[0];
    out["icr_vehicle_body_y"] = s.vehicle_icr_body[1];
    out["icr_target_body_x"] = c.icr_target_body[0];
    out["icr_target_body_y"] = c.icr_target_body[1];
  }
  return out;
}
static double steer_value(const json &s, double t, double duration) {
  auto k = str(s, "kind");
  double a = num(s, "amplitude"), u = t / std::max(duration, 1e-9);
  if (k == "constant")
    return a;
  if (k == "step")
    return t < num(s, "t_step", .5) ? 0 : a;
  if (k == "ramp")
    return num(s, "start") + (a - num(s, "start")) * clip(u, 0, 1);
  if (k == "sine")
    return a * std::sin(2 * pi * num(s, "freq_hz", .5) * t);
  if (k == "sweep") {
    double f = num(s, "f0_hz", .1),
           slope = (num(s, "f1_hz", 2) - f) / std::max(duration, 1e-9);
    return a * std::sin(2 * pi * (f * t + .5 * slope * t * t));
  }
  if (k == "dlc") {
    auto bump = [&](double lo, double hi) {
      return u <= lo || u >= hi
                 ? 0
                 : std::pow(std::sin(pi * (u - lo) / (hi - lo)), 2);
    };
    return a * (bump(.12, .42) - bump(.5, .8));
  }
  throw Error(422, "unknown steer profile");
}
Run Simulation::run() {
  Run r;
  double dt = num(experiment, "dt", .005),
         hz = num(experiment, "record_hz", 50);
  int every = std::max(1, int(std::nearbyint(1 / (hz * dt))));
  double speed = 0;
  for (auto &step : experiment["maneuver"]["steps"]) {
    merge(driver.mode_params, step["mode_params"]);
    double prev = speed;
    if (!step["speed_kmh"].is_null())
      speed = step["speed_kmh"].get<double>() / 3.6;
    double ramp = step["speed_kmh"].is_null() ? 0 : num(step, "speed_ramp_s"),
           duration = num(step, "duration");
    if (!(duration > 0 && duration <= 600))
      throw Error(422, "invalid maneuver duration");
    int n = std::max(1, int(std::nearbyint(duration / dt)));
    r.duration_s += duration;
    for (int k = 0; k < n; k++) {
      double t = k * dt, v = ramp > 0 && t < ramp
                                 ? prev + (speed - prev) * (t / ramp)
                                 : speed;
      driver.mode_params["speed_target_ms"] = v;
      driver.gear = v < 0 ? -1 : 1;
      driver.throttle = std::min(1., std::abs(v) / std::max(model.p.v_max, .1));
      double steer = steer_value(step["steer"], t, duration);
      if (str(step["steer"], "unit") == "front_deg") {
        if (controller.kind != "ideal_ackermann" &&
            controller.kind != "rear_wheel_steer" &&
            controller.kind != "fault_reconfig")
          throw Error(422, "strategy does not support front_deg");
        driver.mode_params["steer_raw_rad"] = rad(steer);
        driver.steering = 0;
      } else {
        driver.mode_params.erase("steer_raw_rad");
        driver.steering = clip(steer, -1, 1);
      }
      advance(dt);
      if (r.n_steps % every == 0) {
        r.t.push_back(model.s.t);
        for (auto &v : sample(model, last, driver))
          r.channels[v.first].push_back(v.second);
      }
      r.n_steps++;
    }
  }
  r.meta = {{"experiment", experiment},
            {"record_hz", hz},
            {"n_samples", r.t.size()},
            {"fault_timeline", faults.status()},
            {"engine", engine_info(model.kind)}};
  return r;
}
} // namespace fourwis
