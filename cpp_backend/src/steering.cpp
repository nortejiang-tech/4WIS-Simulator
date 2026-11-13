#include "steering.hpp"
namespace fourwis {
void Motor::step(double dt, double command, double omega, const json &p) {
  double wmax = std::max(num(p, "no_load_speed_rpm", 8000) * 2 * pi / 60, 1e-6),
         available =
             num(p, "peak_torque", 8) *
             std::max(0., 1 - std::abs(omega) / wmax) *
             (1 - (1 - num(p, "thermal_derate", .65)) * clip(heat - 1, 0, 1));
  saturated = std::abs(command) > available + 1e-9;
  double target = clip(command, -available, available),
         tau = 1 / std::max(2 * pi * num(p, "bandwidth_hz", 40), 1e-6);
  torque += (target - torque) * (1 - std::exp(-dt / tau));
  double load =
      std::pow(torque / std::max(num(p, "continuous_torque", 4.5), 1e-6), 2);
  heat += (load - heat) *
          (1 - std::exp(-dt / std::max(num(p, "thermal_tau_s", 120), 1e-6)));
  speed = omega;
}
Steering::Steering(const VehicleParams &params)
    : p(params), arch(str(p.steering_system, "architecture", "r_eps")) {
  auto map_name = str(p.steering_system, "assist_map", "default");
  if (map_name != "default" && map_name != "none" &&
      !p.steering_system.contains("assist_map_data"))
    throw Error(422, "unknown assist map: " + map_name);
  ratio =
      std::max(p.steer_ratio_low > 0 ? p.steer_ratio_low
                                     : p.steer_wheel_range / 2 /
                                           std::max(deg(p.steer_limit), 1e-6),
               1e-6);
  enabled = yes(p.steering_system, "enabled") &&
            std::set<std::string>{"c_eps", "p_eps",   "dp_eps",  "r_eps",
                                  "sbw",   "eps_rws", "sbw_rws", "4wis"}
                .count(arch);
  bywire = arch == "sbw" || arch == "sbw_rws" || arch == "4wis";
  perwheel = arch == "4wis";
  gear = arch == "c_eps"    ? 20
         : arch == "p_eps"  ? 30
         : arch == "dp_eps" ? 45
                            : 63;
  auto ac = p.steering_system.value("angle_control", json::object());
  if (enabled && yes(ac, "enabled"))
    for (int i = 0; i < 4; i++)
      if ((i < 2 && bywire) ||
          (i >= 2 && (perwheel || arch == "eps_rws" || arch == "sbw_rws")))
        corners[i] = std::make_shared<Corner>(ac, i);
}
V4 Steering::ramp_start(const V4 &target) const {
  V4 start = target;
  if (enabled && !bywire)
    start[0] = start[1] = prev_hand / ratio;
  for (int i = 0; i < 4; i++)
    if (corners[i] && corners[i]->controller.name != "open_loop")
      start[i] = corners[i]->prev_target;
  return start;
}
double Steering::assist(double torque, double speed) const {
  if (str(p.steering_system, "assist_map", "default") == "none")
    return 0;
  static const std::vector<double> tb = {0, .3, .8, 1.5, 2.5, 3.5, 5, 8},
                                   sb = {0, 20, 60, 120};
  static const std::vector<std::vector<double>> grid = {
      {0, 0, 41, 124, 248, 338, 388, 413},
      {0, 0, 33, 102, 206, 284, 330, 355},
      {0, 0, 17, 53, 112, 158, 190, 211},
      {0, 0, 8, 28, 59, 86, 106, 119}};
  auto custom = p.steering_system.value("assist_map_data", json());
  auto t =
      custom.is_object() ? custom.at("tau_bp").get<std::vector<double>>() : tb;
  auto sp = custom.is_object()
                ? custom.at("speed_bp").get<std::vector<double>>()
                : sb;
  auto table = custom.is_object()
                   ? custom.at("table").get<std::vector<std::vector<double>>>()
                   : grid;
  std::vector<double> vals;
  for (auto &row : table)
    vals.push_back(interp(std::abs(torque), t, row));
  return (torque >= 0 ? 1 : -1) * interp(std::abs(speed) * 3.6, sp, vals);
}
double Steering::front(double dt, double target, double rack, double speed,
                       double resistance) {
  double hand_target = target * ratio,
         command_rate = (hand_target - prev_hand) / dt;
  channels = json::object();
  channels["steer_plant_active"] = 1.;
  if (bywire) {
    double wanted = byangle + (target - byangle) *
                                  (1 - std::exp(-dt / (1 / (2 * pi * 10)))),
           step = clip(wanted - byangle, -6 * dt, 6 * dt);
    byangle += step;
    road_filtered +=
        (rack - road_filtered) * (1 - std::exp(-dt / (1 / (2 * pi * 6))));
    double w = std::abs(speed) / (std::abs(speed) + 22),
           road = (.18 + (.55 - .18) * w) * (road_filtered / 1000),
           centring = (.8 + (3.2 - .8) * w) * hand_target,
           friction = std::abs(command_rate) > 1e-6
                          ? std::copysign(.25, command_rate)
                          : 0;
    json mp = {{"peak_torque", 8.},
               {"continuous_torque", 4.},
               {"bandwidth_hz", 60.},
               {"no_load_speed_rpm", 1200.}};
    motor.step(dt, road + centring + .55 * command_rate + friction,
               command_rate, mp);
    channels["steer_hand_angle"] = hand_target;
    channels["steer_hand_torque"] = motor.torque;
    channels["steer_motor_torque"] = motor.torque;
    channels["steer_angle_deviation"] = target - byangle;
    prev_hand = hand_target;
    return byangle;
  }
  auto sys = p.steering_system, col = sys.value("column", json::object()),
       rp = sys.value("rack", json::object()),
       mp = sys.value("motor", json::object());
  double r = std::max(p.pinion_radius, 1e-6),
         ktb = num(col, "torsion_stiffness_nm_per_deg", 2) * 180 / pi,
         j = .002 + num(rp, "mass", 3.2) * r * r +
             num(mp, "inertia", .00012) * gear * gear;
  int n = std::max(1, int(std::ceil(dt / .0005 - 1e-12)));
  double outer_h = dt / n;
  for (int i = 0; i < n; i++) {
    double cmd = prev_hand + (hand_target - prev_hand) * double(i + 1) / n;
    int ns = std::max(1, std::min(256, int(std::ceil(outer_h / .0002))));
    double h = outer_h / ns;
    for (int z2 = 0; z2 < ns; z2++) {
      double driver =
                 num(col, "grip_stiffness", 200) * (cmd - hand) +
                 num(col, "grip_damping", 4.4) * (command_rate - hand_rate),
             limit = num(col, "hand_torque_limit_nm", 25);
      channels["sizing_hand_limited"] = std::abs(driver) > limit ? 1. : 0.;
      driver = clip(driver, -limit, limit);
      double twist = hand - pinion,
             torsion = ktb * twist + num(col, "torsion_damping", .05) *
                                         (hand_rate - pinion_rate),
             sensor = clip(ktb * twist, -num(col, "sensor_range_nm", 10),
                           num(col, "sensor_range_nm", 10)),
             want = assist(sensor, speed),
             boost = std::abs(sensor) < 1e-9
                         ? 0
                         : std::abs(want) / std::abs(sensor);
      if (std::abs(sensor) > 1e-9)
        boost = std::min(boost,
                         num(mp, "peak_torque", 8) * gear / std::abs(sensor));
      double cdamp = 2 * num(sys, "assist_damping_ratio", .6) *
                     std::sqrt(std::max(ktb * (1 + boost) * j, 0.));
      motor.step(h, (want - cdamp * (pinion_rate - hand_rate)) / gear,
                 pinion_rate * gear, mp);
      double assistp = motor.torque * gear,
             visc = num(rp, "viscous_n_per_mps", 900) * r * r * pinion_rate,
             applied = torsion + assistp - rack * r - visc,
             coulomb =
                 (num(rp, "coulomb_friction_n", 260) + std::abs(resistance)) *
                 r,
             hold = applied + j * pinion_rate / h;
      if (std::abs(hold) <= coulomb)
        pinion_rate = 0;
      else
        pinion_rate +=
            (applied - std::copysign(coulomb, std::abs(pinion_rate) > 1e-9
                                                  ? pinion_rate
                                                  : applied)) /
            j * h;
      pinion += pinion_rate * h;
      double colfric =
          std::abs(hand_rate) > 1e-6
              ? std::copysign(num(col, "coulomb_friction", .15), hand_rate)
              : 0;
      hand_rate +=
          (driver - torsion - num(col, "damping", .3) * hand_rate - colfric) /
          std::max(num(col, "inertia", .05), 1e-9) * h;
      hand += hand_rate * h;
      channels["steer_hand_torque"] = driver;
      channels["steer_hand_angle"] = hand;
      channels["steer_torque_sensor"] = sensor;
      channels["steer_assist_torque"] = assistp;
      channels["steer_motor_torque"] = motor.torque;
      channels["steer_motor_speed"] = motor.speed;
    }
  }
  prev_hand = hand_target;
  return pinion / ratio;
}
void Steering::step(const Command &cmd, double dt, const State &s, V4 &actual,
                    V4 &actual_rate) {
  V4 previous = actual;
  for (int i = enabled ? 2 : 0; i < 4; i++)
    actual[i] +=
        clip((cmd.delta_cmd[i] - actual[i]) / std::max(p.steer_tau, .001),
             -p.steer_rate_max, p.steer_rate_max) *
        dt;
  if (enabled)
    actual[0] = actual[1] =
        front(dt, .5 * (cmd.delta_cmd[0] + cmd.delta_cmd[1]),
              s.rack_force[0] + s.rack_force[1], s.vx);
  for (int i = 0; i < 4; i++)
    if (corners[i] && !std::isfinite(cmd.steer_override[i])) {
      auto &tr = *corners[i];
      double target = perwheel ? cmd.delta_cmd[i]
                               : .5 * (cmd.delta_cmd[i < 2 ? 0 : 2] +
                                       cmd.delta_cmd[i < 2 ? 1 : 3]);
      tr.step(dt, target, s.rack_force[i] * std::max(p.pinion_radius, 1e-9),
              s.vx);
      actual[i] = tr.angle;
      channels[std::string("steer_corner_deviation_") + wheel_names[i]] =
          target - tr.angle;
      for (auto it = tr.diagnostics.begin(); it != tr.diagnostics.end(); ++it)
        channels["steer_corner_" + it.key() + "_" + wheel_names[i]] =
            it.value();
    }
  for (int i = 0; i < 4; i++) {
    if (std::isfinite(cmd.steer_override[i])) {
      actual[i] = cmd.steer_override[i];
      actual_rate[i] = cmd.steer_override_rate[i];
      if (corners[i]) {
        corners[i]->reset(actual[i]);
        std::string suffix = std::string("_") + wheel_names[i];
        for (auto it = channels.begin(); it != channels.end(); ++it)
          if (it.key().size() >= suffix.size() &&
              it.key().compare(it.key().size() - suffix.size(), suffix.size(),
                               suffix) == 0)
            it.value() = nan;
      }
    } else
      actual_rate[i] = (actual[i] - previous[i]) / dt;
  }
}
} // namespace fourwis
