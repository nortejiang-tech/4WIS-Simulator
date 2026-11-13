#include "controller.hpp"
namespace fourwis {
Command Controller::compute(const Driver &d, const State &s, double dt) {
  const auto &mp = d.mode_params;
  double v = speed_command(p, d, s, dt),
         cap =
             mp.contains("speed_target_ms") && !mp["speed_target_ms"].is_null()
                 ? INFINITY
                 : std::max(s.mu_avg, 0.) * 9.81,
         cg = p.wheelbase / 2 - p.cg_to_front;
  if (kind == "follow_trajectory")
    return follow(d, s, dt);
  static const std::set<std::string> builtins = {
      "ackermann",      "ideal_ackermann", "rear_wheel_steer",
      "crab",           "zero_radius",     "follow_trajectory",
      "fault_reconfig", "manual_wheel",    "manual_body",
      "user_js"};
  if (!builtins.count(kind)) {
    auto names = plugin_names();
    if (kind != "user_python" &&
        std::find(names.begin(), names.end(), kind) == names.end())
      throw Error(422, "unknown strategy: " + kind);
    if (!plugin)
      plugin = std::make_shared<PluginWorker>(kind, p);
    return plugin->compute(d, s, dt);
  }
  if (kind == "manual_wheel" || kind == "user_js") {
    Command c;
    c.delta_cmd = js;
    if (kind == "manual_wheel") {
      V4 norm{};
      try {
        norm = mp.value("wheel_norm", json(V4{})).get<V4>();
      } catch (...) {
        norm = {};
      }
      for (int i = 0; i < 4; i++)
        c.delta_cmd[i] = clip(norm[i], -1, 1) * p.steer_limit;
    }
    c.wheel_speed_cmd.fill(v / std::max(p.tire_radius, 1e-6));
    return c;
  }
  if (kind == "manual_body") {
    double vx = clip(num(mp, "vx_frac"), -1, 1) * p.v_max,
           vy = clip(num(mp, "vy_frac"), -1, 1) * p.v_max,
           rmax = std::hypot(p.wheelbase / 2,
                             std::max(p.track_front, p.track_rear) / 2),
           r = clip(num(mp, "yaw_frac"), -1, 1) *
               (rmax > 1e-6 ? p.v_max / rmax : 1);
    return commands(
        p,
        {vx, vy, r, std::abs(r) > 1e-6 ? V2{-vy / r, vx / r} : V2{nan, nan}});
  }
  if (kind == "crab") {
    double a = p.steer_limit * d.steering;
    V4 lock;
    lock.fill(a);
    return commands(p, {v * std::cos(a), v * std::sin(a), 0}, lock);
  }
  if (kind == "zero_radius") {
    double rate = num(mp, "spin_rate_max_dps", 30);
    if (!std::isfinite(rate))
      rate = 30;
    double rmax = std::hypot(p.wheelbase / 2,
                             std::max(p.track_front, p.track_rear) / 2),
           w = std::min(rad(clip(rate, 0, 180)),
                        p.v_max / std::max(rmax, 1e-6)) *
               d.throttle * (d.steering == 0 ? 1 : sign(d.steering));
    return commands(p, limit_motion({0, 0, w, {0, 0}}, cap, cg));
  }
  if (kind == "ackermann") {
    double a = p.steer_limit * d.steering;
    Motion q{v, 0, 0};
    if (std::abs(a) >= 1e-6) {
      double yr = p.wheelbase / std::tan(a), w = v / yr;
      q = {v, w * p.wheelbase / 2, w, {-p.wheelbase / 2, yr}};
    }
    return commands(p, limit_motion(q, cap, cg), {nan, nan, 0, 0});
  }
  if (kind == "rear_wheel_steer") {
    double df = mp.contains("steer_raw_rad")
                    ? clip(num(mp, "steer_raw_rad"), -p.steer_limit,
                           p.steer_limit)
                : yes(mp, "steer_bypass_feel")
                    ? p.steer_limit * d.steering
                    : front_angle(p, d.steering, s.vx, s.mu_avg),
           u = std::abs(s.vx);
    double h = std::isfinite(prev_t) && s.t > prev_t
                   ? std::min(.05, s.t - prev_t)
                   : .005;
    prev_t = s.t;
    double a = h / (.08 + h);
    yaw_f += a * (s.yaw_rate - yaw_f);
    double raw = std::isfinite(prev_df) && h > 1e-6 ? (df - prev_df) / h : 0;
    prev_df = df;
    ddf_f += a * (raw - ddf_f);
    auto mode = str(mp, "rws_mode", "speed_schedule");
    double dr;
    if (mode == "fixed_ratio")
      dr = clip(num(mp, "rear_ratio", -.4), -1, 1) * df;
    else if (mode == "yaw_feedback")
      dr = num(mp, "g1", -.1) * df + num(mp, "g2", .12) * yaw_f;
    else if (mode == "transient")
      dr = zero_beta_ratio(p, u) * df - num(mp, "c_transient", .1) * ddf_f;
    else if (mode == "model_following")
      dr = zero_beta_ratio(p, u) * df +
           num(mp, "g_yaw", .08) * (ref_yaw(p, u, df) - yaw_f);
    else {
      std::vector<std::pair<double, double>> pts;
      if (mp.contains("k_curve") && mp["k_curve"].is_array() &&
          mp["k_curve"].size() >= 2) {
        for (auto &q : mp["k_curve"])
          pts.emplace_back(q.at(0).get<double>(), q.at(1).get<double>());
        std::stable_sort(pts.begin(), pts.end());
      } else
        for (double x : {0., 20., 40., 60., 90., 130., 200.})
          pts.emplace_back(x, zero_beta_ratio(p, x / 3.6));
      std::vector<double> x, y;
      for (auto &q : pts) {
        x.push_back(q.first);
        y.push_back(q.second);
      }
      dr = interp(u * 3.6, x, y) * df;
    }
    df = clip(df, -p.steer_limit, p.steer_limit);
    dr = clip(dr, -p.steer_limit, p.steer_limit);
    Motion q{v, 0, 0};
    if (std::abs(df) >= 1e-9 || std::abs(dr) >= 1e-9) {
      if (std::abs(df - dr) < 1e-9) {
        q.vy = v * std::tan(df);
        q.heading = df;
      } else {
        double sf = std::sin(df), cf = std::cos(df), sr = std::sin(dr),
               cr = std::cos(dr), t = p.wheelbase * cr / (sf * cr - sr * cf),
               x = p.wheelbase / 2 - t * sf, y = t * cf,
               w = std::abs(y) > 1e-9 ? v / y : 0;
        q = {v, -w * x, w, {x, y}};
      }
    }
    return commands(p, limit_motion(q, cap, cg));
  }
  double kmax = kappa_max(p), k;
  if (kind == "ideal_ackermann") {
    double df;
    if (mp.contains("steer_raw_rad"))
      df = clip(num(mp, "steer_raw_rad"), -p.steer_limit, p.steer_limit);
    else if (yes(mp, "steer_bypass_feel")) {
      double kk = kmax * d.steering, den = 1 - std::abs(kk) * p.track_front / 2;
      df = std::abs(den) < 1e-12 ? std::copysign(p.steer_limit, kk)
                                 : std::atan(kk * p.wheelbase / 2 / den);
    } else
      df = front_angle(p, d.steering, s.vx, s.mu_avg);
    k = clip(curvature(p, df), -kmax, kmax);
    return commands(p, limit_motion(ideal_motion(k, v), cap, cg));
  }
  if (kind != "fault_reconfig")
    throw Error(422, "unknown strategy: " + kind);
  int wheel = int(num(mp, "fault_wheel", 0));
  if (wheel < 0 || wheel > 3)
    throw Error(422, "invalid fault_wheel");
  double delta_s = num(mp, "fault_angle"),
         detect = num(mp, "fault_time") + num(mp, "detect_delay", .15),
         ky = num(mp, "k_yaw", .5);
  k = mp.contains("steer_raw_rad")
          ? curvature(p, clip(num(mp, "steer_raw_rad"), -p.steer_limit,
                              p.steer_limit))
          : kmax * d.steering;
  bool detected = s.t >= detect;
  if (detected) {
    if (!std::isfinite(v_detect))
      v_detect = std::abs(s.vx);
    double vcap = std::max(num(mp, "v_limit_kmh", 60) / 3.6,
                           v_detect - num(mp, "decel_max", 3) * (s.t - detect));
    v = clip(v, -vcap, vcap);
  }
  auto c = commands(p, ideal_motion(k, v));
  if (!detected)
    return c;
  int partner = wheel ^ 1;
  if (str(mp, "fault_kind", "stuck") == "free")
    c.delta_cmd[partner] *= num(mp, "front_gain", 2);
  else {
    double e = delta_s - c.delta_cmd[wheel];
    c.delta_cmd[wheel] = delta_s;
    c.delta_cmd[partner] -= e;
  }
  double er = v * k - s.yaw_rate,
         h = std::isfinite(prev_t) ? std::min(.05, s.t - prev_t) : 0;
  prev_t = s.t;
  double lim = rad(6) / std::max(ky, 1e-6);
  yaw_int = clip(yaw_int + er * h, -lim, lim);
  double fb = clip(ky * (er + .8 * yaw_int), -rad(8), rad(8));
  for (int i = 0; i < 4; i++) {
    if (i != wheel)
      c.delta_cmd[i] += i < 2 ? fb : -fb;
    c.delta_cmd[i] = clip(c.delta_cmd[i], -p.steer_limit, p.steer_limit);
  }
  return c;
}
Command Controller::follow(const Driver &d, const State &s, double dt) {
  const auto &mp = d.mode_params;
  double v = speed_command(p, d, s, dt), cruise = num(mp, "cruise_speed");
  if (std::abs(v) < .1 && cruise != 0)
    v = cruise;
  auto pts = plan.value("points", json::array()).get<std::vector<V2>>();
  if (pts.size() < 2)
    return commands(p, {v, 0, 0});
  size_t n = pts.size(), near = 0;
  double best = INFINITY;
  for (size_t i = 0; i < n; i++) {
    double q = std::pow(pts[i][0] - s.x, 2) + std::pow(pts[i][1] - s.y, 2);
    if (q < best) {
      best = q;
      near = i;
    }
  }
  bool closed = yes(plan, "closed"), ended = true;
  double speed = std::max(std::abs(s.vx), std::abs(v)),
         ld = std::min(num(mp, "lookahead_max", 14),
                       std::max(num(mp, "lookahead_min", 2.5),
                                num(mp, "lookahead_kv", .6) * speed +
                                    num(mp, "lookahead_base", 3))),
         acc = 0;
  V2 target = pts.back();
  size_t i = near;
  for (size_t z = 0; z < n; z++) {
    size_t j = i + 1;
    if (j >= n) {
      if (!closed)
        break;
      j = 0;
    }
    acc += std::hypot(pts[j][0] - pts[i][0], pts[j][1] - pts[i][1]);
    i = j;
    if (acc >= ld) {
      target = pts[i];
      ended = false;
      break;
    }
    if (closed && i == near)
      break;
  }
  double c = std::cos(s.psi), sn = std::sin(s.psi), dx = target[0] - s.x,
         dy = target[1] - s.y, ex = c * dx + sn * dy, ey = -sn * dx + c * dy,
         chord = ex * ex + ey * ey, k = chord > 1e-6 ? 2 * ey / chord : 0,
         preview = std::max(ld, speed * speed /
                                    (2 * std::max(.1, num(mp, "ax_brake", 2)))),
         km = std::abs(k);
  acc = 0;
  i = near;
  for (size_t z = 0; z < n && acc < preview; z++) {
    size_t j = closed ? (i + 1) % n : i + 1, h = closed ? (i + 2) % n : i + 2;
    if (h >= n)
      break;
    double x1 = pts[j][0] - pts[i][0], y1 = pts[j][1] - pts[i][1],
           x2 = pts[h][0] - pts[j][0], y2 = pts[h][1] - pts[j][1],
           l1 = std::hypot(x1, y1), l2 = std::hypot(x2, y2);
    if (l1 > 1e-6 && l2 > 1e-6) {
      double dh = std::atan2(y2, x2) - std::atan2(y1, x1);
      dh = std::atan2(std::sin(dh), std::cos(dh));
      km = std::max(km, std::abs(dh) / (.5 * (l1 + l2)));
    }
    acc += l1;
    i = j;
  }
  if (km > 1e-6) {
    double curve = std::sqrt(std::max(.1, num(mp, "ay_max", 3)) / km);
    v = clip(v, -curve, curve);
  }
  if (ended && !closed) {
    double dist = std::hypot(pts.back()[0] - s.x, pts.back()[1] - s.y);
    if (dist < 3)
      v *= std::max(0., dist / 3);
  }
  return commands(p, ideal_motion(k, v));
}
void apply_longitudinal(Command &c, const Driver &d, const Model &m) {
  auto &p = m.p;
  auto &s = m.s;
  double f = p.brake_bias_front * d.brake,
         r = (1 - p.brake_bias_front) * d.brake;
  if (d.handbrake)
    r = std::max(r, 1 - p.brake_bias_front);
  c.brake_cmd = {f, f, r, r};
  c.handbrake = d.handbrake;
  c.drive_torque_cmd = {};
  if (p.longitudinal_mode != "torque" || d.gear == 0 || d.brake > .001 ||
      d.handbrake)
    return;
  double demand = clip(d.throttle, 0, 1);
  if (d.mode_params.contains("speed_target_ms")) {
    double err =
        std::abs(num(d.mode_params, "speed_target_ms")) - std::abs(s.vx);
    if (err <= 0)
      return;
    double force =
        p.mass * 1.5 * err + std::abs(sim4wis::resistance(m.bp.data(), s.vx));
    demand = clip(
        force / std::max(4 * p.motor_torque_max / std::max(p.tire_radius, 1e-6),
                         1e-6),
        0, 1);
  }
  double total = demand * 4 * p.motor_torque_max,
         frac = clip(p.drive_front_ratio, 0, 1);
  c.drive_torque_cmd = {frac * total / 2, frac * total / 2,
                        (1 - frac) * total / 2, (1 - frac) * total / 2};
  if (p.diff_type == "open" && s.grip_valid &&
      *std::max_element(s.grip_capacity.begin(), s.grip_capacity.end()) > 0)
    for (int i : {0, 2}) {
      double shared =
          std::min({c.drive_torque_cmd[i], c.drive_torque_cmd[i + 1],
                    p.tire_radius *
                        std::min(s.grip_capacity[i], s.grip_capacity[i + 1])});
      c.drive_torque_cmd[i] = c.drive_torque_cmd[i + 1] = shared;
    }
  double power = 0;
  for (int i = 0; i < 4; i++) {
    c.drive_torque_cmd[i] = std::min(c.drive_torque_cmd[i], p.motor_torque_max);
    power += c.drive_torque_cmd[i] * std::max(std::abs(s.wheel_omega[i]), .5);
  }
  double factor = p.motor_power_max > 0 && power > p.motor_power_max
                      ? p.motor_power_max / power
                      : 1;
  double ceiling = INFINITY;
  if (p.driver_speed_limit > 0)
    ceiling = p.driver_speed_limit;
  if (d.gear < 0)
    ceiling = std::min(ceiling, p.v_max_reverse);
  if (std::isfinite(ceiling)) {
    double band = std::max(.1 * ceiling, 1.);
    factor *= clip(1 - (std::abs(s.vx) - (ceiling - band)) / band, 0, 1);
  }
  if (d.gear < 0)
    factor = -factor;
  for (auto &t : c.drive_torque_cmd)
    t *= factor;
}
} // namespace fourwis
