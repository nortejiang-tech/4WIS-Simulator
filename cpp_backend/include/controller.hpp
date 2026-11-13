#pragma once
#include "model.hpp"
#include "plugin.hpp"
namespace fourwis {
struct Motion {
  double vx = 0, vy = 0, omega = 0;
  V2 icr{nan, nan};
  double heading = 0;
};
inline double speed_command(const VehicleParams &p, const Driver &d,
                            const State &s, double dt) {
  double b = std::max(d.brake, d.handbrake ? 1 - p.brake_bias_front : 0.);
  if (b > .001) {
    double dv = b * s.mu_avg * 9.81 * dt;
    return s.vx > 0   ? std::max(0., s.vx - dv)
           : s.vx < 0 ? std::min(0., s.vx + dv)
                      : 0;
  }
  if (d.mode_params.contains("speed_target_ms") &&
      !d.mode_params["speed_target_ms"].is_null())
    return num(d.mode_params, "speed_target_ms");
  if (d.gear == 0)
    return s.vx;
  double cap = d.gear > 0 ? p.v_max : p.v_max_reverse;
  if (p.driver_speed_limit > 0)
    cap = std::min(cap, p.driver_speed_limit);
  return (d.gear < 0 ? -1. : 1.) * d.throttle * cap;
}
inline double low_ratio(const VehicleParams &p) {
  return p.steer_ratio_low > 0
             ? p.steer_ratio_low
             : (p.steer_wheel_range / 2) / std::max(deg(p.steer_limit), 1e-6);
}
inline double gear_ratio(const VehicleParams &p, double v) {
  double low = low_ratio(p),
         high = p.steer_ratio_high > 0 ? p.steer_ratio_high : 3.5 * low;
  return low + (high - low) * v * v /
                   (v * v + p.steer_ratio_v_ref * p.steer_ratio_v_ref);
}
inline double front_angle(const VehicleParams &p, double steer, double v,
                          double mu) {
  double i = gear_ratio(p, v),
         raw = rad(steer * p.steer_wheel_range / 2 / std::max(i, 1e-6)),
         maxraw = rad(p.steer_wheel_range / 2 / std::max(i, 1e-6)),
         a = p.steer_ay_ref_frac * mu * 9.81 * std::max(p.wheelbase, 1e-6) /
             std::pow(std::max(std::abs(v), 2.), 2),
         allow = a > 0 ? std::atan(a) : p.steer_limit;
  if (allow > 1e-9 && maxraw > 1e-9) {
    double s = maxraw / allow;
    raw = allow * std::tanh(raw / allow) * std::min(s, 1.) / std::tanh(s);
  }
  return clip(raw, -p.steer_limit, p.steer_limit);
}
inline double kappa_max(const VehicleParams &p) {
  return 1 / (p.wheelbase / 2 / std::tan(p.steer_limit) +
              std::max(p.track_front, p.track_rear) / 2);
}
inline double curvature(const VehicleParams &p, double d) {
  double t = std::tan(d);
  return t / (p.wheelbase / 2 + p.track_front / 2 * std::abs(t));
}
inline double zero_beta_ratio(const VehicleParams &p, double u) {
  double a = p.cg_to_front, b = p.wheelbase - a,
         cf = 2 * p.tire_c_alpha * p.tire_c_alpha_front_scale,
         cr = 2 * p.tire_c_alpha * p.tire_c_alpha_rear_scale,
         den = a + b * p.mass * u * u / (cf * p.wheelbase);
  return std::abs(den) < 1e-9
             ? 0
             : (-b + a * p.mass * u * u / (cr * p.wheelbase)) / den;
}
inline double ref_yaw(const VehicleParams &p, double u, double d) {
  if (std::abs(u) < .001)
    return 0;
  double a = p.cg_to_front, b = p.wheelbase - a,
         den = a / u + p.mass * b * u /
                           (2 * p.tire_c_alpha * p.tire_c_alpha_front_scale *
                            p.wheelbase);
  return std::abs(den) < 1e-9 ? 0 : d / den;
}
inline Motion limit_motion(Motion q, double cap, double cg) {
  double demand = std::abs(q.omega) * std::hypot(q.vx, q.vy + cg * q.omega);
  if (std::isfinite(cap) && demand > std::max(cap, 0.) && demand >= 1e-12) {
    double k = std::sqrt(std::max(cap, 0.) / demand);
    q.vx *= k;
    q.vy *= k;
    q.omega *= k;
  }
  return q;
}
inline Command commands(const VehicleParams &p, Motion q,
                        V4 locks = {nan, nan, nan, nan}) {
  Command c;
  c.icr_target_body = q.icr;
  auto w = wheels(p);
  for (int i = 0; i < 4; i++) {
    double vx = q.vx - q.omega * w[i][1], vy = q.vy + q.omega * w[i][0], a;
    if (std::isfinite(locks[i]))
      a = locks[i];
    else {
      if (std::abs(vx) + std::abs(vy) < 1e-12)
        a = std::isfinite(q.icr[0]) && std::isfinite(q.icr[1])
                ? std::atan2(w[i][0] - q.icr[0], q.icr[1] - w[i][1])
                : q.heading;
      else
        a = std::atan2(vy, vx);
      a = halfwrap(a);
    }
    c.delta_cmd[i] = clip(a, -p.steer_limit, p.steer_limit);
    c.wheel_speed_cmd[i] =
        (vx * std::cos(c.delta_cmd[i]) + vy * std::sin(c.delta_cmd[i])) /
        p.tire_radius;
  }
  return c;
}
inline Motion ideal_motion(double k, double v) {
  return {v, 0, v * k, std::abs(k) < 1e-9 ? V2{nan, nan} : V2{0, 1 / k}};
}
class Controller {
public:
  std::shared_ptr<PluginWorker> plugin;
  json plugin_status(bool reload = false) {
    if (!plugin)
      plugin = std::make_shared<PluginWorker>(kind, p);
    return plugin->status(reload);
  }
  std::string kind;
  VehicleParams p;
  V4 js{};
  json plan = {{"points", json::array()}, {"closed", false}};
  double prev_t = nan, prev_df = nan, yaw_f = 0, ddf_f = 0, v_detect = nan,
         yaw_int = 0;
  Controller(std::string name, VehicleParams params)
      : kind(std::move(name)), p(std::move(params)) {}
  Command compute(const Driver &d, const State &s, double dt);
  Command follow(const Driver &d, const State &s, double dt);
};
void apply_longitudinal(Command &c, const Driver &d, const Model &m);
} // namespace fourwis
