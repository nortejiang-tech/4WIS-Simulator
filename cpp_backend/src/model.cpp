#include "model.hpp"
namespace fourwis {
std::vector<double> pack_kc(const VehicleParams &p) {
  std::vector<double> out(10, 0);
  std::array<std::vector<std::pair<double, double>>, 4> curves;
  bool any = false;
  for (int a = 0; a < 2; a++) {
    const char *axle = a == 0 ? "front" : "rear";
    json j =
        p.kc.is_object() ? p.kc.value(axle, json::object()) : json::object();
    auto c = j.value("compliance", json::object());
    // Any supplied K&C measurement suppresses the legacy bump-steer fallback,
    // even when that measured coordinate is not used by the current tire model.
    for (auto key : {"lat_per_fy", "long_per_fx", "lat_per_fy_mm_per_kN",
                     "long_per_fx_mm_per_kN"})
      any |= num(c, key) != 0;
    for (auto key : {"caster", "half_track", "wheelbase"})
      if (j.contains(key) && j[key].is_object())
        any |= j[key].value("jounce_m", json::array()).size() >= 2;
    auto measured = j.value("kinematics", json::object());
    for (auto key : {"caster_deg", "half_track_mm", "wheelbase_mm"})
      any |= measured.value(key, json::array()).size() >= 2;
    const char *si[] = {"toe_per_fy", "toe_per_fx", "toe_per_mz",
                        "camber_per_fy", "camber_per_fz"};
    const char *eng[] = {"toe_per_fy_deg_per_kN", "toe_per_fx_deg_per_kN",
                         "toe_per_mz_deg_per_kNm", "camber_per_fy_deg_per_kN",
                         "camber_per_fz_deg_per_kN"};
    for (int k = 0; k < 5; k++) {
      out[5 * a + k] =
          c.contains(si[k]) ? num(c, si[k]) : num(c, eng[k]) * pi / 180 / 1000;
      any |= out[5 * a + k] != 0;
    }
    for (int k = 0; k < 2; k++) {
      const char *key = k == 0 ? "toe" : "camber";
      auto kin = j.value("kinematics", json::object());
      auto x = kin.value("jounce_mm", json::array()),
           v = kin.value(k == 0 ? "toe_deg" : "camber_deg", json::array());
      double xs = .001, ys = pi / 180;
      if (j.contains(key)) {
        auto q = j[key];
        x = q.value("jounce_m", json::array());
        v = q.value("value", json::array());
        xs = ys = 1;
      }
      if (!x.empty() && !v.empty()) {
        if (x.size() != v.size())
          throw Error(422, "unpaired K&C curve");
        for (size_t i = 0; i < x.size(); i++)
          curves[2 * a + k].push_back(
              {x[i].get<double>() * xs, v[i].get<double>() * ys});
        std::stable_sort(curves[2 * a + k].begin(), curves[2 * a + k].end());
        any |= x.size() > 1;
      }
    }
  }
  if (!any && p.bump_steer_coeff != 0) {
    for (int a = 0; a < 2; a++)
      curves[2 * a] = {{-.1, -.1 * p.bump_steer_coeff},
                       {.1, .1 * p.bump_steer_coeff}};
  }
  for (auto &c : curves) {
    out.push_back(double(c.size()));
    for (auto v : c) {
      out.push_back(v.first);
      out.push_back(v.second);
    }
  }
  return out;
}
void Model::steering(const Command &cmd, double dt) {
  steer->step(cmd, dt, s, act, rate);
  s.steer_rate = rate;
  steering_channels = steer->channels;
}
void Model::derived() {
  s.vehicle_icr_body = std::abs(s.yaw_rate) < 1e-6
                           ? V2{nan, nan}
                           : V2{-s.vy / s.yaw_rate, s.vx / s.yaw_rate};
  auto &g = p.steering_geometry;
  double geom[] = {g.front_outer_x,
                   g.front_outer_y,
                   g.front_inner_x,
                   g.front_inner_y,
                   rad(g.front_rack_axis_deg),
                   g.front_rack_travel_limit,
                   g.rear_outer_x,
                   g.rear_outer_y,
                   g.rear_inner_x,
                   g.rear_inner_y,
                   rad(g.rear_rack_axis_deg),
                   g.rear_rack_travel_limit};
  double met[28];
  sim4wis::linkage(s.delta.data(), geom, p.rack_mech_efficiency,
                   p.steering_arm_length, met);
  for (int i = 0; i < 4; i++) {
    if (std::isfinite(s.vehicle_icr_body[0]) &&
        std::isfinite(s.vehicle_icr_body[1])) {
      double c = std::cos(s.delta[i]), sn = std::sin(s.delta[i]),
             d = (s.vehicle_icr_body[0] - wp[i][0]) * c +
                 (s.vehicle_icr_body[1] - wp[i][1]) * sn;
      s.wheel_icr_dev[i] = d;
      s.wheel_icr_body[i] = {s.vehicle_icr_body[0] - d * c,
                             s.vehicle_icr_body[1] - d * sn};
    } else {
      s.wheel_icr_dev[i] = nan;
      s.wheel_icr_body[i] = {nan, nan};
    }
    s.rack_force[i] = s.torque_steer[i] / std::max(std::abs(met[20 + i]), 1e-6);
    s.motor_torque_demand[i] = s.rack_force[i] *
                               std::max(p.pinion_radius, 1e-6) /
                               (std::max(p.motor_gear_ratio, 1e-6) *
                                std::max(p.rack_mech_efficiency, 1e-6));
    s.linkage_arm_tie_angle[i] = met[i];
    s.linkage_tie_rack_angle[i] = met[4 + i];
    s.linkage_efficiency[i] = met[8 + i];
  }
}
void Model::step(double dt, const Command &cmd, const Scene &scene) {
  if (!(dt > 0 && std::isfinite(dt)))
    throw Error(422, "invalid dt");
  if (kind == "kinematic") {
    s.mu_avg = scene.at(s.x, s.y).mu;
    s.delta = cmd.delta_cmd;
    s.wheel_omega = cmd.wheel_speed_cmd;
    double bx = 0, by = 0, br = 0, den = 0;
    for (int i = 0; i < 4; i++) {
      double v = s.wheel_omega[i] * p.tire_radius, x = v * std::cos(s.delta[i]),
             y = v * std::sin(s.delta[i]);
      bx += x;
      by += y;
      br += -wp[i][1] * x + wp[i][0] * y;
      den += wp[i][0] * wp[i][0] + wp[i][1] * wp[i][1];
    }
    s.vx = bx / 4;
    s.vy = by / 4;
    s.yaw_rate = br / den;
    s.ax = s.ay = 0;
    pose(s, s.vx, s.vy, s.yaw_rate, dt);
    for (int i = 0; i < 4; i++)
      s.torque_steer[i] = p.suspension.kingpin_mu * s.fz[i] *
                          std::max(p.suspension.scrub_radius, .015) *
                          sign(s.delta[i]) *
                          std::min(std::abs(s.delta[i]) / p.steer_limit, 1.);
    s.t += dt;
    derived();
    return;
  }
  V4 mu{}, road{}, fzoff{}, motor{};
  double cp = std::cos(s.psi), sn = std::sin(s.psi);
  for (int i = 0; i < 4; i++) {
    auto q = scene.at(s.x + cp * wp[i][0] - sn * wp[i][1],
                      s.y + sn * wp[i][0] + cp * wp[i][1]);
    mu[i] = q.mu;
    road[i] = q.z;
    fzoff[i] = q.fz;
    if (p.longitudinal_mode == "torque") {
      servos[i] = {};
      motor[i] = cmd.brake_cmd[i] > .02 ? 0 : cmd.drive_torque_cmd[i];
    } else
      motor[i] = servos[i].update(s.wheel_omega[i], cmd.wheel_speed_cmd[i], dt,
                                  cmd.brake_cmd[i] > .02, p);
  }
  s.mu_avg = std::accumulate(mu.begin(), mu.end(), 0.) / 4;
  steering(cmd, dt);
  std::array<double, 60> frozen{};
  V4 held = s.wheel_omega, vxw{}, al{}, fz{}, extra{};
  if (kind == "simplified_dynamic") {
    double zmean = std::accumulate(road.begin(), road.end(), 0.) / 4, zx = 0,
           zy = 0, xx = 0, yy = 0;
    for (int i = 0; i < 4; i++) {
      zx += wp[i][0] * road[i];
      zy += wp[i][1] * road[i];
      xx += wp[i][0] * wp[i][0];
      yy += wp[i][1] * wp[i][1];
    }
    freeze(frozen, act, mu, road, fzbase);
    for (int i = 0; i < 4; i++) {
      double j =
          (s.fz[i] - fzstatic[i]) / std::max(p.suspension.spring_rate, 1.) +
          road[i] - zmean - wp[i][0] * zx / xx - wp[i][1] * zy / yy;
      auto off = kc.offsets(i, j, frozen.data() + 15 * i);
      kc_toe[i] = off[0];
      kc_camber[i] = off[1];
      s.delta[i] =
          clip(clip(act[i] + toe[i], -p.steer_limit, p.steer_limit) + off[0],
               -p.steer_limit, p.steer_limit);
      fz[i] = clip(fzbase[i] + fzoff[i], 0, p.mass * 9.81);
    }
  }
  if (p.tire_relax_length > 0) {
    kin(s.vx, s.vy, s.yaw_rate, s.delta, vxw, al);
    for (int i = 0; i < 4; i++)
      lag[i] += (1 - std::exp(-std::max(std::abs(vxw[i]), .5) * dt /
                              p.tire_relax_length)) *
                (scale[i] * al[i] - lag[i]);
  }
  double e = p.wheelbase / 2 - p.cg_to_front;
  if (kind == "simplified_dynamic") {
    freeze(frozen, s.delta, mu, road, fz);
    double old[] = {s.vx, s.vy, s.yaw_rate}, out[6];
    sim4wis::rk4<3>(old, dt, out, [&](const double *a, double *b) {
      sim4wis::planar_rhs(bp.data(), tp.data(), frozen.data(), a, b);
    });
    s.vx = out[0];
    s.vy = out[1];
    s.yaw_rate = out[2];
    s.ax = out[3] - s.yaw_rate * s.vy - e * s.yaw_rate * s.yaw_rate;
    s.ay = out[4] + s.yaw_rate * s.vx + e * out[5];
    fzbase = vertical_loads(p, s.ax, s.ay);
    double qa = .5 * p.air_density * p.frontal_area * s.vx * s.vx;
    for (int i = 0; i < 4; i++) {
      fzbase[i] = std::max(fzbase[i] - .5 *
                                           (i < 2 ? p.aero_lift_coeff_front
                                                  : p.aero_lift_coeff_rear) *
                                           qa,
                           0.);
      s.fz[i] = clip(fzbase[i] + fzoff[i], 0, p.mass * 9.81);
      extra[i] = kc_camber[i];
    }
    pose(s, .5 * (old[0] + s.vx), .5 * (old[1] + s.vy),
         .5 * (old[2] + s.yaw_rate), dt);
  } else {
    y[0] = s.x;
    y[1] = s.y;
    y[2] = s.psi;
    y[3] = s.z;
    y[4] = s.roll;
    y[5] = s.pitch;
    y[10] = s.vx;
    y[11] = s.vy;
    y[12] = s.yaw_rate;
    for (int i = 0; i < 4; i++)
      y[20 + i] = s.wheel_omega[i];
    freeze(frozen, act, mu, road, fz);
    double out[48];
    sim4wis::rk4<24>(y.data(), dt, out, [&](const double *a, double *b) {
      sim4wis::multibody_rhs(bp.data(), tp.data(), frozen.data(), sp.data(), kc,
                             a, b);
    });
    std::copy(out, out + 24, y.begin());
    s.x = y[0];
    s.y = y[1];
    s.psi = y[2];
    s.z = y[3];
    s.roll = y[4];
    s.pitch = y[5];
    s.vx = y[10];
    s.vy = y[11];
    s.yaw_rate = y[12];
    s.ax = out[34] - s.yaw_rate * s.vy - e * s.yaw_rate * s.yaw_rate;
    s.ay = out[35] + s.yaw_rate * s.vx + e * out[36];
    for (int i = 0; i < 4; i++) {
      s.fz[i] = std::max(sp[16 + i] + p.unsprung_mass * 9.81 +
                             p.tire_vertical_stiffness * (road[i] - y[6 + i]),
                         0.);
      s.susp_defl[i] =
          y[6 + i] - (s.z + wp[i][0] * s.pitch + wp[i][1] * s.roll);
      auto off = kc.offsets(i, s.susp_defl[i], frozen.data() + 15 * i);
      kc_toe[i] = off[0];
      kc_camber[i] = off[1];
      extra[i] = off[1];
      s.delta[i] =
          clip(act[i] + toe[i] + off[0], -p.steer_limit, p.steer_limit);
    }
  }
  kin(s.vx, s.vy, s.yaw_rate, s.delta, vxw, al);
  double in[28], outspin[20], q[] = {dt, p.tire_radius, p.wheel_inertia, .5};
  V4 mueff{};
  for (int i = 0; i < 4; i++) {
    double fnom = std::max(p.mass * 9.80665 / 4, 1.);
    mueff[i] = p.tire_mu_load_sensitivity > 0
                   ? std::max(mu[i] * (1 - p.tire_mu_load_sensitivity *
                                               (s.fz[i] / fnom - 1)),
                              .05 * mu[i])
                   : mu[i];
    double ca = p.tire_c_alpha;
    if (p.tire_load_sensitivity_time_domain)
      ca *=
          std::pow(std::max(s.fz[i] / fnom, .001), p.tire_load_sensitivity_exp);
    ca = std::max(ca, 1.);
    double cam =
        (i % 2 == 0 ? p.suspension.camber : -p.suspension.camber) + extra[i];
    double alf = (p.tire_relax_length > 0 ? lag[i] : scale[i] * al[i]) -
                 p.camber_thrust_coeff * cam * s.fz[i] / ca;
    brakef[i] += dt / (std::max(p.brake_tau, .001) + dt) *
                 (cmd.brake_cmd[i] - brakef[i]);
    double demand = brakef[i] * p.brake_torque_max / 2,
           sgn = std::abs(held[i]) > .001 ? std::copysign(1., held[i])
                 : std::abs(vxw[i]) > .05 ? std::copysign(1., vxw[i])
                                          : 0,
           bt = -sgn * demand, cap = p.tire_radius * mueff[i] * s.fz[i];
    s.wheel_locked[i] = demand >= cap && cap > 0 && std::abs(vxw[i]) > .05;
    double a[] = {held[i], vxw[i], alf, s.fz[i], mueff[i], motor[i] + bt, bt};
    std::copy(a, a + 7, in + 7 * i);
  }
  sim4wis::spin(tp.data(), q, in, outspin);
  s.grip_valid = true;
  for (int i = 0; i < 4; i++) {
    s.wheel_omega[i] = outspin[5 * i];
    fx[i] = outspin[5 * i + 1];
    fy[i] = outspin[5 * i + 2];
    mz[i] = outspin[5 * i + 3];
    kappa[i] = outspin[5 * i + 4];
    alpha[i] = al[i];
    kc_fx[i] = fx[i];
    kc_fy[i] = fy[i];
    kc_fz[i] = s.fz[i];
    kc_mz[i] = mz[i];
    double cap = std::max(mueff[i] * s.fz[i], 0.), ca = tp[1];
    if (tp[4] > 0 && tp[5] > 0)
      ca *= std::pow(std::max(s.fz[i] / tp[4], .001), tp[5]);
    s.grip_capacity[i] = cap;
    s.grip_util[i] =
        cap > 1e-6 ? std::hypot(fx[i], fy[i]) / std::max(cap, 1e-6) : 0;
    s.grip_margin_lat[i] = std::max(
        std::sqrt(std::max(cap * cap - fx[i] * fx[i], 0.)) - std::abs(fy[i]),
        0.);
    s.grip_margin_long[i] = std::max(
        std::sqrt(std::max(cap * cap - fy[i] * fy[i], 0.)) - std::abs(fx[i]),
        0.);
    s.grip_alpha_peak[i] =
        (tp[0] == 0 ? 1. : peaky * p.tire_cy) * cap / std::max(ca, 1e-6);
    s.grip_kappa_peak[i] = (tp[0] == 0 ? 1. : peakx * p.tire_cx) * cap /
                           std::max(p.tire_c_kappa, 1e-6);
    s.grip_beyond_peak_lat[i] =
        std::abs(vxw[i]) > .5 && std::abs(al[i]) > s.grip_alpha_peak[i];
    s.grip_beyond_peak_long[i] =
        std::abs(vxw[i]) > .5 && std::abs(kappa[i]) > s.grip_kappa_peak[i];
    s.torque_steer[i] =
        fy[i] * p.tire_radius * std::tan(p.suspension.caster_angle) +
        fx[i] * p.suspension.scrub_radius * (i % 2 == 0 ? 1. : -1.) - mz[i] +
        s.fz[i] * std::sin(p.suspension.kingpin_inclination) *
            p.suspension.scrub_radius * std::sin(s.delta[i]);
  }
  s.t += dt;
  derived();
  for (double v : {s.x, s.y, s.psi, s.vx, s.vy, s.yaw_rate})
    if (!std::isfinite(v))
      throw Error(422, "non-finite physical state");
}
} // namespace fourwis
