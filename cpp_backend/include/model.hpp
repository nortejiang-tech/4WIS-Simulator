#pragma once
#include "core.hpp"
#include "state.hpp"
#include "steering.hpp"
namespace fourwis {
struct Servo {
  double integral = 0, prev = nan, rate = 0;
  double update(double actual, double cmd, double dt, bool brake,
                const VehicleParams &p) {
    if (brake) {
      integral = rate = 0;
      prev = cmd;
      return 0;
    }
    double err = cmd - actual,
           raw = std::isfinite(prev) && dt > 1e-9 ? (cmd - prev) / dt : 0;
    prev = cmd;
    rate += dt / (.12 + dt) * (raw - rate);
    double u =
        p.servo_kp * err + p.servo_ki * integral +
        (p.wheel_inertia + p.mass * p.tire_radius * p.tire_radius / 4) * rate;
    double t = clip(u, -p.motor_torque_max, p.motor_torque_max);
    if (t == u || err * u < 0)
      integral = clip(integral + err * dt, -50, 50);
    return t;
  }
};
inline V4 vertical_loads(const VehicleParams &p, double ax, double ay) {
  double eps = p.suspension.roll_stiffness_front_frac;
  if (eps <= 0)
    eps = (p.wheelbase - p.cg_to_front) / p.wheelbase;
  eps = clip(eps, 0, 1);
  double front =
             clip(p.mass * 9.81 * (p.wheelbase - p.cg_to_front) / p.wheelbase -
                      p.mass * ax * p.cg_height / p.wheelbase,
                  0, p.mass * 9.81),
         rear = p.mass * 9.81 - front;
  double df = clip(eps * p.mass * ay * p.cg_height / p.track_front, -front / 2,
                   front / 2),
         dr = clip((1 - eps) * p.mass * ay * p.cg_height / p.track_rear,
                   -rear / 2, rear / 2);
  return {front / 2 - df, front / 2 + df, rear / 2 - dr, rear / 2 + dr};
}
inline void pose(State &s, double u, double v, double r, double dt) {
  double h = .5 * r * dt,
         q = std::abs(h) > 1e-8 ? std::sin(h) / h : 1 - h * h / 6,
         c = std::cos(s.psi + h), si = std::sin(s.psi + h);
  s.x += dt * q * (u * c - v * si);
  s.y += dt * q * (u * si + v * c);
  s.psi += 2 * h;
}
inline double peak_x(double c, double e) {
  if (c <= 1e-6)
    return INFINITY;
  double target = std::tan(pi / (2 * c)), x = std::max(target, .1);
  for (int i = 0; i < 64; i++) {
    double y = x - e * (x - std::atan(x)), dy = 1 - e * (1 - 1 / (1 + x * x));
    if (std::abs(dy) < 1e-12)
      break;
    double step = (y - target) / dy;
    x -= step;
    if (std::abs(step) < 1e-14)
      break;
  }
  return std::abs(x);
}
std::vector<double> pack_kc(const VehicleParams &p);
class Model {
public:
  VehicleParams p;
  std::string kind;
  State s;
  Wheels wp;
  V4 act{}, rate{}, fzbase{}, fzstatic{}, lag{}, brakef{}, fx{}, fy{}, mz{},
      alpha{}, kappa{}, kc_fx{}, kc_fy{}, kc_fz{}, kc_mz{}, kc_camber{},
      kc_toe{}, toe{}, scale{};
  std::array<Servo, 4> servos{};
  std::array<double, 19> bp{};
  std::array<double, 11> tp{};
  std::array<double, 20> sp{};
  std::array<double, 24> y{};
  std::vector<double> kcp;
  sim4wis::KC kc;
  json steering_channels = json::object();
  double peakx = 0, peaky = 0;
  std::shared_ptr<Steering> steer;
  Model(const VehicleParams &params, const std::string &model)
      : p(params), kind(model), wp(wheels(p)), kcp(pack_kc(p)),
        kc(kcp.data(), kcp.size()) {
    if (kind != "kinematic" && kind != "simplified_dynamic" &&
        kind != "multibody")
      throw Error(422, "unknown model");
    bp = {p.mass,
          p.inertia_z,
          p.wheelbase,
          p.cg_to_front,
          p.tire_radius,
          p.rolling_resistance_coeff,
          p.air_density,
          p.drag_coeff_cd,
          p.frontal_area,
          p.camber_thrust_coeff,
          p.suspension.camber,
          p.tire_c_alpha,
          double(p.tire_load_sensitivity_time_domain),
          p.tire_load_sensitivity_exp,
          p.tire_mu_load_sensitivity,
          p.tire_relax_length,
          p.steer_limit,
          p.track_front,
          p.track_rear};
    tp = {p.tire_model == "pacejka" ? 1. : 0.,
          p.tire_c_alpha,
          p.tire_c_kappa,
          p.tire_t_pneumatic,
          p.tire_load_sensitivity_time_domain ? p.mass * 9.80665 / 4 : 0.,
          p.tire_load_sensitivity_time_domain ? p.tire_load_sensitivity_exp
                                              : 0.,
          p.tire_cx,
          p.tire_cy,
          p.tire_ex,
          p.tire_ey,
          .2};
    peakx = peak_x(p.tire_cx, p.tire_ex);
    peaky = peak_x(p.tire_cy, p.tire_ey);
    toe = {-p.static_toe_front, p.static_toe_front, -p.static_toe_rear,
           p.static_toe_rear};
    scale = {p.tire_c_alpha_front_scale, p.tire_c_alpha_front_scale,
             p.tire_c_alpha_rear_scale, p.tire_c_alpha_rear_scale};
    double ms = p.mass - 4 * p.unsprung_mass,
           eps = clip(p.suspension.roll_stiffness_front_frac, 0, 1),
           arb = p.suspension.anti_roll_rate;
    sp = {
        p.mass,
        p.unsprung_mass,
        ms,
        p.cg_height,
        p.suspension.spring_rate,
        p.suspension.damper_rate,
        p.tire_vertical_stiffness,
        eps > 0 ? arb * eps : 0,
        eps > 0 ? arb * (1 - eps) : 0,
        eps > 0 ? 0 : arb,
        ms *
            (std::pow(std::max(p.track_front, p.track_rear), 2) +
             4 * p.cg_height * p.cg_height) /
            12,
        ms *
            (2.25 * p.wheelbase * p.wheelbase + 4 * p.cg_height * p.cg_height) /
            12,
        p.mass * (p.wheelbase / 2 - p.cg_to_front) / ms,
        .5 * p.air_density * p.frontal_area,
        p.aero_lift_coeff_front,
        p.aero_lift_coeff_rear,
        0,
        0,
        0,
        0};
    fzstatic = vertical_loads(p, 0, 0);
    for (int i = 0; i < 4; i++)
      sp[16 + i] = fzstatic[i] - p.unsprung_mass * 9.81;
    reset();
  }
  void reset() {
    s = State{};
    s.wheel_pos_body = wp;
    s.fz = fzstatic;
    fzbase = fzstatic;
    kc_fz = fzstatic;
    act = {};
    rate = {};
    lag = {};
    brakef = {};
    fx = {};
    fy = {};
    mz = {};
    alpha = {};
    kappa = {};
    kc_fx = {};
    kc_fy = {};
    kc_mz = {};
    kc_camber = {};
    kc_toe = {};
    servos = {};
    y = {};
    steering_channels = json::object();
    steer = std::make_shared<Steering>(p);
  }
  void steering(const Command &cmd, double dt);
  void step(double dt, const Command &cmd, const Scene &scene);
  void derived();
  void kin(double u, double v, double r, const V4 &delta, V4 &vxw,
           V4 &al) const {
    for (int i = 0; i < 4; i++) {
      double c = std::cos(delta[i]), sn = std::sin(delta[i]),
             vx = u - r * wp[i][1], vy = v + r * wp[i][0];
      vxw[i] = c * vx + sn * vy;
      al[i] = std::atan2(-sn * vx + c * vy, std::max(std::abs(vxw[i]), .5));
    }
  }
  void freeze(std::array<double, 60> &f, const V4 &delta, const V4 &mu,
              const V4 &road, const V4 &fz) const {
    for (int i = 0; i < 4; i++) {
      double a[] = {wp[i][0],     wp[i][1],         delta[i], mu[i],
                    fz[i],        s.wheel_omega[i], scale[i], lag[i],
                    kc_camber[i], toe[i],           road[i],  kc_fx[i],
                    kc_fy[i],     kc_fz[i],         kc_mz[i]};
      std::copy(a, a + 15, f.begin() + i * 15);
    }
  }
};
} // namespace fourwis
