#include "steering.hpp"
#include <Eigen/Eigenvalues>
namespace fourwis {
Mat matrix_exp(const Mat &m, double dt) {
  double norm = m.cwiseAbs().colwise().sum().maxCoeff();
  int n = std::max(0, int(std::ceil(std::log2(std::max(dt * norm, 1e-12)))));
  Mat a = m * (dt / std::pow(2., n)), out = Mat::Identity(m.rows(), m.cols()),
      term = out;
  for (int i = 1; i <= 24; i++) {
    term = (term * a / double(i)).eval();
    out += term;
  }
  for (int i = 0; i < n; i++)
    out = (out * out).eval();
  return out;
}
Mat dare(const Mat &a, const Mat &b, const Mat &q, double r, Mat k) {
  Mat p = q;
  for (int iter = 0; iter < 40; iter++) {
    Mat ac = a - b * k;
    p = q + k.transpose() * r * k;
    for (int n = 0; n < 200; n++) {
      Mat step = ac.transpose() * p * ac;
      p += step;
      ac = (ac * ac).eval();
      if (step.cwiseAbs().maxCoeff() <
          1e-13 * std::max(1., p.cwiseAbs().maxCoeff()))
        break;
    }
    Mat next = (b.transpose() * p * a) / (r + (b.transpose() * p * b)(0, 0));
    if ((next - k).cwiseAbs().maxCoeff() < 1e-9)
      return p;
    k = next;
  }
  throw Error(422, "DARE Newton iteration did not converge");
}
static Mat pole(double j, double damping, double wn, bool positive) {
  double pp = 1.8 * wn, kd = j * (pp + 1.8 * wn) - damping,
         kp = j * (wn * wn + 1.8 * wn * pp), ki = j * pp * wn * wn;
  Mat k(1, 3);
  k << -ki, positive ? kp : -kp, kd;
  return k;
}
TrackerControl::TrackerControl(std::string n, json c)
    : name(std::move(n)), cfg(std::move(c)),
      ff(cfg.value("ff", json::object())) {
  if (ff.is_null())
    ff = json::object();
  auto blocks = ff.value("blocks", json::array());
  ff_state.resize(blocks.size());
  for (auto &x : blocks)
    if ((name == "dob" || name == "adrc") &&
        (str(x, "type") == "rack_force" || str(x, "type") == "friction"))
      throw Error(422,
                  name + " refuses double-counted disturbance feedforward");
  rate.tau = num(cfg, "deriv_tau_s", name == "pid_single" ? .01 : .005);
  acc.tau = .005;
  double j = num(cfg, "inertia", .6), d = num(cfg, "damping", 4),
         dt = num(cfg, "dt", .0005);
  if (!(j > 0 && dt > 0))
    throw Error(422, "controller inertia and dt must be positive");
  if (name == "dob") {
    json bc = {{"kp", 1253.}, {"ki", 0.}, {"kd", 43.83}};
    merge(bc, cfg.value("base", json::object()));
    bc["ff"] = ff;
    base = std::make_shared<TrackerControl>("pid_single", bc);
  } else if (name == "lqr" || name == "mpc") {
    Mat ac = Mat::Zero(4, 4);
    ac(0, 1) = name == "lqr" ? 1 : -1;
    ac(1, 2) = name == "lqr" ? -1 : 1;
    ac(2, 2) = -d / j;
    ac(2, 3) = 1 / j;
    Mat e = matrix_exp(ac, dt);
    a = e.topLeftCorner(3, 3);
    b = e.block(0, 3, 3, 1);
    Mat q = Mat::Zero(3, 3);
    q(0, 0) = num(cfg, "q_integral", name == "lqr" ? 50 : 103.5);
    q(1, 1) = num(cfg, "q_angle", name == "lqr" ? 19700 : 20000);
    q(2, 2) = num(cfg, "q_rate", name == "lqr" ? .1 : 1);
    double r = num(cfg, "r", .001);
    Mat p =
        dare(a, b, q, r, pole(j, d, name == "lqr" ? 25 : 30, name == "mpc"));
    k = (b.transpose() * p * a) / (r + (b.transpose() * p * b)(0, 0));
    if (name == "mpc") {
      int horizon = std::max(2, int(num(cfg, "horizon", 10)));
      if (horizon > 128)
        throw Error(422, "MPC horizon exceeds 128");
      Mat mm = Mat::Zero(3 * horizon, 3), nn = Mat::Zero(3 * horizon, horizon),
          ap = Mat::Identity(3, 3), qb = Mat::Zero(3 * horizon, 3 * horizon);
      std::vector<Mat> powers{ap};
      for (int i = 1; i < horizon; i++)
        powers.push_back(a * powers.back());
      for (int i = 0; i < horizon; i++) {
        mm.block(3 * i, 0, 3, 3) = powers[i];
        for (int j2 = 0; j2 < i; j2++)
          nn.block(3 * i, j2, 3, 1) = powers[i - 1 - j2] * b;
        nn.block(3 * i, i, 3, 1) = b;
        qb.block(3 * i, 3 * i, 3, 3) = q;
      }
      qb.bottomRightCorner(3, 3) += p;
      h_inv = (nn.transpose() * qb * nn + r * Mat::Identity(horizon, horizon))
                  .inverse();
      nqm = nn.transpose() * qb * mm;
      nq = nn.transpose() * qb;
    }
  } else if (name == "adrc") {
    Mat block = Mat::Zero(4, 4);
    block(0, 1) = block(1, 2) = 1;
    block(1, 3) = 1 / j;
    Mat e = matrix_exp(block, .0005);
    a = e.topLeftCorner(3, 3);
    b = e.block(0, 3, 3, 1);
    double rho = std::exp(-num(cfg, "omega_o", 400) * .0005);
    Mat at = a.transpose(), ct = Mat::Zero(3, 1);
    ct(0) = 1;
    Mat ctrb(3, 3);
    ctrb << ct, at * ct, at * at * ct;
    Mat d0 = at - rho * Mat::Identity(3, 3);
    Mat e3 = Mat::Zero(1, 3);
    e3(2) = 1;
    l = (e3 * ctrb.inverse() * d0 * d0 * d0).transpose();
    z = Vec::Zero(3);
  } else if (name == "h_inf") {
    Mat ac = Mat::Zero(3, 3), bc = Mat::Zero(3, 1), q = Mat::Zero(3, 3);
    ac(0, 1) = 1;
    ac(1, 2) = -1;
    ac(2, 2) = -d / j;
    bc(2) = 1 / j;
    q(0, 0) = num(cfg, "q_integral", 6324);
    q(1, 1) = num(cfg, "q_angle", 20000);
    q(2, 2) = num(cfg, "q_rate", 1);
    double rr = num(cfg, "r", .01), gamma = num(cfg, "gamma", 2);
    Mat h(6, 6);
    h << ac, -bc * bc.transpose() * (1 / rr - 1 / (gamma * gamma)), -q,
        -ac.transpose();
    Eigen::ComplexEigenSolver<Mat> es(h);
    if (es.info() != Eigen::Success)
      throw Error(422, "Hamiltonian eigensolve failed");
    Eigen::MatrixXcd x1(3, 3), x2(3, 3);
    int idx = 0;
    for (int i = 0; i < 6; i++) {
      double re = es.eigenvalues()[i].real();
      if (std::abs(re) <= 1e-9)
        throw Error(422, "infeasible H-infinity attenuation");
      if (re < 0) {
        if (idx == 3)
          throw Error(422, "Hamiltonian eigenspace is degenerate");
        x1.col(idx) = es.eigenvectors().col(i).head(3);
        x2.col(idx) = es.eigenvectors().col(i).tail(3);
        idx++;
      }
    }
    if (idx != 3)
      throw Error(422, "Hamiltonian eigenspace is degenerate");
    Mat p = (x2 * x1.inverse()).real();
    p = (.5 * (p + p.transpose())).eval();
    if (Eigen::SelfAdjointEigenSolver<Mat>(p).eigenvalues().minCoeff() < -1e-6)
      throw Error(422, "game Riccati solution is not positive semidefinite");
    k = bc.transpose() * p / rr;
  } else if (name != "open_loop" && name != "pid_single" &&
             name != "pid_cascade" && name != "smc" && name != "file_gain")
    throw Error(422, "unknown angle controller: " + name);
}
void TrackerControl::reset() {
  integral = integral_v = prev_target_rate = dhat = last_u = 0;
  rate = {rate.tau, nan, 0};
  acc = {.005, nan, 0};
  if (z.size())
    z.setZero();
  if (base)
    base->reset();
}
double TrackerControl::limit(double u, double fallback) const {
  double cap =
      cfg.contains("torque_limit_nm")
          ? (cfg["torque_limit_nm"].is_null() ? INFINITY
                                              : num(cfg, "torque_limit_nm"))
          : fallback;
  return clip(u, -cap, cap);
}
json TrackerControl::gains(double speed) const {
  json g = cfg;
  auto sch = cfg.value("schedule", json());
  if (!sch.is_object() || sch.empty())
    return g;
  std::vector<std::pair<double, json>> points;
  for (auto it = sch.begin(); it != sch.end(); ++it)
    points.push_back({std::stod(it.key()), it.value()});
  std::sort(points.begin(), points.end(),
            [](auto &x, auto &y) { return x.first < y.first; });
  if (points.size() < 2)
    throw Error(422, "gain schedule requires two breakpoints");
  size_t i = 0;
  while (i + 2 < points.size() && speed >= points[i + 1].first)
    i++;
  double w = clip((speed - points[i].first) /
                      std::max(points[i + 1].first - points[i].first, 1e-12),
                  0, 1);
  std::set<std::string> cols;
  for (auto &p : points)
    for (auto it = p.second.begin(); it != p.second.end(); ++it)
      cols.insert(it.key());
  for (auto &key : cols)
    g[key] = (1 - w) * num(points[i].second, key, nan) +
             w * num(points[i + 1].second, key, nan);
  return g;
}
double TrackerControl::feed(double dt, double vr, double ar, double load) {
  double out = 0;
  auto blocks = ff.value("blocks", json::array());
  for (size_t i = 0; i < blocks.size(); i++) {
    auto x = blocks[i];
    if (!yes(x, "enabled", true))
      continue;
    std::string n = str(x, "type");
    if (n == "rack_force") {
      double hz = num(x, "lowpass_hz", 0), value = load;
      if (hz > 0) {
        ff_state[i] += (load - ff_state[i]) * (1 - std::exp(-2 * pi * hz * dt));
        value = ff_state[i];
      }
      out += num(x, "gain", 1) * value;
    } else if (n == "velocity")
      out += num(x, "damping", 4) * vr +
             (yes(x, "accel_term", true) ? num(x, "inertia", .6) * ar : 0);
    else if (n == "friction")
      out += num(x, "friction_nm", .5) *
             std::tanh(vr / std::max(num(x, "eps_rad_s", .01), 1e-9));
    else
      throw Error(422, "unknown feedforward: " + n);
  }
  return out;
}
TrackResult TrackerControl::step(double dt, double target, double vr, double fb,
                                 double angle, double load, double speed) {
  dt = std::max(dt, 1e-9);
  TrackResult o;
  double e = target - fb, u = 0, j = num(cfg, "inertia", .6),
         d = num(cfg, "damping", 4);
  if (name == "open_loop") {
    double desired =
               (target - angle) *
               (1 -
                std::exp(-dt /
                         (1 / std::max(2 * pi * num(cfg, "bandwidth_hz", 10),
                                       1e-6)))),
           cap = num(cfg, "rate_limit", 6) * dt,
           step = clip(desired, -cap, cap);
    o.angle = angle + step;
    o.diagnostics = {{"rate_limited", std::abs(desired) > cap ? 1. : 0.},
                     {"deviation", target - o.angle}};
    return o;
  }
  if (name == "pid_single" || name == "pid_cascade") {
    json g = gains(speed);
    double w = rate.step(fb, dt), ta = (vr - prev_target_rate) / dt;
    prev_target_rate = vr;
    if (name == "pid_single") {
      double cap = num(cfg, "integral_clamp", 2);
      integral = clip(integral + e * dt, -cap, cap);
      u = num(g, "kp", 2165) * e + num(g, "ki", 8170) * integral -
          num(g, "kd", 47.91) * w + feed(dt, vr, ta, load);
      o.diagnostics = {
          {"error_rad", e}, {"integral", integral}, {"rate_meas", w}};
    } else {
      double cp = num(cfg, "integral_clamp_pos", 2),
             cv = num(cfg, "integral_clamp_vel", 2),
             rc = num(cfg, "rate_ref_clamp", 6);
      integral = clip(integral + e * dt, -cp, cp);
      double ref =
          clip(num(g, "kp_pos", 45.17) * e + num(g, "ki_pos", 2) * integral,
               -rc, rc);
      integral_v = clip(integral_v + (ref - w) * dt, -cv, cv);
      u = num(g, "kp_vel", 44.86) * (ref - w) +
          num(g, "ki_vel", 327.3) * integral_v + feed(dt, vr, ta, load);
      o.diagnostics = {{"error_rad", e}, {"rate_ref", ref}, {"rate_meas", w}};
    }
    u = limit(u);
    o.diagnostics["torque_cmd"] = u;
  } else if (name == "dob") {
    double ub = base->step(dt, target, vr, fb, angle, load, speed).torque,
           w = rate.step(angle, dt), ar = acc.step(w, dt),
           raw = last_u - (j * ar + d * w),
           cap = num(cfg, "d_hat_limit_nm", 60);
    dhat =
        clip(dhat + (raw - dhat) *
                        (1 - std::exp(-2 * pi * num(cfg, "q_hz", 8.09) * dt)),
             -cap, cap);
    u = limit(ub + dhat, 260);
    last_u = u;
    o.diagnostics = {{"d_hat", dhat}, {"torque_cmd", u}};
  } else if (name == "adrc") {
    double wc = num(cfg, "omega_c", 60);
    u = limit((wc * wc * (target - z[0]) - 2 * wc * z[1] - z[2]) * j +
                  feed(dt, vr, 0, load),
              260);
    o.diagnostics = {{"f_hat", z[2]}, {"theta_hat", z[0]}, {"omega_hat", z[1]}};
    z = (a * z + b * last_u + l * (angle - z[0])).eval();
    last_u = u;
  } else if (name == "smc") {
    double w = rate.step(angle, dt);
    e = target - angle;
    double sf = vr - w + num(cfg, "lam", 25) * e;
    u = limit(d * vr +
              num(cfg, "switching_gain", 60) *
                  std::tanh(sf / num(cfg, "phi", .05)) +
              feed(dt, vr, 0, load));
    o.diagnostics = {{"surface", sf}, {"error_rad", e}};
  } else if (name == "lqr" || name == "h_inf") {
    integral += e * dt;
    double w = rate.step(fb, dt);
    u = limit(-(k(0) * integral + k(1) * e + k(2) * w) + feed(dt, vr, 0, load),
              name == "h_inf" ? 260 : INFINITY);
    o.diagnostics = {{"error_rad", e}, {"integral", integral}};
    if (name == "lqr")
      o.diagnostics["torque_cmd"] = u;
  } else if (name == "mpc") {
    double w = rate.step(fb, dt);
    integral += e * dt;
    int n = int(h_inv.rows());
    Vec x(3);
    x << integral, fb, w;
    Vec ref(3 * n);
    for (int i = 0; i < n; i++)
      ref.segment(3 * i, 3) << 0, target, vr;
    Vec c = nqm * x - nq * ref, v = -h_inv * c, lambda = Vec::Zero(2 * n);
    double cap = num(cfg, "u_max", 260);
    if (v.cwiseAbs().maxCoeff() > cap + 1e-10) {
      bool ok = false;
      for (int iter = 0; iter < 4000; iter++) {
        Vec prev = lambda;
        for (int i = 0; i < 2 * n; i++) {
          int ix = i % n;
          double sg = i < n ? 1 : -1, den = h_inv(ix, ix);
          if (den < 1e-12)
            continue;
          lambda[i] = std::max(0., lambda[i] + (sg * v[ix] - cap) / den);
          v = -h_inv * (c + lambda.head(n) - lambda.tail(n));
        }
        if ((lambda - prev).cwiseAbs().maxCoeff() < 1e-10) {
          ok = true;
          break;
        }
      }
      if (!ok)
        throw Error(422, "Hildreth QP did not converge");
    }
    u = v[0] + feed(dt, vr, 0, load);
    o.diagnostics = {{"error_rad", e}, {"u0_mpc", u}};
  } else
    throw Error(422, "external controller not initialized");
  o.torque = u;
  return o;
}
double Corner::sensor() {
  double v = angle, noise = num(cfg, "sensor_noise_std_rad", 0);
  if (noise > 0)
    v += noise * rng.normal();
  double q = num(cfg, "sensor_quant_rad", .0005);
  if (q > 0)
    v = q * std::nearbyint(v / q);
  queue.push_back(v);
  double out = queue.front();
  queue.pop_front();
  return out;
}
void Corner::plant(double dt, double torque, double load) {
  double peak = num(cfg, "plant_peak_torque_nm", 260),
         j = num(cfg, "plant_inertia_kgm2", .6),
         d = num(cfg, "plant_damping_nms_per_rad", 4),
         fc = num(cfg, "plant_friction_nm", .5),
         lim = num(cfg, "plant_rate_limit_rad_s", INFINITY);
  torque = clip(torque, -peak, peak);
  auto net_force = [&](double net, double rate0) {
    if (std::abs(rate0) <= 1e-9 && std::abs(net) <= fc)
      return 0.;
    return net - fc * (std::abs(rate0) <= 1e-9 ? sign(net) : sign(rate0));
  };
  double stiffness = num(cfg, "plant_transmission_stiffness_nms_per_rad", 0);
  if (stiffness > 0) {
    double frac = clip(num(cfg, "plant_motor_inertia_fraction", .2), .05, .95),
           backlash = num(cfg, "plant_backlash_rad", 0),
           diff = motor_angle - angle,
           coupling = std::abs(diff) <= backlash
                          ? 0
                          : stiffness * (diff - std::copysign(backlash, diff));
    motor_rate = clip(motor_rate + (torque - coupling - d * frac * motor_rate) /
                                       std::max(j * frac, 1e-9) * dt,
                      -lim, lim);
    motor_angle += motor_rate * dt;
    rate += net_force(coupling - load - d * (1 - frac) * rate, rate) /
            std::max(j * (1 - frac), 1e-9) * dt;
    angle += rate * dt;
  } else {
    rate = clip(rate + net_force(torque - load - d * rate, rate) /
                           std::max(j, 1e-9) * dt,
                -lim, lim);
    angle += rate * dt;
  }
}
void Corner::step(double dt, double target, double load, double speed) {
  int n = controller.name == "open_loop"
              ? 1
              : std::max(1, int(std::ceil(dt * 2000 - 1e-12)));
  double h = dt / n,
         vr = controller.name == "open_loop" ? 0 : (target - prev_target) / dt;
  for (int k2 = 0; k2 < n; k2++) {
    double t = prev_target + (target - prev_target) * double(k2 + 1) / n;
    auto out = controller.step(h, t, vr, sensor(), angle, load, speed);
    if (std::isfinite(out.angle)) {
      angle = motor_angle = out.angle;
      rate = motor_rate = 0;
    } else
      plant(h, out.torque, load);
    diagnostics = out.diagnostics;
  }
  prev_target = target;
}
} // namespace fourwis
