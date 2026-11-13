#include "metrics.hpp"
#include <Eigen/Dense>
#include <complex>
namespace fourwis {
namespace {
using Vec = std::vector<double>;
double mean(const Vec &x) {
  return x.empty() ? nan : std::accumulate(x.begin(), x.end(), 0.) / x.size();
}
double median(Vec x) {
  if (x.empty())
    return nan;
  std::sort(x.begin(), x.end());
  size_t n = x.size();
  return n % 2 ? x[n / 2] : (x[n / 2 - 1] + x[n / 2]) / 2;
}
Vec slice(const Vec &v, size_t first, size_t last = SIZE_MAX) {
  return {v.begin() + std::min(first, v.size()),
          v.begin() + std::min(last, v.size())};
}
Vec difference(const Vec &v) {
  Vec d;
  for (size_t i = 1; i < v.size(); i++)
    d.push_back(v[i] - v[i - 1]);
  return d;
}
double amplitude(const Vec &v) {
  double m = mean(v), a = 0;
  for (double x : v)
    a = std::max(a, std::abs(x - m));
  return a;
}
bool finite(const Vec &v) {
  return std::all_of(v.begin(), v.end(),
                     [](double x) { return std::isfinite(x); });
}
const Vec &channel(const Run &r, const std::string &name) {
  auto it = r.channels.find(name);
  if (it == r.channels.end() || it->second.size() != r.t.size())
    throw Error(422, "required channel missing or length mismatch: " + name);
  return it->second;
}
Vec gradient(const Vec &x, const Vec &t) {
  Vec d(x.size());
  if (x.size() < 2)
    throw Error(422, "trace too short for gradient");
  d.front() = (x[1] - x[0]) / (t[1] - t[0]);
  d.back() = (x.back() - x[x.size() - 2]) / (t.back() - t[t.size() - 2]);
  for (size_t i = 1; i + 1 < x.size(); i++) {
    double a = t[i] - t[i - 1], b = t[i + 1] - t[i];
    d[i] = -b / (a * (a + b)) * x[i - 1] + (b - a) / (a * b) * x[i] +
           a / (b * (a + b)) * x[i + 1];
  }
  return d;
}
V2 sine_fit(const Vec &t, const Vec &x, double f) {
  Eigen::MatrixXd A(t.size(), 3);
  Eigen::VectorXd b(x.size());
  for (size_t i = 0; i < t.size(); i++) {
    A(i, 0) = std::sin(2 * pi * f * t[i]);
    A(i, 1) = std::cos(2 * pi * f * t[i]);
    A(i, 2) = 1;
    b(i) = x[i];
  }
  auto v = A.colPivHouseholderQr().solve(b).eval();
  return {std::hypot(v[0], v[1]), std::atan2(v[1], v[0])};
}
V2 branch_fit(const Vec &x, const Vec &y, const Vec &rate, double window,
              bool up) {
  std::vector<size_t> ids;
  for (size_t i = 0; i < x.size(); i++)
    if ((up ? rate[i] > 0 : rate[i] < 0) && std::abs(x[i]) <= window)
      ids.push_back(i);
  if (ids.size() < 4)
    throw Error(422, "too few samples near zero to fit hysteresis branch");
  Eigen::MatrixXd A(ids.size(), 2);
  Eigen::VectorXd b(ids.size());
  for (size_t i = 0; i < ids.size(); i++) {
    A(i, 0) = x[ids[i]];
    A(i, 1) = 1;
    b(i) = y[ids[i]];
  }
  auto c = A.colPivHouseholderQr().solve(b).eval();
  return {c[0], c[1]};
}
V2 loop_fit(const Vec &x, const Vec &y, const Vec &rate, double window) {
  auto a = branch_fit(x, y, rate, window, true),
       b = branch_fit(x, y, rate, window, false);
  return {(a[0] + b[0]) / 2, (a[1] + b[1]) / 2};
}
double loop_width(const Vec &x, const Vec &y, const Vec &rate) {
  Vec up, down;
  for (size_t i = 0; i + 1 < x.size(); i++) {
    double a = x[i], b = x[i + 1];
    if (a == b || (a < 0) == (b < 0))
      continue;
    double v = y[i] - a / (b - a) * (y[i + 1] - y[i]);
    ((rate[i] + rate[i + 1]) * .5 > 0 ? up : down).push_back(v);
  }
  if (up.empty() || down.empty())
    throw Error(422,
                "hysteresis loop has no complete rising and falling crossing");
  return std::abs(mean(up) - mean(down));
}
json tracking_step(const Run &r) {
  if (r.t.size() < 20)
    throw Error(422, "trace too short for a step analysis");
  double dt = median(difference(r.t));
  json out = json::object();
  for (auto w : {"fl", "rl"}) {
    auto &cmd = channel(r, std::string("delta_cmd_") + w);
    auto &resp = channel(r, std::string("delta_") + w);
    double c0 = cmd.front(), end = cmd.back(), amp = end - c0;
    size_t step = cmd.size();
    for (size_t i = 0; i < cmd.size(); i++)
      if (std::abs(cmd[i] - c0) > std::max(.05 * std::abs(amp), 1e-12)) {
        step = i;
        break;
      }
    if (step == cmd.size())
      throw Error(422, "commanded channel never steps");
    if (step < 10 || cmd.size() - step < size_t(.5 / std::max(dt, 1e-9)))
      throw Error(422, "not enough samples around the step");
    for (size_t i = step; i < cmd.size(); i++)
      if (std::abs(cmd[i] - end) > .01 * std::abs(amp) + 1e-12)
        throw Error(422,
                    "command drifts after the step; not a step-response run");
    Vec rb, cb;
    for (size_t i = 0; i < resp.size(); i++) {
      rb.push_back(resp[i] - resp[step - 1]);
      cb.push_back(cmd[i] - c0);
    }
    size_t ss =
        std::min(resp.size(), std::max(size_t(resp.size() * .2), size_t(8)));
    double rise = 0, over = 0, settle = 0, err = 0, peak = 0;
    if (std::abs(amp) < .002) {
      for (size_t i = resp.size() - ss; i < resp.size(); i++)
        err += rb[i] - cb[i];
      err /= ss;
      for (size_t i = step; i < resp.size(); i++)
        peak = std::max(peak, std::abs(rb[i] - cb[i]));
    } else {
      size_t i10 = step, i90 = step, last_out = step;
      bool got10 = false, got90 = false, outside = false;
      double top = -INFINITY;
      for (size_t i = 0; i < rb.size(); i++) {
        double y = rb[i] / amp;
        if (!got10 && y >= .1) {
          i10 = i;
          got10 = true;
        }
        if (!got90 && y >= .9) {
          i90 = i;
          got90 = true;
        }
        if (i >= step)
          top = std::max(top, y);
        if (std::abs(y - 1) > .02) {
          last_out = i + 1;
          outside = true;
        }
      }
      rise = std::max((double(i90) - double(i10)) * dt, 0.);
      over = std::max(top - 1, 0.) * 100;
      settle = (double(outside ? last_out : step) - double(step)) * dt;
      err = mean(slice(resp, resp.size() - ss)) - resp[step - 1] - amp;
      for (size_t i = i90; i < rb.size(); i++)
        peak = std::max(peak, std::abs(rb[i] - cb[i]));
    }
    std::string suffix = "_" + std::string(w);
    out["trk_rise_s" + suffix] = rise;
    out["trk_overshoot_pct" + suffix] = over;
    out["trk_settle_s" + suffix] = settle;
    out["trk_ss_err_rad" + suffix] = err;
    out["trk_peak_dev_rad" + suffix] = peak;
  }
  return out;
}
json disturbance(const Run &r) {
  if (r.t.size() < 200)
    throw Error(422, "trace too short for a disturbance analysis");
  auto &rack = channel(r, "rack_force_fl");
  auto &cmd = channel(r, "delta_cmd_fl");
  auto &resp = channel(r, "delta_fl");
  double dt = median(difference(r.t));
  size_t w = size_t(std::max(std::nearbyint(.1 / dt), 2.));
  if (w >= rack.size())
    throw Error(422, "disturbance window exceeds trace");
  double range = *std::max_element(rack.begin(), rack.end()) -
                 *std::min_element(rack.begin(), rack.end());
  size_t start = 0;
  double step = -INFINITY;
  for (size_t i = 0; i + w < rack.size(); i++) {
    double v = std::abs(rack[i + w] - rack[i]);
    if (v > step) {
      step = v;
      start = i;
    }
  }
  if (range < 1 || step < .25 * range || step < 100)
    throw Error(422, "no significant rack-force step for disturbance analysis");
  double peak = 0;
  size_t last = 0;
  bool outside = false;
  for (size_t i = start; i < resp.size(); i++) {
    double dev = std::abs(resp[i] - cmd[i]);
    peak = std::max(peak, dev);
    if (dev > .01) {
      outside = true;
      last = i - start;
    }
  }
  json recover = std::abs(resp.back() - cmd.back()) > .01
                     ? json()
                     : json(outside ? (last + 1) * dt : 0.);
  return {{"trk_dist_onset_s", r.t[start + w / 2]},
          {"trk_dist_peak_dev_rad", peak},
          {"trk_dist_recover_s", recover},
          {"trk_dist_rack_step_n", step}};
}
json tracking_sweep(const Run &r) {
  if (r.t.size() < 100)
    throw Error(422, "trace too short for sweep analysis");
  auto &cmd = channel(r, "delta_cmd_fl");
  auto &resp = channel(r, "delta_fl");
  double dt = median(difference(r.t)), center = mean(cmd);
  Vec ct;
  for (size_t i = 0; i + 1 < cmd.size(); i++) {
    double a = cmd[i] - center, b = cmd[i + 1] - center;
    if (std::signbit(a) != std::signbit(b))
      ct.push_back(r.t[i] + std::abs(a) /
                                std::max(std::abs(a) + std::abs(b), 1e-12) *
                                (r.t[i + 1] - r.t[i]));
  }
  if (ct.size() < 6)
    throw Error(422, "too few zero crossings for sweep");
  Vec freq;
  for (size_t i = 1; i < ct.size(); i++)
    freq.push_back(1 / (2 * (ct[i] - ct[i - 1])));
  std::vector<size_t> bounds{0};
  for (size_t i = 1; i < freq.size(); i++)
    if (std::abs(freq[i] - freq[i - 1]) >
        .25 * std::max(std::abs(freq[i - 1]), 1e-9))
      bounds.push_back(i);
  bounds.push_back(freq.size());
  if (bounds.size() < 3)
    throw Error(422, "command frequency is not stepped");
  std::map<double, V2> points;
  for (size_t k = 0; k + 1 < bounds.size(); k++) {
    size_t a = bounds[k], b = bounds[k + 1];
    if (b - a < 3)
      continue;
    size_t i0 = std::lower_bound(r.t.begin(), r.t.end(), ct[a]) - r.t.begin(),
           i1 = std::lower_bound(r.t.begin(), r.t.end(),
                                 ct[std::min(b, ct.size() - 1)]) -
                r.t.begin();
    double f =
        median(b - a > 3 ? slice(freq, a + 1, b - 1) : slice(freq, a, b));
    if (i1 - i0 < size_t(2 / std::max(f, 1e-6) / dt))
      throw Error(422,
                  "frequency segment contains fewer than two full periods");
    auto c = sine_fit(slice(r.t, i0, i1), slice(cmd, i0, i1), f),
         v = sine_fit(slice(r.t, i0, i1), slice(resp, i0, i1), f);
    if (c[0] < 1e-6)
      throw Error(422, "near-zero command amplitude");
    points[std::nearbyint(f * 1e4) / 1e4] = {v[0] / c[0],
                                             deg(wrap(c[1] - v[1]))};
  }
  if (points.size() < 2)
    throw Error(422, "fewer than two valid frequency points");
  json out = json::object();
  for (auto pair : std::vector<std::pair<double, std::string>>{
           {.5, "0_5"}, {1, "1"}, {2, "2"}, {5, "5"}, {10, "10"}}) {
    for (auto &v : points)
      if (std::abs(v.first - pair.first) <= .05 * std::max(pair.first, 1e-9)) {
        out["trk_amp_ratio_" + pair.second + "hz"] = v.second[0];
        if (pair.first == 2)
          out["trk_phase_lag_2hz"] = v.second[1];
        break;
      }
  }
  auto prev = points.begin();
  for (auto i = std::next(prev); i != points.end(); ++i) {
    if (prev->second[0] >= .707 && i->second[0] < .707) {
      double fraction =
          (std::log(prev->second[0]) - std::log(.707)) /
          std::max(std::log(prev->second[0]) - std::log(i->second[0]), 1e-12);
      out["trk_bw_hz"] =
          std::exp(std::log(prev->first) +
                   fraction * (std::log(i->first) - std::log(prev->first)));
      break;
    }
    prev = i;
  }
  return out;
}
json weave(const Run &r) {
  auto &active = channel(r, "steer_plant_active");
  for (double x : active)
    if (!std::isfinite(x) || x <= .5)
      throw Error(422, "steering plant was not active throughout this run");
  Vec t = r.t, angle = channel(r, "steer_hand_angle"),
      torque = channel(r, "steer_hand_torque");
  if (t.size() < 16 || !finite(angle) || !finite(torque))
    throw Error(422, "invalid or insufficient hand angle/torque samples");
  double center = mean(angle), amp = amplitude(angle), h = .25 * amp;
  Vec crossings;
  int state = 0;
  for (size_t i = 0; i < angle.size(); i++) {
    double x = angle[i] - center;
    int next = state <= 0 && x > h ? 1 : state >= 0 && x < -h ? -1 : state;
    if (next != state) {
      if (state != 0) {
        size_t j = i;
        while (j > 0 && (angle[j] - center < 0) == (angle[j - 1] - center < 0))
          j--;
        double when = t.front();
        if (j > 0) {
          double a = angle[j - 1] - center, b = angle[j] - center;
          when = a == b ? t[j] : t[j - 1] - a / (b - a) * (t[j] - t[j - 1]);
        }
        crossings.push_back(when);
      }
      state = next;
    }
  }
  double cycles = crossings.size() / 2.;
  if (cycles < 2)
    throw Error(422, "at least two weave cycles are required");
  double freq =
      (crossings.size() - 1) / (2 * (crossings.back() - crossings.front()));
  size_t first =
      std::lower_bound(t.begin(), t.end(), t.front() + 1 / freq) - t.begin();
  if (t.size() - first <= 16)
    first = 0;
  t = slice(t, first);
  angle = slice(angle, first);
  torque = slice(torque, first);
  amp = amplitude(angle);
  if (amp < rad(1))
    throw Error(422, "handwheel amplitude is below one degree");
  auto rate = gradient(angle, t);
  auto fit = loop_fit(angle, torque, rate, .25 * amp);
  json out = {{"onc_torque_gradient_nm_per_deg", fit[0] * pi / 180},
              {"onc_torque_hysteresis_nm", loop_width(angle, torque, rate)},
              {"onc_torque_deadband_deg", deg(loop_width(torque, angle, rate))},
              {"onc_sw_amplitude_deg", deg(amp)},
              {"onc_frequency_hz", freq},
              {"onc_ay_amplitude_g", 0}};
  if (r.channels.count("ay")) {
    auto ay = slice(channel(r, "ay"), first);
    if (finite(ay)) {
      double ayamp = amplitude(ay);
      if (ayamp > 3.43)
        throw Error(
            422, "lateral acceleration exceeds on-centre procedure envelope");
      out["onc_ay_amplitude_g"] = ayamp / 9.81;
      if (ayamp > .2)
        try {
          auto ay_rate = gradient(ay, t);
          double window = std::min(.981, ayamp);
          auto f = loop_fit(ay, torque, ay_rate, window);
          out["onc_torque_gradient_nm_per_g"] = f[0] * 9.81;
          out["onc_torque_at_0_1g_nm"] = f[0] * .981 + f[1];
          auto a = loop_fit(ay, angle, ay_rate, window);
          out["onc_angle_gradient_deg_per_g"] = deg(a[0] * 9.81);
        } catch (const Error &) {
        }
    }
  }
  if (r.channels.count("yaw_rate")) {
    auto yaw = slice(channel(r, "yaw_rate"), first);
    if (finite(yaw)) {
      double a_mean = mean(angle), b_mean = mean(yaw);
      std::complex<double> a{}, b{};
      for (size_t i = 0; i < t.size(); i++) {
        auto basis = std::exp(std::complex<double>(0, -2 * pi * freq * t[i]));
        a += (angle[i] - a_mean) * basis;
        b += (yaw[i] - b_mean) * basis;
      }
      if (std::abs(a) >= 1e-12 && std::abs(b) >= 1e-12)
        out["onc_yaw_phase_lag_deg"] = -deg(std::arg(b / a));
    }
  }
  return out;
}
} // namespace
json procedure_metrics(const Run &r, const std::string &kind) {
  if (kind == "weave")
    return weave(r);
  if (kind == "tracking_step")
    return tracking_step(r);
  if (kind == "tracking_sweep")
    return tracking_sweep(r);
  if (kind == "tracking_disturbance")
    return disturbance(r);
  throw Error(422, "unknown procedure");
}
json collect_metrics(const json &requested, const json &base, const Run &r) {
  json values = json::object(), errors = json::object();
  std::map<std::string, json> cache;
  std::map<std::string, std::string> failed;
  for (auto &m : requested) {
    auto name = m.is_string() ? m.get<std::string>() : str(m, "name");
    try {
      if (m.is_object()) {
        values[name] = expression(str(m, "expr"), r);
        continue;
      }
      std::string proc;
      if (name.rfind("onc_", 0) == 0)
        proc = "weave";
      else if (name.rfind("trk_dist_", 0) == 0)
        proc = "tracking_disturbance";
      else if (name.rfind("trk_amp_", 0) == 0 || name == "trk_bw_hz" ||
               name == "trk_phase_lag_2hz")
        proc = "tracking_sweep";
      else if (name.rfind("trk_", 0) == 0)
        proc = "tracking_step";
      if (!proc.empty()) {
        if (failed.count(proc))
          throw Error(422, failed[proc]);
        if (!cache.count(proc)) {
          try {
            cache[proc] = procedure_metrics(r, proc);
          } catch (const std::exception &e) {
            failed[proc] = e.what();
            throw;
          }
        }
        if (!cache[proc].contains(name) || cache[proc][name].is_null())
          throw Error(422, "procedure did not produce metric; condition or "
                           "measurement range not met");
        values[name] = cache[proc][name];
      } else if (base.contains(name) && base[name].is_number())
        values[name] = base[name];
      else
        throw Error(422, "metric was not produced by this run; verify maneuver "
                         "and model capabilities");
    } catch (const std::exception &e) {
      errors[name] = e.what();
    }
  }
  return {{"values", values}, {"errors", errors}};
}
} // namespace fourwis
