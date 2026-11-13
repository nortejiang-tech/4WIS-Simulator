#include "storage.hpp"
namespace fourwis {
json kpis(const Run &r, const json &exp, bool recorded) {
  json out = json::object();
  if (r.t.empty())
    return out;
  auto at = [&](const std::string &name, size_t i) {
    auto it = r.channels.find(name);
    return it == r.channels.end() || i >= it->second.size() ? nan
                                                            : it->second[i];
  };
  auto peak = [&](const std::string &name) {
    double result = 0;
    bool found = false;
    for (size_t i = 0; i < r.t.size(); i++) {
      double x = at(name, i);
      if (std::isfinite(x)) {
        result = std::max(result, std::abs(x));
        found = true;
      }
    }
    return found ? result : nan;
  };
  double yaw = peak("yaw_rate"), vy = peak("vy");
  if (!recorded || std::isfinite(yaw))
    out["yaw_rate_peak_dps"] = deg(yaw);
  if (!recorded || std::isfinite(vy))
    out["vy_peak_kmh"] = 3.6 * vy;
  if (recorded)
    return out;
  double icr_peak = 0, icr_sum = 0, energy = 0, rack = 0, slip = 0;
  size_t count = 0;
  for (auto w : wheel_names) {
    std::string suffix = std::string("_") + w;
    rack = std::max(rack, peak("rack_force" + suffix));
    slip = std::max(slip, peak("slip_alpha" + suffix));
    for (size_t i = 0; i < r.t.size(); i++) {
      double v = at("icr_dev" + suffix, i);
      if (std::abs(at("yaw_rate", i)) > .05 && std::isfinite(v)) {
        icr_peak = std::max(icr_peak, std::abs(v));
        icr_sum += v * v;
        count++;
      }
      energy += std::abs(at("motor_torque" + suffix, i)) *
                (i ? r.t[i] - r.t[i - 1] : 0);
    }
  }
  out["icr_dev_peak_m"] = icr_peak;
  out["icr_dev_rms_m"] = count ? std::sqrt(icr_sum / count) : 0;
  out["steer_energy_nms"] = energy;
  out["rack_force_peak_n"] = rack;
  out["slip_alpha_peak_deg"] = deg(slip);
  double cursor = 0, target = nan, errors = 0;
  size_t samples = 0;
  bool first_step = false;
  for (auto &s :
       exp.value("maneuver", json::object()).value("steps", json::array())) {
    double duration = num(s, "duration"), begin = cursor;
    bool changed = s.contains("speed_kmh") && !s["speed_kmh"].is_null();
    if (changed) {
      target = num(s, "speed_kmh") / 3.6;
      begin += 2 + num(s, "speed_ramp_s");
    }
    if (std::isfinite(target))
      for (size_t i = 0; i < r.t.size(); i++)
        if (r.t[i] >= begin && r.t[i] < cursor + duration) {
          double e = at("vx", i) - target;
          errors += e * e;
          samples++;
        }
    auto st = s.value("steer", json::object());
    if (!first_step && str(st, "kind") == "step" &&
        std::abs(num(st, "amplitude")) > 1e-6) {
      first_step = true;
      double amplitude = num(st, "amplitude"),
             on = cursor + num(st, "t_step", .5);
      std::vector<double> t, y;
      for (size_t i = 0; i < r.t.size(); i++)
        if (r.t[i] >= on && r.t[i] <= cursor + duration) {
          t.push_back(r.t[i]);
          y.push_back(deg(at("yaw_rate", i)) * sign(amplitude));
        }
      if (t.size() >= 5) {
        size_t n = std::max(size_t(3), size_t(.2 * y.size()));
        double steady = std::accumulate(y.end() - n, y.end(), 0.) / n;
        if (std::abs(steady) >= 1e-6) {
          out["yaw_gain_dps"] = steady / std::abs(amplitude);
          out["yaw_overshoot_pct"] =
              std::max(0., (*std::max_element(y.begin(), y.end()) - steady) /
                               std::abs(steady) * 100);
          auto crossing = [&](double level) {
            for (size_t i = 0; i < y.size(); i++)
              if (y[i] >= level) {
                if (i == 0 || std::abs(y[i] - y[i - 1]) < 1e-12)
                  return t[i];
                return t[i - 1] + (level - y[i - 1]) / (y[i] - y[i - 1]) *
                                      (t[i] - t[i - 1]);
              }
            return nan;
          };
          double t10 = crossing(.1 * steady), t90 = crossing(.9 * steady);
          if (std::isfinite(t10) && std::isfinite(t90) && t90 >= t10)
            out["yaw_rise_time_s"] = t90 - t10;
          double settle = 0;
          for (size_t i = 0; i < y.size(); i++)
            if (std::abs(y[i] - steady) > .05 * std::abs(steady))
              settle = t[i] - t[0];
          out["yaw_settling_time_s"] = settle;
        }
      }
    }
    cursor += duration;
  }
  if (samples)
    out["speed_error_rms_kmh"] = 3.6 * std::sqrt(errors / samples);
  return out;
}
} // namespace fourwis
