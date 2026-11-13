#include "study.hpp"
namespace fourwis {
namespace {
json failure_metrics(const Run &r, const Run &ref, double time) {
  auto &x = r.channels.at("pose_x");
  auto &y = r.channels.at("pose_y");
  auto &rx = ref.channels.at("pose_x");
  auto &ry = ref.channels.at("pose_y");
  size_t first = std::lower_bound(r.t.begin(), r.t.end(), time) - r.t.begin(),
         last = std::lower_bound(r.t.begin(), r.t.end(),
                                 std::min(time + 5, r.t.back() - 1e-9)) -
                r.t.begin();
  if (first >= last)
    throw Error(422, "fault evaluation window is empty");
  std::vector<double> cross, post, dyaw;
  double maxcross = 0, ttld = INFINITY, yawpeak = 0, aypeak = 0, betapeak = 0;
  auto gradient = [](const Run &v, size_t i) {
    auto &y = v.channels.at("vy");
    if (i == 0)
      return (y[1] - y[0]) / (v.t[1] - v.t[0]);
    if (i + 1 == v.t.size())
      return (y[i] - y[i - 1]) / (v.t[i] - v.t[i - 1]);
    double a = v.t[i] - v.t[i - 1], b = v.t[i + 1] - v.t[i];
    return -b / (a * (a + b)) * y[i - 1] + (b - a) / (a * b) * y[i] +
           a / (b * (a + b)) * y[i + 1];
  };
  for (size_t i = first; i < last; i++) {
    double distance = INFINITY;
    for (size_t k = 0; k + 1 < rx.size(); k++) {
      double dx = rx[k + 1] - rx[k], dy = ry[k + 1] - ry[k],
             f = clip(((x[i] - rx[k]) * dx + (y[i] - ry[k]) * dy) /
                          std::max(dx * dx + dy * dy, 1e-12),
                      0, 1);
      double a = x[i] - (rx[k] + f * dx), b = y[i] - (ry[k] + f * dy);
      distance = std::min(distance, std::sqrt(a * a + b * b));
    }
    cross.push_back(distance);
    post.push_back(r.t[i] - time);
    maxcross = std::max(maxcross, distance);
    if (distance > .90 && !std::isfinite(ttld))
      ttld = post.back();
    double dyaw_i =
        r.channels.at("yaw_rate")[i] - ref.channels.at("yaw_rate")[i];
    dyaw.push_back(dyaw_i);
    yawpeak = std::max(yawpeak, std::abs(dyaw_i));
    double ay = gradient(r, i) +
                r.channels.at("yaw_rate")[i] * r.channels.at("vx")[i],
           reference = gradient(ref, i) + ref.channels.at("yaw_rate")[i] *
                                              ref.channels.at("vx")[i];
    aypeak = std::max(aypeak, std::abs(ay - reference));
    betapeak = std::max(
        betapeak, std::abs(std::atan2(r.channels.at("vy")[i],
                                      std::max(r.channels.at("vx")[i], .5))));
  }
  auto at = [&](double t) {
    size_t i = std::lower_bound(post.begin(), post.end(), t) - post.begin();
    return cross[std::min(i, cross.size() - 1)];
  };
  size_t i2 = std::lower_bound(r.t.begin(), r.t.end(), time + 2) - r.t.begin();
  double resid =
      std::abs(std::accumulate(dyaw.end() - std::min<size_t>(50, dyaw.size()),
                               dyaw.end(), 0.) /
               std::min<size_t>(50, dyaw.size()));
  auto &psi = r.channels.at("pose_psi");
  auto &pr = ref.channels.at("pose_psi");
  auto classification =
      !std::isfinite(ttld) && maxcross < .45 && deg(resid) < 1 && aypeak < 2
          ? "C1"
      : ttld >= 1.2 && aypeak < 5 ? "C2"
                                  : "C3";
  return {{"xtrack_react", at(1.2)},
          {"xtrack_2_5s", at(2.5)},
          {"ttld_s", ttld},
          {"dyaw_peak_dps", deg(yawpeak)},
          {"dyaw_resid_dps", deg(resid)},
          {"beta_peak_deg", deg(betapeak)},
          {"dpsi_2s_deg",
           deg(std::abs((psi[std::min(i2, psi.size() - 1)] - psi[first]) -
                        (pr[std::min(i2, pr.size() - 1)] -
                         pr[std::min(first, pr.size() - 1)])))},
          {"c_class", classification}};
}
} // namespace
json verify_golden(const Storage &store) {
  auto data = store.root / "cpp_backend/data";
  if (!fs::exists(data))
    data = store.root / "app/data";
  auto cases = read_json(data / "golden_cases.json"),
       baseline = read_json(store.root / "docs/golden_experiments.json");
  json results = json::object(), failures = json::array();
  for (auto &c : cases) {
    std::string name = str(c, "name");
    try {
      json metrics;
      size_t samples = 0;
      std::string classification;
      if (str(c, "kind") == "procedure") {
        auto spec =
            normalize_study(store, yaml_json(store.root / str(c, "path")));
        auto exp = spec["baseline"];
        exp["model_type"] = spec["model"];
        if (!str(c, "library").empty()) {
          auto twin =
              store.experiment(yaml_json(store.root / str(c, "library")));
          for (auto field : {"model_type", "strategy", "mode_params", "path",
                             "maneuver", "dt", "record_hz"})
            if (exp[field] != twin[field])
              throw Error(422, "golden procedure/library drift in " +
                                   std::string(field));
        }
        Simulation sim(exp, store.params(exp["vehicle"]));
        auto r = sim.run();
        samples = r.t.size();
        metrics = collect_metrics(spec["metrics"], kpis(r, exp), r)["values"];
      } else {
        auto exp = store.experiment(c["experiment"]),
             reference = store.experiment(c["reference"]);
        Simulation sim(exp, store.params(exp["vehicle"])),
            ref(reference, store.params(reference["vehicle"]));
        auto r = sim.run(), rr = ref.run();
        samples = r.t.size();
        metrics = failure_metrics(r, rr, num(c, "t_fault"));
        classification = str(metrics, "c_class");
        metrics.erase("c_class");
      }
      auto want = baseline["experiments"][name];
      if (samples != want["n_samples"].get<size_t>())
        failures.push_back(name + ": sample count mismatch");
      for (auto it = baseline["tolerances"][name].begin();
           it != baseline["tolerances"][name].end(); ++it) {
        auto metric = it.key();
        if (!metrics.contains(metric) || !metrics[metric].is_number() ||
            !want["kpis"].contains(metric)) {
          failures.push_back(name + "." + metric + ": missing metric");
          continue;
        }
        double expected = want["kpis"][metric], got = metrics[metric],
               allowed =
                   std::max(num(it.value(), "abs_tol"),
                            std::abs(expected) * num(it.value(), "rel_tol"));
        if (!std::isfinite(got - expected) ||
            std::abs(got - expected) > allowed)
          failures.push_back(name + "." + metric +
                             ": deviation exceeded frozen tolerance");
      }
      if (baseline["classifications"].contains(name) &&
          classification !=
              baseline["classifications"][name].get<std::string>())
        failures.push_back(name + ": classification mismatch");
      results[name] = {{"n_samples", samples},
                       {"kpis", metrics},
                       {"c_class", classification}};
    } catch (const std::exception &e) {
      failures.push_back(name + ": " + e.what());
    }
  }
  std::string tail;
  for (auto it = results.begin(); it != results.end(); ++it)
    tail += it.key() + ": " +
            std::to_string(it.value()["n_samples"].get<size_t>()) +
            " samples\n";
  for (auto &f : failures)
    tail += f.get<std::string>() + "\n";
  return {{"exit_code", failures.empty() ? 0 : 1},
          {"tail", tail},
          {"cases", results},
          {"failures", failures},
          {"engine", engine_info()}};
}
} // namespace fourwis
