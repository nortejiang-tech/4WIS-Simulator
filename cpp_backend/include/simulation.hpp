#pragma once
#include "build_info.hpp"
#include "controller.hpp"
#include "fault.hpp"
#include "path.hpp"
namespace fourwis {
inline json engine_info(const std::string &model = "") {
  return {{"requested", "native"},
          {"native_available", true},
          {"numerical_core", "cpp17"},
          {"body_core", model == "kinematic" ? "cpp17_kinematic" : "cpp17"},
          {"abi", 1},
          {"source_fingerprint", FOURWIS_SOURCE_FINGERPRINT},
          {"source_revision", FOURWIS_SOURCE_REVISION},
          {"service_runtime", "cpp17"},
          {"body_fallback_reason", nullptr},
          {"integration",
           "RK4 body + nonlinear backward-Euler wheel spin; partitioned"}};
}
struct Run {
  std::vector<double> t;
  std::map<std::string, std::vector<double>> channels;
  double duration_s = 0;
  int n_steps = 0;
  json meta = json::object();
  json output() const {
    return {{"t", t},
            {"channels", channels},
            {"duration_s", duration_s},
            {"n_steps", n_steps},
            {"meta", meta}};
  }
};
json normalize_experiment(json exp);
std::map<std::string, double> sample(const Model &m, const Command &c,
                                     const Driver &d, bool all = false);
class Simulation {
public:
  json experiment;
  Model model;
  Controller controller;
  Scene scene;
  Faults faults;
  Driver driver;
  Command last;
  Simulation(json exp, const VehicleParams &p)
      : experiment(normalize_experiment(exp)),
        model(p, str(experiment, "model_type")),
        controller(str(experiment, "strategy"), p) {
    if (experiment["scene"].is_object())
      merge(scene.definition, experiment["scene"]);
    driver.mode_params = experiment["mode_params"];
    faults.replace(experiment["faults"],
                   experiment["fault_seed"].get<uint32_t>(), &model);
    if (experiment["path"].is_object()) {
      auto path = experiment["path"];
      if (!str(path, "template").empty())
        controller.plan = plan_template(str(path, "template"),
                                        path.value("params", json::object()));
      else if (path.contains("waypoints") && !path["waypoints"].empty())
        controller.plan = plan_waypoints(path["waypoints"].get<Points>(),
                                         yes(path, "closed"));
    }
  }
  void advance(double dt) {
    last = controller.compute(driver, model.s, dt);
    apply_longitudinal(last, driver, model);
    double t0 = model.s.t, target = t0 + dt;
    Command c = last;
    V4 begin = model.steer->ramp_start(c.delta_cmd);
    while (model.s.t < target - 1e-12) {
      double end = faults.boundary(model.s.t, target);
      if (model.kind != "kinematic")
        end = std::min(end, model.s.t + .00125);
      double h = end - model.s.t;
      Command inner = c;
      for (int i = 0; i < 4; i++)
        inner.delta_cmd[i] =
            begin[i] + (c.delta_cmd[i] - begin[i]) * (end - t0) / dt;
      last = faults.command(inner, model, h);
      model.step(h, last, scene);
      faults.sample(model);
    }
  }
  Run run();
};
} // namespace fourwis
