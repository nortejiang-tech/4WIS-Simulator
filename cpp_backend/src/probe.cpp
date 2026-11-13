#include "metrics.hpp"
#include "path.hpp"
#include "random.hpp"
#include "simulation.hpp"
#include <iostream>
using namespace fourwis;
int main() {
  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      auto req = json::parse(line);
      if (str(req, "action") == "metrics") {
        Run r;
        r.t = req.at("t").get<std::vector<double>>();
        for (auto i = req["channels"].begin(); i != req["channels"].end(); ++i)
          for (auto &v : i.value())
            r.channels[i.key()].push_back(v.is_null() ? fourwis::nan
                                                      : v.get<double>());
        json out;
        if (req.contains("expr"))
          out = {{"value", expression(str(req, "expr"), r)}};
        else if (req.contains("procedure"))
          out = procedure_metrics(r, str(req, "procedure"));
        else
          out = kpis(r, req.at("experiment"));
        std::cout << out.dump() << std::endl;
        continue;
      }
      if (str(req, "action") == "random") {
        Random rng(req.at("seed").get<uint64_t>());
        json vals = json::array(), raw = json::array();
        for (int i = 0; i < req.value("count", 1000); i++)
          vals.push_back(rng.normal());
        Random r2(req.at("seed").get<uint64_t>());
        for (int i = 0; i < 20; i++)
          raw.push_back(r2.next());
        auto hash = sha256(str(req, "text"));
        std::cout
            << json{{"normal", vals}, {"raw", raw}, {"sha256", hash}}.dump()
            << std::endl;
        continue;
      }
      if (str(req, "action") == "tracking") {
        TrackerControl control(str(req, "controller"),
                               req.value("kwargs", json::object()));
        json rows = json::array();
        for (auto &x : req.at("steps")) {
          auto o = control.step(num(x, "dt", .0005), num(x, "target_angle"),
                                num(x, "target_rate"), num(x, "feedback_angle"),
                                num(x, "plant_angle"), num(x, "load_torque"),
                                num(x, "speed_ms"));
          rows.push_back(json{{"angle_set", o.angle},
                              {"torque_cmd", o.torque},
                              {"diagnostics", o.diagnostics}});
        }
        std::cout << json{{"rows", rows}}.dump() << std::endl;
        continue;
      }
      if (str(req, "action") == "path") {
        auto p = req.contains("template")
                     ? plan_template(str(req, "template"),
                                     req.value("parameters", json::object()))
                     : plan_waypoints(req.at("points").get<Points>(),
                                      yes(req, "closed"));
        if (req.contains("transform")) {
          auto t = req["transform"];
          transform_plan(p, t[0], t[1], t[2]);
        }
        std::cout << p.dump() << std::endl;
        continue;
      }
      VehicleParams p = req.at("params").get<VehicleParams>();
      if (str(req, "action") == "experiment") {
        Simulation sim(req.at("experiment"), p);
        std::cout << sim.run().output().dump() << std::endl;
        continue;
      }
      Model m(p, str(req, "model", "simplified_dynamic"));
      if (str(req, "action") == "controller") {
        Controller ctrl(str(req, "strategy"), p);
        json cmds = json::array();
        for (auto &step : req.at("steps")) {
          step.at("state").get_to(m.s);
          Driver d = step.at("driver").get<Driver>();
          auto c = ctrl.compute(d, m.s, .005);
          apply_longitudinal(c, d, m);
          cmds.push_back(c);
        }
        std::cout << json{{"commands", cmds}}.dump() << std::endl;
        continue;
      }
      Scene scene;
      merge(scene.definition, req.value("scene", json::object()));
      if (req.contains("state"))
        req["state"].get_to(m.s);
      json rows = json::array();
      for (auto &step : req.at("steps")) {
        Command c = step.at("command").get<Command>();
        double dt = num(step, "dt", .00125);
        int count = int(num(step, "count", 1));
        for (int i = 0; i < count; i++) {
          m.step(dt, c, scene);
          m.s.delta_reported = m.s.delta;
          if (req.value("record_every", 1) > 0 &&
              (i + 1) % req.value("record_every", 1) == 0) {
            json row = m.s;
            row["slip_alpha"] = m.alpha;
            row["slip_kappa"] = m.kappa;
            row["tire_fx"] = m.fx;
            row["tire_fy"] = m.fy;
            if (req.value("steering_channels", false))
              row["steering_channels"] = m.steering_channels;
            rows.push_back(row);
          }
        }
      }
      std::cout << json{{"rows", rows}, {"engine", "cpp17"}}.dump()
                << std::endl;
    } catch (const std::exception &e) {
      std::cout << json{{"error", e.what()}}.dump() << std::endl;
    }
  }
}
