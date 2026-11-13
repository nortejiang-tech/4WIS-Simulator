#include "runtime.hpp"
namespace fourwis {
static void strict_overrides(const json &data, const json &reference) {
  if (!data.is_object())
    throw Error(422, "vehicle overrides must be an object");
  for (auto i = data.begin(); i != data.end(); ++i) {
    if (!reference.contains(i.key()))
      throw Error(422, "unknown vehicle override: " + i.key());
    if (reference[i.key()].is_object())
      strict_overrides(i.value(), reference[i.key()]);
  }
}
AgentSession::AgentSession(json c, const Storage &store)
    : config(std::move(c)), id(uid()) {
  auto &exp = config["experiment"];
  exp = store.experiment(exp);
  auto model = str(exp, "model_type"), strategy = str(exp, "strategy");
  if (!std::set<std::string>{"ackermann", "ideal_ackermann", "rear_wheel_steer",
                             "crab", "zero_radius", "manual_wheel",
                             "manual_body", "follow_trajectory",
                             "fault_reconfig"}
           .count(strategy))
    throw Error(
        422, "isolated sessions support built-in structured strategies only");
  if (strategy == "follow_trajectory" && exp["path"].is_null())
    throw Error(422, "follow_trajectory requires explicit isolated path");
  double dt = num(exp, "dt");
  if (dt < .0005 || dt > .02)
    throw Error(422, "session dt must be within 0.0005..0.02 s");
  if (!exp["maneuver"]["steps"].empty())
    throw Error(422, "session control comes from step requests");
  for (auto name : {"speed_target_ms", "steer_raw_rad", "wheel_norm", "vx_frac",
                    "vy_frac", "yaw_frac"})
    if (exp["mode_params"].contains(name))
      throw Error(422, "dynamic controls belong in step.control");
  strict_overrides(exp["vehicle"]["overrides"], store.defaults);
  auto p = store.params(exp["vehicle"]);
  sim = std::make_unique<Simulation>(exp, p);
}
json AgentSession::observe() const {
  auto &s = sim->model.s;
  json tele = json::object();
  if (!record.t.empty())
    for (auto &c : record.channels)
      tele[c.first] = c.second.back();
  json wheels = json::array();
  for (int i = 0; i < 4; i++) {
    std::string name = wheel_names[i];
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return char(std::toupper(c)); });
    wheels.push_back({{"name", name}, {"delta", s.delta[i]}, {"fz", s.fz[i]}});
  }
  return {
      {"contract", "4wis.agent.v1"},
      {"session_id", id},
      {"label", str(config, "label", "Agent session")},
      {"engine", engine_info(sim->model.kind)},
      {"status", status},
      {"revision", revision},
      {"steps", steps},
      {"samples", record.t.size()},
      {"dt", num(config["experiment"], "dt")},
      {"remaining_steps", 20000 - steps},
      {"error", error},
      {"model_type", sim->model.kind},
      {"strategy", sim->controller.kind},
      {"state",
       {{"t", s.t},
        {"pose", {{"x", s.x}, {"y", s.y}, {"psi", s.psi}}},
        {"telemetry", tele},
        {"velocity", {{"vx", s.vx}, {"vy", s.vy}, {"yaw_rate", s.yaw_rate}}},
        {"accel", {{"ax", s.ax}, {"ay", s.ay}, {"valid", s.grip_valid}}},
        {"wheels", wheels}}},
      {"run_id", exported_revision == revision ? json(exported) : json()},
      {"fault_timeline", sim->faults.status()},
      {"validity",
       "internal verification only; no independent vehicle correlation"}};
}
std::optional<json> AgentSession::replay(const json &request,
                                         const std::string &digest) {
  touched = epoch();
  auto rid = str(request, "request_id");
  auto found = receipts.find(rid);
  if (found != receipts.end()) {
    if (found->second.first != digest)
      throw Error(409, "request_id already used with a different body",
                  "request_conflict");
    auto response = found->second.second;
    response["replayed"] = true;
    return response;
  }
  if (status != "ready")
    throw Error(409, "create a new session after a numerical failure",
                "session_failed");
  if (request.at("expected_revision").get<int>() != revision)
    throw Error(409, "expected current revision " + std::to_string(revision),
                "revision_conflict");
  return {};
}
void AgentSession::receipt(const json &request, const std::string &digest,
                           const json &response) {
  receipts[str(request, "request_id")] = {digest, response};
}
json AgentSession::step(const json &request) {
  auto digest = hex_sha(request.dump());
  if (auto response = replay(request, digest))
    return *response;
  int n = request.at("steps").get<int>();
  if (steps + n > 20000)
    throw Error(
        409, "session sample limit reached; export then create a new session",
        "step_limit");
  const auto &c = request.at("control");
  auto strategy = sim->controller.kind;
  auto has = [&](const char *key) {
    return c.contains(key) && !c[key].is_null();
  };
  auto unsupported = [](const std::string &msg) {
    throw Error(422, msg, "unsupported_control");
  };
  if (has("speed_target_ms") &&
      (strategy == "manual_body" || strategy == "zero_radius"))
    unsupported("strategy has no longitudinal speed_target_ms input");
  if (has("front_angle_rad") && strategy != "ideal_ackermann" &&
      strategy != "rear_wheel_steer" && strategy != "fault_reconfig")
    unsupported("front_angle_rad is unsupported by this strategy");
  if (has("wheel_norm") && strategy != "manual_wheel")
    unsupported("wheel_norm requires manual_wheel");
  if (has("body_fraction") && strategy != "manual_body")
    unsupported("body_fraction requires manual_body");
  for (auto field : {"wheel_norm", "body_fraction"})
    if (has(field))
      for (auto &v : c[field])
        if (!v.is_number() || std::abs(v.get<double>()) > 1)
          throw Error(422, "fractions must be within [-1,1]");
  Driver driver = c.get<Driver>();
  driver.mode_params = config["experiment"]["mode_params"];
  for (auto key : {"speed_target_ms", "wheel_norm"})
    if (has(key))
      driver.mode_params[key] = c[key];
  if (has("front_angle_rad"))
    driver.mode_params["steer_raw_rad"] = c["front_angle_rad"];
  if (has("body_fraction")) {
    int i = 0;
    for (auto key : {"vx_frac", "vy_frac", "yaw_frac"})
      driver.mode_params[key] = c["body_fraction"][i++];
  }
  sim->driver = driver;
  int before = steps;
  try {
    for (int i = 0; i < n; i++) {
      sim->advance(num(config["experiment"], "dt"));
      record.t.push_back(sim->model.s.t);
      for (auto &v : sample(sim->model, sim->last, sim->driver, true))
        record.channels[v.first].push_back(v.second);
      steps++;
    }
  } catch (const std::exception &e) {
    status = "failed";
    error = std::string(e.what());
  }
  revision++;
  json log = request;
  log["completed_steps"] = steps - before;
  log["status"] = status;
  command_log.push_back(log);
  auto out = observe();
  out["completed_steps"] = steps - before;
  out["replayed"] = false;
  receipt(request, digest, out);
  return out;
}
json AgentSession::schedule(const json &request) {
  auto digest = hex_sha("fault-schedule:" + request.dump());
  if (auto response = replay(request, digest))
    return *response;
  double now = sim->model.s.t;
  for (auto &fault : request.at("faults"))
    if (yes(fault, "active", true) && num(fault, "t_start") < now - 1e-10)
      throw Error(
          422, "new fault starts must be at or after current simulation time",
          "past_fault");
  sim->faults.replace(request["faults"], request.value("seed", uint32_t(0)),
                      &sim->model, now);
  config["experiment"]["faults"] = request["faults"];
  config["experiment"]["fault_seed"] = request.value("seed", 0);
  revision++;
  json log = request;
  log["operation"] = "set_fault_schedule";
  log["time"] = now;
  command_log.push_back(log);
  auto out = observe();
  out["replayed"] = false;
  receipt(request, digest, out);
  return out;
}
Run AgentSession::result() const {
  Run r = record;
  double dt = num(config["experiment"], "dt");
  r.duration_s = steps * dt;
  r.n_steps = steps;
  r.meta = {{"experiment", config["experiment"]},
            {"engine", engine_info(sim->model.kind)},
            {"source", "agent"},
            {"contract", "4wis.agent.v1"},
            {"session_id", id},
            {"resolved_vehicle", json(sim->model.p)},
            {"config_sha256", hex_sha(config.dump())},
            {"revision", revision},
            {"execution_status", status},
            {"error", error},
            {"record_hz", 1 / dt},
            {"sampling",
             "every outer integration step; events split internal integration"},
            {"fault_timeline", sim->faults.status()},
            {"command_log", command_log}};
  return r;
}
std::string AgentSession::export_run(const Storage &store) {
  if (record.t.empty())
    throw Error(409, "advance the session before exporting", "no_samples");
  if (exported_revision == revision)
    return exported;
  auto r = result();
  auto metrics = kpis(r, config["experiment"]);
  if (sim->model.kind == "kinematic")
    for (auto key :
         {"steer_energy_nms", "rack_force_peak_n", "slip_alpha_peak_deg"})
      metrics.erase(key);
  exported = store.save_run(r, str(config, "label", "Agent session"), metrics);
  exported_revision = revision;
  return exported;
}
json AgentSession::preview(int maximum) const {
  size_t n = record.t.size(), count = std::min(n, size_t(maximum));
  std::set<size_t> indices;
  if (n > 1 && count > 1)
    for (size_t i = 0; i < count; i++)
      indices.insert(size_t(
          std::nearbyint(double(i) * double(n - 1) / double(count - 1))));
  else if (n)
    indices.insert(0);
  json data = {{"t", json::array()}};
  for (auto name : {"pose_x", "pose_y", "pose_psi", "vx", "vy", "yaw_rate",
                    "delta_fl", "delta_fr", "delta_rl", "delta_rr"}) {
    data[name] = json::array();
    if (n)
      for (auto i : indices)
        data[name].push_back(record.channels.at(name)[i]);
  }
  for (auto i : indices)
    data["t"].push_back(record.t[i]);
  return {{"session_id", id},
          {"revision", revision},
          {"total_samples", n},
          {"returned_samples", indices.size()},
          {"decimated", indices.size() < n},
          {"data", data}};
}
} // namespace fourwis
