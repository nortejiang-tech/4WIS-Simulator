#include "runtime.hpp"
#include <iomanip>
#include <sstream>
namespace fourwis {
Live::Live(Storage &storage) : store(storage) {
  sim = std::make_unique<Simulation>(
      json{{"model_type", "kinematic"}, {"strategy", "ideal_ackermann"}},
      store.defaults.get<VehicleParams>());
  record_names = store.catalog["recording_default_channels"]
                     .get<std::vector<std::string>>();
}
void Live::reset() {
  sim->model.reset();
  sim->faults.reset();
  simulation_error = nullptr;
}
void Live::replace(const VehicleParams &p, const std::string &model) {
  auto exp = sim->experiment;
  exp["model_type"] = model;
  exp["strategy"] = sim->controller.kind;
  exp["faults"] = json::array();
  for (auto &f : sim->faults.faults)
    exp["faults"].push_back(f.spec());
  exp["fault_seed"] = sim->faults.seed;
  auto candidate = std::make_unique<Simulation>(exp, p);
  candidate->scene = sim->scene;
  candidate->driver = sim->driver;
  candidate->controller.plan = plan;
  sim = std::move(candidate);
}
void Live::strategy(const std::string &name) {
  auto strategies = store.catalog["strategies"].get<std::vector<std::string>>();
  auto extra = plugin_names();
  strategies.insert(strategies.end(), extra.begin(), extra.end());
  if (std::find(strategies.begin(), strategies.end(), name) == strategies.end())
    throw Error(400, "unknown strategy: " + name);
  sim->controller = Controller(name, sim->model.p);
  sim->controller.plan = plan;
  sim->model.servos = {};
}
void Live::release() {
  json mode = sim->driver.mode_params;
  for (auto k : {"wheel_norm", "vx_frac", "vy_frac", "yaw_frac",
                 "speed_target_ms", "steer_raw_rad"})
    mode.erase(k);
  sim->driver = Driver{};
  sim->driver.mode_params = mode;
  input_owner.clear();
  input_seen = 0;
}
void Live::driver(const json &v) {
  auto &d = sim->driver;
  auto has = [&](const char *k) { return v.contains(k) && !v[k].is_null(); };
  if (has("throttle") && !has("brake")) {
    double t = clip(num(v, "throttle"), -1, 1);
    d.throttle = std::max(t, 0.);
    d.brake = std::max(-t, 0.);
  } else if (has("throttle"))
    d.throttle = clip(num(v, "throttle"), 0, 1);
  if (has("brake"))
    d.brake = clip(num(v, "brake"), 0, 1);
  if (has("gear")) {
    int gear = v["gear"].get<int>();
    if (gear < -1 || gear > 1)
      throw Error(422, "gear must be -1/0/1");
    d.gear = gear;
  }
  if (has("steering"))
    d.steering = clip(num(v, "steering"), -1, 1);
  if (has("handbrake"))
    d.handbrake = v["handbrake"].get<int>();
  if (has("mode_params"))
    d.mode_params = v["mode_params"];
}
void Live::tick() {
  if (input_seen && !script_running && epoch() - input_seen > .75)
    release();
  if (paused)
    return;
  try {
    script_tick();
    sim->controller.plan = plan;
    sim->advance(.005);
    push_counter++;
    if (recording && (full_rate || push_counter % 3 == 0))
      record_sample();
  } catch (const std::exception &e) {
    paused = true;
    simulation_error = std::string(e.what());
  }
}
json Live::script_status() const {
  return {{"running", script_running},
          {"current_action_idx", action_index},
          {"t_in_script", script_running
                              ? std::max(0., sim->model.s.t - script_start)
                              : last_elapsed},
          {"script_name", script.is_object() ? str(script, "name") : ""},
          {"loop_count", loop_count},
          {"error", script_error},
          {"clock", "simulation_time"}};
}
json Live::interaction() const {
  return {{"paused", paused},
          {"source", script_running        ? "script"
                     : input_owner.empty() ? "idle"
                                           : "manual"},
          {"script", script_status()},
          {"recording", recording_status()}};
}
void Live::load_script(const json &definition) {
  if (script_running)
    throw Error(409, "script is running — stop it first");
  json candidate = definition.value("script", definition);
  if (!candidate.is_object())
    throw Error(400, "script must be an object");
  if (!candidate.contains("name"))
    candidate["name"] = "untitled";
  if (!candidate.contains("actions"))
    candidate["actions"] = json::array();
  if (!candidate["actions"].is_array())
    throw Error(400, "actions must be an array");
  std::map<std::string, std::vector<std::string>> required = {
      {"drive", {"throttle", "steering"}},
      {"set_strategy", {"name"}},
      {"set_mode_params", {"params"}},
      {"throttle_ramp", {"from_", "to", "duration"}},
      {"steer_ramp", {"from_", "to", "duration"}},
      {"brake", {"duration"}},
      {"wait_until", {}},
      {"reset", {}},
      {"stop", {}}};
  for (auto &a : candidate["actions"]) {
    if (!a.is_object() || !a.contains("action") || !a.contains("t") ||
        !required.count(str(a, "action")))
      throw Error(400, "invalid script action");
    if (num(a, "t") < 0)
      throw Error(400, "negative action time");
    for (auto &key : required[str(a, "action")])
      if (!a.contains(key))
        throw Error(400, "missing action argument: " + key);
    for (auto key : {"duration", "distance", "speed_below", "until_t"})
      if (a.contains(key) && num(a, key) < 0)
        throw Error(400, "negative action duration");
    for (auto key : {"throttle", "steering", "from_", "to"})
      if (a.contains(key) && std::abs(num(a, key)) > 1)
        throw Error(400, "script input outside [-1,1]");
  }
  std::stable_sort(
      candidate["actions"].begin(), candidate["actions"].end(),
      [](const json &a, const json &b) { return num(a, "t") < num(b, "t"); });
  script = candidate;
  action_index = -1;
}
void Live::start_script() {
  if (script.is_null())
    throw Error(400, "no script loaded");
  if (script_running)
    return;
  script_running = true;
  script_error = nullptr;
  loop_count = 0;
  action_index = 0;
  script_start = sim->model.s.t;
  script_pose = {sim->model.s.x, sim->model.s.y};
  action_started = false;
  next_script_tick = script_start;
}
void Live::stop_script() {
  if (script_running)
    last_elapsed = std::max(0., sim->model.s.t - script_start);
  script_running = false;
  action_index = -1;
  action_started = false;
  release();
}
void Live::script_tick() {
  if (!script_running || sim->model.s.t < next_script_tick - 1e-10)
    return;
  next_script_tick = sim->model.s.t + .02;
  try {
    for (int dispatch = 0; dispatch < 10000 && script_running; dispatch++) {
      auto &actions = script["actions"];
      if (size_t(action_index) >= actions.size()) {
        loop_count++;
        if (yes(script, "loop")) {
          script_start = sim->model.s.t;
          script_pose = {sim->model.s.x, sim->model.s.y};
          action_index = 0;
          action_started = false;
          return;
        }
        stop_script();
        return;
      }
      const auto &a = actions[action_index];
      auto name = str(a, "action");
      double now = sim->model.s.t;
      if (!action_started) {
        if (now < script_start + num(a, "t") - 1e-10)
          return;
        action_start = now;
        action_started = true;
      }
      bool done = true;
      if (name == "drive")
        driver({{"throttle", num(a, "throttle")},
                {"steering", num(a, "steering")}});
      else if (name == "set_strategy")
        strategy(str(a, "name"));
      else if (name == "set_mode_params")
        driver({{"mode_params", a["params"]}});
      else if (name == "throttle_ramp" || name == "steer_ramp") {
        double duration = num(a, "duration"),
               alpha = duration <= 0
                           ? 1
                           : clip((now - action_start) / duration, 0, 1);
        driver({{name == "throttle_ramp" ? "throttle" : "steering",
                 num(a, "from_") + (num(a, "to") - num(a, "from_")) * alpha}});
        done = alpha >= 1;
      } else if (name == "brake") {
        driver({{"throttle", 0}, {"brake", 1}});
        done = now - action_start >= num(a, "duration") ||
               std::abs(sim->model.s.vx) < .05;
        if (done)
          driver({{"brake", 0}});
      } else if (name == "wait_until") {
        bool any = false;
        done = false;
        if (a.contains("until_t")) {
          any = true;
          done |= now >= num(a, "until_t");
        }
        if (a.contains("distance")) {
          any = true;
          done |=
              std::hypot(sim->model.s.x - script_pose[0],
                         sim->model.s.y - script_pose[1]) >= num(a, "distance");
        }
        if (a.contains("speed_below")) {
          any = true;
          done |= std::abs(sim->model.s.vx) < num(a, "speed_below");
        }
        if (!any)
          done = true;
      } else if (name == "reset") {
        double elapsed = now - script_start;
        reset();
        script_start = sim->model.s.t - elapsed;
        script_pose = {sim->model.s.x, sim->model.s.y};
        next_script_tick = sim->model.s.t + .02;
      } else if (name == "stop") {
        loop_count++;
        stop_script();
        return;
      }
      if (!done)
        return;
      action_index++;
      action_started = false;
    }
  } catch (const std::exception &e) {
    script_error = std::string(e.what());
    stop_script();
  }
}
json Live::recording_status() const {
  return {{"recording", recording},
          {"samples", record.t.size()},
          {"from_t", record.t.empty() ? json() : json(record.t.front())},
          {"to_t", record.t.empty() ? json() : json(record.t.back())},
          {"channels", record_names},
          {"buffer_seconds", 1800.},
          {"available_channels",
           store.catalog["agent_capabilities"]["telemetry_channels"].empty()
               ? json::array()
               : store.catalog["available_channels"]},
          {"generation", generation},
          {"dropped_samples", dropped},
          {"complete", dropped == 0 && stop_reason.is_null()},
          {"stop_reason", stop_reason}};
}
void Live::start_record(const json &config) {
  if (config.contains("channels") && !config["channels"].is_null()) {
    auto names = config["channels"].get<std::vector<std::string>>();
    auto available =
        store.catalog["available_channels"].get<std::vector<std::string>>();
    for (auto &n : names)
      if (std::find(available.begin(), available.end(), n) == available.end())
        throw Error(422, "unknown recording channel: " + n);
    record_names = names;
  }
  full_rate = yes(config, "full_rate");
  record = Run{};
  record_strategies.clear();
  saved_recording.clear();
  generation++;
  dropped = 0;
  stop_reason = nullptr;
  recording = true;
  recording_meta = {{"experiment",
                     {{"name", "realtime_recording"},
                      {"strategy", sim->controller.kind},
                      {"model_type", sim->model.kind}}},
                    {"vehicle_at_start", json(sim->model.p)},
                    {"fault_timeline_at_start", sim->faults.status()},
                    {"sampling", full_rate ? "every integration step"
                                           : "state-stream cadence"}};
}
void Live::stop_record() {
  recording = false;
  recording_meta["fault_timeline"] = sim->faults.status();
  recording_meta["execution_status"] =
      simulation_error.is_null() ? "recorded" : "failed";
  recording_meta["error"] = simulation_error;
}
void Live::clear_record() {
  if (recording)
    throw Error(409, "stop recording before clearing");
  record = Run{};
  record_strategies.clear();
  recording_meta = json::object();
  saved_recording.clear();
  generation++;
  dropped = 0;
  stop_reason = nullptr;
}
void Live::record_sample() {
  if (!recording)
    return;
  double t = sim->model.s.t;
  if (!record.t.empty() && t <= record.t.back()) {
    stop_reason = "simulation_time_reset";
    stop_record();
    return;
  }
  record.t.push_back(t);
  record_strategies.push_back(sim->controller.kind);
  auto values = sample(sim->model, sim->last, sim->driver, true);
  for (auto &n : record_names)
    record.channels[n].push_back(values.at(n));
  size_t max_samples = size_t(1800 * (full_rate ? 200 : 1 / .015)) + 60;
  if (record.t.size() > max_samples) {
    size_t n = record.t.size() - max_samples;
    dropped += int(n);
    record.t.erase(record.t.begin(), record.t.begin() + n);
    record_strategies.erase(record_strategies.begin(),
                            record_strategies.begin() + n);
    for (auto &c : record.channels)
      c.second.erase(c.second.begin(), c.second.begin() + n);
  }
}
std::string Live::save_record() {
  if (recording)
    throw Error(409, "stop recording before saving");
  if (record.t.empty())
    throw Error(409, "no recorded samples");
  if (!saved_recording.empty())
    return saved_recording;
  Run result = record;
  result.duration_s = record.t.back() - record.t.front();
  result.n_steps = int(record.t.size());
  auto meta = recording_meta;
  meta["vehicle_at_end"] = json(sim->model.p);
  meta["configuration_changed"] =
      meta["vehicle_at_end"] != meta["vehicle_at_start"];
  meta["source"] = "realtime";
  std::vector<std::string> catalog;
  for (auto &s : record_strategies) {
    auto pos = std::find(catalog.begin(), catalog.end(), s);
    if (pos == catalog.end()) {
      catalog.push_back(s);
      pos = catalog.end() - 1;
    }
    result.channels["strategy_index"].push_back(double(pos - catalog.begin()));
  }
  meta["strategy_catalog"] = catalog;
  meta["recording"] = recording_status();
  meta["record_hz"] = full_rate ? 200 : 1 / .015;
  meta["n_steps"] = full_rate ? json(record.t.size()) : json();
  meta["complete"] = dropped == 0 && stop_reason.is_null();
  meta["engine"] = engine_info(sim->model.kind);
  result.meta = meta;
  saved_recording =
      store.save_run(result, "实时驾驶 / 脚本录制", kpis(result, json(), true));
  return saved_recording;
}
std::string Live::csv(const std::vector<std::string> &names, double from,
                      double to, bool strategy_col) const {
  std::vector<std::string> selected;
  if (names.empty())
    selected = record_names;
  else
    for (auto &n : names)
      if (record.channels.count(n))
        selected.push_back(n);
  std::ostringstream s;
  s << std::setprecision(17) << "t";
  for (auto &n : selected)
    s << ',' << n;
  if (strategy_col)
    s << ",strategy";
  s << '\n';
  for (size_t i = 0; i < record.t.size(); i++)
    if (record.t[i] >= from && record.t[i] <= to) {
      s << record.t[i];
      for (auto &n : selected) {
        s << ',';
        double v = record.channels.at(n)[i];
        if (std::isfinite(v))
          s << v;
      }
      if (strategy_col)
        s << ',' << record_strategies[i];
      s << '\n';
    }
  return s.str();
}
json Live::state() const {
  auto &m = sim->model;
  auto &s = m.s;
  auto &c = sim->last;
  auto &p = m.p;
  auto &d = sim->driver;
  json out = {
      {"type", "state"},
      {"t", s.t},
      {"wall", epoch()},
      {"strategy", sim->controller.kind},
      {"interaction",
       {{"paused", paused},
        {"source", script_running        ? "script"
                   : input_owner.empty() ? "idle"
                                         : "manual"}}},
      {"fault_active", !sim->faults.enabled.empty()},
      {"fault_timeline", sim->faults.status()},
      {"simulation_error", simulation_error},
      {"driver",
       {{"throttle", d.throttle},
        {"brake", d.brake},
        {"gear", d.gear},
        {"steering", d.steering},
        {"handbrake", d.handbrake},
        {"steer_ratio", gear_ratio(p, s.vx)},
        {"steer_delta_eff_deg",
         deg(front_angle(p, d.steering, s.vx, s.mu_avg))},
        {"mode_params", d.mode_params}}},
      {"pose", {{"x", s.x}, {"y", s.y}, {"psi", s.psi}}},
      {"attitude", {{"z", s.z}, {"roll", s.roll}, {"pitch", s.pitch}}},
      {"velocity", {{"vx", s.vx}, {"vy", s.vy}, {"yaw_rate", s.yaw_rate}}},
      {"accel",
       {{"ax", s.ax},
        {"ay", s.ay},
        {"envelope", s.mu_avg * 9.81},
        {"valid", s.grip_valid}}},
      {"model_type", m.kind},
      {"icr_vehicle_body", s.vehicle_icr_body},
      {"icr_target_body", c.icr_target_body},
      {"scene", sim->scene.definition},
      {"path_version", path_version},
      {"scenario_version", scenario_version}};
  json params = {{"wheelbase", p.wheelbase},
                 {"track_front", p.track_front},
                 {"track_rear", p.track_rear},
                 {"tire_radius", p.tire_radius},
                 {"steer_limit", p.steer_limit},
                 {"v_max", p.v_max},
                 {"steer_wheel_range", p.steer_wheel_range},
                 {"steer_ratio_low", low_ratio(p)},
                 {"steer_ratio_high", p.steer_ratio_high > 0
                                          ? p.steer_ratio_high
                                          : 3.5 * low_ratio(p)},
                 {"steer_ratio_v_ref", p.steer_ratio_v_ref}};
  out["params"] = params;
  out["wheels"] = json::array();
  double cp = std::cos(s.psi), sn = std::sin(s.psi), left = 0, right = 0;
  std::string source =
      m.kind == "kinematic" ? "kinematic_estimate" : "tire_model";
  for (int i = 0; i < 4; i++) {
    double side =
        std::sin(s.delta[i]) * m.fx[i] + std::cos(s.delta[i]) * m.fy[i];
    (i % 2 == 0 ? left : right) += side;
    double mu = sim->scene
                    .at(s.x + cp * m.wp[i][0] - sn * m.wp[i][1],
                        s.y + sn * m.wp[i][0] + cp * m.wp[i][1])
                    .mu;
    out["wheels"].push_back(
        {{"delta", s.delta_reported[i]},
         {"delta_true", s.delta[i]},
         {"delta_cmd", c.delta_cmd[i]},
         {"omega", s.wheel_omega[i]},
         {"fz", s.fz[i]},
         {"torque_steer", s.torque_steer[i]},
         {"susp_defl", s.susp_defl[i]},
         {"locked", bool(s.wheel_locked[i])},
         {"slip_alpha", m.alpha[i]},
         {"slip_kappa", m.kappa[i]},
         {"mu", mu},
         {"pos_body", m.wp[i]},
         {"icr_body", s.wheel_icr_body[i]},
         {"icr_dev", s.wheel_icr_dev[i]},
         {"rack_force", s.rack_force[i]},
         {"motor_torque_demand", s.motor_torque_demand[i]},
         {"linkage_arm_tie_angle", s.linkage_arm_tie_angle[i]},
         {"linkage_tie_rack_angle", s.linkage_tie_rack_angle[i]},
         {"linkage_efficiency", s.linkage_efficiency[i]},
         {"tire_fx", m.fx[i]},
         {"tire_fy", m.fy[i]},
         {"grip_capacity", s.grip_capacity[i]},
         {"grip_util", s.grip_util[i]},
         {"grip_margin_lat", s.grip_margin_lat[i]},
         {"grip_margin_long", s.grip_margin_long[i]},
         {"grip_alpha_peak", s.grip_alpha_peak[i]},
         {"grip_kappa_peak", s.grip_kappa_peak[i]},
         {"grip_beyond_peak_lat", bool(s.grip_beyond_peak_lat[i])},
         {"grip_beyond_peak_long", bool(s.grip_beyond_peak_long[i])},
         {"side_force_body_y", side},
         {"force_source", source}});
  }
  out["side_force_summary"] = {{"left", left},
                               {"right", right},
                               {"total", left + right},
                               {"source", source}};
  return out;
}
} // namespace fourwis
