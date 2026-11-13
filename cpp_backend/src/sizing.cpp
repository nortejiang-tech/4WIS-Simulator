#include "analysis.hpp"
#include "study.hpp"
namespace fourwis {
json sizing(const Storage &store, const VehicleParams &p) {
  auto data = store.root / "cpp_backend/data";
  if (!fs::exists(data))
    data = store.root / "app/data";
  auto cases = read_json(data / "sizing_scenarios.json");
  json scenarios = json::array(), measurements = json::array();
  std::map<std::string, std::pair<double, std::string>> worst;
  double weighted = 0, weights = 0;
  std::set<std::string> tainted;
  json saturated = json::array();
  auto rnd = [](double x, int places) {
    double k = std::pow(10., places);
    return std::nearbyint(x * k) / k;
  };
  for (auto &scenario : cases) {
    double speed = num(scenario, "speed_kmh") / 3.6, mu = num(scenario, "mu"),
           amp = rad(num(scenario, "amplitude_deg")),
           rate = rad(num(scenario, "rate_deg_s")),
           accel = rad(std::max(num(scenario, "accel_deg_s2"), 1e-6)),
           dt = .002;
    auto id = str(scenario, "id");
    json request = {{"speeds", json::array({speed})},
                    {"angles", json::array()},
                    {"wheel_index", 0},
                    {"mu", mu}};
    for (int i = 0; i < 25; i++)
      request["angles"].push_back(p.steer_limit * i / 24);
    auto rows = load_sweep(p, request)["rows"];
    std::vector<std::pair<double, double>> pairs;
    for (auto &r : rows)
      if (int(num(r, "wheel_index")) == 0)
        pairs.emplace_back(std::abs(num(r, "delta_cmd")),
                           std::abs(num(r, "rack_force")));
    std::stable_sort(pairs.begin(), pairs.end(),
                     [](auto &a, auto &b) { return a.first < b.first; });
    std::vector<double> xs, ys;
    for (auto &pair : pairs) {
      xs.push_back(pair.first);
      ys.push_back(pair.second);
    }
    Steering plant(p);
    plant.bywire = false;
    std::vector<double> targets;
    for (int i = 0; i < std::max(1, int(num(scenario, "cycles", 1))); i++)
      for (double value : {amp, -amp, 0.})
        targets.push_back(value);
    double span = amp;
    for (size_t i = 0; i < targets.size() - 2; i++)
      span += 2 * amp;
    span += amp;
    double total =
        span / std::max(rate, 1e-6) + 2 * targets.size() * rate / accel + .5;
    int n = std::max(int(total / dt), 10), steps = 0, limited = 0, sat = 0;
    size_t leg = 0;
    double delta = 0, delta_rate = 0, settle = 0, sq = 0;
    json values = {{"peak_motor_torque_nm", 0.}, {"peak_motor_speed_rpm", 0.},
                   {"peak_power_w", 0.},         {"peak_thermal_state", 0.},
                   {"peak_rack_force_n", 0.},    {"peak_hand_torque_nm", 0.}};
    for (int tick = 0; tick < n; tick++) {
      double target = targets[std::min(leg, targets.size() - 1)],
             gap = target - delta;
      if (std::abs(gap) < .0001 && std::abs(delta_rate) < .001) {
        if (leg < targets.size() - 1)
          leg++;
        else {
          settle += dt;
          if (settle > .1)
            break;
        }
      }
      double braking = std::sqrt(2 * accel * std::abs(gap)),
             want = gap ? std::copysign(std::min(rate, braking), gap) : 0;
      delta_rate += clip(want - delta_rate, -accel * dt, accel * dt);
      delta += delta_rate * dt;
      double angle = plant.pinion / plant.ratio,
             mag = 2 * interp(std::abs(angle), xs, ys),
             rolling = std::abs(speed) / (std::abs(speed) + 1.5),
             aligning = std::copysign(rolling * mag, angle),
             scrub = (1 - rolling) * mag;
      plant.front(dt, delta, aligning, speed, scrub);
      steps++;
      auto update = [&](const std::string &key, double x) {
        values[key] = std::max(num(values, key), std::abs(x));
      };
      update("peak_motor_torque_nm", plant.motor.torque);
      update("peak_motor_speed_rpm", plant.motor.speed * 60 / (2 * pi));
      update("peak_power_w", plant.motor.torque * plant.motor.speed);
      update("peak_thermal_state", plant.motor.heat);
      update("peak_rack_force_n", std::abs(aligning) + std::abs(scrub));
      update("peak_hand_torque_nm", num(plant.channels, "steer_hand_torque"));
      sq += plant.motor.torque * plant.motor.torque;
      if (plant.motor.saturated)
        sat++;
      if (num(plant.channels, "sizing_hand_limited") > .5)
        limited++;
    }
    values["rms_motor_torque_nm"] = std::sqrt(sq / std::max(steps, 1));
    bool beyond = double(limited) / std::max(steps, 1) > .05;
    if (beyond)
      tainted.insert(id);
    else if (double(sat) / std::max(steps, 1) > .05)
      saturated.push_back(id);
    for (auto it = values.begin(); it != values.end(); ++it) {
      double v = it.value();
      auto found = worst.find(it.key());
      if (found == worst.end() || v > found->second.first)
        worst[it.key()] = {v, id};
      measurements.push_back(
          {{"metric", it.key()},
           {"value", v},
           {"at", {{"scenario", id}}},
           {"label", scenario["label"]},
           {"source", "sizing:" + id},
           {"trusted", !beyond},
           {"untrusted_reason",
            beyond ? "工况超出作动器能力，饱和后的数值不可信" : ""}});
    }
    double weight = num(scenario, "duty_weight", 1);
    weighted += weight * std::pow(num(values, "rms_motor_torque_nm"), 2);
    weights += weight;
    json item = {
        {"id", id},
        {"label", scenario["label"]},
        {"exposes", scenario["exposes"]},
        {"saturated_fraction", rnd(double(sat) / std::max(steps, 1), 3)},
        {"hand_limited_fraction", rnd(double(limited) / std::max(steps, 1), 3)},
        {"beyond_capability", beyond}};
    for (auto it = values.begin(); it != values.end(); ++it) {
      auto key = it.key();
      int precision = key == "peak_motor_speed_rpm" || key == "peak_power_w" ||
                              key == "peak_rack_force_n"
                          ? 1
                      : key == "peak_hand_torque_nm" ? 2
                                                     : 3;
      item[key] = rnd(it.value(), precision);
    }
    if (beyond)
      item["note"] = "手力矩已达上限，车轮没有到达指令角度 —— "
                     "只能读出「不够」，读不出「差多少」";
    scenarios.push_back(item);
  }
  double torque = worst["peak_motor_torque_nm"].first,
         rms = std::sqrt(weighted / (weights ? weights : 1)),
         speed = worst["peak_motor_speed_rpm"].first * 2 * pi / 60;
  json requirement = {{"peak_torque_nm", rnd(torque, 3)},
                      {"peak_speed_rpm",
                       rnd(worst["peak_motor_speed_rpm"].first, 1)},
                      {"rms_torque_nm", rnd(rms, 3)},
                      {"peak_power_w", rnd(worst["peak_power_w"].first, 1)},
                      {"peak_thermal_state",
                       rnd(worst["peak_thermal_state"].first, 3)}},
       driven = {{"peak_torque", worst["peak_motor_torque_nm"].second},
                 {"peak_speed", worst["peak_motor_speed_rpm"].second},
                 {"peak_power", worst["peak_power_w"].second},
                 {"thermal", worst["peak_thermal_state"].second}};
  auto motor = p.steering_system.value("motor", json::object());
  json verdict = {{"pass", true}, {"checks", json::object()}};
  for (auto check : std::vector<std::tuple<std::string, double, double>>{
           {"peak_torque", torque, num(motor, "peak_torque", 8)},
           {"continuous_torque", rms, num(motor, "continuous_torque", 4.5)},
           {"speed", speed,
            num(motor, "no_load_speed_rpm", 8000) * 2 * pi / 60}}) {
    auto name = std::get<0>(check);
    double need = std::get<1>(check), have = std::get<2>(check);
    verdict["checks"][name] = {
        {"required", rnd(need, 3)},
        {"available", rnd(have, 3)},
        {"margin_pct",
         have ? json(rnd((have - need) / have * 100, 1)) : json()},
        {"pass", need <= have}};
    if (need > have)
      verdict["pass"] = false;
  }
  if (!tainted.empty()) {
    verdict["beyond_capability_in"] = tainted;
    verdict["note"] = "这些工况下手力矩已达上限、车轮未到达指令角度，其峰值数字"
                      "不可信；先换更大的电机再读余量。";
    verdict["pass"] = false;
  }
  if (!saturated.empty()) {
    verdict["assist_saturated_in"] = saturated;
    verdict["saturation_note"] = "助力在这些工况下饱和：转向比标定意图更重，但"
                                 "数值有效 —— 这是一条发现，不是一次失效。";
  }
  for (auto pair : std::vector<std::pair<std::string, std::string>>{
           {"required_peak_torque_nm", "peak_motor_torque_nm"},
           {"required_peak_speed_rpm", "peak_motor_speed_rpm"},
           {"required_peak_power_w", "peak_power_w"},
           {"required_thermal_state", "peak_thermal_state"}}) {
    auto v = worst[pair.second];
    bool bad = tainted.count(v.second);
    measurements.push_back(
        {{"metric", pair.first},
         {"value", v.first},
         {"at", json::object()},
         {"label", "包络（" + v.second + "）"},
         {"source", "sizing:requirement"},
         {"trusted", !bad},
         {"untrusted_reason", bad ? "工况超出作动器能力" : ""}});
  }
  measurements.push_back(
      {{"metric", "required_rms_torque_nm"},
       {"value", rms},
       {"at", json::object()},
       {"label", "占空比 RMS"},
       {"source", "sizing:requirement"},
       {"trusted", tainted.empty()},
       {"untrusted_reason", tainted.empty() ? "" : "工况超出作动器能力"}});
  return {{"result",
           {{"requirement", requirement},
            {"driven_by", driven},
            {"scenarios", scenarios},
            {"verdict", verdict}}},
          {"measurements", measurements}};
}
} // namespace fourwis
