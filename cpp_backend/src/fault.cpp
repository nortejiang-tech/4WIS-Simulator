#include "fault.hpp"
#include <iomanip>
#include <regex>
#include <sstream>
namespace fourwis {
static const std::map<std::string, std::string> aliases = {
    {"motor_stuck_zero", "stuck_zero"},
    {"motor_stuck_angle", "stuck_value"},
    {"motor_limited_range", "limited"},
    {"motor_stuck_hold", "stuck_hold"},
    {"motor_free_caster", "free_caster"}};
json Fault::spec() const {
  return {{"fault_type", type},
          {"wheel", wheel},
          {"value", value},
          {"t_start", start},
          {"t_end", std::isfinite(end) ? json(end) : json(nullptr)},
          {"active", active},
          {"id", id},
          {"eta_rev", eta},
          {"j_steer", inertia},
          {"c_damp", damping},
          {"tau_coulomb", friction}};
}
json Fault::legacy() const {
  auto j = spec();
  j["wheel"] = wheel_names[wheel];
  for (auto &a : aliases)
    if (a.second == type) {
      j["fault_type"] = a.first;
      break;
    }
  return j;
}
void Faults::replace(const json &defs, uint32_t random_seed, Model *model,
                     double now) {
  if (!defs.is_array() || defs.size() > 64)
    throw Error(422, "at most 64 faults per timeline");
  std::vector<Fault> next;
  std::set<std::string> ids;
  const std::set<std::string> keys = {
      "fault_type", "wheel",   "value",   "t_start", "t_end",      "active",
      "id",         "eta_rev", "j_steer", "c_damp",  "tau_coulomb"};
  for (size_t i = 0; i < defs.size(); i++) {
    const auto &d = defs[i];
    if (!d.is_object())
      throw Error(422, "fault must be an object");
    for (auto it = d.begin(); it != d.end(); ++it)
      if (!keys.count(it.key()))
        throw Error(422, "unknown fault field: " + it.key());
    Fault f;
    f.type = str(d, "fault_type");
    if (aliases.count(f.type))
      f.type = aliases.at(f.type);
    f.value = num(d, "value");
    f.start = num(d, "t_start");
    f.end = num(d, "t_end", INFINITY);
    f.active = yes(d, "active", true);
    f.eta = num(d, "eta_rev", .6);
    f.inertia = num(d, "j_steer", 3);
    f.damping = num(d, "c_damp", 80);
    f.friction = num(d, "tau_coulomb", 5);
    if (!d.contains("wheel"))
      throw Error(422, "wheel is required");
    auto w = d.at("wheel");
    if (w.is_string()) {
      auto name = w.get<std::string>();
      std::transform(name.begin(), name.end(), name.begin(),
                     [](unsigned char c) { return char(std::tolower(c)); });
      f.wheel = -1;
      for (int k = 0; k < 4; k++)
        if (name == wheel_names[k])
          f.wheel = k;
    } else if (w.is_number_integer() && !w.is_boolean())
      f.wheel = w.get<int>();
    else
      throw Error(422, "wheel must be an integer or wheel name");
    if (f.wheel < 0 || f.wheel > 3)
      throw Error(422, "invalid wheel");
    std::ostringstream id;
    id << "fault-" << std::setfill('0') << std::setw(3) << (i + 1);
    f.id = str(d, "id", id.str());
    if (!std::regex_match(f.id, std::regex("[A-Za-z0-9_.:-]{1,64}")) ||
        !ids.insert(f.id).second)
      throw Error(422, "invalid or duplicate fault id");
    if (!f.actuator() && f.type != "sensor_bias" && f.type != "sensor_noise" &&
        f.type != "sensor_dropout")
      throw Error(422, "unknown fault type");
    for (double v : {f.value, f.start, f.eta, f.inertia, f.damping, f.friction})
      if (!std::isfinite(v))
        throw Error(422, "non-finite fault parameter");
    if (f.start < 0 || f.end <= f.start || f.eta < 0 || f.eta > 1 ||
        f.inertia <= 0 || f.damping < 0 || f.friction < 0)
      throw Error(422, "invalid fault window or mechanism parameter");
    if ((f.type == "limited" || f.type == "sensor_noise") && f.value < 0)
      throw Error(422, "range/noise must be nonnegative");
    if ((f.type == "stuck_value" || f.type == "limited") &&
        std::abs(f.value) > pi / 2)
      throw Error(422, "target exceeds pi/2");
    if (model) {
      if (f.physical() && model->kind == "kinematic")
        throw Error(422, "physical fault requires force-based dynamic model");
      if ((f.type == "stuck_value" || f.type == "limited") &&
          std::abs(f.value) > model->p.steer_limit)
        throw Error(422, "fault angle exceeds vehicle steer_limit");
      if (f.actuator() && yes(model->p.steering_system, "enabled")) {
        if (str(model->p.steering_system, "architecture") != "4wis")
          throw Error(
              422,
              "per-wheel actuator faults require independent 4wis steering");
        if (!yes(
                model->p.steering_system.value("angle_control", json::object()),
                "enabled"))
          throw Error(422, "4wis faults require enabled angle control");
      }
    }
    for (auto &b : next)
      if (f.wheel == b.wheel &&
          std::max(f.start, b.start) < std::min(f.end, b.end) &&
          ((f.actuator() && b.actuator()) || f.type == b.type))
        throw Error(422, "overlapping incompatible faults on " +
                             std::string(wheel_names[f.wheel]));
    next.push_back(f);
  }
  auto history = events;
  for (auto &id : enabled)
    history.push_back({{"id", id}, {"event", "removed"}, {"time", now}});
  faults = std::move(next);
  seed = random_seed;
  reset(model ? model->s.delta : V4{});
  events = history;
  time = now;
}
void Faults::sync(double t, Model *model) {
  time = t;
  std::set<std::string> current;
  for (auto &f : faults)
    if (f.enabled(t))
      current.insert(f.id);
  for (auto &id : enabled)
    if (!current.count(id)) {
      events.push_back({{"id", id}, {"event", "deactivated"}, {"time", time}});
      hold.erase(id);
      free.erase(id);
    }
  for (auto &id : current)
    if (!enabled.count(id)) {
      auto it = std::find_if(faults.begin(), faults.end(),
                             [&](const Fault &f) { return f.id == id; });
      const auto &f = *it;
      events.push_back({{"id", id},
                        {"event", "activated"},
                        {"time", time},
                        {"scheduled_time", f.start},
                        {"wheel", f.wheel},
                        {"fault_type", f.type},
                        {"config", f.spec()}});
      if (model) {
        double a = model->kind == "kinematic" ? model->s.delta[f.wheel]
                                              : model->act[f.wheel],
               r = model->kind == "kinematic" ? 0 : model->rate[f.wheel];
        hold[id] = a;
        free[id] = {a, r};
      }
    }
  enabled = current;
}
double Faults::boundary(double start, double end) const {
  for (auto &f : faults)
    if (f.active)
      for (double t : {f.start, f.end})
        if (start + 1e-10 < t && t < end - 1e-10)
          end = t;
  return end;
}
Command Faults::command(Command c, Model &m, double dt) {
  sync(m.s.t, &m);
  c.steer_override.fill(nan);
  c.steer_override_rate.fill(0);
  for (auto &f : faults)
    if (f.enabled(time)) {
      int w = f.wheel;
      if (f.type == "stuck_zero")
        c.delta_cmd[w] = 0;
      else if (f.type == "stuck_value")
        c.delta_cmd[w] = f.value;
      else if (f.type == "limited")
        c.delta_cmd[w] = clip(c.delta_cmd[w], -f.value, f.value);
    }
  for (auto &id : enabled) {
    auto it = std::find_if(faults.begin(), faults.end(),
                           [&](const Fault &f) { return f.id == id; });
    const auto &f = *it;
    int w = f.wheel;
    if (f.type == "stuck_hold")
      c.steer_override[w] = hold.at(id);
    else if (f.type == "free_caster") {
      auto q = free.at(id);
      double mom = f.inertia * q[1] + dt * (-f.eta * m.s.torque_steer[w]);
      q[1] = std::copysign(std::max(std::abs(mom) - dt * f.friction, 0.), mom) /
             (f.inertia + dt * f.damping);
      q[0] += dt * q[1];
      if (q[0] >= m.p.steer_limit) {
        q[0] = m.p.steer_limit;
        q[1] = std::min(q[1], 0.);
      } else if (q[0] <= -m.p.steer_limit) {
        q[0] = -m.p.steer_limit;
        q[1] = std::max(q[1], 0.);
      }
      free[id] = q;
      c.steer_override[w] = q[0];
      c.steer_override_rate[w] = q[1];
    }
    if (f.physical())
      c.delta_cmd[w] = c.steer_override[w];
  }
  return c;
}
void Faults::sample(Model &m) {
  sync(m.s.t, &m);
  V4 reported = m.s.delta;
  std::vector<const Fault *> active;
  for (auto &f : faults)
    if (f.enabled(time))
      active.push_back(&f);
  std::stable_sort(active.begin(), active.end(), [](auto a, auto b) {
    return std::tie(a->wheel, a->type, a->start) <
           std::tie(b->wheel, b->type, b->start);
  });
  for (auto *f : active) {
    if (f->type == "sensor_bias")
      reported[f->wheel] += f->value;
    else if (f->type == "sensor_noise") {
      std::string key = std::to_string(f->wheel) + ":" + f->type + ":" +
                        python_float(f->start);
      if (!rng.count(key)) {
        auto digest = sha256(std::to_string(seed) + ":" + key);
        uint64_t x = 0;
        for (int i = 0; i < 8; i++)
          x |= uint64_t(digest[i]) << (8 * i);
        rng.emplace(key, Random(x));
      }
      reported[f->wheel] += rng.at(key).normal() * f->value;
    }
  }
  for (auto *f : active)
    if (f->type == "sensor_dropout")
      reported[f->wheel] = last_reported[f->wheel];
  last_reported = reported;
  m.s.delta_reported = reported;
  m.s.fault_mode = {};
  m.s.sensor_fault_active = {};
  for (auto *f : active) {
    int mode = f->type == "stuck_zero" || f->type == "stuck_value" ? 1
               : f->type == "limited"                              ? 2
               : f->type == "stuck_hold"                           ? 3
               : f->type == "free_caster"                          ? 4
                                                                   : 0;
    if (mode)
      m.s.fault_mode[f->wheel] = mode;
    else
      m.s.sensor_fault_active[f->wheel] = 1;
  }
}
json Faults::status() const {
  json defs = json::array();
  for (auto &f : faults) {
    auto d = f.legacy();
    d["effective_active"] = f.enabled(time);
    d["phase"] = !f.active                ? "disabled"
                 : time < f.start - 1e-10 ? "scheduled"
                 : f.enabled(time)        ? "active"
                                          : "completed";
    defs.push_back(d);
  }
  return {{"clock", "simulation"},
          {"time", time},
          {"seed", seed},
          {"faults", defs},
          {"events", events}};
}
} // namespace fourwis
