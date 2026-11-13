#pragma once
#include "model.hpp"
#include "random.hpp"
#include <set>
namespace fourwis {
struct Fault {
  std::string type, id;
  int wheel = 0;
  double value = 0, start = 0, end = INFINITY, eta = .6, inertia = 3,
         damping = 80, friction = 5;
  bool active = true;
  bool actuator() const {
    return type == "stuck_zero" || type == "stuck_value" || type == "limited" ||
           physical();
  }
  bool physical() const {
    return type == "stuck_hold" || type == "free_caster";
  }
  bool enabled(double t) const {
    return active && t >= start - 1e-10 && t < end - 1e-10;
  }
  json spec() const;
  json legacy() const;
};
class Faults {
public:
  std::vector<Fault> faults;
  uint32_t seed = 0;
  double time = 0;
  std::set<std::string> enabled;
  std::map<std::string, double> hold;
  std::map<std::string, V2> free;
  std::map<std::string, Random> rng;
  V4 last_reported{};
  json events = json::array();
  void replace(const json &defs, uint32_t random_seed, Model *model = nullptr,
               double now = 0);
  void reset(V4 actual = {}) {
    time = 0;
    enabled.clear();
    hold.clear();
    free.clear();
    rng.clear();
    last_reported = actual;
    events = json::array();
  }
  void sync(double t, Model *model = nullptr);
  double boundary(double start, double end) const;
  Command command(Command c, Model &m, double dt);
  void sample(Model &m);
  json status() const;
};
} // namespace fourwis
