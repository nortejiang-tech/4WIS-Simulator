#pragma once
#include "random.hpp"
#include "state.hpp"
#include <Eigen/Dense>
#include <deque>
#include <memory>
namespace fourwis {
using Mat = Eigen::MatrixXd;
using Vec = Eigen::VectorXd;
Mat matrix_exp(const Mat &m, double dt);
Mat dare(const Mat &a, const Mat &b, const Mat &q, double r, Mat k);
struct Derivative {
  double tau = .005, prev = nan, value = 0;
  double step(double x, double dt) {
    if (!std::isfinite(prev)) {
      prev = x;
      return 0;
    }
    double raw = (x - prev) / dt;
    prev = x;
    value += (raw - value) * dt / (tau + dt);
    return value;
  }
};
struct TrackResult {
  double angle = nan, torque = nan;
  json diagnostics = json::object();
};
class TrackerControl {
public:
  std::string name;
  json cfg, ff;
  std::vector<double> ff_state;
  double integral = 0, integral_v = 0, prev_target_rate = 0, dhat = 0,
         last_u = 0;
  Derivative rate, acc;
  Mat a, b, k, h_inv, nqm, nq;
  Vec z, l;
  std::shared_ptr<TrackerControl> base;
  TrackerControl(std::string n, json c = json::object());
  void reset();
  double feed(double dt, double target_rate, double target_accel, double load);
  double limit(double u, double fallback = INFINITY) const;
  json gains(double speed) const;
  TrackResult step(double dt, double target, double target_rate,
                   double feedback, double angle, double load, double speed);
};
struct Corner {
  json cfg;
  TrackerControl controller;
  Random rng;
  std::deque<double> queue;
  double angle = 0, rate = 0, motor_angle = 0, motor_rate = 0, prev_target = 0;
  json diagnostics = json::object();
  Corner(json c, int i)
      : cfg(c), controller(str(c, "controller", "open_loop"),
                           c.value("controller_kwargs", json::object())),
        rng(uint64_t(num(c, "sensor_seed", 1234)) + i),
        queue(size_t(std::max(0, int(num(c, "sensor_delay_steps", 1)))) + 1,
              0) {}
  void reset(double a = 0) {
    angle = motor_angle = prev_target = a;
    rate = motor_rate = 0;
    diagnostics = json::object();
    controller.reset();
  }
  double sensor();
  void plant(double dt, double torque, double load);
  void step(double dt, double target, double load, double speed);
};
struct Motor {
  double torque = 0, heat = 0, speed = 0;
  bool saturated = false;
  void step(double dt, double command, double omega, const json &p);
};
class Steering {
public:
  VehicleParams p;
  std::string arch;
  bool enabled = false, bywire = false, perwheel = false;
  double ratio = 1, gear = 63, prev_hand = 0, pinion = 0, pinion_rate = 0,
         hand = 0, hand_rate = 0, byangle = 0, road_filtered = 0;
  Motor motor;
  std::array<std::shared_ptr<Corner>, 4> corners;
  json channels = json::object();
  explicit Steering(const VehicleParams &params);
  V4 ramp_start(const V4 &target) const;
  void step(const Command &cmd, double dt, const State &s, V4 &actual,
            V4 &actual_rate);
  double front(double dt, double target, double rack, double speed,
               double resistance = 0);
  double assist(double torque, double speed) const;
};
} // namespace fourwis
