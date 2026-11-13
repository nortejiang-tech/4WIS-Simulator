#include "analysis.hpp"
#include "service.hpp"
namespace fourwis {
std::array<double, 28> linkage_metrics(const VehicleParams &p, const V4 &d) {
  auto &g = p.steering_geometry;
  double geom[] = {g.front_outer_x,
                   g.front_outer_y,
                   g.front_inner_x,
                   g.front_inner_y,
                   rad(g.front_rack_axis_deg),
                   g.front_rack_travel_limit,
                   g.rear_outer_x,
                   g.rear_outer_y,
                   g.rear_inner_x,
                   g.rear_inner_y,
                   rad(g.rear_rack_axis_deg),
                   g.rear_rack_travel_limit};
  std::array<double, 28> out{};
  sim4wis::linkage(d.data(), geom, p.rack_mech_efficiency,
                   p.steering_arm_length, out.data());
  return out;
}
V4 analysis_stiffness(const VehicleParams &p, const V4 &fz) {
  V4 c{};
  for (int i = 0; i < 4; i++)
    c[i] = p.tire_c_alpha *
           std::pow(std::max(fz[i] / std::max(p.mass * 9.80665 / 4, 1.), .001),
                    p.tire_load_sensitivity_exp) *
           (i < 2 ? p.tire_c_alpha_front_scale : p.tire_c_alpha_rear_scale);
  return c;
}
static V4 analysis_loads(const VehicleParams &p, double speed) {
  V4 f = vertical_loads(p, 0, 0);
  double q = .5 * p.air_density * p.frontal_area * speed * speed;
  for (int i = 0; i < 4; i++)
    f[i] = std::max(
        f[i] -
            .5 * q * (i < 2 ? p.aero_lift_coeff_front : p.aero_lift_coeff_rear),
        0.);
  return f;
}
V2 steady_body(const VehicleParams &p, double speed, const V4 &d,
               const V4 &ca) {
  double v = std::max(speed, .001), A = 0, B = 0, C = 0, D = 0, E = 0;
  auto wp = wheels(p);
  for (int i = 0; i < 4; i++) {
    A += ca[i];
    B += ca[i] * wp[i][0];
    C += ca[i] * wp[i][0] * wp[i][0];
    D += ca[i] * d[i];
    E += ca[i] * wp[i][0] * d[i];
  }
  double cg = p.wheelbase / 2 - p.cg_to_front, b = B / v + p.mass * v,
         c = B - cg * A, e = (C - cg * B) / v, rhs = E - cg * D,
         det = A * e - b * c;
  if (det == 0)
    return {};
  double beta = (D * e - b * rhs) / det, yaw = (A * rhs - c * D) / det;
  if (!std::isfinite(beta) || !std::isfinite(yaw))
    return {};
  return {beta, yaw};
}
static json zero_cross(const std::vector<double> &x,
                       const std::vector<double> &y) {
  if (x.size() < 2)
    return nullptr;
  std::vector<double> roots;
  for (size_t i = 1; i < x.size(); i++) {
    double a = y[i - 1], b = y[i];
    if (!std::isfinite(a) || !std::isfinite(b))
      continue;
    if (std::abs(a) < 1e-9)
      roots.push_back(x[i - 1]);
    if ((a < 0 && b > 0) || (b < 0 && a > 0))
      roots.push_back(x[i - 1] - a * (x[i] - x[i - 1]) / (b - a));
  }
  if (std::abs(y.back()) < 1e-9)
    roots.push_back(x.back());
  if (roots.empty())
    return nullptr;
  return *std::min_element(roots.begin(), roots.end(), [](double a, double b) {
    return std::abs(a) < std::abs(b);
  });
}
static double interpolate(const std::vector<double> &x,
                          const std::vector<double> &y, double q) {
  if (q <= x.front())
    return y.front();
  if (q >= x.back())
    return y.back();
  for (size_t i = 1; i < x.size(); i++)
    if (q <= x[i]) {
      if (std::abs(x[i] - x[i - 1]) < 1e-12)
        return y[i];
      return y[i - 1] + (q - x[i - 1]) / (x[i] - x[i - 1]) * (y[i] - y[i - 1]);
    }
  return y.back();
}
static json load_summary(const json &rows, int wheel) {
  if (rows.empty())
    return json::object();
  auto peak = [&](const std::string &field, bool absolute, bool minimum) {
    json best = rows[0];
    for (auto &r : rows) {
      double a = num(r, field), b = num(best, field);
      if (absolute) {
        a = std::abs(a);
        b = std::abs(b);
      }
      if (minimum ? a < b : a > b)
        best = r;
    }
    return best;
  };
  auto rack = peak("rack_force", true, false),
       motor = peak("motor_torque", true, false),
       eff = peak("geometry_efficiency", false, true),
       util = peak("friction_utilization", false, false);
  auto where = [](const json &r) {
    return json{{"speed", r["speed"]},
                {"requested_angle", r["requested_angle"]},
                {"wheel_label", r["wheel_label"]},
                {"delta", r["delta"]}};
  };
  json warnings = json::array();
  for (auto &r : rows)
    if (!yes(r, "linkage_valid", true)) {
      warnings.push_back("存在超出齿条行程或硬点不可达的转角；该区间负载为外推"
                         "值，不能用于选型验收");
      break;
    }
  if (num(eff, "geometry_efficiency") < .2)
    warnings.push_back("低几何效率：存在接近奇异或力臂不足的转角区间");
  if (num(util, "friction_utilization") > .95)
    warnings.push_back("轮胎接近附着极限：侧向力/转向阻力可能进入饱和");
  std::map<double, std::vector<json>> buckets;
  for (auto &r : rows)
    if (num(r, "wheel_index") == wheel)
      buckets[num(r, "speed")].push_back(r);
  json equilibrium = json::array();
  for (auto &b : buckets) {
    std::stable_sort(
        b.second.begin(), b.second.end(), [](const json &a, const json &b) {
          return num(a, "requested_angle") < num(b, "requested_angle");
        });
    std::vector<double> x, y, t;
    for (auto &r : b.second) {
      x.push_back(num(r, "requested_angle"));
      y.push_back(num(r, "rack_force"));
      t.push_back(num(r, "torque_steer"));
    }
    auto eq = zero_cross(x, y);
    std::string source = "rack";
    if (eq.is_null()) {
      eq = zero_cross(x, t);
      source = eq.is_null() ? "none" : "torque";
    }
    equilibrium.push_back(
        {{"speed", b.first},
         {"delta_eq", eq},
         {"delta_eq_deg",
          eq.is_null() ? json() : json(eq.get<double>() * 180 / pi)},
         {"rack_at_zero", interpolate(x, y, 0)},
         {"torque_at_zero", interpolate(x, t, 0)},
         {"source", source},
         {"found", !eq.is_null()}});
  }
  return {{"peak_abs_rack_force", std::abs(num(rack, "rack_force"))},
          {"peak_abs_rack_force_at", where(rack)},
          {"peak_abs_motor_torque", std::abs(num(motor, "motor_torque"))},
          {"peak_abs_motor_torque_at", where(motor)},
          {"min_geometry_efficiency", eff["geometry_efficiency"]},
          {"min_geometry_efficiency_at", where(eff)},
          {"max_friction_utilization", util["friction_utilization"]},
          {"warnings", warnings},
          {"per_speed_equilibrium", equilibrium}};
}
json load_sweep(const VehicleParams &p, const json &request) {
  int wheel = int(clip(num(request, "wheel_index"), 0, 3));
  auto mode = str(request, "mode", "single_wheel"),
       coupling = str(request, "body_coupling", "vehicle");
  if (coupling != "isolated")
    coupling = "vehicle";
  auto speeds = request.value("speeds", json::array({0, 2, 5, 10, 20})),
       angles = request.value("angles", json::array());
  if (speeds.empty())
    speeds = json::array({0, 5, 15});
  if (angles.empty())
    angles = json::array({0});
  double mu = num(request, "mu", .85);
  json rows = json::array();
  auto wp = wheels(p);
  for (size_t si = 0; si < std::min<size_t>(80, speeds.size()); si++) {
    double speed = clip(speeds[si], 0, std::max(p.v_max, .1));
    auto fz = analysis_loads(p, speed), ca = analysis_stiffness(p, fz);
    double low = std::exp(-std::pow(
               std::abs(speed) / std::max(p.low_speed_blend_ms, .05), 2)),
           moving = 1 - low,
           drive = (p.rolling_resistance_coeff * p.mass * 9.80665 +
                    .5 * p.air_density * p.drag_coeff_cd * p.frontal_area *
                        speed * speed) /
                   4;
    for (size_t ai = 0; ai < std::min<size_t>(161, angles.size()); ai++) {
      double angle = clip(angles[ai], -p.steer_limit, p.steer_limit);
      V4 cmd{};
      cmd[wheel] = angle;
      if (mode != "single_wheel")
        try {
          Controller controller(mode, p);
          Driver driver;
          driver.steering =
              p.steer_limit > 1e-9 ? clip(angle / p.steer_limit, -1, 1) : 0;
          driver.throttle = p.v_max > 1e-9 ? clip(speed / p.v_max, -1, 1) : 0;
          State s;
          s.vx = speed;
          cmd = controller.compute(driver, s, .005).delta_cmd;
        } catch (const Error &) {
        }
      V4 d{}, alpha{};
      for (int i = 0; i < 4; i++)
        d[i] = cmd[i] + (i % 2 == 0 ? -1. : 1.) *
                            (i < 2 ? p.static_toe_front : p.static_toe_rear);
      if (coupling == "vehicle" && speed > 1) {
        auto body = steady_body(p, speed, d, ca);
        for (int i = 0; i < 4; i++)
          alpha[i] =
              body[0] + body[1] * wp[i][0] / std::max(speed, .001) - d[i];
      } else
        for (int i = 0; i < 4; i++)
          alpha[i] = std::atan2(-speed * std::sin(d[i]),
                                std::max(std::abs(speed * std::cos(d[i])), .5));
      auto geom = linkage_metrics(p, d);
      for (int i = 0; i < 4; i++) {
        double cam = (i % 2 == 0 ? 1. : -1.) * p.suspension.camber,
               camber = p.camber_thrust_coeff * cam * fz[i];
        double tp[] = {1,         ca[i],     p.tire_c_kappa, 0,         0, 0,
                       p.tire_cx, p.tire_cy, p.tire_ex,      p.tire_ey, .2};
        auto force =
            sim4wis::tire(tp, alpha[i] - camber / std::max(ca[i], 1.),
                          drive / std::max(p.tire_c_kappa, 1.), fz[i], mu);
        double turn = std::tanh(
                   d[i] / rad(std::max(p.static_tire_deflection_deg, .5))),
               side = (p.parking_lateral_coeff > 0 ? p.parking_lateral_coeff
                                                   : p.parking_scrub_coeff) *
                      mu * fz[i] * turn * low,
               parking = (p.parking_torque_coeff > 0 ? p.parking_torque_coeff
                                                     : p.parking_scrub_coeff) *
                         mu * fz[i] * p.contact_patch_radius * turn * low;
        double fx = force.fx * moving, fy = force.fy * moving + side,
               mz = -force.fy * p.tire_t_pneumatic *
                    std::max(0., 1 - std::abs(alpha[i]) / .2) * moving;
        double ideal_fx = drive * moving,
               ideal_fy = (-ca[i] * alpha[i] + camber) * moving + side,
               ideal_mz = -ideal_fy * p.tire_t_pneumatic;
        auto torque = [&](double x, double y, double z) {
          return y * p.tire_radius * std::tan(p.suspension.caster_angle) +
                 x * p.suspension.scrub_radius * (i % 2 == 0 ? 1. : -1.) - z +
                 fz[i] * std::sin(p.suspension.kingpin_inclination) *
                     p.suspension.scrub_radius * std::sin(d[i]) +
                 parking;
        };
        double tau = torque(fx, fy, mz),
               ideal = torque(ideal_fx, ideal_fy, ideal_mz),
               lever = std::max(std::abs(geom[20 + i]), 1e-6),
               rack = tau / lever, ideal_rack = ideal / lever,
               motor_scale = std::max(p.pinion_radius, 1e-6) /
                             (std::max(p.motor_gear_ratio, 1e-6) *
                              std::max(p.rack_mech_efficiency, 1e-6));
        rows.push_back(
            {{"speed", speed},
             {"requested_angle", angle},
             {"wheel_index", i},
             {"wheel_label",
              std::array<const char *, 4>{"FL", "FR", "RL", "RR"}[i]},
             {"delta", d[i]},
             {"delta_cmd", cmd[i]},
             {"slip_alpha", alpha[i]},
             {"fz", fz[i]},
             {"tire_fx", fx},
             {"tire_fy", fy},
             {"tire_mz", mz},
             {"fx_drive", drive * moving},
             {"fy_camber", camber * moving},
             {"torque_steer", tau},
             {"torque_steer_ideal", ideal},
             {"rack_force", rack},
             {"rack_force_ideal", ideal_rack},
             {"motor_torque", rack * motor_scale},
             {"motor_torque_ideal", ideal_rack * motor_scale},
             {"side_force_body_y", std::sin(d[i]) * fx + std::cos(d[i]) * fy},
             {"side_force_body_y_ideal",
              std::sin(d[i]) * ideal_fx + std::cos(d[i]) * ideal_fy},
             {"arm_tie_angle", geom[i]},
             {"tie_rack_angle", geom[4 + i]},
             {"geometry_efficiency", geom[8 + i]},
             {"rack_travel", geom[16 + i]},
             {"linkage_valid", geom[24 + i] != 0},
             {"friction_utilization",
              std::min(1., std::hypot(fx, fy) / std::max(mu * fz[i], 1e-9))}});
      }
    }
  }
  return {{"mode", mode},
          {"wheel_index", wheel},
          {"wheel_label",
           std::array<const char *, 4>{"FL", "FR", "RL", "RR"}[wheel]},
          {"body_coupling", coupling},
          {"rows", rows},
          {"summary", load_summary(rows, wheel)}};
}
json rack_solve(const VehicleParams &p, double travel, int i, double hint) {
  auto &g = p.steering_geometry;
  bool f = i < 2;
  double mirror = i % 2 == 0 ? 1. : -1.,
         ox = f ? g.front_outer_x : g.rear_outer_x,
         oy = mirror * (f ? g.front_outer_y : g.rear_outer_y),
         ix = f ? g.front_inner_x : g.rear_inner_x,
         iy = mirror * (f ? g.front_inner_y : g.rear_inner_y),
         a = rad(f ? g.front_rack_axis_deg : g.rear_rack_axis_deg),
         ax = std::cos(a), ay = mirror * std::sin(a), an = std::hypot(ax, ay);
  ax /= an;
  ay /= an;
  double limit = std::max(
             f ? g.front_rack_travel_limit : g.rear_rack_travel_limit, 0.),
         t = limit > 0 ? clip(travel, -limit, limit) : travel, x = ix + t * ax,
         y = iy + t * ay, arm = std::max(std::hypot(ox, oy), 1e-6),
         tie = std::max(std::hypot(ix - ox, iy - oy), 1e-6),
         dist = std::hypot(x, y);
  auto result = [&](bool valid, double delta, double angle, double rack,
                    double efficiency) {
    return json{{"valid", valid},         {"delta", delta},
                {"rack_travel", t},       {"arm_tie_angle", angle},
                {"tie_rack_angle", rack}, {"efficiency", efficiency}};
  };
  if (dist < 1e-9)
    return result(false, hint, 0, 0, 1e-6);
  double along = (arm * arm - tie * tie + dist * dist) / (2 * dist),
         h2 = arm * arm - along * along;
  if (h2 < -1e-9)
    return result(false, hint, 0, 0, 1e-6);
  double h = std::sqrt(std::max(h2, 0.)), theta = std::atan2(oy, ox), best = 0,
         score = INFINITY, bx = 0, by = 0;
  auto wrap = [](double d) {
    double v = std::fmod(d + pi, 2 * pi);
    if (v < 0)
      v += 2 * pi;
    return v - pi;
  };
  for (double sign : {1., -1.}) {
    double cx = x * (along / dist) - sign * h * y / dist,
           cy = y * (along / dist) + sign * h * x / dist,
           delta = wrap(std::atan2(cy, cx) - theta),
           s = std::abs(wrap(delta - hint));
    if (s < score) {
      score = s;
      best = delta;
      bx = cx;
      by = cy;
    }
  }
  double tx = x - bx, ty = y - by, la = std::max(std::hypot(bx, by), 1e-9),
         lt = std::max(std::hypot(tx, ty), 1e-9),
         dot = (bx * tx + by * ty) / (la * lt),
         cross = (bx * ty - by * tx) / (la * lt),
         proj = std::abs((tx * ax + ty * ay) / lt);
  return result(true, best, std::acos(clip(dot, -1, 1)),
                std::acos(clip(proj, 0, 1)),
                std::max(std::abs(cross) * proj, 1e-6));
}
std::optional<Response> Service::analysis(const std::string &path,
                                          const json &body) {
  if (path.rfind("/api/load-analysis/", 0) != 0 &&
      path.rfind("/api/model/demo/", 0) != 0)
    return {};
  json raw = live.sim->model.p;
  if (body.contains("params") && body["params"].is_object())
    merge(raw, body["params"]);
  auto p = store.params(json{{"overrides", raw}});
  int wheel = int(num(body, "wheel_index"));
  if (path == "/api/load-analysis/sweep")
    return Response(load_sweep(p, body));
  if (path == "/api/load-analysis/rack-solve") {
    json solutions = json::array();
    double hint = num(body, "hint_delta");
    auto travels = body.value("rack_travels", json::array());
    for (size_t i = 0; i < std::min<size_t>(512, travels.size()); i++) {
      auto sol = rack_solve(p, travels[i], wheel, hint);
      if (yes(sol, "valid"))
        hint = num(sol, "delta");
      solutions.push_back(sol);
    }
    return Response(json{{"wheel_index", wheel}, {"solutions", solutions}});
  }
  if (path == "/api/load-analysis/sensitivity") {
    json points = json::array();
    double speed = clip(num(body, "speed", 8), 0, std::max(p.v_max, .1));
    auto values = body.value("values", json::array());
    auto key = str(body, "vary_param");
    for (size_t i = 0; i < std::min<size_t>(64, values.size()); i++) {
      json candidate = p;
      try {
        set_dotted(candidate, key, values[i]);
        auto pp = candidate.get<VehicleParams>();
        auto request = body;
        request["speeds"] = json::array({speed});
        request["mode"] = "single_wheel";
        auto sweep = load_sweep(pp, request);
        auto eq = sweep["summary"]["per_speed_equilibrium"][0];
        points.push_back({{"value", values[i]},
                          {"delta_eq", eq["delta_eq"]},
                          {"delta_eq_deg", eq["delta_eq_deg"]},
                          {"rack_at_zero", eq["rack_at_zero"]},
                          {"torque_at_zero", eq["torque_at_zero"]},
                          {"found", eq["found"]}});
      } catch (const Error &) {
      }
    }
    return Response(
        json{{"vary_param", key},
             {"speed", speed},
             {"wheel_index", wheel},
             {"body_coupling", str(body, "body_coupling", "vehicle")},
             {"points", points}});
  }
  if (path == "/api/model/demo/bicycle-gain") {
    json speeds = body.value("speeds_kmh", json::array()),
         points = json::array();
    if (!body.contains("speeds_kmh"))
      for (int i = 0; i <= 200; i += 10)
        speeds.push_back(i);
    double delta = rad(num(body, "ref_delta_deg", 1)),
           peak = rad(num(body, "alpha_peak_deg", 7));
    auto wp = wheels(p);
    for (size_t i = 0; i < std::min<size_t>(80, speeds.size()); i++) {
      double v = std::max(speeds[i].get<double>() / 3.6, .001);
      V4 d{};
      d[wheel] = delta;
      auto ca = analysis_stiffness(p, analysis_loads(p, v));
      auto steady = steady_body(p, v, d, ca);
      double gain =
          std::abs((steady[0] + steady[1] * wp[wheel][0] / v - delta) / delta);
      points.push_back({{"speed_kmh", speeds[i]},
                        {"slip_gain", gain},
                        {"delta_sat_deg",
                         gain > 1e-9 ? json(peak / gain * 180 / pi) : json()}});
    }
    return Response(json{{"wheel_index", wheel},
                         {"ref_delta_deg", num(body, "ref_delta_deg", 1)},
                         {"points", points}});
  }
  if (path == "/api/model/demo/tire-curve") {
    double fz = num(body, "fz", 7000), mu = num(body, "mu", .85),
           maximum = rad(num(body, "alpha_max_deg", 15)),
           ca =
               p.tire_c_alpha *
               std::pow(std::max(fz / std::max(p.mass * 9.80665 / 4, 1.), .001),
                        p.tire_load_sensitivity_exp);
    int n = int(clip(num(body, "points", 81), 11, 241));
    double tp[] = {1,         ca,        p.tire_c_kappa, 0,         0, 0,
                   p.tire_cx, p.tire_cy, p.tire_ex,      p.tire_ey, .2};
    json curve = json::array();
    for (int i = 0; i < n; i++) {
      double a = -maximum + 2 * maximum * i / (n - 1);
      curve.push_back({{"alpha_deg", a * 180 / pi},
                       {"fy", sim4wis::tire(tp, a, 0, fz, mu).fy}});
    }
    return Response(json{{"fz", fz},
                         {"mu", mu},
                         {"c_alpha", ca},
                         {"mu_fz", mu * fz},
                         {"curve", curve}});
  }
  if (path == "/api/model/demo/kingpin-breakdown") {
    int n = int(clip(num(body, "points", 61), 11, 161));
    double maximum = rad(num(body, "angle_max_deg", 20)),
           speed = num(body, "speed_kmh", 30);
    json request = body;
    request["angles"] = json::array();
    for (int i = 0; i < n; i++)
      request["angles"].push_back(-maximum + 2 * maximum * i / (n - 1));
    request["speeds"] = json::array({speed / 3.6});
    request["mode"] = "single_wheel";
    auto sweep = load_sweep(p, request);
    json curve = json::array();
    for (auto &r : sweep["rows"])
      if (num(r, "wheel_index") == wheel)
        curve.push_back(
            {{"delta_cmd_deg", num(r, "delta_cmd") * 180 / pi},
             {"m_fy", num(r, "tire_fy") * p.tire_radius *
                          std::tan(p.suspension.caster_angle)},
             {"m_fx", num(r, "tire_fx") * p.suspension.scrub_radius},
             {"m_mz", -num(r, "tire_mz")},
             {"m_kpi",
              num(r, "fz") * std::sin(p.suspension.kingpin_inclination) *
                  p.suspension.scrub_radius * std::sin(num(r, "delta"))},
             {"torque_steer", r["torque_steer"]}});
    return Response(
        json{{"wheel_index", wheel}, {"speed_kmh", speed}, {"curve", curve}});
  }
  throw Error(404, "analysis operation not found");
}
} // namespace fourwis
