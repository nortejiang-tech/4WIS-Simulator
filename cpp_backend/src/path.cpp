#include "path.hpp"
#include <iomanip>
#include <sstream>
namespace fourwis {
static const char *ORANGE = "#f97316", *RED = "#dc2626", *GREEN = "#22c55e",
                  *YELLOW = "#eab308", *BLUE = "#38bdf8", *PAINT = "#e8e8e8",
                  *DIM = "#94a3b8";
static std::string fmt(double x) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(2) << x;
  auto r = s.str();
  while (r.size() && r.back() == '0')
    r.pop_back();
  if (r.size() && r.back() == '.')
    r.pop_back();
  return r;
}
Points resample(const Points &pts, double step) {
  if (pts.size() < 2)
    return pts;
  std::vector<double> s{0}, x, y;
  for (size_t i = 1; i < pts.size(); i++)
    s.push_back(s.back() + std::hypot(pts[i][0] - pts[i - 1][0],
                                      pts[i][1] - pts[i - 1][1]));
  double total = s.back();
  if (total < 1e-9)
    return {pts[0]};
  for (auto p : pts) {
    x.push_back(p[0]);
    y.push_back(p[1]);
  }
  int n = std::max(2, int(std::nearbyint(total / step)) + 1);
  Points out;
  for (int i = 0; i < n; i++) {
    double u = total * double(i) / (n - 1);
    out.push_back({interp(u, s, x), interp(u, s, y)});
  }
  return out;
}
Points catmull_rom(const Points &pts, bool closed) {
  if (pts.size() < 3)
    return pts;
  Points ext;
  ext.push_back(closed ? pts.back() : pts.front());
  ext.insert(ext.end(), pts.begin(), pts.end());
  ext.push_back(closed ? pts.front() : pts.back());
  if (closed)
    ext.push_back(pts[1]);
  Points out;
  for (size_t i = 0; i + 3 < ext.size(); i++)
    for (int j = 0; j < 16; j++) {
      double t = double(j) / 16, t2 = t * t, t3 = t * t * t;
      V2 v;
      for (int k = 0; k < 2; k++)
        v[k] = .5 * (2 * ext[i + 1][k] + (-ext[i][k] + ext[i + 2][k]) * t +
                     (2 * ext[i][k] - 5 * ext[i + 1][k] + 4 * ext[i + 2][k] -
                      ext[i + 3][k]) *
                         t2 +
                     (-ext[i][k] + 3 * ext[i + 1][k] - 3 * ext[i + 2][k] +
                      ext[i + 3][k]) *
                         t3);
      out.push_back(v);
    }
  out.push_back(ext[ext.size() - 2]);
  return out;
}
json empty_plan() {
  return {{"name", ""},
          {"label", ""},
          {"notes", ""},
          {"closed", false},
          {"points", json::array()},
          {"cones", json::array()},
          {"marks", json::array()}};
}
json plan_waypoints(const Points &pts, bool closed, const std::string &name) {
  json p = empty_plan();
  p["name"] = name;
  p["label"] = "自定义航点";
  p["closed"] = closed;
  p["points"] = resample(catmull_rom(pts, closed));
  return p;
}
static json cone(double x, double y, const char *color = ORANGE,
                 const char *kind = "cone", double height = .75) {
  return {
      {"x", x}, {"y", y}, {"kind", kind}, {"color", color}, {"height", height}};
}
static json mark(const Points &pts, const char *color = PAINT,
                 double width = .12, bool dash = false) {
  return {{"points", pts}, {"color", color}, {"width", width}, {"dash", dash}};
}
static void gate(json &cs, double x, double lo, double hi, const char *color) {
  cs.push_back(cone(x, hi, color));
  cs.push_back(cone(x, lo, color));
}
static json cross(double x, double lo, double hi, const char *color = PAINT,
                  double width = .2) {
  return mark({{x, lo}, {x, hi}}, color, width);
}
static json box(double a, double b, double lo, double hi,
                const char *color = PAINT, double width = .12) {
  return mark({{a, lo}, {b, lo}, {b, hi}, {a, hi}, {a, lo}}, color, width);
}
static void row(json &cs, double x, double y, double u, double v,
                double step = 3, const char *color = ORANGE) {
  int n = std::max(
      1, int(std::nearbyint(std::hypot(u - x, v - y) / std::max(step, .5))));
  for (int i = 0; i <= n; i++)
    cs.push_back(cone(x + (u - x) * i / n, y + (v - y) * i / n, color));
}
static void cones_arc(json &cs, double cx, double cy, double r, double a,
                      double b, double step = 6, const char *color = ORANGE) {
  int n = std::max(
      2, int(std::nearbyint(std::abs(r * (b - a)) / std::max(step, .5))));
  for (int i = 0; i < n; i++) {
    double th = a + (b - a) * i / n;
    cs.push_back(cone(cx + r * std::cos(th), cy + r * std::sin(th), color));
  }
}
static Points arc_points(double cx, double cy, double r, double a, double b,
                         int n = 96) {
  Points out;
  for (int i = 0; i <= n; i++) {
    double th = a + (b - a) * i / n;
    out.push_back({cx + r * std::cos(th), cy + r * std::sin(th)});
  }
  return out;
}
static void lane_course(json &p,
                        const std::vector<std::array<double, 3>> &sections,
                        const std::vector<double> &gaps, double entry,
                        double runout, double step = 3) {
  auto &cs = p["cones"];
  auto &ms = p["marks"];
  Points wp = {{0, 0}, {entry * .6, 0}};
  double x = entry;
  std::vector<V4> spans;
  for (size_t i = 0; i < sections.size(); i++) {
    auto s = sections[i];
    spans.push_back({x, x + s[0], s[2], s[1] / 2});
    x += s[0] + (i < gaps.size() ? gaps[i] : 0);
  }
  for (size_t i = 0; i < spans.size(); i++) {
    auto s = spans[i];
    double a = s[0], b = s[1], cy = s[2], hw = s[3];
    auto color = std::abs(cy) > 1e-6 ? YELLOW : ORANGE;
    row(cs, a, cy + hw, b, cy + hw, step, color);
    row(cs, a, cy - hw, b, cy - hw, step, color);
    if (i == 0)
      gate(cs, a, cy - hw, cy + hw, GREEN);
    if (i + 1 == spans.size())
      gate(cs, b, cy - hw, cy + hw, RED);
    ms.push_back(box(a, b, cy - hw, cy + hw));
    wp.push_back({a, cy});
    wp.push_back({(a + b) / 2, cy});
    wp.push_back({b, cy});
    if (i + 1 < spans.size())
      wp.push_back({(b + spans[i + 1][0]) / 2, (cy + spans[i + 1][2]) / 2});
  }
  ms.push_back(cross(0, -1.5, 1.5, PAINT, .3));
  ms.push_back(mark({{0, 0}, {entry, 0}}, DIM, .1, true));
  wp.push_back({spans.back()[1] + runout, spans.back()[2]});
  p["points"] = resample(catmull_rom(wp));
}
json plan_template(const std::string &name, const json &k) {
  json p = empty_plan();
  p["name"] = name;
  auto &cs = p["cones"];
  auto &ms = p["marks"];
  Points pts;
  double width = num(k, "lane_width", name == "straight" ? 3.5 : 4),
         hw = std::max(1.5, width) / 2, b = num(k, "vehicle_width", 1.78);
  if (name == "straight") {
    double len = std::max(20., num(k, "length", 120)),
           spacing = num(k, "gate_spacing", 20);
    pts = resample({{0, 0}, {len, 0}});
    ms = {mark({{0, hw}, {len, hw}}, PAINT, .14),
          mark({{0, -hw}, {len, -hw}}, PAINT, .14),
          mark({{0, 0}, {len, 0}}, DIM, .1, true), cross(0, -hw, hw, PAINT, .3),
          cross(len, -hw, hw, PAINT, .3)};
    for (double x = 0; x <= len + 1e-6; x += std::max(5., spacing))
      gate(cs, x, -hw, hw, x < 1e-6 ? GREEN : x >= len - 1e-6 ? RED : ORANGE);
    if (!cs.empty() && std::abs(num(cs.back(), "x") - len) > 1e-6)
      gate(cs, len, -hw, hw, RED);
    p["label"] = "直线";
    p["notes"] = "直线 " + fmt(len) + " m · 车道宽 " + fmt(width) + " m · 每 " +
                 fmt(spacing) + " m 一道门（绿=起点，红=终点）";
  } else if (name == "arc" || name == "skidpad") {
    bool closed = name == "skidpad";
    double r = std::max(closed ? 6. : 4., num(k, "radius", closed ? 30 : 25)),
           sweep = num(k, "sweep_deg", 180), sw = rad(sweep);
    int n = closed ? std::max(48, int(2 * pi * r / .5))
                   : std::max(8, int(std::abs(r * sw) / .5));
    for (int i = 0; i < n; i++) {
      double th = closed ? -pi / 2 + (2 * pi / n) * i : (sw / (n - 1)) * i;
      pts.push_back(closed ? V2{r * std::cos(th), r + r * std::sin(th)}
                           : V2{r * std::sin(th), r * (1 - std::cos(th))});
    }
    double a = closed ? 0 : -pi / 2, z = closed ? 2 * pi : -pi / 2 + sw;
    ms = {mark(arc_points(0, r, r + hw, a, z), PAINT, .14),
          mark(arc_points(0, r, r - hw, a, z), PAINT, .14),
          mark(arc_points(0, r, r, a, z), DIM, .1, true)};
    if (closed)
      ms.push_back(cross(0, -hw, hw, PAINT, .3));
    cones_arc(cs, 0, r, r + hw, a, z, 8);
    cones_arc(cs, 0, r, r - hw, a, z, 8, closed ? BLUE : ORANGE);
    gate(cs, 0, -hw, hw, GREEN);
    p["closed"] = closed;
    p["label"] = closed ? "定圆 (ISO 4138)" : "圆弧";
    p["notes"] = closed ? "稳态回转 R=" + fmt(r) + " m（中线）· 车道宽 " +
                              fmt(width) + " m · 外圈橙桩 / 内圈蓝桩"
                        : "定半径 R=" + fmt(r) + " m · 转角 " + fmt(sweep) +
                              "° · 车道宽 " + fmt(width) + " m";
  } else if (name == "slalom") {
    int n = std::max(2, int(num(k, "n_gates", 6)));
    double spacing = std::max(6., num(k, "spacing", 18)),
           entry = std::max(5., num(k, "entry", 20)),
           off = num(k, "offset") > 0 ? num(k, "offset") : b / 2 + 1.2,
           runout = entry + (n - 1) * spacing + spacing, gh = off + 1.5;
    for (int i = 0; i < n; i++)
      cs.push_back(cone(entry + i * spacing, 0,
                        i == 0       ? GREEN
                        : i == n - 1 ? RED
                                     : "#ef4444",
                        "pole", 1.5));
    gate(cs, entry - spacing / 2, -gh, gh, GREEN);
    gate(cs, runout, -gh, gh, RED);
    ms = {mark({{0, 0}, {runout + 10, 0}}, DIM, .1, true),
          cross(entry - spacing / 2, -gh, gh, PAINT, .25),
          cross(runout, -gh, gh, PAINT, .25)};
    Points wp = {{0, 0}, {entry - spacing / 2, 0}};
    for (int i = 0; i < n; i++) {
      wp.push_back({entry + i * spacing, (i % 2 == 0 ? 1 : -1) * off});
      wp.push_back({entry + i * spacing + spacing / 2, 0});
    }
    wp.push_back({runout + 10, 0});
    pts = resample(catmull_rom(wp));
    p["label"] = "绕桩 (Slalom)";
    p["notes"] = std::to_string(n) + " 根标杆 · 桩距 " + fmt(spacing) +
                 " m · 路线侧向幅值 ±" + fmt(off) + " m（车宽 " + fmt(b) +
                 " m）· 引入段 " + fmt(entry) + " m";
  } else if (name == "lane_change" || name == "double_lane_change" ||
             name == "iso3888_1") {
    b = std::max(1.2, b);
    double w1 = 1.1 * b + .25, w2 = name == "iso3888_1" ? 1.2 * b + .25 : b + 1,
           w3 = 1.3 * b + .25,
           off = name == "double_lane_change" ? w1 / 2 + 1 + w2 / 2 : 3.5,
           entry = num(k, "entry",
                       name == "double_lane_change" ? 15
                       : name == "iso3888_1"        ? 20
                                                    : 25);
    if (name == "lane_change") {
      off = num(k, "offset", 3.5);
      double transition = num(k, "transition", 25);
      lane_course(p, {{20, w1, 0}, {25, w1, off}}, {std::max(8., transition)},
                  entry, 20);
      p["label"] = "单移线";
      p["notes"] = "单移线 · 车道宽 " + fmt(w1) + " m（车宽 " + fmt(b) +
                   " m）· 横向偏移 " + fmt(off) + " m · 换道段 " +
                   fmt(transition) + " m";
    } else {
      bool iso = name == "iso3888_1";
      if (iso)
        lane_course(p, {{15, w1, 0}, {25, w2, off}, {15, w3, 0}}, {30, 25},
                    entry, 25, 4);
      else
        lane_course(p, {{12, w1, 0}, {11, w2, off}, {12, w3, 0}}, {13.5, 12.5},
                    entry, 20);
      p["label"] = iso ? "双移线 · ISO 3888-1" : "双移线 · ISO 3888-2 (麋鹿)";
      p["notes"] =
          std::string(iso ? "ISO 3888-1" : "ISO 3888-2") + " · 车宽 " + fmt(b) +
          " m → 段1 " + fmt(w1) + " m / 侧道 " + fmt(w2) + " m / 段3 " +
          fmt(w3) + " m · 横向偏移 " + fmt(off) + " m · 段长 " +
          (iso ? "15/25/15 m，间隔 30/25 m" : "12/11/12 m，间隔 13.5/12.5 m");
    }
    return p;
  } else if (name == "figure_eight") {
    double r = std::max(5., num(k, "radius", 15));
    int n = std::max(24, int(2 * pi * r / .5));
    for (int i = 0; i < n; i++) {
      double th = (2 * pi / n) * i;
      pts.push_back({r * std::sin(th), r - r * std::cos(th)});
    }
    for (int i = n - 1; i >= 0; i--) {
      double th = (2 * pi / n) * i;
      pts.push_back({r * std::sin(th), -r + r * std::cos(th)});
    }
    double ri = std::max(2., r - 2);
    cones_arc(cs, 0, r, ri, 0, 2 * pi);
    cones_arc(cs, 0, -r, ri, 0, 2 * pi, 6, BLUE);
    cs.push_back(cone(0, 0, GREEN, "pole", 1.5));
    ms = {mark(arc_points(0, r, ri, 0, 2 * pi), DIM, .1, true),
          mark(arc_points(0, -r, ri, 0, 2 * pi), DIM, .1, true)};
    p["closed"] = true;
    p["label"] = "八字";
    p["notes"] =
        "两个相切圆 R=" + fmt(r) + " m · 绕内圈桩阵行驶（橙/蓝各一圈）";
  } else if (name == "parking") {
    double lv = std::max(2., num(k, "vehicle_length", 3.79)),
           bv = std::max(1.2, b), margin = num(k, "margin", 1.5),
           len = lv + std::max(.5, margin), w = bv + .8,
           approach = std::max(5., num(k, "approach", 14)), far = -3,
           near = far + w, slot = (far + near) / 2, a = approach, z = a + len;
    ms = {mark({{0, far}, {z + 12, far}}, PAINT, .16),
          mark({{0, 0}, {z + 12, 0}}, DIM, .1, true),
          box(a, z, far, near, PAINT, .14)};
    cs = {cone(a, far, GREEN), cone(z, far, RED),
          cone(a, near, GREEN, "pole", 1.2), cone(z, near, RED, "pole", 1.2)};
    pts = resample(catmull_rom({{0, 0},
                                {approach * .7, 0},
                                {a + len * .35, slot * .45},
                                {a + len * .75, slot},
                                {z - .4, slot}}),
                   .3);
    p["label"] = "侧方停车";
    p["notes"] = "车位 " + fmt(len) + " × " + fmt(w) + " m（车长 " + fmt(lv) +
                 " + " + fmt(margin) + " / 车宽 " + fmt(bv) +
                 " + 0.8）· 跟踪策略只能前进入位，倒库请手动驾驶";
  } else
    throw Error(404, "unknown path template: " + name);
  p["points"] = pts;
  return p;
}
void transform_plan(json &p, double dx, double dy, double heading) {
  double c = std::cos(heading), s = std::sin(heading);
  auto trans = [&](double x, double y) {
    return V2{x * c - y * s + dx, x * s + y * c + dy};
  };
  for (auto &point : p["points"]) {
    auto v = point.get<V2>();
    point = trans(v[0], v[1]);
  }
  for (auto &cone : p["cones"]) {
    auto v = trans(num(cone, "x"), num(cone, "y"));
    cone["x"] = v[0];
    cone["y"] = v[1];
  }
  for (auto &mark : p["marks"])
    for (auto &point : mark["points"]) {
      auto v = point.get<V2>();
      point = trans(v[0], v[1]);
    }
}
} // namespace fourwis
