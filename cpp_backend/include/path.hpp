#pragma once
#include "common.hpp"
namespace fourwis {
using Points = std::vector<V2>;
Points resample(const Points &points, double step = .5);
Points catmull_rom(const Points &points, bool closed = false);
json empty_plan();
json plan_waypoints(const Points &points, bool closed = false,
                    const std::string &name = "custom");
json plan_template(const std::string &name,
                   const json &params = json::object());
void transform_plan(json &plan, double x, double y, double heading);
} // namespace fourwis
