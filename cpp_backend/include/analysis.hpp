#pragma once
#include "storage.hpp"
namespace fourwis {
json load_sweep(const VehicleParams &p, const json &request);
json rack_solve(const VehicleParams &p, double travel, int wheel, double hint);
V2 steady_body(const VehicleParams &p, double speed, const V4 &delta,
               const V4 &ca);
V4 analysis_stiffness(const VehicleParams &p, const V4 &fz);
std::array<double, 28> linkage_metrics(const VehicleParams &p, const V4 &delta);
} // namespace fourwis
