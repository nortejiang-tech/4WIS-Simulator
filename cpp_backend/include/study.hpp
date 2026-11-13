#pragma once
#include "metrics.hpp"
#include <atomic>
namespace fourwis {
std::string canonical(const json &value);
json normalize_study(const Storage &store, json spec);
json dry_study(const Storage &store, const json &spec);
std::pair<std::string, json>
run_study(const Storage &store, const json &spec,
          const std::atomic<bool> *running = nullptr);
json target_sets(const Storage &store);
json target_set(const Storage &store, const std::string &ref);
json target_catalog(const Storage &store);
json compliance(const json &target, const json &measurements);
json sizing(const Storage &store, const VehicleParams &p);
std::string study_html(const json &result);
} // namespace fourwis
namespace fourwis {
json verify_golden(const Storage &store);
}
