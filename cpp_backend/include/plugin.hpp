#pragma once
#include "state.hpp"
namespace fourwis {
void configure_plugins(const std::filesystem::path &root);
json discover_plugins();
std::vector<std::string> plugin_names();
class PluginWorker {
  struct Process;
  std::unique_ptr<Process> process;

public:
  PluginWorker(const std::string &name, const VehicleParams &params);
  ~PluginWorker();
  json request(const json &message, double timeout = 3);
  Command compute(const Driver &driver, const State &state, double dt);
  json status(bool reload = false);
};
} // namespace fourwis
