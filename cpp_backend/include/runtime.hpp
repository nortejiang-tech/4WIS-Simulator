#pragma once
#include "storage.hpp"
#include <functional>
namespace fourwis {
class AgentSession {
public:
  json config;
  std::string id, status = "ready", exported;
  int revision = 0, steps = 0, exported_revision = -1;
  json error = nullptr, command_log = json::array();
  double touched = epoch();
  std::unique_ptr<Simulation> sim;
  Run record;
  std::map<std::string, std::pair<std::string, json>> receipts;
  AgentSession(json config_, const Storage &store);
  json observe() const;
  json step(const json &request);
  json schedule(const json &request);
  Run result() const;
  std::string export_run(const Storage &store);
  json preview(int maximum) const;

private:
  std::optional<json> replay(const json &request, const std::string &digest);
  void receipt(const json &request, const std::string &digest,
               const json &response);
};
class Live {
public:
  Storage &store;
  std::unique_ptr<Simulation> sim;
  json plan = empty_plan(), scenario = nullptr;
  int path_version = 0, scenario_version = 0, push_counter = 0;
  bool paused = false;
  json simulation_error = nullptr;
  std::string input_owner;
  double input_seen = 0;
  bool recording = false, full_rate = false;
  int generation = 0, dropped = 0;
  json stop_reason = nullptr, recording_meta = json::object();
  Run record;
  std::vector<std::string> record_names, record_strategies;
  std::string saved_recording;
  json script = nullptr, script_error = nullptr;
  bool script_running = false, action_started = false;
  int action_index = -1, loop_count = 0;
  double script_start = 0, action_start = 0, last_elapsed = 0,
         next_script_tick = 0;
  V2 script_pose{};
  explicit Live(Storage &storage);
  void reset();
  void replace(const VehicleParams &p, const std::string &model);
  void strategy(const std::string &name);
  void release();
  void driver(const json &input);
  void tick();
  json state() const;
  json interaction() const;
  json script_status() const;
  void load_script(const json &definition);
  void start_script();
  void stop_script();
  void script_tick();
  json recording_status() const;
  void start_record(const json &config);
  void stop_record();
  void clear_record();
  void record_sample();
  std::string save_record();
  std::string csv(const std::vector<std::string> &names, double from, double to,
                  bool strategy_column) const;
};
} // namespace fourwis
