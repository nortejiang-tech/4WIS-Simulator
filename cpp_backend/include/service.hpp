#pragma once
#include "runtime.hpp"
#include <atomic>
#include <thread>
namespace fourwis {
struct Response {
  int status = 200;
  std::string content_type = "application/json; charset=utf-8", body;
  Response(json data, int code = 200)
      : status(code),
        body(data.dump(-1, ' ', false, json::error_handler_t::replace)) {}
  Response(std::string data, std::string mime, int code = 200)
      : status(code), content_type(std::move(mime)), body(std::move(data)) {};
};
struct Job {
  json data;
  double started = epoch(), finished = 0;
  std::atomic<bool> cancel{false};
};
class Service {
public:
  Storage store;
  Live live;
  std::recursive_mutex mutex;
  std::map<std::string, std::shared_ptr<AgentSession>> sessions;
  std::map<std::string, std::shared_ptr<Job>> jobs, study_jobs;
  std::vector<std::thread> workers;
  std::atomic<bool> running{true};
  size_t subscribers = 0;
  json scenarios;
  std::unique_ptr<Controller> python_probe;
  explicit Service(const fs::path &root) : store(root), live(store) {
    auto data = root / "cpp_backend/data";
    if (!fs::exists(data))
      data = root / "app/data";
    scenarios = read_json(data / "scenarios.json");
    configure_plugins(root);
  }
  ~Service() {
    running = false;
    for (auto &j : jobs)
      j.second->cancel = true;
    for (auto &t : workers)
      if (t.joinable())
        t.join();
  }
  Response request(const std::string &method, const std::string &path,
                   json body = json(),
                   const std::map<std::string, std::string> &query = {});
  void ws(const json &body, const std::string &owner);
  void disconnect(const std::string &owner);
  json validate_body(const std::string &method, const std::string &path,
                     const json &body) const;
  std::string batch(const json &request);
  json job_status(const std::shared_ptr<Job> &job) const;
  std::optional<Response> analysis(const std::string &path, const json &body);
  std::optional<Response>
  studies(const std::string &method, const std::vector<std::string> &parts,
          const json &body, const std::map<std::string, std::string> &query);
};
std::vector<std::string> split(const std::string &value, char delimiter);
void set_dotted(json &object, const std::string &path, const json &value);
} // namespace fourwis
