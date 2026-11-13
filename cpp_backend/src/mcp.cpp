#include "http.hpp"
#include "study.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>
namespace fourwis {
namespace {
std::string url(const std::string &s) {
  std::ostringstream out;
  out << std::hex << std::uppercase;
  for (unsigned char c : s) {
    if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~')
      out << c;
    else
      out << '%' << std::setw(2) << std::setfill('0') << int(c);
  }
  return out.str();
}
json invoke(const std::string &name, json args, int port,
            const fs::path &root) {
  std::string base = "http://127.0.0.1:" + std::to_string(port);
  auto request = [&](const std::string &method, const std::string &path,
                     const json &body = json()) {
    auto response = http_call(port, method, path, body);
    json data = json::parse(response.body, nullptr, false);
    if (data.is_discarded())
      data = response.body;
    if (response.status >= 400) {
      Error error(response.status, "backend request failed");
      error.detail = data;
      throw error;
    }
    return data;
  };
  json result;
  if (name == "describe_capabilities") {
    result = request("GET", "/api/study/capabilities");
    result["backend_version"] = request("GET", "/api/version")["version"];
    result["mcp_facade"] = {
        {"version", "0.107.0"}, {"base", base}, {"runtime", "cpp17"}};
    result["interaction"] = request("GET", "/api/agent/capabilities");
  } else if (name == "list_studies")
    result = request("GET", "/api/study");
  else if (name == "list_runs")
    result = request("GET", "/api/runs");
  else if (name == "get_study")
    result = request("GET", "/api/study/" + url(str(args, "study_id")));
  else if (name == "get_trace")
    result = request(
        "GET", "/api/study/trace/" + url(str(args, "run_id")) +
                   "?channels=" + url(str(args, "channels")) + "&max_points=" +
                   std::to_string(int(num(args, "max_points", 200))));
  else if (name == "read_report") {
    auto id = str(args, "study_id");
    auto study = request("GET", "/api/study/" + url(id));
    auto report = http_call(port, "GET", "/api/study/" + url(id) + "/report");
    result = {{"study_id", id},
              {"report_path", study.value("report_path", json())},
              {"served", report.status == 200}};
  } else if (name == "verify_golden") {
    Storage store(root);
    result = verify_golden(store);
  } else if (name == "create_session")
    result = request("POST", "/api/agent/sessions", args);
  else if (name == "list_sessions")
    result = request("GET", "/api/agent/sessions");
  else if (std::set<std::string>{"observe_session", "step_session",
                                 "set_fault_schedule", "export_session",
                                 "close_session"}
               .count(name)) {
    auto id = url(str(args, "session_id"));
    args.erase("session_id");
    std::string method = name == "observe_session" ? "GET"
                         : name == "close_session" ? "DELETE"
                                                   : "POST",
                suffix = name == "step_session"         ? "/step"
                         : name == "set_fault_schedule" ? "/faults"
                         : name == "export_session"     ? "/export"
                                                        : "";
    result = request(method, "/api/agent/sessions/" + id + suffix,
                     (name == "step_session" || name == "set_fault_schedule")
                         ? args
                         : json());
  } else if (name == "get_run_artifacts")
    result =
        request("GET", "/api/runs/" + url(str(args, "run_id")) + "/artifacts");
  else if (name == "get_study_job")
    result = request("GET", "/api/study/jobs/" + url(str(args, "job_id")));
  else if (name == "start_study")
    result = request("POST", "/api/study/run", args.at("spec"));
  else if (name == "run_study") {
    if (yes(args, "dry_run"))
      result = {
          {"dry_run", request("POST", "/api/study/dry-run", args.at("spec"))}};
    else {
      auto job = request("POST", "/api/study/run", args.at("spec"));
      auto id = str(job, "job_id");
      double deadline = epoch() + 600;
      for (;;) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        job = request("GET", "/api/study/jobs/" + id);
        auto status = str(job, "status");
        if (status == "error") {
          result = {
              {"job_id", id}, {"status", "error"}, {"error", job["error"]}};
          break;
        }
        if (status == "done") {
          auto data = job["result"];
          result = {{"job_id", id},
                    {"study_id", job["study_id"]},
                    {"report_path", data["report_path"]},
                    {"result", data}};
          break;
        }
        if (epoch() > deadline)
          throw Error(504, "study timeout; poll get_study_job with " + id);
      }
    }
  } else
    throw Error(404, "unknown MCP tool");
  if (result.is_object() && result.contains("outputs"))
    for (auto it = result["outputs"].begin(); it != result["outputs"].end();
         ++it) {
      auto value = it.value().get<std::string>();
      if (!value.empty() && value[0] == '/')
        it.value() = base + value;
    }
  return result;
}
} // namespace
int mcp_stdio(Service *, int port, const fs::path &root) {
  auto data = root / "cpp_backend/data";
  if (!fs::exists(data))
    data = root / "app/data";
  auto tools = read_json(data / "mcp_tools.json");
  for (auto &t : tools)
    if (str(t, "name") == "verify_golden")
      t["description"] =
          "Run all six frozen golden regression cases directly in C++, without "
          "Python; return exit code, metrics and failures.";
  bool initialized = false, ready = false;
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty())
      continue;
    json id = nullptr, response;
    try {
      if (line.size() > 32 * 1024 * 1024)
        throw Error(-32600, "MCP message too large");
      auto message = json::parse(line);
      if (!message.is_object() || str(message, "jsonrpc") != "2.0")
        throw Error(-32600, "invalid JSON-RPC message");
      id = message.value("id", json());
      auto method = str(message, "method");
      if (!message.contains("id")) {
        if (method == "notifications/initialized" && initialized)
          ready = true;
        continue;
      }
      auto params = message.value("params", json::object());
      json result;
      if (method == "initialize") {
        auto version = str(params, "protocolVersion");
        if (!std::set<std::string>{"2024-11-05", "2025-03-26", "2025-06-18",
                                   "2025-11-25"}
                 .count(version))
          version = "2025-11-25";
        result = {{"protocolVersion", version},
                  {"capabilities", {{"tools", {{"listChanged", false}}}}},
                  {"serverInfo",
                   {{"name", "sim4wis-mcp-cpp"}, {"version", "0.107.0"}}},
                  {"instructions",
                   "Use isolated Agent sessions for multi-point faults. "
                   "Built-in simulation and service operations run in C++. "
                   "Export data before closing a session."}};
        initialized = true;
      } else if (method == "ping")
        result = json::object();
      else {
        if (!ready)
          throw Error(-32000,
                      "initialize and send notifications/initialized first");
        if (method == "tools/list")
          result = {{"tools", tools}};
        else if (method == "tools/call") {
          auto name = str(params, "name");
          json schema;
          for (auto &t : tools)
            if (str(t, "name") == name)
              schema = t["inputSchema"];
          if (schema.is_null())
            throw Error(-32602, "unknown tool");
          try {
            auto args = validate_json(params.value("arguments", json::object()),
                                      schema, schema);
            auto output = invoke(name, args, port, root);
            bool error =
                output.is_object() &&
                (str(output, "status") == "failed" ||
                 str(output, "status") == "error" ||
                 (name == "verify_golden" && num(output, "exit_code") != 0));
            result = {{"content",
                       json::array(
                           {{{"type", "text"},
                             {"text",
                              output.dump(1, ' ', false,
                                          json::error_handler_t::replace)}}})},
                      {"structuredContent",
                       output.is_object() ? output : json{{"result", output}}},
                      {"isError", error}};
          } catch (const Error &e) {
            json error = {{"error",
                           {{"code", "backend_error"},
                            {"http_status", e.status},
                            {"detail", e.detail}}}};
            result = {{"content", json::array({{{"type", "text"},
                                                {"text", error.dump()}}})},
                      {"structuredContent", error},
                      {"isError", true}};
          } catch (const std::exception &e) {
            result = {
                {"content",
                 json::array({{{"type", "text"},
                               {"text", std::string("error: ") + e.what()}}})},
                {"isError", true}};
          }
        } else
          throw Error(-32601, "method not found");
      }
      response = {{"jsonrpc", "2.0"}, {"id", id}, {"result", result}};
    } catch (const json::parse_error &) {
      response = {{"jsonrpc", "2.0"},
                  {"id", nullptr},
                  {"error", {{"code", -32700}, {"message", "Parse error"}}}};
    } catch (const Error &e) {
      response = {{"jsonrpc", "2.0"},
                  {"id", id},
                  {"error", {{"code", e.status}, {"message", e.what()}}}};
    } catch (const std::exception &e) {
      response = {{"jsonrpc", "2.0"},
                  {"id", id},
                  {"error", {{"code", -32603}, {"message", e.what()}}}};
    }
    std::cout << response.dump(-1, ' ', false, json::error_handler_t::replace)
              << std::endl;
  }
  return 0;
}
} // namespace fourwis
