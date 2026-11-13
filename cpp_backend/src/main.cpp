#include "http.hpp"
#include "study.hpp"
#include <csignal>
#include <iostream>
#ifdef _WIN32
#define NOMINMAX
#include <fcntl.h>
#include <io.h>
#include <windows.h>
#endif
namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
} // namespace
int run_main(int argc, char **argv) {
  using namespace fourwis;
  mg_init_library(0);
#ifdef _WIN32
  _setmode(_fileno(stdin), _O_BINARY);
  _setmode(_fileno(stdout), _O_BINARY);
#endif
#ifndef _WIN32
  std::signal(SIGPIPE, SIG_IGN);
#endif
  try {
    fs::path root = fs::absolute(fs::u8path(argv[0])).parent_path();
    for (int i = 0; i < 5 && !fs::exists(root / "cpp_backend/data") &&
                    !fs::exists(root / "app/data");
         i++)
      root = root.parent_path();
    int port = 8020;
    std::string mode = "serve", input;
    for (int i = 1; i < argc; i++) {
      std::string arg = argv[i];
      if (arg == "--root" && i + 1 < argc)
        root = fs::absolute(fs::u8path(argv[++i]));
      else if (arg == "--port" && i + 1 < argc)
        port = std::stoi(argv[++i]);
      else if (arg == "--mcp")
        mode = "mcp";
      else if (arg == "--self-test")
        mode = "self-test";
      else if (arg == "--probe-api")
        mode = "probe-api";
      else if (arg == "--print-config")
        mode = "config";
      else if (arg == "--run-experiment" && i + 1 < argc) {
        mode = "experiment";
        input = argv[++i];
      } else if (arg == "--version") {
        std::cout << "0.107.0 cpp17\n";
        return 0;
      } else if (arg == "--serve")
        mode = "serve";
      else if (arg == "--help") {
        std::cout << "sim4wis-cpp [--root PATH] [--port 8020] "
                     "[--serve|--mcp|--print-config|--self-test|--probe-api|--"
                     "run-experiment FILE]\n";
        return 0;
      } else
        throw Error(422, "unknown command-line argument: " + arg);
    }
    if (mode == "config") {
      json config = {
          {"mcpServers",
           {{"4wis-cpp",
             {{"command", fs::absolute(fs::u8path(argv[0])).u8string()},
              {"args", json::array({"--mcp", "--root", root.u8string(),
                                    "--port", std::to_string(port)})}}}}}};
      std::cout << config.dump(2) << '\n';
      return 0;
    }
    if (mode == "mcp") {
      bool connected = false;
      try {
        auto response = http_call(port, "GET", "/api/version");
        if (response.status == 200) {
          auto version = json::parse(response.body);
          if (str(version, "version") != "0.107.0" ||
              str(version.value("engine", json::object()), "service_runtime") !=
                  "cpp17")
            throw Error(409, "port is occupied by a different backend; select "
                             "another --port");
          connected = true;
        }
      } catch (const Error &e) {
        if (e.status != 503)
          throw;
      }
      if (connected)
        return mcp_stdio(nullptr, port, root);
      Service service(root);
      HttpServer server(service, port);
      std::cerr << "Started C++ MCP backend on 127.0.0.1:" << port << '\n';
      return mcp_stdio(&service, port, root);
    }
    Service service(root);
    if (mode == "self-test") {
      auto result = verify_golden(service.store);
      std::cout << result.dump(2, ' ', false, json::error_handler_t::replace)
                << '\n';
      return result["exit_code"].get<int>();
    }
    if (mode == "experiment") {
      auto definition =
          input == "-" ? json::parse(std::cin) : yaml_json(fs::u8path(input));
      auto exp = service.store.experiment(definition);
      Simulation sim(exp, service.store.params(exp["vehicle"]));
      auto run = sim.run();
      auto result = run.output();
      result["kpis"] = kpis(run, exp);
      std::cout << result.dump() << '\n';
      return 0;
    }
    if (mode == "probe-api") {
      std::string line;
      while (std::getline(std::cin, line)) {
        try {
          auto r = json::parse(line);
          auto response = service.request(
              str(r, "method", "GET"), str(r, "path"), r.value("body", json()),
              r.value("query", std::map<std::string, std::string>{}));
          json body = json::parse(response.body, nullptr, false);
          if (body.is_discarded())
            body = response.body;
          std::cout << json{{"status", response.status},
                            {"content_type", response.content_type},
                            {"body", body}}
                           .dump()
                    << std::endl;
        } catch (const Error &e) {
          std::cout << json{{"status", e.status},
                            {"body", {{"detail", e.detail}}}}
                           .dump()
                    << std::endl;
        } catch (const std::exception &e) {
          std::cout
              << json{{"status", 500}, {"body", {{"detail", e.what()}}}}.dump()
              << std::endl;
        }
      }
      return 0;
    }
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    {
      HttpServer server(service, port);
      std::cerr << "4WIS C++ 0.107.0 ready: http://127.0.0.1:" << port << '\n';
      while (!stopped)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "4WIS C++: " << e.what() << '\n';
    return 1;
  }
}

#ifdef _WIN32
int wmain(int argc, wchar_t **argv) {
  std::vector<std::string> utf8;
  for (int i = 0; i < argc; ++i) {
    int length = WideCharToMultiByte(CP_UTF8, 0, argv[i], -1, nullptr, 0,
                                     nullptr, nullptr);
    if (length <= 0)
      return 1;
    std::string value(length, '\0');
    WideCharToMultiByte(CP_UTF8, 0, argv[i], -1, value.data(), length, nullptr,
                        nullptr);
    value.pop_back();
    utf8.push_back(std::move(value));
  }
  std::vector<char *> pointers;
  for (auto &value : utf8)
    pointers.push_back(value.data());
  pointers.push_back(nullptr);
  return run_main(argc, pointers.data());
}
#else
int main(int argc, char **argv) { return run_main(argc, argv); }
#endif
