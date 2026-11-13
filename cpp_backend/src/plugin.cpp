#include "plugin.hpp"
#include "storage.hpp"
#include <cerrno>
#include <chrono>
#include <thread>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <signal.h>
#include <spawn.h>
#include <sys/select.h>
#include <sys/wait.h>
#include <unistd.h>
extern char **environ;
#endif
namespace fourwis {
namespace {
fs::path plugin_root;
fs::path directory() {
  if (const char *v = std::getenv("SIM4WIS_PLUGINS_DIR"))
    return fs::u8path(v);
  const char *base = std::getenv("SIM4WIS_DATA_DIR");
  return (base ? fs::u8path(base) : plugin_root) / "plugins/strategies";
}
fs::path python() {
  if (const char *v = std::getenv("SIM4WIS_PLUGIN_PYTHON"))
    return fs::u8path(v);
  for (auto &p : {plugin_root / "app/python/python.exe",
                  plugin_root / "app/python/bin/python3",
                  plugin_root / "backend/.venv/bin/python",
                  plugin_root / "backend/.venv/Scripts/python.exe"})
    if (fs::exists(p))
      return p;
  throw Error(503, "Custom Python/FMU/MATLAB plugins require the optional "
                   "compatibility runtime; set SIM4WIS_PLUGIN_PYTHON.");
}
} // namespace
void configure_plugins(const fs::path &root) { plugin_root = root; }
json discover_plugins() {
  json items = json::array();
  int loaded = 0;
  auto root = directory();
  if (fs::exists(root)) {
    std::vector<fs::path> files;
    for (auto &p : fs::directory_iterator(root))
      if (p.is_regular_file() &&
          (p.path().extension() == ".fmu" || p.path().extension() == ".slx"))
        files.push_back(p.path());
    std::sort(files.begin(), files.end());
    for (auto &path : files) {
      json row = {{"name", path.stem().u8string()},
                  {"path", path.u8string()},
                  {"ok", false},
                  {"error", nullptr}};
      auto sidecar = path;
      sidecar += ".yaml";
      try {
        if (!fs::exists(sidecar))
          throw Error(400, "missing sidecar " + sidecar.filename().u8string());
        auto config = yaml_json(sidecar);
        row["name"] = str(config, "name", path.stem().u8string());
        row["ok"] = true;
        loaded++;
      } catch (const std::exception &e) {
        row["error"] = e.what();
      }
      items.push_back(row);
    }
  }
  return {{"total", items.size()},
          {"loaded", loaded},
          {"failed", items.size() - loaded},
          {"items", items}};
}
std::vector<std::string> plugin_names() {
  std::vector<std::string> out;
  auto report = discover_plugins();
  for (auto &item : report["items"])
    if (yes(item, "ok"))
      out.push_back(str(item, "name"));
  return out;
}
struct PluginWorker::Process {
#ifdef _WIN32
  HANDLE input = nullptr, output = nullptr, child = nullptr;
#else
  int input = -1, output = -1;
  pid_t child = -1;
#endif
  std::string pending;
  ~Process() {
#ifdef _WIN32
    if (input)
      CloseHandle(input);
    if (child) {
      if (WaitForSingleObject(child, 200) != WAIT_OBJECT_0) {
        TerminateProcess(child, 1);
        WaitForSingleObject(child, 1000);
      }
      CloseHandle(child);
    }
    if (output)
      CloseHandle(output);
#else
    if (input >= 0)
      close(input);
    if (child > 0) {
      int status;
      bool done = false;
      for (int i = 0; i < 20; i++) {
        if (waitpid(child, &status, WNOHANG) != 0) {
          done = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if (!done) {
        kill(child, SIGKILL);
        waitpid(child, &status, 0);
      }
    }
    if (output >= 0)
      close(output);
#endif
  }
  void start(const fs::path &executable, const fs::path &script) {
#ifdef _WIN32
    SECURITY_ATTRIBUTES security{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
    HANDLE child_in = nullptr, child_out = nullptr;
    if (!CreatePipe(&child_in, &input, &security, 0) ||
        !CreatePipe(&output, &child_out, &security, 0))
      throw Error(503, "Cannot create plugin pipes");
    SetHandleInformation(input, HANDLE_FLAG_INHERIT, 0);
    SetHandleInformation(output, HANDLE_FLAG_INHERIT, 0);
    auto quote = [](const std::wstring &s) {
      std::wstring out = L"\"";
      size_t slashes = 0;
      for (wchar_t c : s) {
        if (c == L'\\') {
          slashes++;
          continue;
        }
        out.append(slashes * (c == L'"' ? 2 : 1), L'\\');
        slashes = 0;
        if (c == L'"')
          out += L'\\';
        out += c;
      }
      out.append(slashes * 2, L'\\');
      return out + L"\"";
    };
    std::wstring command = quote(executable.wstring()) + L" -u " +
                           quote(script.wstring()) + L" --root " +
                           quote(plugin_root.wstring());
    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    startup.dwFlags = STARTF_USESTDHANDLES;
    startup.hStdInput = child_in;
    startup.hStdOutput = child_out;
    startup.hStdError = GetStdHandle(STD_ERROR_HANDLE);
    PROCESS_INFORMATION info{};
    bool ok = CreateProcessW(executable.c_str(), command.data(), nullptr,
                             nullptr, TRUE, CREATE_NO_WINDOW, nullptr,
                             plugin_root.c_str(), &startup, &info);
    CloseHandle(child_in);
    CloseHandle(child_out);
    if (!ok)
      throw Error(503, "Cannot start plugin runtime");
    child = info.hProcess;
    CloseHandle(info.hThread);
#else
    int in[2], out[2];
    if (pipe(in) < 0)
      throw Error(503, "Cannot create plugin input pipe");
    if (pipe(out) < 0) {
      close(in[0]);
      close(in[1]);
      throw Error(503, "Cannot create plugin output pipe");
    }
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, in[0], STDIN_FILENO);
    posix_spawn_file_actions_adddup2(&actions, out[1], STDOUT_FILENO);
    for (int fd : {in[0], in[1], out[0], out[1]})
      posix_spawn_file_actions_addclose(&actions, fd);
    std::string exe = executable.string(), file = script.string(),
                root = plugin_root.string();
    char *args[] = {exe.data(),  const_cast<char *>("-u"),
                    file.data(), const_cast<char *>("--root"),
                    root.data(), nullptr};
    int result =
        posix_spawn(&child, exe.c_str(), &actions, nullptr, args, environ);
    posix_spawn_file_actions_destroy(&actions);
    close(in[0]);
    close(out[1]);
    input = in[1];
    output = out[0];
    if (result != 0)
      throw Error(503, "Cannot start plugin runtime");
#endif
  }
  void write(const std::string &text) {
    size_t pos = 0;
    while (pos < text.size()) {
#ifdef _WIN32
      DWORD n = 0;
      if (!WriteFile(input, text.data() + pos, DWORD(text.size() - pos), &n,
                     nullptr) ||
          n == 0)
        throw Error(503, "Plugin pipe closed");
#else
      auto n = ::write(input, text.data() + pos, text.size() - pos);
      if (n < 0 && errno == EINTR)
        continue;
      if (n <= 0)
        throw Error(503, "Plugin pipe closed");
#endif
      pos += size_t(n);
    }
  }
  std::string read(double timeout) {
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(int(timeout * 1000));
    for (;;) {
      auto end = pending.find('\n');
      if (end != std::string::npos) {
        auto line = pending.substr(0, end);
        pending.erase(0, end + 1);
        return line;
      }
      if (std::chrono::steady_clock::now() >= deadline)
        throw Error(504, "Plugin computation timed out");
      char bytes[8192];
#ifdef _WIN32
      DWORD count = 0, n = 0;
      if (!PeekNamedPipe(output, nullptr, 0, nullptr, &count, nullptr))
        throw Error(503, "Plugin process terminated");
      if (!count) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      if (!ReadFile(output, bytes, std::min<DWORD>(count, sizeof(bytes)), &n,
                    nullptr) ||
          n == 0)
        throw Error(503, "Plugin process terminated");
#else
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(output, &fds);
      timeval tv{0, 20000};
      int ready = select(output + 1, &fds, nullptr, nullptr, &tv);
      if (ready < 0 && errno == EINTR)
        continue;
      if (ready < 0)
        throw Error(503, "Plugin pipe failed");
      if (!ready)
        continue;
      auto n = ::read(output, bytes, sizeof(bytes));
      if (n <= 0)
        throw Error(503, "Plugin process terminated");
#endif
      pending.append(bytes, size_t(n));
      if (pending.size() > 4 * 1024 * 1024)
        throw Error(413, "Plugin response exceeds limit");
    }
  }
};
PluginWorker::PluginWorker(const std::string &name, const VehicleParams &params)
    : process(std::make_unique<Process>()) {
  auto script = plugin_root / "cpp_backend/tools/plugin_worker.py";
  if (!fs::exists(script))
    script = plugin_root / "app/plugin_worker.py";
  process->start(python(), script);
  request({{"operation", "init"},
           {"strategy", name},
           {"params", params},
           {"plugins_dir", directory().u8string()}},
          30);
}
PluginWorker::~PluginWorker() = default;
json PluginWorker::request(const json &message, double timeout) {
  if (!process)
    throw Error(503, "Plugin worker has stopped");
  try {
    process->write(message.dump() + "\n");
    auto result = json::parse(process->read(timeout));
    if (result.contains("error"))
      throw Error(422, "Plugin error: " + str(result, "error"));
    return result.at("result");
  } catch (...) {
    process.reset();
    throw;
  }
}
json PluginWorker::status(bool reload) {
  return request({{"operation", reload ? "reload" : "status"}});
}
Command PluginWorker::compute(const Driver &d, const State &s, double dt) {
  auto result = request(
      {{"operation", "compute"}, {"driver", d}, {"state", s}, {"dt", dt}});
  Command c;
  for (auto pair : std::vector<std::pair<std::string, V4 *>>{
           {"delta_cmd", &c.delta_cmd},
           {"wheel_speed_cmd", &c.wheel_speed_cmd},
           {"drive_torque_cmd", &c.drive_torque_cmd},
           {"brake_cmd", &c.brake_cmd}}) {
    if (result.contains(pair.first)) {
      *pair.second = result.at(pair.first).get<V4>();
      for (double value : *pair.second)
        if (!std::isfinite(value))
          throw Error(422, "Plugin produced non-finite command");
    }
  }
  if (result.contains("icr_target_body"))
    for (int i = 0; i < 2; i++)
      c.icr_target_body[i] = result["icr_target_body"][i].is_null()
                                 ? nan
                                 : result["icr_target_body"][i].get<double>();
  c.handbrake = int(num(result, "handbrake"));
  return c;
}
} // namespace fourwis
