#include "storage.hpp"
#include <chrono>
#include <iomanip>
#include <random>
#include <sstream>
#include <yaml-cpp/yaml.h>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif
namespace fourwis {
double epoch() {
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
std::string uid() {
  static std::mutex mu;
  static std::mt19937_64 engine(std::random_device{}());
  std::lock_guard<std::mutex> lock(mu);
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << engine()
      << std::setw(16) << engine();
  return out.str();
}
std::string stamp(const char *format) {
  auto now = std::time(nullptr);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &now);
#else
  localtime_r(&now, &tm);
#endif
  char b[128];
  std::strftime(b, sizeof(b), format, &tm);
  return b;
}
std::string hex_sha(const std::string &text) {
  std::ostringstream s;
  s << std::hex << std::setfill('0');
  for (auto b : sha256(text))
    s << std::setw(2) << unsigned(b);
  return s.str();
}
std::string file_text(const fs::path &p) {
  std::ifstream f(p, std::ios::binary);
  if (!f)
    throw Error(404, "file not found: " + p.filename().string());
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}
void atomic_text(const fs::path &p, const std::string &text) {
  fs::create_directories(p.parent_path());
  auto tmp = p;
  tmp += "." + uid() + ".tmp";
  try {
    std::ofstream f(tmp, std::ios::binary);
    if (!f)
      throw Error(500, "cannot create data file");
    f.write(text.data(), std::streamsize(text.size()));
    f.close();
    if (!f)
      throw Error(500, "cannot complete data file");
#ifdef _WIN32
    if (!MoveFileExW(tmp.c_str(), p.c_str(),
                     MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
      throw Error(500, "atomic data replacement failed");
#else
    fs::rename(tmp, p);
#endif
  } catch (...) {
    std::error_code ec;
    fs::remove(tmp, ec);
    throw;
  }
}
static json from_yaml(const YAML::Node &node) {
  if (node.IsNull() || !node)
    return nullptr;
  if (node.IsSequence()) {
    json a = json::array();
    for (auto x : node)
      a.push_back(from_yaml(x));
    return a;
  }
  if (node.IsMap()) {
    json o = json::object();
    for (auto x : node)
      o[x.first.as<std::string>()] = from_yaml(x.second);
    return o;
  }
  auto s = node.Scalar();
  if (node.Tag() == "!")
    return s;
  if (s == "true" || s == "True")
    return true;
  if (s == "false" || s == "False")
    return false;
  if (s == "null" || s == "Null" || s == "~")
    return nullptr;
  try {
    auto v = json::parse(s);
    if (v.is_number() || v.is_null())
      return v;
  } catch (...) {
  }
  try {
    size_t used = 0;
    double value = std::stod(s, &used);
    if (used == s.size() && std::isfinite(value))
      return value;
  } catch (...) {
  }
  return s;
}
json parse_yaml(const std::string &text) { return from_yaml(YAML::Load(text)); }
json yaml_json(const fs::path &p) { return parse_yaml(file_text(p)); }
std::string safe_name(const std::string &s) {
  if (s.empty() || s.size() > 256 || s == "." || s == "..")
    throw Error(400, "invalid name");
  for (unsigned char c : s)
    if (c < 128 && !std::isalnum(c) && c != '_' && c != '-')
      throw Error(400, "invalid name");
  return s;
}
std::string escaped(const std::string &s) {
  std::string out;
  for (char c : s) {
    switch (c) {
    case '&':
      out += "&amp;";
      break;
    case '<':
      out += "&lt;";
      break;
    case '>':
      out += "&gt;";
      break;
    case '"':
      out += "&quot;";
      break;
    default:
      out += c;
    }
  }
  return out;
}
static bool starts(const std::string &s, const std::string &p) {
  return s.rfind(p, 0) == 0;
}
std::string unit(const std::string &n) {
  if (n == "t")
    return "s";
  if (n == "pose_x" || n == "pose_y" || n == "pose_z" || starts(n, "icr_") ||
      starts(n, "susp_defl_"))
    return "m";
  if (n == "vx" || n == "vy")
    return "m/s";
  if (n == "ax" || n == "ay")
    return "m/s^2";
  if (n == "yaw_rate" || n == "steer_motor_speed" || starts(n, "omega_") ||
      starts(n, "wheel_speed_cmd_") || starts(n, "steer_rate_"))
    return "rad/s";
  if (n == "pose_psi" || n == "roll" || n == "pitch" ||
      n == "steer_hand_angle" || n == "steer_angle_deviation" ||
      starts(n, "delta_") || starts(n, "deltacmd_") ||
      starts(n, "slip_alpha_") || starts(n, "grip_alpha_peak_") ||
      starts(n, "steer_corner_deviation_") ||
      starts(n, "linkage_arm_tie_angle_") ||
      starts(n, "linkage_tie_rack_angle_"))
    return "rad";
  for (auto p : {"fz_", "rack_force_", "tire_fx_", "tire_fy_", "grip_capacity_",
                 "grip_margin_"})
    if (starts(n, p))
      return "N";
  if (n == "steer_hand_torque" || n == "steer_torque_sensor" ||
      n == "steer_assist_torque" || n == "steer_motor_torque" ||
      starts(n, "torque_steer_") || starts(n, "motor_torque_") ||
      starts(n, "drive_torque_cmd_"))
    return "N*m";
  if (n == "steer_plant_active" || n == "mu_avg" || n == "strategy_index")
    return "1";
  for (auto p :
       {"fault_mode_", "sensor_fault_active_", "driver_", "slip_kappa_",
        "grip_", "wheel_locked_", "brake_cmd_", "linkage_efficiency_"})
    if (starts(n, p))
      return "1";
  return "unspecified";
}
json validate_json(json v, const json &schema, const json &doc,
                   const std::string &where, int depth) {
  if (depth > 64)
    throw Error(422, where + ": nesting too deep");
  if ((str(schema, "title") == "FaultSpec" ||
       str(schema, "$ref") == "#/components/schemas/FaultSpec") &&
      v.is_object()) {
    if (v.contains("wheel") && v["wheel"].is_string()) {
      auto w = v["wheel"].get<std::string>();
      std::transform(w.begin(), w.end(), w.begin(),
                     [](unsigned char c) { return char(std::tolower(c)); });
      for (int i = 0; i < 4; i++)
        if (w == wheel_names[i])
          v["wheel"] = i;
    }
    const std::map<std::string, std::string> aliases = {
        {"motor_stuck_zero", "stuck_zero"},
        {"motor_stuck_angle", "stuck_value"},
        {"motor_limited_range", "limited"},
        {"motor_stuck_hold", "stuck_hold"},
        {"motor_free_caster", "free_caster"}};
    auto alias = aliases.find(str(v, "fault_type"));
    if (alias != aliases.end())
      v["fault_type"] = alias->second;
  }
  if (schema.contains("$ref"))
    return validate_json(
        v, doc.at(json::json_pointer(str(schema, "$ref").substr(1))), doc,
        where, depth + 1);
  if (schema.contains("anyOf")) {
    for (auto &s : schema["anyOf"])
      try {
        return validate_json(v, s, doc, where, depth + 1);
      } catch (const Error &) {
      }
    throw Error(422, where + ": invalid value");
  }
  if (schema.contains("allOf"))
    for (auto &s : schema["allOf"])
      v = validate_json(v, s, doc, where, depth + 1);
  if (schema.contains("type") && schema["type"].is_array()) {
    for (auto &t : schema["type"]) {
      auto branch = schema;
      branch["type"] = t;
      try {
        return validate_json(v, branch, doc, where, depth + 1);
      } catch (const Error &) {
      }
    }
    throw Error(422, where + ": invalid type");
  }
  auto type = str(schema, "type");
  bool valid = type.empty() || (type == "null" && v.is_null()) ||
               (type == "object" && v.is_object()) ||
               (type == "array" && v.is_array()) ||
               (type == "boolean" && v.is_boolean()) ||
               (type == "string" && v.is_string()) ||
               (type == "number" && v.is_number()) ||
               (type == "integer" && v.is_number_integer());
  if (!valid)
    throw Error(422, where + ": expected " + type);
  if (schema.contains("enum") &&
      std::find(schema["enum"].begin(), schema["enum"].end(), v) ==
          schema["enum"].end())
    throw Error(422, where + ": invalid enum");
  if (schema.contains("const") && schema["const"] != v)
    throw Error(422, where + ": invalid constant");
  if (v.is_number()) {
    if (type == "number")
      v = v.get<double>();
    double x = v.get<double>();
    if (!std::isfinite(x))
      throw Error(422, where + ": non-finite value");
    for (auto key :
         {"minimum", "maximum", "exclusiveMinimum", "exclusiveMaximum"})
      if (schema.contains(key)) {
        double bound = num(schema, key);
        if ((std::string(key) == "minimum" && x < bound) ||
            (std::string(key) == "maximum" && x > bound) ||
            (std::string(key) == "exclusiveMinimum" && x <= bound) ||
            (std::string(key) == "exclusiveMaximum" && x >= bound))
          throw Error(422, where + ": outside physical bounds");
      }
  }
  if (v.is_string()) {
    auto s = v.get<std::string>();
    size_t length = 0;
    for (unsigned char c : s)
      if ((c & 0xc0) != 0x80)
        length++;
    if (length < num(schema, "minLength", 0) ||
        length > num(schema, "maxLength", INFINITY))
      throw Error(422, where + ": invalid string length");
    if (schema.contains("pattern") &&
        !std::regex_search(s, std::regex(str(schema, "pattern"))))
      throw Error(422, where + ": invalid format");
  }
  if (v.is_array()) {
    if (v.size() < num(schema, "minItems", 0) ||
        v.size() > num(schema, "maxItems", INFINITY))
      throw Error(422, where + ": invalid array length");
    if (schema.contains("items"))
      for (size_t i = 0; i < v.size(); i++)
        v[i] = validate_json(v[i], schema["items"], doc,
                             where + "[" + std::to_string(i) + "]", depth + 1);
  }
  if (v.is_object()) {
    auto props = schema.value("properties", json::object());
    for (auto &key : schema.value("required", json::array()))
      if (!v.contains(key.get<std::string>()))
        throw Error(422, where + ": missing " + key.get<std::string>());
    bool strict = schema.contains("additionalProperties") &&
                  schema["additionalProperties"] == false;
    for (auto it = v.begin(); it != v.end(); ++it) {
      if (props.contains(it.key()))
        it.value() = validate_json(it.value(), props[it.key()], doc,
                                   where + "." + it.key(), depth + 1);
      else if (strict)
        throw Error(422, where + ": unknown field " + it.key());
      else if (schema.contains("additionalProperties") &&
               schema["additionalProperties"].is_object())
        it.value() = validate_json(it.value(), schema["additionalProperties"],
                                   doc, where + "." + it.key(), depth + 1);
    }
    for (auto it = props.begin(); it != props.end(); ++it)
      if (!v.contains(it.key()) && it.value().contains("default"))
        v[it.key()] = validate_json(it.value()["default"], it.value(), doc,
                                    where + "." + it.key(), depth + 1);
  }
  return v;
}
Storage::Storage(fs::path path) : root(std::move(path)) {
  auto data = root / "cpp_backend/data";
  if (!fs::exists(data))
    data = root / "app/data";
  defaults = read_json(data / "default_params.json");
  contract = read_json(data / "openapi.json");
  catalog = read_json(data / "catalog.json");
}
fs::path Storage::dir(const std::string &name) const {
  std::string e = "SIM4WIS_";
  for (char c : name)
    e += char(std::toupper(static_cast<unsigned char>(c)));
  e += "_DIR";
  auto v = env_path(e.c_str());
  fs::path out = v ? *v : env_path("SIM4WIS_DATA_DIR").value_or(root) / name;
  fs::create_directories(out);
  return out;
}
std::vector<std::string> Storage::names(const std::string &folder) const {
  std::vector<std::string> v;
  for (auto &p : fs::directory_iterator(dir(folder)))
    if (p.is_regular_file() && p.path().extension() == ".yaml")
      v.push_back(p.path().stem().u8string());
  std::sort(v.begin(), v.end());
  return v;
}
json Storage::load(const std::string &folder, const std::string &name) const {
  return yaml_json(dir(folder) / fs::u8path(safe_name(name) + ".yaml"));
}
fs::path Storage::save(const std::string &folder, const std::string &name,
                       const json &data) const {
  auto p = dir(folder) / fs::u8path(safe_name(name) + ".yaml");
  atomic_text(p,
              data.dump(2, ' ', false, json::error_handler_t::replace) + "\n");
  return p;
}
json Storage::profile(const std::string &name) const {
  json v;
  auto path = dir("vehicle_profiles") / fs::u8path(safe_name(name) + ".yaml");
  if (fs::is_regular_file(path))
    v = yaml_json(path);
  else if (name == "LS9" || name == "im_ls9")
    v = catalog.at("builtin_profile");
  else
    throw Error(404, "vehicle profile not found");
  json p = defaults;
  merge(p, v.value("vehicle", json::object()));
  v["vehicle"] = p;
  if (!v.contains("profile"))
    v["profile"] = json::object();
  if (!v["profile"].contains("name"))
    v["profile"]["name"] = name;
  if (!v["profile"].contains("label"))
    v["profile"]["label"] = name;
  return v;
}
VehicleParams Storage::params(const json &vehicle) const {
  json p = defaults;
  auto name = str(vehicle, "profile");
  if (!name.empty())
    p = profile(name).at("vehicle");
  auto overrides = vehicle.value("overrides", json::object());
  merge(p, overrides);
  VehicleParams out = p.get<VehicleParams>();
  if (!(out.mass > 0 && out.inertia_z > 0 && out.wheelbase > 0 &&
        out.tire_radius > 0 && out.track_front > 0 && out.track_rear > 0 &&
        out.cg_to_front > 0 && out.cg_to_front < out.wheelbase &&
        out.unsprung_mass * 4 < out.mass))
    throw Error(422, "invalid vehicle mass, dimensions or CG");
  if (p.contains("kc_profile") && !p["kc_profile"].is_null() &&
      !str(p, "kc_profile").empty())
    out.kc = load("kc_profiles", str(p, "kc_profile"));
  return out;
}
json Storage::experiment(const json &d) const {
  return normalize_experiment(validate_json(
      d, contract["components"]["schemas"]["Experiment"], contract));
}
std::string run_csv(const Run &r, const std::vector<std::string> &selection) {
  std::vector<std::string> names = selection;
  if (names.empty())
    for (auto &c : r.channels)
      names.push_back(c.first);
  std::ostringstream s;
  s << std::setprecision(17) << "t";
  for (auto &n : names)
    s << ',' << n;
  s << "\r\n";
  for (size_t i = 0; i < r.t.size(); i++) {
    s << r.t[i];
    for (auto &n : names) {
      s << ',';
      double x = r.channels.at(n)[i];
      if (std::isfinite(x))
        s << x;
    }
    s << "\r\n";
  }
  return s.str();
}
std::string Storage::save_run(const Run &r, const std::string &label,
                              const json &metrics, const json &job) const {
  std::string id = stamp("%Y%m%d_%H%M%S") + "_" + uid().substr(0, 6);
  auto path = dir("runs") / id;
  fs::create_directory(path);
  json names = json::array();
  for (auto &c : r.channels)
    names.push_back(c.first);
  json meta = {{"run_id", id},
               {"label", label},
               {"created_at", stamp("%Y-%m-%dT%H:%M:%S")},
               {"sim4wis_version", "0.107.0"},
               {"duration_s", r.duration_s},
               {"n_steps", r.n_steps},
               {"n_samples", r.t.size()},
               {"channels", names},
               {"kpis", metrics},
               {"job_id", job}};
  merge(meta, r.meta);
  atomic_text(path / "data.csv", run_csv(r));
  atomic_text(path / "meta.json", meta.dump(1) + "\n");
  return id;
}
json Storage::run_meta(const std::string &id) const {
  return json::parse(file_text(dir("runs") / safe_name(id) / "meta.json"));
}
static std::vector<std::string> split_csv(std::string line) {
  if (!line.empty() && line.back() == '\r')
    line.pop_back();
  std::vector<std::string> out;
  size_t pos = 0;
  for (;;) {
    auto i = line.find(',', pos);
    out.push_back(line.substr(pos, i == line.npos ? i : i - pos));
    if (i == line.npos)
      break;
    pos = i + 1;
  }
  return out;
}
json Storage::run_data(const std::string &id,
                       const std::vector<std::string> &selection,
                       int stride) const {
  std::istringstream in(file_text(dir("runs") / safe_name(id) / "data.csv"));
  std::string line;
  std::getline(in, line);
  auto names = split_csv(line);
  std::vector<size_t> indices{0};
  for (size_t i = 1; i < names.size(); i++)
    if (selection.empty() || std::find(selection.begin(), selection.end(),
                                       names[i]) != selection.end())
      indices.push_back(i);
  json out = json::object();
  for (auto i : indices)
    out[names[i]] = json::array();
  size_t row = 0;
  while (std::getline(in, line)) {
    if (row++ % std::max(1, stride))
      continue;
    auto vals = split_csv(line);
    for (auto i : indices) {
      if (i >= vals.size())
        throw Error(422, "truncated run CSV");
      if (vals[i].empty())
        out[names[i]].push_back(nullptr);
      else
        out[names[i]].push_back(std::stod(vals[i]));
    }
  }
  return out;
}
json Storage::runs() const {
  std::vector<fs::path> paths;
  for (auto &p : fs::directory_iterator(dir("runs")))
    if (p.is_directory() && fs::is_regular_file(p.path() / "meta.json"))
      paths.push_back(p.path());
  std::sort(paths.rbegin(), paths.rend());
  json out = json::array();
  for (auto &p : paths)
    try {
      auto m = read_json(p / "meta.json"),
           e = m.value("experiment", json::object());
      out.push_back({{"run_id", m.value("run_id", p.filename().string())},
                     {"label", str(m, "label")},
                     {"created_at", str(m, "created_at")},
                     {"duration_s", m.value("duration_s", json())},
                     {"n_samples", m.value("n_samples", json())},
                     {"strategy", str(e, "strategy")},
                     {"model_type", str(e, "model_type")},
                     {"experiment_name", str(e, "name")},
                     {"kpis", m.value("kpis", json::object())},
                     {"job_id", m.value("job_id", json())}});
    } catch (...) {
    }
  return out;
}
json Storage::manifest(const std::string &id) const {
  auto m = run_meta(id);
  auto data = file_text(dir("runs") / id / "data.csv");
  auto names = split_csv(data.substr(0, data.find('\n')));
  json channels = json::array();
  for (auto &n : names)
    channels.push_back({{"name", n}, {"unit", unit(n)}});
  std::string base = "/api/runs/" + id;
  return {
      {"contract", "4wis.artifacts.v1"},
      {"run_id", id},
      {"source", str(m, "source", "experiment")},
      {"samples", m["n_samples"]},
      {"channels", channels},
      {"sampling",
       m.value("sampling", json("recorded at requested " +
                                (m.contains("record_hz") ? m["record_hz"].dump()
                                                         : "unknown") +
                                " Hz"))},
      {"full_data_decimated", false},
      {"nonfinite_encoding", "empty CSV field / JSON null"},
      {"complete", yes(m, "complete", str(m, "execution_status") != "failed")},
      {"execution_status", str(m, "execution_status", "recorded")},
      {"dropped_samples",
       m.value("recording", json::object()).value("dropped_samples", 0)},
      {"csv_sha256", hex_sha(data)},
      {"csv_bytes", data.size()},
      {"validity", "Internal verification; no independent vehicle correlation. "
                   "Model validity remains in metadata."},
      {"outputs",
       {{"csv", base + "/export.csv"},
        {"json", base + "/export.json"},
        {"bundle", base + "/bundle.zip"},
        {"metadata", base},
        {"trajectory", base + "/plot.svg?kind=trajectory"},
        {"chart", base + "/plot.svg?kind=chart&channel=yaw_rate"},
        {"analysis", "/?run=" + id}}}};
}
json Storage::rows(const std::string &id) const {
  auto data = run_data(id);
  json rows = json::array();
  for (size_t i = 0; i < data["t"].size(); i++) {
    json row = json::object();
    for (auto it = data.begin(); it != data.end(); ++it)
      row[it.key()] = it.value()[i];
    rows.push_back(row);
  }
  return {
      {"contract", "4wis.data.v1"}, {"manifest", manifest(id)}, {"rows", rows}};
}
std::string Storage::plot(const std::string &id, const std::string &kind,
                          const std::string &channel) const {
  auto meta = run_meta(id);
  int stride = std::max(1, int(std::ceil(num(meta, "n_samples") / 2000)));
  auto data = run_data(id,
                       kind == "trajectory"
                           ? std::vector<std::string>{"pose_x", "pose_y"}
                           : std::vector<std::string>{channel},
                       stride);
  auto xn = kind == "trajectory" ? "pose_x" : "t",
       yn = kind == "trajectory" ? "pose_y" : channel.c_str();
  if (!data.contains(xn) || !data.contains(yn))
    throw Error(422, "unknown plot channel");
  std::vector<V2> pairs;
  double xmin = INFINITY, xmax = -INFINITY, ymin = INFINITY, ymax = -INFINITY;
  for (size_t i = 0; i < data[xn].size(); i++)
    if (data[xn][i].is_number() && data[yn][i].is_number()) {
      double x = data[xn][i], y = data[yn][i];
      pairs.push_back({x, y});
      xmin = std::min(xmin, x);
      xmax = std::max(xmax, x);
      ymin = std::min(ymin, y);
      ymax = std::max(ymax, y);
    }
  if (pairs.empty())
    throw Error(422, "no finite samples to plot");
  double dx = std::max(xmax - xmin, 1e-6), dy = std::max(ymax - ymin, 1e-6);
  if (kind == "trajectory") {
    double scale = std::min(670 / dx, 270 / dy);
    dx = 670 / scale;
    dy = 270 / scale;
    xmin = (xmin + xmax - dx) / 2;
    ymin = (ymin + ymax - dy) / 2;
  } else {
    double pad = std::max(dy * .06, .01);
    ymin -= pad;
    dy += 2 * pad;
  }
  std::ostringstream s;
  s << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 800 400\" "
       "role=\"img\"><rect width=\"800\" height=\"400\" fill=\"white\"/><g "
       "font-family=\"sans-serif\" font-size=\"12\" fill=\"#334155\"><text "
       "x=\"70\" y=\"28\" font-size=\"18\">"
    << escaped(kind == "trajectory" ? "Trajectory (equal scale)"
                                    : channel + " [" + unit(channel) + "]")
    << "</text>";
  for (int i = 0; i < 6; i++)
    s << "<text x=\"62\" y=\"" << 320 - i * 54 << "\" text-anchor=\"end\">"
      << std::setprecision(4) << ymin + dy * i / 5
      << "</text><line x1=\"70\" x2=\"740\" y1=\"" << 320 - i * 54 << "\" y2=\""
      << 320 - i * 54 << "\" stroke=\"#e2e8f0\"/><text x=\"" << 70 + i * 134
      << "\" y=\"340\" text-anchor=\"middle\">" << xmin + dx * i / 5
      << "</text>";
  s << "<polyline points=\"" << std::fixed << std::setprecision(3);
  for (auto v : pairs)
    s << 70 + (v[0] - xmin) / dx * 670 << ',' << 320 - (v[1] - ymin) / dy * 270
      << ' ';
  s << "\" fill=\"none\" stroke=\"#2563eb\" stroke-width=\"2\"/><text "
       "x=\"400\" y=\"363\" text-anchor=\"middle\">"
    << xn << " [" << unit(xn)
    << "]</text><text x=\"70\" y=\"385\">Preview: every " << stride
    << " recorded sample(s); full data available separately. " << escaped(id)
    << "</text></g></svg>";
  return s.str();
}
std::string Storage::bundle(const std::string &id) const {
  std::vector<std::pair<std::string, std::string>> files = {
      {"data.csv", file_text(dir("runs") / safe_name(id) / "data.csv")},
      {"meta.json", run_meta(id).dump(2)},
      {"manifest.json", manifest(id).dump(2)}};
  for (auto kind : {"trajectory", "chart"})
    try {
      files.emplace_back(std::string(kind) + ".svg",
                         plot(id, kind, "yaw_rate"));
    } catch (const Error &e) {
      files.emplace_back(std::string(kind) + "-unavailable.txt", e.what());
    }
  return zip_store(files);
}
std::string
zip_store(const std::vector<std::pair<std::string, std::string>> &files) {
  std::string out, central;
  auto u16 = [](std::string &s, uint16_t x) {
    s += char(x & 255);
    s += char(x >> 8);
  };
  auto u32 = [&](std::string &s, uint32_t x) {
    u16(s, uint16_t(x));
    u16(s, uint16_t(x >> 16));
  };
  for (auto &f : files) {
    if (f.second.size() > UINT32_MAX || out.size() > UINT32_MAX)
      throw Error(422, "ZIP64 size not supported");
    uint32_t crc = 0xffffffff;
    for (unsigned char c : f.second) {
      crc ^= c;
      for (int i = 0; i < 8; i++)
        crc = (crc >> 1) ^ (0xedb88320u & uint32_t(-int(crc & 1)));
    }
    crc ^= 0xffffffff;
    uint32_t off = uint32_t(out.size()), sz = uint32_t(f.second.size());
    u32(out, 0x04034b50);
    u16(out, 20);
    u16(out, 0x800);
    u16(out, 0);
    u16(out, 0);
    u16(out, 33);
    u32(out, crc);
    u32(out, sz);
    u32(out, sz);
    u16(out, uint16_t(f.first.size()));
    u16(out, 0);
    out += f.first;
    out += f.second;
    u32(central, 0x02014b50);
    u16(central, 20);
    u16(central, 20);
    u16(central, 0x800);
    u16(central, 0);
    u16(central, 0);
    u16(central, 33);
    u32(central, crc);
    u32(central, sz);
    u32(central, sz);
    u16(central, uint16_t(f.first.size()));
    u16(central, 0);
    u16(central, 0);
    u16(central, 0);
    u16(central, 0);
    u32(central, 0);
    u32(central, off);
    central += f.first;
  }
  uint32_t offset = uint32_t(out.size());
  out += central;
  u32(out, 0x06054b50);
  u16(out, 0);
  u16(out, 0);
  u16(out, uint16_t(files.size()));
  u16(out, uint16_t(files.size()));
  u32(out, uint32_t(central.size()));
  u32(out, offset);
  u16(out, 0);
  return out;
}
} // namespace fourwis
