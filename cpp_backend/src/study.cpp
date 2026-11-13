#include "study.hpp"
#include "service.hpp"
#include <iomanip>
#include <sstream>
namespace fourwis {
namespace {
json metric_names(const json &spec) {
  json names = json::array();
  for (auto &m : spec["metrics"])
    names.push_back(m.is_string() ? m : json(str(m, "name")));
  return names;
}
json axes(const json &spec) {
  json names = json::array();
  for (auto it = spec["sweep"].begin(); it != spec["sweep"].end(); ++it)
    names.push_back(it.key());
  return names;
}
bool walk(const json &root, const std::string &path) {
  auto parts = split(path, '.');
  const json *cur = &root;
  try {
    for (auto &part : parts) {
      if (cur->is_array())
        cur = &cur->at(std::stoul(part));
      else if (cur->is_object())
        cur = &cur->at(part);
      else
        return false;
    }
  } catch (...) {
    return false;
  }
  return true;
}
std::string label_value(const json &v) {
  std::string s;
  if (v.is_number_float()) {
    std::ostringstream out;
    out << std::setprecision(6) << v.get<double>();
    s = out.str();
  } else if (v.is_string())
    s = v.get<std::string>();
  else if (v.is_boolean())
    s = v.get<bool>() ? "True" : "False";
  else
    s = v.dump();
  s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
  std::replace(s.begin(), s.end(), '/', '-');
  return s;
}
json cells(const json &spec) {
  json out = json::array({{{"coords", json::object()},
                           {"label", str(spec, "study")},
                           {"overrides", json::object()}}});
  for (auto it = spec["sweep"].begin(); it != spec["sweep"].end(); ++it) {
    json next = json::array();
    for (auto &cell : out)
      for (auto &value : it.value()["values"]) {
        auto c = cell;
        c["coords"][it.key()] = value;
        c["overrides"][str(it.value(), "bind")] = value;
        std::string label;
        for (auto coord = c["coords"].begin(); coord != c["coords"].end();
             ++coord) {
          if (!label.empty())
            label += ' ';
          label += coord.key() + "=" + label_value(coord.value());
        }
        c["label"] = label;
        next.push_back(c);
        if (next.size() > 1000)
          throw Error(422, "study exceeds 1000 runs");
      }
    out = std::move(next);
  }
  return out;
}
std::string digest(const json &s) {
  return hex_sha(canonical(s)).substr(0, 16);
}
} // namespace
json normalize_study(const Storage &store, json spec) {
  bool explicit_model = spec.contains("baseline") &&
                        spec["baseline"].is_object() &&
                        spec["baseline"].contains("model_type");
  spec =
      validate_json(spec, store.contract["components"]["schemas"]["StudySpec"],
                    store.contract);
  auto model = str(spec, "model");
  if (model != "kinematic" && model != "simplified_dynamic" &&
      model != "multibody")
    throw Error(422, "unknown study model");
  if (explicit_model && str(spec["baseline"], "model_type") != model)
    throw Error(422, "study model conflicts with baseline.model_type");
  spec["baseline"] = store.experiment(spec["baseline"]);
  if (!explicit_model)
    spec["baseline"]["model_type"] = "simplified_dynamic";
  if (spec["metrics"].empty())
    throw Error(422, "a study without metrics measures nothing");
  auto names = metric_names(spec);
  std::set<std::string> known;
  for (auto &n : names)
    if (!known.insert(n.get<std::string>()).second)
      throw Error(422, "duplicate metric name");
  for (auto &criterion : spec["criteria"]) {
    if (!known.count(str(criterion, "metric")))
      throw Error(422, "criterion refers to an unrequested metric");
    holds(0, str(criterion, "must"));
  }
  for (auto &axis : spec["sweep"]) {
    bool values = axis.contains("values") && !axis["values"].is_null(),
         solver = axis.contains("solve_for") && !axis["solve_for"].is_null();
    if (values == solver)
      throw Error(422, "sweep axis needs exactly one of values or solve_for");
    if (values && axis["values"].empty())
      throw Error(422, "sweep values cannot be empty");
    if (solver && axis["bracket"].is_null())
      throw Error(422, "solver requires bracket");
  }
  if (spec["compare"].is_object()) {
    auto group = str(spec["compare"], "group_by");
    if (!group.empty() && !spec["sweep"].contains(group))
      throw Error(422, "unknown compare.group_by axis");
    auto against = spec["compare"].value("against", json());
    if (against.is_object())
      for (auto i = against.begin(); i != against.end(); ++i)
        if (!spec["sweep"].contains(i.key()))
          throw Error(422, "unknown compare.against axis");
  }
  return spec;
}
json dry_study(const Storage &store, const json &spec) {
  json problems = json::array(), warnings = json::array();
  auto base = spec["baseline"];
  base["model_type"] = spec["model"];
  size_t count = 1;
  for (auto it = spec["sweep"].begin(); it != spec["sweep"].end(); ++it) {
    auto &axis = it.value();
    if (!axis["solve_for"].is_null())
      problems.push_back("solver axes are not implemented yet: " + it.key());
    auto path = str(axis, "bind");
    bool open = path.rfind("vehicle.overrides.", 0) == 0 ||
                path.rfind("mode_params.", 0) == 0 ||
                path.rfind("scene.", 0) == 0;
    auto checked = open ? path.substr(0, path.rfind('.')) : path;
    if (!walk(base, checked))
      problems.push_back("axis " + it.key() +
                         ": bind container does not exist: " + path);
    if (axis["values"].is_array()) {
      if (axis["values"].size() > 1000 ||
          count > 1000 / std::max<size_t>(1, axis["values"].size()))
        problems.push_back("study exceeds 1000 runs");
      else
        count *= axis["values"].size();
    }
  }
  std::set<std::string> known, procedure;
  for (auto &m : store.catalog["study_capabilities"]["metrics"]) {
    known.insert(str(m, "name"));
    if (str(m, "source") == "procedure")
      procedure.insert(str(m, "name"));
  }
  Run shape;
  for (auto &name : store.catalog["study_capabilities"]["channels"])
    shape.channels[name.get<std::string>()] = {0};
  for (auto &m : spec["metrics"]) {
    if (m.is_string()) {
      auto name = m.get<std::string>();
      if (!known.count(name))
        problems.push_back("unknown metric: " + name);
    } else
      try {
        expression(str(m, "expr"), shape, true);
      } catch (const Error &e) {
        problems.push_back(e.what());
      }
  }
  for (auto &c : spec["criteria"])
    try {
      auto selector = selectors(str(c, "at", "all"));
      for (auto it = selector.begin(); it != selector.end(); ++it)
        if (!spec["sweep"].contains(it.key()))
          problems.push_back("criterion selector names unknown axis: " +
                             it.key());
    } catch (const Error &e) {
      problems.push_back(e.what());
    }
  auto names = metric_names(spec);
  bool needs_plant = false, needs_step = false, has_step = false;
  for (auto &n : names) {
    auto name = n.get<std::string>();
    needs_plant |= procedure.count(name) != 0;
    needs_step |=
        std::set<std::string>{"yaw_gain_dps", "yaw_rise_time_s",
                              "yaw_overshoot_pct", "yaw_settling_time_s"}
            .count(name) != 0;
  }
  for (auto &s : base["maneuver"]["steps"])
    if (str(s["steer"], "kind") == "step")
      has_step = true;
  auto sys =
      base["vehicle"]["overrides"].value("steering_system", json::object());
  if (needs_plant && !yes(sys, "enabled"))
    warnings.push_back("procedure metrics need steering_system.enabled; absent "
                       "plant channels will be reported missing");
  if (needs_step && !has_step)
    warnings.push_back(
        "step-response metrics require a step-kind steer segment");
  auto target = str(spec, "targets");
  if (!target.empty())
    try {
      auto ts = target_set(store, target);
      for (auto &e : ts["entries"])
        if (str(e, "severity") == "must" &&
            std::find(names.begin(), names.end(), e["metric"]) == names.end()) {
          warnings.push_back("target includes required metrics this study does "
                             "not compute; coverage will remain incomplete");
          break;
        }
    } catch (const Error &e) {
      problems.push_back(e.what());
    }
  if (!problems.empty())
    count = 0;
  double duration = 0;
  for (auto &s : base["maneuver"]["steps"])
    duration += num(s, "duration");
  json labels = json::array();
  if (problems.empty())
    for (auto &c : cells(spec))
      labels.push_back(c["label"]);
  return {{"ok", problems.empty()},
          {"problems", problems},
          {"warnings", warnings},
          {"grid", count},
          {"runs", count},
          {"sim_seconds", std::nearbyint(count * duration * 100) / 100},
          {"est_wall_seconds", std::nearbyint(count * duration * .5) / 10},
          {"metrics", names},
          {"axes", axes(spec)},
          {"targets", spec["targets"]},
          {"cells", labels},
          {"spec_digest", digest(spec)}};
}
std::string study_html(const json &r) {
  std::ostringstream out;
  out << "<!doctype html><html lang=\"zh-CN\"><meta charset=\"utf-8\"><meta "
         "name=\"viewport\" "
         "content=\"width=device-width,initial-scale=1\"><title>"
      << escaped(str(r, "study", "4WIS analysis"))
      << "</title><style>body{font:16px/1.6 "
         "system-ui,sans-serif;max-width:1200px;margin:40px auto;padding:0 "
         "24px;color:#172033;background:#f8fafc}h1{font-size:32px}table{border-"
         "collapse:collapse;background:white;width:100%;margin:24px "
         "0}th,td{border:1px solid "
         "#cbd5e1;padding:10px;text-align:left}th{background:#e2e8f0}pre{white-"
         "space:pre-wrap;background:white;padding:20px;border:1px solid "
         "#cbd5e1}.missing{color:#9a3412}</style><h1>"
      << escaped(str(r, "study", "4WIS analysis")) << "</h1><p>"
      << escaped(str(r, "question"))
      << "</p><p>Version 0.107.0 · C++ runtime · internal simulation evidence; "
         "independent vehicle correlation remains pending.</p>";
  if (r.contains("rows") && r.contains("metrics")) {
    out << "<table><thead><tr><th>Condition</th>";
    for (auto &m : r["metrics"])
      out << "<th>" << escaped(m.get<std::string>()) << "</th>";
    out << "</tr></thead><tbody>";
    for (auto &row : r["rows"]) {
      out << "<tr><td>" << escaped(str(row, "label")) << "</td>";
      for (auto &m : r["metrics"]) {
        auto n = m.get<std::string>();
        out << "<td>";
        if (row["metrics"].contains(n))
          out << escaped(row["metrics"][n].dump());
        else
          out << "<span class=\"missing\">PENDING: "
              << escaped(row.value("errors", json::object())
                             .value(n, "missing measurement"))
              << "</span>";
        out << "</td>";
      }
      out << "</tr>";
    }
    out << "</tbody></table>";
  }
  out << "<h2>Evidence and provenance</h2><pre>"
      << escaped(r.dump(2, ' ', false, json::error_handler_t::replace))
      << "</pre></html>";
  return out.str();
}
std::pair<std::string, json> run_study(const Storage &store, const json &spec,
                                       const std::atomic<bool> *running) {
  double start = epoch();
  auto check = dry_study(store, spec);
  if (!yes(check, "ok")) {
    Error e(400, "invalid study");
    e.detail = {{"problems", check["problems"]}};
    throw e;
  }
  json rows = json::array(), warnings = check["warnings"];
  auto base = spec["baseline"];
  base["model_type"] = spec["model"];
  auto names = metric_names(spec);
  for (auto &cell : cells(spec)) {
    if (running && !*running)
      throw Error(409, "server stopping");
    json exp = base;
    for (auto it = cell["overrides"].begin(); it != cell["overrides"].end();
         ++it)
      set_dotted(exp, it.key(), it.value());
    exp = store.experiment(exp);
    Simulation simulation(exp, store.params(exp["vehicle"]));
    auto result = simulation.run();
    auto base_metrics = kpis(result, exp);
    auto id = store.save_run(result, str(cell, "label"), base_metrics);
    auto metrics = collect_metrics(spec["metrics"], base_metrics, result);
    json row = {{"coords", cell["coords"]},
                {"label", cell["label"]},
                {"run_id", id},
                {"metrics", metrics["values"]}};
    if (!metrics["errors"].empty())
      row["errors"] = metrics["errors"];
    rows.push_back(row);
  }
  json compare = nullptr;
  if (spec["compare"].is_object()) {
    compare = comparison(rows, names, axes(spec), spec["compare"]);
    if (compare.contains("warnings")) {
      for (auto &w : compare["warnings"])
        warnings.push_back(w);
      compare.erase("warnings");
    }
  }
  auto verdicts = criteria(spec["criteria"], rows, axes(spec));
  json target = nullptr;
  if (!str(spec, "targets").empty()) {
    json measures = json::array();
    for (auto &row : rows)
      for (auto it = row["metrics"].begin(); it != row["metrics"].end(); ++it)
        measures.push_back(
            {{"metric", it.key()},
             {"value", it.value()},
             {"at", row["coords"]},
             {"label", row["label"]},
             {"source", "study:" + str(spec, "study")},
             {"trusted", it.value().is_number() &&
                             std::isfinite(it.value().get<double>())}});
    target = compliance(target_set(store, str(spec, "targets")), measures);
  }
  bool passed = true;
  for (auto &v : verdicts)
    passed &= yes(v, "passed");
  auto params = store.params(base["vehicle"]);
  json result = {
      {"study", spec["study"]},
      {"question", spec["question"]},
      {"model", spec["model"]},
      {"spec_digest", digest(spec)},
      {"provenance",
       {{"sim4wis_version", "0.107.0"},
        {"git_sha", FOURWIS_SOURCE_REVISION},
        {"spec_digest", digest(spec)},
        {"model", spec["model"]},
        {"strategy", base["strategy"]},
        {"dt", base["dt"]},
        {"record_hz", base["record_hz"]},
        {"params_hash", digest(json(params))},
        {"created_at", stamp("%Y-%m-%dT%H:%M:%S")},
        {"engine", engine_info()}}},
      {"metrics", names},
      {"rows", rows},
      {"comparison", compare},
      {"verdicts", verdicts},
      {"compliance", target},
      {"warnings", warnings},
      {"report_path", nullptr},
      {"elapsed_s", std::nearbyint((epoch() - start) * 1000) / 1000},
      {"all_passed", verdicts.empty() ? json() : json(passed)},
      {"compliant", target.is_object() ? target["verdict"] : json()}};
  auto id = stamp("%Y%m%d_%H%M%S") + "_" + str(spec, "study").substr(0, 40) +
            "_" + uid().substr(0, 4);
  auto dir = store.dir("studies") / id;
  fs::create_directories(dir);
  if (spec["report"].is_object()) {
    auto path = dir / "report.html";
    auto custom = str(spec["report"], "out");
    if (!custom.empty())
      path = fs::u8path(custom);
    result["report_path"] = fs::absolute(path).u8string();
    atomic_text(path, study_html(result));
    if (fs::absolute(path) != fs::absolute(dir / "report.html"))
      atomic_text(dir / "report.html", study_html(result));
  }
  atomic_text(dir / "spec.json", spec.dump(1));
  atomic_text(dir / "result.json", result.dump(1));
  return {id, result};
}
std::optional<Response>
Service::studies(const std::string &method,
                 const std::vector<std::string> &parts, const json &body,
                 const std::map<std::string, std::string> &query) {
  if (parts.size() < 2)
    return {};
  auto q = [&](const std::string &key, std::string fallback = "") {
    auto it = query.find(key);
    return it == query.end() ? fallback : it->second;
  };
  bool get = method == "GET", post = method == "POST";
  if (parts[1] == "targets") {
    if (parts.size() == 2 && get)
      return Response(json{{"sets", target_catalog(store)}});
    if (parts.size() >= 3) {
      auto target = target_set(store, parts[2]);
      if (parts.size() == 3 && get)
        return Response(target);
      if (parts.size() == 4 && parts[3] == "check" && post) {
        auto p = store.params(json{{"profile", body.value("vehicle", json())}});
        p.steering_system["enabled"] = true;
        auto sized = sizing(store, p);
        auto checked = compliance(target, sized["measurements"]);
        json result = {{"compliance", checked}, {"sizing", sized["result"]}};
        if (yes(body, "html"))
          return Response(study_html(result), "text/html; charset=utf-8");
        return Response(result);
      }
    }
    throw Error(404, "target endpoint not found");
  }
  if (parts[1] != "study")
    return {};
  if (parts.size() == 3 && parts[2] == "capabilities") {
    auto result = store.catalog["study_capabilities"];
    result["engine"] = engine_info();
    result["target_sets"] = target_catalog(store);
    return Response(result);
  }
  if (parts.size() == 3 && post &&
      (parts[2] == "dry-run" || parts[2] == "run")) {
    auto spec = normalize_study(store, body), check = dry_study(store, spec);
    if (parts[2] == "dry-run")
      return Response(check);
    if (!yes(check, "ok")) {
      Error e(400, "invalid study");
      e.detail = {{"problems", check["problems"]}};
      throw e;
    }
    auto job = std::make_shared<Job>();
    std::string id = uid().substr(0, 12);
    job->data = {{"job_id", id},        {"study", spec["study"]},
                 {"status", "running"}, {"total", check["runs"]},
                 {"error", ""},         {"study_id", nullptr},
                 {"result", nullptr}};
    study_jobs[id] = job;
    workers.emplace_back([this, spec, job] {
      try {
        auto result = run_study(store, spec, &running);
        std::lock_guard<std::recursive_mutex> lock(mutex);
        job->data["study_id"] = result.first;
        job->data["result"] = result.second;
        job->data["status"] = "done";
      } catch (const std::exception &e) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        job->data["status"] = "error";
        job->data["error"] = e.what();
      }
      std::lock_guard<std::recursive_mutex> lock(mutex);
      job->finished = epoch();
    });
    return Response(
        json{{"job_id", id}, {"total", check["runs"]}, {"estimate", check}});
  }
  if (parts.size() >= 3 && parts[2] == "jobs") {
    if (parts.size() == 3) {
      std::vector<std::shared_ptr<Job>> sorted;
      for (auto &j : study_jobs)
        sorted.push_back(j.second);
      std::sort(sorted.begin(), sorted.end(),
                [](auto &a, auto &b) { return a->started > b->started; });
      json list = json::array();
      for (auto &j : sorted) {
        auto status = job_status(j);
        status.erase("result");
        list.push_back(status);
      }
      return Response(json{{"jobs", list}});
    }
    if (parts.size() == 4) {
      auto it = study_jobs.find(parts[3]);
      if (it == study_jobs.end())
        throw Error(404, "unknown study job");
      return Response(job_status(it->second));
    }
  }
  if (parts.size() == 4 && parts[2] == "trace" && get) {
    auto names = split(q("channels"), ',');
    if (names.empty())
      throw Error(422, "channels must name at least one channel");
    int maximum = std::stoi(q("max_points", "200"));
    if (maximum < 2 || maximum > 2000)
      throw Error(422, "invalid trace point limit");
    auto meta = store.run_meta(parts[3]);
    int count = int(num(meta, "n_samples")),
        stride = std::max(1, (count + maximum - 1) / maximum);
    auto data = store.run_data(parts[3], names, stride);
    json missing = json::array();
    for (auto &n : names)
      if (!data.contains(n))
        missing.push_back(n);
    return Response(json{{"run_id", parts[3]},
                         {"decimate", stride},
                         {"n_samples", data["t"].size()},
                         {"of_total", count},
                         {"missing_channels", missing},
                         {"channels", data}});
  }
  if (parts.size() == 2 && get) {
    std::vector<fs::path> paths;
    for (auto &p : fs::directory_iterator(store.dir("studies")))
      if (p.is_directory() && fs::exists(p.path() / "result.json"))
        paths.push_back(p.path());
    std::sort(paths.rbegin(), paths.rend());
    json list = json::array();
    for (auto &p : paths)
      try {
        auto r = read_json(p / "result.json");
        list.push_back(
            {{"study_id", p.filename().u8string()},
             {"study", str(r, "study")},
             {"question", str(r, "question")},
             {"model", str(r, "model")},
             {"spec_digest", str(r, "spec_digest")},
             {"rows", r["rows"].size()},
             {"all_passed", r.value("all_passed", json())},
             {"created_at",
              r.value("provenance", json::object()).value("created_at", "")},
             {"report_path", r.value("report_path", json())}});
      } catch (...) {
      }
    return Response(json{{"studies", list}});
  }
  if (parts.size() >= 3 && get) {
    auto id = safe_name(parts[2]);
    auto dir = store.dir("studies") / id;
    if (parts.size() == 3)
      return Response(json::parse(file_text(dir / "result.json")));
    if (parts.size() == 4 && parts[3] == "report")
      return Response(file_text(dir / "report.html"),
                      "text/html; charset=utf-8");
  }
  throw Error(404, "study endpoint not found");
}
} // namespace fourwis
