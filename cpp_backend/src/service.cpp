#include "service.hpp"
namespace fourwis {
std::vector<std::string> split(const std::string &s, char delimiter) {
  std::vector<std::string> out;
  size_t i = 0;
  while (i < s.size()) {
    auto next = s.find(delimiter, i);
    auto part = s.substr(i, next == s.npos ? next : next - i);
    if (!part.empty())
      out.push_back(part);
    if (next == s.npos)
      break;
    i = next + 1;
  }
  return out;
}
void set_dotted(json &object, const std::string &path, const json &value) {
  auto parts = split(path, '.');
  if (parts.empty())
    throw Error(422, "empty override path");
  json *cur = &object;
  for (size_t i = 0; i < parts.size(); i++) {
    if (cur->is_null())
      *cur = json::object();
    if (cur->is_array()) {
      size_t pos = std::stoul(parts[i]);
      if (pos >= cur->size())
        throw Error(422, "override index out of range");
      cur = &cur->at(pos);
    } else if (cur->is_object())
      cur = &(*cur)[parts[i]];
    else
      throw Error(422, "cannot traverse override path");
  }
  *cur = value;
}
json Service::validate_body(const std::string &method, const std::string &path,
                            const json &body) const {
  std::string verb = method;
  std::transform(verb.begin(), verb.end(), verb.begin(),
                 [](unsigned char c) { return char(std::tolower(c)); });
  auto parts = split(path, '/');
  for (auto it = store.contract["paths"].begin();
       it != store.contract["paths"].end(); ++it) {
    auto p = split(it.key(), '/');
    if (p.size() != parts.size() || !it.value().contains(verb))
      continue;
    bool matched = true;
    for (size_t i = 0; i < p.size(); i++)
      if (p[i] != parts[i] && p[i][0] != '{')
        matched = false;
    if (!matched)
      continue;
    auto operation = it.value()[verb];
    if (!operation.contains("requestBody"))
      return body;
    auto request = operation["requestBody"];
    if (body.is_null() && !yes(request, "required"))
      return body;
    auto schema = request["content"]["application/json"]["schema"];
    return validate_json(body, schema, store.contract);
  }
  return body;
}
void Service::reap_jobs() {
  // Called with mutex held. finished is the worker's last protected update;
  // after releasing that guard it returns without acquiring another lock.
  for (auto it = workers.begin(); it != workers.end();) {
    if (it->first->finished > 0) {
      if (it->second.joinable())
        it->second.join();
      it = workers.erase(it);
    } else
      ++it;
  }
  for (auto *registry : {&jobs, &study_jobs}) {
    while (registry->size() > 50) {
      auto oldest = registry->end();
      for (auto it = registry->begin(); it != registry->end(); ++it)
        if (it->second->finished > 0 &&
            (oldest == registry->end() ||
             it->second->started < oldest->second->started))
          oldest = it;
      if (oldest == registry->end())
        break;
      registry->erase(oldest);
    }
  }
}
json Service::job_status(const std::shared_ptr<Job> &job) const {
  auto data = job->data;
  data["elapsed_s"] =
      std::round(
          ((job->finished > 0 ? job->finished : epoch()) - job->started) *
          1000) /
      1000;
  return data;
}
std::string Service::batch(const json &request) {
  json exp;
  if (request.contains("experiment") && !request["experiment"].is_null())
    exp = store.experiment(request["experiment"]);
  else if (!str(request, "name").empty())
    exp = store.experiment(store.load("experiments", str(request, "name")));
  else
    throw Error(400, "provide name or inline experiment");
  if (exp["maneuver"]["steps"].empty())
    throw Error(400, "experiment has no maneuver steps");
  std::vector<std::pair<std::string, json>> items;
  auto variants = request.value("variants", json::array());
  if (variants.empty())
    items.emplace_back(str(exp, "name"), exp);
  else
    for (size_t i = 0; i < variants.size(); i++) {
      auto e = exp;
      auto overrides = variants[i].value("overrides", json::object());
      for (auto v = overrides.begin(); v != overrides.end(); ++v)
        set_dotted(e, v.key(), v.value());
      items.emplace_back(
          str(variants[i], "label", "variant_" + std::to_string(i)),
          store.experiment(e));
    }
  if (items.size() > 1000)
    throw Error(422, "batch exceeds 1000 runs");
  auto job = std::make_shared<Job>();
  std::string id = uid().substr(0, 12);
  job->data = {{"job_id", id},          {"status", "running"},
               {"total", items.size()}, {"done", 0},
               {"current_label", ""},   {"error", ""},
               {"runs", json::array()}};
  jobs[id] = job;
  workers.emplace_back(
      job, std::thread([this, job, items, id] {
        try {
          for (auto &item : items) {
            if (job->cancel || !running) {
              std::lock_guard<std::recursive_mutex> guard(mutex);
              job->data["status"] = "cancelled";
              break;
            }
            {
              std::lock_guard<std::recursive_mutex> guard(mutex);
              job->data["current_label"] = item.first;
            }
            Simulation sim(item.second, store.params(item.second["vehicle"]));
            auto result = sim.run();
            auto metrics = kpis(result, item.second);
            auto run = store.save_run(result, item.first, metrics, id);
            {
              std::lock_guard<std::recursive_mutex> guard(mutex);
              job->data["runs"].push_back(
                  {{"run_id", run}, {"label", item.first}, {"kpis", metrics}});
              job->data["done"] = job->data["done"].get<int>() + 1;
            }
          }
          std::lock_guard<std::recursive_mutex> guard(mutex);
          if (str(job->data, "status") == "running")
            job->data["status"] = "done";
        } catch (const std::exception &e) {
          std::lock_guard<std::recursive_mutex> guard(mutex);
          job->data["status"] = "error";
          job->data["error"] = e.what();
        }
        std::lock_guard<std::recursive_mutex> guard(mutex);
        job->data["current_label"] = "";
        job->finished = epoch();
      }));
  return id;
}
void Service::disconnect(const std::string &owner) {
  std::lock_guard<std::recursive_mutex> guard(mutex);
  if (live.input_owner == owner && !live.script_running)
    live.release();
}
void Service::ws(const json &data, const std::string &owner) {
  std::lock_guard<std::recursive_mutex> guard(mutex);
  if (!data.is_object())
    return;
  auto type = str(data, "type");
  if (type == "release_input") {
    if (live.input_owner == owner && !live.script_running)
      live.release();
  } else if (type == "driver") {
    if (!live.script_running) {
      live.driver(data);
      live.input_owner = owner;
      live.input_seen = epoch();
    }
  } else if (type == "strategy")
    live.strategy(str(data, "name"));
  else if (type == "reset")
    live.reset();
  else if (type == "steer_cmd" && live.sim->controller.kind == "user_js")
    for (int i = 0; i < 4; i++)
      live.sim->controller.js[i] = num(data, wheel_names[i]);
}
Response Service::request(const std::string &method, const std::string &path,
                          json body,
                          const std::map<std::string, std::string> &query) {
  std::lock_guard<std::recursive_mutex> guard(mutex);
  reap_jobs();
  if (path != "/api/study/run" && path != "/api/study/dry-run")
    body = validate_body(method, path, body);
  if (body.is_null())
    body = json::object();
  auto parts = split(path, '/');
  auto q = [&](const std::string &key, std::string fallback = "") {
    auto it = query.find(key);
    return it == query.end() ? fallback : it->second;
  };
  auto qnum = [&](const std::string &key, double fallback) {
    auto s = q(key);
    return s.empty() ? fallback : std::stod(s);
  };
  auto get = method == "GET", post = method == "POST", del = method == "DELETE";
  auto &sim = *live.sim;
  if (path == "/health")
    return Response(json{{"status", "ok"}});
  if (path == "/openapi.json")
    return Response(store.contract);
  if (path == "/api/version")
    return Response(json{{"version", "0.107.0"}, {"engine", engine_info()}});
  if (path == "/api/strategies") {
    json names = store.catalog["strategies"];
    for (auto &n : plugin_names())
      if (std::find(names.begin(), names.end(), n) == names.end())
        names.push_back(n);
    return Response(json{{"strategies", names}});
  }
  if (path == "/api/plugins" || path == "/api/plugins/reload")
    return Response(discover_plugins());
  if (path == "/api/user_python/status" || path == "/api/user_python/reload") {
    bool reload = path == "/api/user_python/reload";
    if (reload && sim.controller.kind != "user_python")
      throw Error(400, "user_python is not the active strategy");
    try {
      if (sim.controller.kind == "user_python")
        return Response(sim.controller.plugin_status(reload));
      if (!python_probe)
        python_probe = std::make_unique<Controller>("user_python", sim.model.p);
      return Response(python_probe->plugin_status());
    } catch (const std::exception &e) {
      return Response(json{{"status", "error"},
                           {"error", e.what()},
                           {"last_reload", nullptr},
                           {"file_path", ""}});
    }
  }
  if (path == "/api/status")
    return Response(json{{"strategy", sim.controller.kind},
                         {"t", sim.model.s.t},
                         {"pose",
                          {{"x", sim.model.s.x},
                           {"y", sim.model.s.y},
                           {"psi", sim.model.s.psi}}},
                         {"running", running.load()},
                         {"subscribers", subscribers}});
  if (path == "/api/strategy" && post) {
    live.strategy(str(body, "name"));
    return Response(json{{"strategy", live.sim->controller.kind}});
  }
  if (path == "/api/reset" && post) {
    live.reset();
    return Response(json{{"status", "reset"}});
  }
  if (path == "/api/model") {
    auto layer = [](const std::string &model) {
      return model == "multibody" ? "research" : "primary";
    };
    if (post) {
      live.replace(sim.model.p, str(body, "model_type"));
      live.reset();
      return Response(json{{"model_type", live.sim->model.kind},
                           {"layer", layer(live.sim->model.kind)}});
    }
    return Response(
        json{{"model_type", sim.model.kind},
             {"layer", layer(sim.model.kind)},
             {"available",
              json::array({"kinematic", "simplified_dynamic", "multibody"})},
             {"models", store.catalog["models"]},
             {"primary", store.catalog["primary_models"]}});
  }
  if (path == "/api/params") {
    if (post) {
      json p = sim.model.p, patch = body;
      for (auto it = patch.begin(); it != patch.end();)
        if (it.value().is_null())
          it = patch.erase(it);
        else
          ++it;
      merge(p, patch);
      live.replace(store.params(json{{"overrides", p}}), sim.model.kind);
    }
    return Response(json(live.sim->model.p));
  }
  if (path == "/api/interaction/status")
    return Response(live.interaction());
  if (path == "/api/interaction/control" && post) {
    auto action = str(body, "action");
    if (action == "stop") {
      live.stop_script();
      live.release();
    } else
      live.paused = action == "pause";
    return Response(live.interaction());
  }
  if (path == "/api/scene" && get)
    return Response(sim.scene.definition);
  if (path == "/api/scene/mu" && post) {
    sim.scene.definition["base_mu"] = body["mu"];
    return Response(json{{"base_mu", body["mu"]}});
  }
  if (path == "/api/scene/clear" && post) {
    size_t n = sim.scene.definition["disturbances"].size();
    sim.scene.definition["disturbances"] = json::array();
    return Response(json{{"status", "cleared"},
                         {"removed", n},
                         {"base_mu", sim.scene.definition["base_mu"]}});
  }
  if (parts.size() >= 3 && parts[1] == "scene" && parts[2] == "disturbances") {
    auto &ds = sim.scene.definition["disturbances"];
    auto normalize = [](json d) {
      json clean;
      for (auto k : {"type", "id", "x", "y", "width", "length", "heading"})
        clean[k] = d.value(k, k == std::string("heading") ? json(0) : json());
      auto type = str(d, "type");
      if (type == "ice_patch")
        clean["mu"] = num(d, "mu", .2);
      else if (type == "split_mu") {
        clean["mu_left"] = num(d, "mu_left", .2);
        clean["mu_right"] = num(d, "mu_right", .85);
      } else if (type == "speed_bump") {
        clean["height"] = num(d, "height", .05);
        clean["stiffness"] = num(d, "stiffness", 1e5);
      } else if (type == "slope")
        clean["angle"] = num(d, "angle");
      else
        throw Error(422, "unknown disturbance");
      return clean;
    };
    if (parts.size() == 3 && post) {
      std::set<std::string> ids;
      for (auto &d : ds)
        ids.insert(str(d, "id"));
      int n = 1;
      auto type = str(body, "type");
      while (ids.count(type + "_" + std::to_string(n)))
        n++;
      body["id"] = type + "_" + std::to_string(n);
      body = normalize(body);
      ds.push_back(body);
      return Response(body);
    }
    if (parts.size() == 4) {
      for (size_t i = 0; i < ds.size(); i++)
        if (str(ds[i], "id") == parts[3]) {
          if (del) {
            ds.erase(ds.begin() + i);
            return Response(json{{"status", "deleted"}, {"id", parts[3]}});
          }
          if (method == "PUT") {
            auto d = ds[i];
            for (auto it = body.begin(); it != body.end(); ++it)
              if (!it.value().is_null())
                d[it.key()] = it.value();
            ds[i] = normalize(d);
            return Response(ds[i]);
          }
        }
      throw Error(404, "disturbance not found");
    }
  }
  if (parts.size() >= 2 && parts[1] == "faults") {
    auto &f = sim.faults;
    if (parts.size() == 3 && parts[2] == "schedule") {
      if (post)
        f.replace(body["faults"], body.value("seed", uint32_t(0)), &sim.model,
                  sim.model.s.t);
      return Response(f.status());
    }
    if (parts.size() == 2) {
      if (get)
        return Response(json{{"faults", f.status()["faults"]}});
      if (post) {
        if (!body.contains("id") || body["id"].is_null())
          body["id"] = uid().substr(0, 12);
        json defs = json::array();
        for (auto &item : f.faults)
          defs.push_back(item.spec());
        defs.push_back(body);
        Faults candidate;
        candidate.replace(defs, f.seed, &sim.model);
        f.faults.push_back(candidate.faults.back());
        return Response(f.faults.back().legacy(), 201);
      }
      if (del) {
        for (auto &id : f.enabled)
          f.events.push_back(
              {{"id", id}, {"event", "removed"}, {"time", f.time}});
        f.faults.clear();
        f.enabled.clear();
        f.hold.clear();
        f.free.clear();
        return Response(json{{"cleared", true}});
      }
    }
    if (parts.size() == 3) {
      auto it = std::find_if(f.faults.begin(), f.faults.end(),
                             [&](auto &item) { return item.id == parts[2]; });
      if (it == f.faults.end())
        throw Error(404, "fault not found");
      if (method == "PATCH") {
        it->active = yes(body, "active");
        return Response(it->legacy());
      }
      if (del) {
        if (f.enabled.erase(parts[2]))
          f.events.push_back(
              {{"id", parts[2]}, {"event", "removed"}, {"time", f.time}});
        f.hold.erase(parts[2]);
        f.free.erase(parts[2]);
        f.faults.erase(it);
        return Response(json{{"deleted", parts[2]}});
      }
    }
  }
  if (path == "/api/path/templates")
    return Response(store.catalog["path_templates"]);
  if (path == "/api/path" && get) {
    auto out = live.plan;
    out["version"] = live.path_version;
    return Response(out);
  }
  if (parts.size() == 3 && parts[1] == "path" && post) {
    json plan;
    if (parts[2] == "clear")
      plan = empty_plan();
    else if (parts[2] == "waypoints")
      plan = plan_waypoints(body.at("points").get<Points>(),
                            yes(body, "closed"), str(body, "name", "custom"));
    else if (parts[2] == "template") {
      json params = {
          {"vehicle_width",
           std::max(sim.model.p.track_front, sim.model.p.track_rear) * 1.14},
          {"vehicle_length", sim.model.p.wheelbase * 1.2}};
      merge(params, body.value("params", json::object()));
      plan = plan_template(str(body, "name"), params);
      auto anchor = str(body, "anchor");
      if (!anchor.empty()) {
        if (live.scenario.is_null() ||
            !live.scenario["anchors"].contains(anchor))
          throw Error(400, "unknown scenario anchor");
        auto pose = live.scenario["anchors"][anchor];
        transform_plan(plan, pose[0], pose[1], pose[2]);
        plan["notes"] = str(plan, "notes") + " · 锚点 " + anchor;
      }
    } else
      throw Error(404, "unknown path operation");
    live.plan = plan;
    live.path_version++;
    plan["version"] = live.path_version;
    return Response(plan);
  }
  if (path == "/api/scenarios") {
    json list = json::array();
    for (auto it = scenarios.begin(); it != scenarios.end(); ++it)
      list.push_back({{"name", it.key()}, {"label", it.value()["label"]}});
    return Response(json{{"scenarios", list}});
  }
  if (path == "/api/scenario")
    return Response(
        json{{"version", live.scenario_version}, {"scenario", live.scenario}});
  if (path == "/api/scenario/clear" && post) {
    live.scenario = nullptr;
    return Response(
        json{{"version", ++live.scenario_version}, {"scenario", nullptr}});
  }
  if (parts.size() == 4 && parts[1] == "scenarios" && parts[3] == "load" &&
      post) {
    if (!scenarios.contains(parts[2]))
      throw Error(404, "unknown scenario");
    live.scenario = scenarios[parts[2]];
    auto pose = live.scenario["spawn"];
    for (auto &spawn : live.scenario.value("spawns", json::array()))
      if (str(spawn, "name") == q("spawn"))
        pose = json::array({spawn["x"], spawn["y"], spawn["heading"]});
    live.reset();
    sim.model.s.x = pose[0];
    sim.model.s.y = pose[1];
    sim.model.s.psi = pose[2];
    return Response(json{{"version", ++live.scenario_version},
                         {"scenario", live.scenario}});
  }
  if (path == "/api/script/status")
    return Response(live.script_status());
  if (path == "/api/script/load" && post) {
    try {
      auto definition = parse_yaml(str(body, "yaml"));
      live.load_script(definition.is_null() ? json::object() : definition);
    } catch (const Error &e) {
      if (e.status == 409)
        throw;
      throw Error(400, std::string("script parse error: ") + e.what());
    } catch (const std::exception &e) {
      throw Error(400, std::string("script parse error: ") + e.what());
    }
    bool path_set = false;
    auto path = live.script.value("path_template", json());
    if (path.is_object() && !str(path, "name").empty())
      try {
        live.plan = plan_template(str(path, "name"),
                                  path.value("params", json::object()));
        live.path_version++;
        path_set = true;
      } catch (const Error &) {
      }
    return Response(json{{"status", "loaded"},
                         {"script_name", str(live.script, "name")},
                         {"actions", live.script["actions"].size()},
                         {"path_template", path_set}});
  }
  if (path == "/api/script/start" && post) {
    live.start_script();
    return Response(json{{"status", "started"}});
  }
  if (path == "/api/script/stop" && post) {
    live.stop_script();
    return Response(json{{"status", "stopped"}});
  }
  if (path == "/api/script/library")
    return Response(json{{"scripts", store.names("scripts_lib")}});
  if (parts.size() == 4 && parts[1] == "script" && parts[2] == "library")
    return Response(
        json{{"name", parts[3]},
             {"yaml", file_text(store.dir("scripts_lib") /
                                fs::u8path(safe_name(parts[3]) + ".yaml"))}});
  if (parts.size() == 3 && parts[1] == "recording") {
    if (parts[2] == "start" && post)
      live.start_record(body);
    else if (parts[2] == "stop" && post)
      live.stop_record();
    else if (parts[2] == "clear" && post)
      live.clear_record();
    else if (parts[2] == "save" && post)
      return Response(store.manifest(live.save_record()));
    else if (parts[2] == "export.csv" && get)
      return Response(live.csv(split(q("channels"), ','),
                               qnum("from_t", -INFINITY),
                               qnum("to_t", INFINITY),
                               q("include_strategy", "true") != "false"),
                      "text/csv; charset=utf-8");
    else if (parts[2] != "status")
      throw Error(404, "unknown recording operation");
    return Response(live.recording_status());
  }
  if (parts.size() >= 2 && parts[1] == "projects") {
    if (parts.size() == 2 && get)
      return Response(json{{"projects", store.names("projects")}});
    if (parts.size() >= 3) {
      auto name = safe_name(parts[2]);
      auto resource = store.root / "cpp_backend/data";
      if (!fs::exists(resource))
        resource = store.root / "app/data";
      json project = read_json(resource / "project_defaults.json");
      if (parts.size() == 3 && post) {
        json p = sim.model.p;
        for (auto it = project["vehicle"].begin();
             it != project["vehicle"].end(); ++it)
          if (p.contains(it.key()))
            it.value() = p[it.key()];
        project["vehicle"]["model"] = sim.model.kind;
        project["suspension"] = p["suspension"];
        project["steering_geometry"] = p["steering_geometry"];
        project["controller"]["type"] = sim.controller.kind;
        project["scene"] = sim.scene.definition;
        project["project"] = {{"name", name},
                              {"description", str(body, "description")}};
        auto saved = store.save("projects", name, project);
        return Response(json{
            {"status", "saved"}, {"name", name}, {"path", saved.u8string()}});
      }
      merge(project, store.load("projects", name));
      auto schema = read_json(resource / "project_schema.json");
      project = validate_json(project, schema, schema);
      if (parts.size() == 3 && get)
        return Response(project);
      if (parts.size() == 4 && parts[3] == "load" && post) {
        json p = project["vehicle"];
        p.erase("model");
        p["suspension"] = project["suspension"];
        p["steering_geometry"] = project["steering_geometry"];
        auto params = store.params(json{{"overrides", p}});
        Controller check(str(project["controller"], "type"), params);
        live.replace(params, str(project["vehicle"], "model"));
        live.strategy(check.kind);
        live.sim->scene.definition = project["scene"];
        live.reset();
        return Response(
            json{{"status", "loaded"},
                 {"name", name},
                 {"strategy", live.sim->controller.kind},
                 {"model_type", live.sim->model.kind},
                 {"disturbances",
                  live.sim->scene.definition["disturbances"].size()}});
      }
    }
  }
  if (parts.size() >= 2 && parts[1] == "vehicle-profiles") {
    if (parts.size() == 2 && get) {
      json items =
          json::array({{{"name", "LS9"}, {"label", "LS9"}, {"builtin", true}}});
      for (auto &n : store.names("vehicle_profiles")) {
        auto p =
            store.load("vehicle_profiles", n).value("profile", json::object());
        json item = {{"name", str(p, "name", n)},
                     {"label", str(p, "label", n)},
                     {"builtin", false}};
        if (str(item, "name") == "LS9") {
          item["builtin"] = yes(p, "builtin");
          items[0] = item;
        } else
          items.push_back(item);
      }
      return Response(json{{"profiles", items}});
    }
    if (parts.size() >= 3) {
      auto name = safe_name(parts[2]);
      if (parts.size() == 4 && parts[3] == "apply" && post) {
        live.replace(store.params(json{{"profile", name}}), sim.model.kind);
        live.reset();
        return Response(json{{"status", "applied"}, {"name", name}});
      }
      if (parts.size() == 3) {
        if (get)
          return Response(store.profile(name));
        if (del) {
          if (name == "LS9" || name == "im_ls9")
            throw Error(400, "built-in profile cannot be deleted");
          if (!fs::remove(store.dir("vehicle_profiles") /
                          fs::u8path(name + ".yaml")))
            throw Error(404, "vehicle profile not found");
          return Response(json{{"status", "deleted"}, {"name", name}});
        }
        if (post) {
          json p = sim.model.p;
          if (body.contains("vehicle") && body["vehicle"].is_object())
            merge(p, body["vehicle"]);
          auto params = store.params(json{{"overrides", p}});
          auto saved =
              store.save("vehicle_profiles", name,
                         json{{"profile",
                               {{"name", name},
                                {"label", str(body, "label", name)},
                                {"description", str(body, "description")},
                                {"builtin", false}}},
                              {"vehicle", params}});
          return Response(json{
              {"status", "saved"}, {"name", name}, {"path", saved.u8string()}});
        }
      }
    }
  }
  if (path == "/api/maneuver-templates")
    return Response(json{
        {"steer_kinds",
         json::array({"constant", "step", "ramp", "sine", "sweep", "dlc"})},
        {"path_templates", store.catalog["path_templates"]["templates"]}});
  if (parts.size() >= 2 && parts[1] == "experiments") {
    if (parts.size() == 2 && get) {
      json items = json::array();
      for (auto &name : store.names("experiments")) {
        json item = {{"name", name},
                     {"description", "(unreadable)"},
                     {"strategy", ""},
                     {"model_type", ""},
                     {"maneuver", ""}};
        try {
          auto d = store.load("experiments", name);
          for (auto key : {"description", "strategy", "model_type"})
            item[key] = str(d, key);
          item["maneuver"] = str(d.value("maneuver", json::object()), "name");
        } catch (...) {
        }
        items.push_back(item);
      }
      return Response(json{{"experiments", items}});
    }
    if (parts.size() == 3) {
      auto name = safe_name(parts[2]);
      if (get)
        return Response(store.experiment(store.load("experiments", name)));
      if (post) {
        body["name"] = name;
        auto saved = store.save("experiments", name, store.experiment(body));
        return Response(json{
            {"status", "saved"}, {"name", name}, {"path", saved.u8string()}});
      }
      if (del) {
        if (!fs::remove(store.dir("experiments") / fs::u8path(name + ".yaml")))
          throw Error(404, "experiment not found");
        return Response(json{{"status", "deleted"}, {"name", name}});
      }
    }
  }
  if (parts.size() >= 2 && parts[1] == "batch") {
    if (parts.size() == 2) {
      if (post)
        return Response(json{{"job_id", batch(body)}});
      if (get) {
        std::vector<std::shared_ptr<Job>> ordered;
        for (auto &j : jobs)
          ordered.push_back(j.second);
        std::sort(ordered.begin(), ordered.end(),
                  [](auto &a, auto &b) { return a->started > b->started; });
        json list = json::array();
        for (auto &j : ordered)
          list.push_back(job_status(j));
        return Response(json{{"jobs", list}});
      }
    }
    if (parts.size() >= 3) {
      auto it = jobs.find(parts[2]);
      if (it == jobs.end())
        throw Error(404, "job not found");
      if (parts.size() == 3 && get)
        return Response(job_status(it->second));
      if (parts.size() == 4 && parts[3] == "cancel" && post) {
        if (str(it->second->data, "status") != "running")
          throw Error(409, "job not running");
        it->second->cancel = true;
        return Response(json{{"status", "cancelling"}, {"job_id", parts[2]}});
      }
    }
  }
  if (parts.size() >= 2 && parts[1] == "runs") {
    if (parts.size() == 2 && get)
      return Response(json{{"runs", store.runs()}});
    if (parts.size() >= 3) {
      auto id = safe_name(parts[2]);
      if (parts.size() == 3) {
        if (get)
          return Response(store.run_meta(id));
        if (del) {
          if (!fs::is_directory(store.dir("runs") / id))
            throw Error(404, "run not found");
          fs::remove_all(store.dir("runs") / id);
          return Response(json{{"status", "deleted"}, {"run_id", id}});
        }
      }
      if (parts.size() == 4 && get) {
        auto op = parts[3];
        if (op == "data") {
          int n = int(qnum("decimate", 1));
          if (n < 1 || n > 1000)
            throw Error(422, "invalid decimation");
          return Response(store.run_data(id, split(q("channels"), ','), n));
        }
        if (op == "artifacts")
          return Response(store.manifest(id));
        if (op == "export.csv")
          return Response(file_text(store.dir("runs") / id / "data.csv"),
                          "text/csv; charset=utf-8");
        if (op == "export.json")
          return Response(store.rows(id));
        if (op == "plot.svg") {
          auto kind = q("kind", "chart");
          if (kind != "chart" && kind != "trajectory")
            throw Error(422, "invalid plot kind");
          return Response(store.plot(id, kind, q("channel", "yaw_rate")),
                          "image/svg+xml");
        }
        if (op == "bundle.zip")
          return Response(store.bundle(id), "application/zip");
      }
    }
  }
  if (parts.size() >= 2 && parts[1] == "agent") {
    if (path == "/api/agent/capabilities") {
      json data = store.catalog["agent_capabilities"];
      data["engine"] = engine_info();
      return Response(data);
    }
    for (auto it = sessions.begin(); it != sessions.end();)
      if (epoch() - it->second->touched > 3600)
        it = sessions.erase(it);
      else
        ++it;
    if (parts.size() >= 3 && parts[2] == "sessions") {
      if (parts.size() == 3) {
        if (post) {
          if (sessions.size() >= 4)
            throw Error(429, "close a session before creating another",
                        "session_limit");
          auto session = std::make_shared<AgentSession>(body, store);
          sessions[session->id] = session;
          return Response(session->observe(), 201);
        }
        if (get) {
          json list = json::array();
          for (auto &s : sessions)
            list.push_back(s.second->observe());
          return Response(
              json{{"contract", "4wis.agent.v1"}, {"sessions", list}});
        }
      }
      if (parts.size() >= 4) {
        auto it = sessions.find(parts[3]);
        if (it == sessions.end())
          throw Error(404, "unknown or expired session", "session_not_found");
        auto session = it->second;
        session->touched = epoch();
        if (parts.size() == 4) {
          if (get)
            return Response(session->observe());
          if (del) {
            session->status = "closed";
            sessions.erase(it);
            return Response(
                json{{"session_id", session->id}, {"status", "closed"}});
          }
        }
        if (parts.size() == 5) {
          auto op = parts[4];
          if (op == "step" && post) {
            auto result = session->step(body);
            return Response(result,
                            str(result, "status") == "failed" ? 422 : 200);
          }
          if (op == "faults" && post)
            return Response(session->schedule(body));
          if (op == "preview" && get) {
            int n = int(qnum("max_points", 300));
            if (n < 2 || n > 1000)
              throw Error(422, "invalid preview limit");
            return Response(session->preview(n));
          }
          if (op == "export" && post)
            return Response(store.manifest(session->export_run(store)));
        }
      }
    }
  }
  if (auto result = analysis(path, body))
    return *result;
  if (auto result = studies(method, parts, body, query))
    return *result;
  throw Error(404, "endpoint not found");
}
} // namespace fourwis
