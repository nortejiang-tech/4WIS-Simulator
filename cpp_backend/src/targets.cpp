#include "analysis.hpp"
#include "service.hpp"
#include "study.hpp"
#include <iomanip>
#include <sstream>
namespace fourwis {
std::string canonical(const json &v) {
  if (v.is_number_float())
    return python_float(v.get<double>());
  if (v.is_array()) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); i++) {
      if (i)
        s += ",";
      s += canonical(v[i]);
    }
    return s + "]";
  }
  if (v.is_object()) {
    std::string s = "{";
    bool first = true;
    for (auto i = v.begin(); i != v.end(); ++i) {
      if (!first)
        s += ",";
      first = false;
      s += json(i.key()).dump(-1, ' ', true) + ":" + canonical(i.value());
    }
    return s + "}";
  }
  return v.dump(-1, ' ', true);
}
namespace {
std::string general(double x) {
  std::ostringstream s;
  s << std::setprecision(6) << x;
  return s.str();
}
struct Band {
  json lo = nullptr, hi = nullptr;
  bool ls = false, hs = false, absolute = false;
  explicit Band(const json &v) {
    if (v.is_object()) {
      lo = v.value("lo", json());
      hi = v.value("hi", json());
      absolute = yes(v, "use_abs");
      auto text = str(v, "text");
      ls = text.find('>') != text.npos;
      hs = text.find('<') != text.npos;
    } else if (v.is_array() && v.size() == 2) {
      lo = v[0].get<double>();
      hi = v[1].get<double>();
    } else if (v.is_string()) {
      std::regex re("^\\s*(abs\\s+)?(<=|>=|<|>|==)\\s*(-?\\d+(?:\\.\\d+)?(?:["
                    "eE][-+]?\\d+)?)\\s*$");
      std::smatch m;
      auto s = v.get<std::string>();
      if (!std::regex_match(s, m, re))
        throw Error(422, "invalid target band");
      absolute = m[1].matched;
      std::string op = m[2];
      double value = std::stod(m[3]);
      if (op == "<" || op == "<=") {
        hi = value;
        hs = op == "<";
      } else if (op == ">" || op == ">=") {
        lo = value;
        ls = op == ">";
      } else {
        lo = hi = value;
      }
    } else
      throw Error(422, "target band must be a clause or [low,high]");
    if (lo.is_null() && hi.is_null())
      throw Error(422, "unbounded target");
    if (lo.is_number() && hi.is_number() && lo.get<double>() > hi.get<double>())
      throw Error(422, "inverted target band");
  }
  bool contains(double x) const {
    if (absolute)
      x = std::abs(x);
    return (lo.is_null() ||
            (ls ? x > lo.get<double>() : x >= lo.get<double>())) &&
           (hi.is_null() ||
            (hs ? x < hi.get<double>() : x <= hi.get<double>()));
  }
  json margin(double x) const {
    if (absolute)
      x = std::abs(x);
    if (lo.is_number() && hi.is_number()) {
      double half = (hi.get<double>() - lo.get<double>()) / 2;
      if (half <= 0)
        return nullptr;
      return (half - std::abs(x - (hi.get<double>() + lo.get<double>()) / 2)) /
             half;
    }
    if (hi.is_number()) {
      double h = hi;
      if (h == 0)
        return nullptr;
      return (h - x) / std::abs(h);
    }
    if (lo.is_number()) {
      double l = lo;
      if (l == 0)
        return nullptr;
      return (x - l) / std::abs(l);
    }
    return nullptr;
  }
  std::string describe() const {
    std::string prefix = absolute ? "|·| " : "";
    if (lo.is_number() && hi.is_number()) {
      if (lo == hi)
        return prefix + "= " + general(lo);
      return prefix + "∈ [" + general(lo) + ", " + general(hi) + "]";
    }
    if (hi.is_number())
      return prefix + (hs ? "< " : "≤ ") + general(hi);
    return prefix + (ls ? "> " : "≥ ") + general(lo);
  }
  json data() const {
    return {
        {"lo", lo}, {"hi", hi}, {"use_abs", absolute}, {"text", describe()}};
  }
};
json normalize_target(json v) {
  for (auto name : {"name", "applies_to", "entries"})
    if (!v.contains(name))
      throw Error(422, "target missing " + std::string(name));
  safe_name(str(v, "name"));
  if (v["entries"].empty() || !v["entries"].is_array())
    throw Error(422, "target entries required");
  json result = {{"name", v["name"]},        {"version", v.value("version", 1)},
                 {"title", str(v, "title")}, {"applies_to", v["applies_to"]},
                 {"owner", str(v, "owner")}, {"notes", str(v, "notes")},
                 {"entries", json::array()}};
  std::set<std::string> ids;
  for (auto &e : v["entries"]) {
    for (auto key : {"id", "metric", "limit", "source"})
      if (!e.contains(key))
        throw Error(422, "target entry missing field");
    if (!ids.insert(str(e, "id")).second)
      throw Error(422, "duplicate target id");
    if (str(e, "source").empty())
      throw Error(422, "target source required");
    Band limit(e["limit"]);
    json target = nullptr;
    if (e.contains("target") && !e["target"].is_null()) {
      Band wanted(e["target"]);
      if (wanted.absolute != limit.absolute ||
          (limit.lo.is_number() &&
           (wanted.lo.is_null() ||
            wanted.lo.get<double>() < limit.lo.get<double>())) ||
          (limit.hi.is_number() &&
           (wanted.hi.is_null() ||
            wanted.hi.get<double>() > limit.hi.get<double>())))
        throw Error(422, "target goal is outside requirement limit");
      target = wanted.data();
    }
    auto severity = str(e, "severity", "must");
    if (severity != "must" && severity != "should")
      throw Error(422, "unknown target severity");
    result["entries"].push_back({{"id", e["id"]},
                                 {"metric", e["metric"]},
                                 {"at", str(e, "at", "all")},
                                 {"limit", limit.data()},
                                 {"target", target},
                                 {"severity", severity},
                                 {"unit", str(e, "unit")},
                                 {"source", e["source"]},
                                 {"rationale", str(e, "rationale")}});
  }
  result["digest"] = hex_sha(canonical(result)).substr(0, 16);
  result["ref"] =
      str(result, "name") + "@" + std::to_string(result["version"].get<int>());
  return result;
}
} // namespace
json target_sets(const Storage &store) {
  auto data = store.root / "cpp_backend/data";
  if (!fs::exists(data))
    data = store.root / "app/data";
  json sets = read_json(data / "targets.json"), builtins = sets;
  std::vector<fs::path> paths;
  for (auto &p : fs::directory_iterator(store.dir("targets")))
    if (p.is_regular_file() &&
        (p.path().extension() == ".yaml" || p.path().extension() == ".yml"))
      paths.push_back(p.path());
  std::sort(paths.begin(), paths.end());
  for (auto &p : paths) {
    auto target = normalize_target(yaml_json(p));
    auto ref = str(target, "ref");
    if (builtins.contains(ref))
      throw Error(422,
                  "shipped target version is immutable; increment its version");
    sets[ref] = target;
  }
  return sets;
}
json target_set(const Storage &store, const std::string &ref) {
  auto sets = target_sets(store);
  if (sets.contains(ref))
    return sets[ref];
  json found;
  for (auto &v : sets)
    if (str(v, "name") == ref &&
        (found.is_null() || num(v, "version") > num(found, "version")))
      found = v;
  if (found.is_null())
    throw Error(404, "unknown target set: " + ref);
  return found;
}
json target_catalog(const Storage &store) {
  json out = json::array();
  for (auto &t : target_sets(store)) {
    json metrics = json::array();
    int must = 0;
    for (auto &e : t["entries"]) {
      if (std::find(metrics.begin(), metrics.end(), e["metric"]) ==
          metrics.end())
        metrics.push_back(e["metric"]);
      if (str(e, "severity") == "must")
        must++;
    }
    auto ref = str(t, "ref");
    out.push_back(
        {{"name", t["name"]},
         {"version", t["version"]},
         {"ref", ref},
         {"title", t["title"]},
         {"applies_to", t["applies_to"]},
         {"owner", t["owner"]},
         {"entries", t["entries"].size()},
         {"must", must},
         {"metrics", metrics},
         {"builtin", ref == "eps_actuator@1" || ref == "steering_feel@3"},
         {"digest", t["digest"]},
         {"notes", t["notes"]}});
  }
  return out;
}
json compliance(const json &t, const json &measurements) {
  json rows = json::array(), sources = json::array();
  std::set<std::string> src;
  for (auto &m : measurements) {
    auto source = str(m, "source");
    if (!source.empty())
      src.insert(source.substr(0, source.find(':')));
  }
  for (auto &s : src)
    sources.push_back(s);
  int must = 0, evaluated = 0, violated = 0, marginal = 0, missing = 0;
  bool must_fail = false, must_missing = false;
  for (auto &e : t["entries"]) {
    Band limit(e["limit"]);
    std::optional<Band> target;
    if (!e["target"].is_null())
      target.emplace(e["target"]);
    json row = {{"id", e["id"]},
                {"metric", e["metric"]},
                {"at", e["at"]},
                {"unit", e["unit"]},
                {"severity", e["severity"]},
                {"limit", limit.describe()},
                {"target", target ? json(target->describe()) : json()},
                {"status", "not_evaluated"},
                {"status_label", "未评估"},
                {"checked", 0},
                {"violated", 0},
                {"marginal", 0},
                {"worst_label", nullptr},
                {"worst_value", nullptr},
                {"margin_pct", nullptr},
                {"source", e["source"]},
                {"rationale", e["rationale"]},
                {"note", ""}};
    bool required = str(e, "severity") == "must";
    if (required)
      must++;
    try {
      auto filters = selectors(str(e, "at"));
      json selected = json::array();
      std::set<std::string> axes;
      int same = 0;
      for (auto &m : measurements)
        if (m["metric"] == e["metric"]) {
          same++;
          for (auto i = m["at"].begin(); i != m["at"].end(); ++i)
            axes.insert(i.key());
          if (matches(m["at"], filters))
            selected.push_back(m);
        }
      if (!same)
        throw Error(422, "no measured value for the requested metric");
      for (auto i = filters.begin(); i != filters.end(); ++i)
        if (!axes.count(i.key()))
          throw Error(422, "selector refers to unavailable measurement axis: " +
                               i.key());
      if (selected.empty())
        throw Error(422, "selector matched no measurement");
      int checked = 0, bad = 0, edge = 0;
      json worst = nullptr, worst_margin = nullptr, first = nullptr,
           undef = nullptr;
      for (auto &m : selected) {
        if (!yes(m, "trusted", true) || !m["value"].is_number() ||
            !std::isfinite(m["value"].get<double>()))
          continue;
        checked++;
        if (first.is_null())
          first = m;
        double value = m["value"];
        bool inside = limit.contains(value);
        if (!inside)
          bad++;
        else if (target && !target->contains(value))
          edge++;
        auto margin = limit.margin(value);
        if (margin.is_null()) {
          if (!inside && undef.is_null())
            undef = m;
        } else if (worst.is_null() ||
                   margin.get<double>() < worst_margin.get<double>()) {
          worst = m;
          worst_margin = margin;
        }
      }
      if (!checked)
        throw Error(422, "untrusted measurements cannot establish compliance");
      if (worst.is_null())
        worst = undef.is_null() ? first : undef;
      row["status"] = bad ? "violated" : edge ? "marginal" : "met";
      row["status_label"] = bad ? "超标" : edge ? "边际" : "达标";
      row["checked"] = checked;
      row["violated"] = bad;
      row["marginal"] = edge;
      row["worst_label"] = str(worst, "label", str(worst, "source"));
      row["worst_value"] = worst["value"];
      if (worst_margin.is_number())
        row["margin_pct"] =
            std::nearbyint(worst_margin.get<double>() * 1000) / 10;
      if (selected.size() > size_t(checked))
        row["note"] = std::to_string(selected.size() - checked) +
                      " untrusted measurements excluded";
    } catch (const Error &err) {
      row["note"] = err.what();
    }
    auto status = str(row, "status");
    if (status == "violated") {
      violated++;
      if (required)
        must_fail = true;
    }
    if (status == "marginal")
      marginal++;
    if (status == "not_evaluated") {
      missing++;
      if (required)
        must_missing = true;
    } else if (required)
      evaluated++;
    rows.push_back(row);
  }
  std::string verdict = must_fail      ? "non_compliant"
                        : must_missing ? "incomplete"
                                       : "compliant";
  return {{"target_set", t["name"]},
          {"version", t["version"]},
          {"ref", t["ref"]},
          {"digest", t["digest"]},
          {"applies_to", t["applies_to"]},
          {"measured_from", sources},
          {"verdict", verdict},
          {"verdict_label", must_fail      ? "不符合"
                            : must_missing ? "覆盖不全"
                                           : "符合"},
          {"coverage",
           must ? std::nearbyint(double(evaluated) / must * 1000) / 1000 : 1.},
          {"counts",
           {{"total", rows.size()},
            {"must", must},
            {"must_evaluated", evaluated},
            {"violated", violated},
            {"marginal", marginal},
            {"not_evaluated", missing}}},
          {"rows", rows}};
}
} // namespace fourwis
