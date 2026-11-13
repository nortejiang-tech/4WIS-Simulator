#include "metrics.hpp"
#include <sstream>
namespace fourwis {
namespace {
std::string trim(std::string s) {
  auto a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
  return a == s.npos ? "" : s.substr(a, b - a + 1);
}
bool close(const json &a, const json &b) {
  if (a.is_number() && b.is_number()) {
    double x = a, y = b;
    return std::abs(x - y) <=
           std::max(1e-12, 1e-9 * std::max(std::abs(x), std::abs(y)));
  }
  return a == b;
}
} // namespace
json selectors(const std::string &input) {
  auto s = trim(input), low = s;
  std::transform(low.begin(), low.end(), low.begin(),
                 [](unsigned char c) { return char(std::tolower(c)); });
  if (low.empty() || low == "all" || low == "*")
    return json::object();
  std::regex conjunction("\\s+and\\s+", std::regex::icase),
      eq("^\\s*([A-Za-z_][A-Za-z_0-9]*)\\s*==\\s*(.+?)\\s*$");
  json out = json::object();
  for (std::sregex_token_iterator it(s.begin(), s.end(), conjunction, -1), end;
       it != end; ++it) {
    std::smatch match;
    auto term = it->str();
    if (!std::regex_match(term, match, eq))
      throw Error(422, "unparseable criterion selector: " + input);
    auto text = trim(match[2]);
    while (!text.empty() && (text.front() == '\'' || text.front() == '"'))
      text.erase(text.begin());
    while (!text.empty() && (text.back() == '\'' || text.back() == '"'))
      text.pop_back();
    json v;
    try {
      v = json::parse(text);
      if (!v.is_number() && !v.is_boolean())
        v = text;
    } catch (...) {
      std::string l = text;
      std::transform(l.begin(), l.end(), l.begin(),
                     [](unsigned char c) { return char(std::tolower(c)); });
      v = l == "true" ? json(true) : l == "false" ? json(false) : json(text);
    }
    out[match[1].str()] = v;
  }
  return out;
}
bool matches(const json &coords, const json &filters) {
  for (auto it = filters.begin(); it != filters.end(); ++it)
    if (!coords.contains(it.key()) || !close(coords[it.key()], it.value()))
      return false;
  return true;
}
bool holds(double v, const std::string &clause) {
  std::regex re("^\\s*(abs\\s+)?(<=|>=|<|>|==|!=)\\s*(-?\\d+(?:\\.\\d+)?(?:[eE]"
                "[-+]?\\d+)?)\\s*$");
  std::smatch m;
  if (!std::regex_match(clause, m, re))
    throw Error(422, "unparseable criterion: " + clause);
  if (m[1].matched)
    v = std::abs(v);
  double threshold = std::stod(m[3]);
  auto op = m[2].str();
  if (std::isnan(v))
    return false;
  if (op == "<")
    return v < threshold;
  if (op == "<=")
    return v <= threshold;
  if (op == ">")
    return v > threshold;
  if (op == ">=")
    return v >= threshold;
  bool equal =
      std::abs(v - threshold) <=
      std::max(1e-12, 1e-9 * std::max(std::abs(v), std::abs(threshold)));
  return op == "==" ? equal : !equal;
}
json criteria(const json &specs, const json &rows, const json &axes) {
  json out = json::array();
  for (auto &c : specs) {
    std::string name = str(c, "metric"), must = str(c, "must"),
                at = str(c, "at", "all");
    json v = {{"metric", name},         {"must", must},           {"at", at},
              {"passed", false},        {"checked", 0},           {"failed", 0},
              {"worst_label", nullptr}, {"worst_value", nullptr}, {"note", ""}};
    try {
      auto filters = selectors(at);
      for (auto it = filters.begin(); it != filters.end(); ++it)
        if (std::find(axes.begin(), axes.end(), json(it.key())) == axes.end())
          throw Error(422, "selector names unknown axis: " + it.key());
      int selected = 0, checked = 0, missing = 0, failed = 0;
      double worst = 0;
      bool abs = must.find("abs") != must.npos,
           upper = must.find('<') != must.npos;
      for (auto &r : rows)
        if (matches(r["coords"], filters)) {
          selected++;
          auto metrics = r["metrics"];
          if (!metrics.contains(name)) {
            missing++;
            continue;
          }
          checked++;
          double x =
              metrics[name].is_number() ? metrics[name].get<double>() : nan;
          if (!holds(x, must)) {
            double score = abs ? std::abs(x) : x;
            if (!failed || (upper ? score > worst : score < worst)) {
              worst = score;
              v["worst_label"] = r["label"];
              v["worst_value"] = x;
            }
            failed++;
          }
        }
      v["checked"] = checked;
      v["failed"] = failed;
      v["passed"] = checked > 0 && failed == 0;
      if (!selected)
        v["note"] = "selector matched no cells — a criterion checked over "
                    "nothing is not satisfied";
      else if (!checked)
        v["note"] = "metric is missing from all selected cells";
      else if (missing)
        v["note"] = std::to_string(missing) + " cell(s) lacked the metric";
    } catch (const Error &e) {
      v["note"] = e.what();
    }
    out.push_back(v);
  }
  return out;
}
json comparison(const json &rows, const json &names, const json &axes,
                const json &spec) {
  json out = json::object();
  auto group = str(spec, "group_by");
  if (!group.empty()) {
    out["group_by"] = group;
    out["groups"] = json::array();
    std::vector<json> keys;
    for (auto &r : rows)
      if (r["coords"].contains(group)) {
        auto key = r["coords"][group];
        if (std::find(keys.begin(), keys.end(), key) == keys.end())
          keys.push_back(key);
      }
    for (auto &key : keys) {
      json metrics = json::object();
      int n = 0;
      for (auto &r : rows)
        if (r["coords"].value(group, json()) == key)
          n++;
      for (auto &name : names) {
        std::vector<double> v;
        for (auto &r : rows)
          if (r["coords"].value(group, json()) == key &&
              r["metrics"].contains(name.get<std::string>()) &&
              r["metrics"][name.get<std::string>()].is_number()) {
            double x = r["metrics"][name.get<std::string>()];
            if (!std::isnan(x))
              v.push_back(x);
          }
        if (!v.empty()) {
          double lo = *std::min_element(v.begin(), v.end()),
                 hi = *std::max_element(v.begin(), v.end());
          metrics[name.get<std::string>()] = {
              {"mean", std::accumulate(v.begin(), v.end(), 0.) / v.size()},
              {"min", lo},
              {"max", hi},
              {"spread", hi - lo},
              {"n", v.size()}};
        }
      }
      out["groups"].push_back({{group, key}, {"n", n}, {"metrics", metrics}});
    }
  }
  auto against = spec.value("against", json());
  if (against.is_object() && !against.empty()) {
    json refs = json::array();
    for (auto &r : rows)
      if (matches(r["coords"], against))
        refs.push_back(r);
    if (refs.empty())
      out["warnings"] =
          json::array({"compare.against matched no cell — no deltas computed"});
    else {
      out["against"] = against;
      out["reference_labels"] = json::array();
      for (auto &r : refs)
        out["reference_labels"].push_back(r["label"]);
      out["deltas"] = json::array();
      auto key = [&](const json &r) {
        json values = json::array();
        for (auto &a : axes)
          if (!against.contains(a.get<std::string>()))
            values.push_back(r["coords"].value(a.get<std::string>(), json()));
        return values.dump();
      };
      std::map<std::string, json> index;
      for (auto &r : refs)
        index.emplace(key(r), r);
      for (auto &r : rows) {
        auto it = index.find(key(r));
        if (it == index.end())
          continue;
        auto &ref = it->second;
        json metrics = json::object();
        for (auto &m : names) {
          auto n = m.get<std::string>();
          if (!r["metrics"].contains(n) || !ref["metrics"].contains(n) ||
              !r["metrics"][n].is_number() || !ref["metrics"][n].is_number())
            continue;
          double v = r["metrics"][n], rv = ref["metrics"][n];
          metrics[n] = {
              {"value", v},
              {"reference", rv},
              {"delta", v - rv},
              {"delta_pct",
               rv == 0 ? json() : json((v - rv) / std::abs(rv) * 100)}};
        }
        out["deltas"].push_back({{"label", r["label"]},
                                 {"coords", r["coords"]},
                                 {"reference_label", ref["label"]},
                                 {"metrics", metrics}});
      }
    }
  }
  return out;
}
} // namespace fourwis
