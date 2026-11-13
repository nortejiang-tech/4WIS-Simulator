#pragma once
#include "simulation.hpp"
#include <regex>
namespace fourwis {
namespace fs = std::filesystem;
double epoch();
std::string uid();
std::string stamp(const char *format);
std::string hex_sha(const std::string &data);
std::string file_text(const fs::path &path);
void atomic_text(const fs::path &path, const std::string &text);
json yaml_json(const fs::path &path);
json parse_yaml(const std::string &text);
std::string safe_name(const std::string &name);
std::string unit(const std::string &name);
std::string escaped(const std::string &text);
std::string
zip_store(const std::vector<std::pair<std::string, std::string>> &files);
json validate_json(json value, const json &schema, const json &document,
                   const std::string &where = "body", int depth = 0);
json kpis(const Run &r, const json &experiment, bool recorded = false);
std::string run_csv(const Run &r, const std::vector<std::string> &names = {});
class Storage {
public:
  fs::path root;
  json defaults, contract, catalog;
  explicit Storage(fs::path path);
  fs::path dir(const std::string &name) const;
  std::vector<std::string> names(const std::string &folder) const;
  json load(const std::string &folder, const std::string &name) const;
  fs::path save(const std::string &folder, const std::string &name,
                const json &data) const;
  json profile(const std::string &name) const;
  VehicleParams params(const json &vehicle) const;
  json experiment(const json &definition) const;
  std::string save_run(const Run &r, const std::string &label,
                       const json &metrics, const json &job = nullptr) const;
  json run_meta(const std::string &id) const;
  json run_data(const std::string &id,
                const std::vector<std::string> &names = {},
                int stride = 1) const;
  json runs() const;
  json manifest(const std::string &id) const;
  json rows(const std::string &id) const;
  std::string plot(const std::string &id, const std::string &kind,
                   const std::string &channel) const;
  std::string bundle(const std::string &id) const;
};
} // namespace fourwis
