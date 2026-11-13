#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <numeric>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
namespace fourwis {
using json = nlohmann::json;
// Windows CRT narrow environment values use the system code page, not UTF-8.
// Read native wide paths so Chinese installation/data locations remain intact.
inline std::optional<std::filesystem::path> env_path(const char *name) {
#ifdef _WIN32
  std::wstring key;
  while (*name)
    key.push_back(static_cast<wchar_t>(*name++));
  const wchar_t *value = _wgetenv(key.c_str());
  if (value && *value)
    return std::filesystem::path(value);
#else
  const char *value = std::getenv(name);
  if (value && *value)
    return std::filesystem::u8path(value);
#endif
  return std::nullopt;
}
inline constexpr const char *wheel_names[] = {"fl", "fr", "rl", "rr"};
using V4 = std::array<double, 4>;
using V2 = std::array<double, 2>;
using Wheels = std::array<V2, 4>;
constexpr double pi = 3.14159265358979323846;
inline constexpr double nan = std::numeric_limits<double>::quiet_NaN();
inline double clip(double v, double lo, double hi) {
  return std::min(std::max(v, lo), hi);
}
inline double sign(double x) { return (x > 0) - (x < 0); }
inline double rad(double x) { return x * pi / 180.0; }
inline double deg(double x) { return x * 180.0 / pi; }
inline double wrap(double x) {
  double y = std::fmod(x + pi, 2 * pi);
  return (y < 0 ? y + 2 * pi : y) - pi;
}
inline double halfwrap(double x) {
  double y = wrap(x);
  return y > pi / 2 ? y - pi : y < -pi / 2 ? y + pi : y;
}
inline double num(const json &j, const std::string &k, double d = 0) {
  auto i = j.find(k);
  return i != j.end() && !i->is_null() ? i->get<double>() : d;
}
inline std::string str(const json &j, const std::string &k,
                       std::string d = "") {
  auto i = j.find(k);
  return i != j.end() && !i->is_null() ? i->get<std::string>() : d;
}
inline bool yes(const json &j, const std::string &k, bool d = false) {
  auto i = j.find(k);
  return i != j.end() && !i->is_null() ? i->get<bool>() : d;
}
inline json read_json(const std::filesystem::path &p) {
  std::ifstream f(p);
  if (!f)
    throw std::runtime_error("cannot read " + p.string());
  json j;
  f >> j;
  return j;
}
inline void merge(json &dst, const json &patch) {
  for (auto it = patch.begin(); it != patch.end(); ++it) {
    if (it.value().is_object() && dst.contains(it.key()) &&
        dst[it.key()].is_object())
      merge(dst[it.key()], it.value());
    else
      dst[it.key()] = it.value();
  }
}
inline double interp(double v, const std::vector<double> &x,
                     const std::vector<double> &y) {
  if (x.empty())
    return 0;
  if (v <= x.front())
    return y.front();
  if (v >= x.back())
    return y.back();
  auto k = std::upper_bound(x.begin(), x.end(), v) - x.begin();
  return y[k - 1] + (y[k] - y[k - 1]) * (v - x[k - 1]) / (x[k] - x[k - 1]);
}
struct Error : std::runtime_error {
  int status;
  json detail;
  Error(int code, const std::string &s)
      : std::runtime_error(s), status(code), detail(s) {}
  Error(int status_, const std::string &message, const std::string &code)
      : std::runtime_error(message), status(status_),
        detail({{"code", code}, {"message", message}}) {}
};
} // namespace fourwis
