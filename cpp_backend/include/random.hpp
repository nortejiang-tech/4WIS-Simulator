#pragma once
#include "common.hpp"
#include <charconv>
#include <cstdint>
#ifdef _MSC_VER
#include <intrin.h>
#endif
#include "pcg64.h"
#ifdef inline
#undef inline
#endif
namespace fourwis {
// SeedSequence/PCG64/Ziggurat compatibility with NumPy 2.4.6. See vendored
// BSD/MIT licenses.
class Random {
  pcg64_random_t state{};

public:
  explicit Random(uint64_t seed = 0);
  uint64_t next() { return pcg64_random_r(&state); }
  double uniform() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
  double normal();
};
std::array<uint8_t, 32> sha256(const std::string &input);
inline std::string python_float(double x) {
  char buf[128];
  auto fmt = std::abs(x) != 0 && (std::abs(x) < 1e-4 || std::abs(x) >= 1e16)
                 ? std::chars_format::scientific
                 : std::chars_format::fixed;
  auto r = std::to_chars(buf, buf + sizeof(buf), x, fmt);
  if (r.ec != std::errc())
    throw std::runtime_error("float formatting failed");
  std::string s(buf, r.ptr);
  if (s.find('.') == s.npos && s.find('e') == s.npos)
    s += ".0";
  return s;
}
} // namespace fourwis
